#include "ax_model_damage.hpp"
#include "ax_freetype_overlay.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>  // 添加此行以使用 std::accumulate
#include <algorithm>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fstream>
#include <ctime>
#include <chrono>
#include <set>
#include "../../utilities/json.hpp"
#include "../../camera/camera_controller.hpp"

// 把当前帧关联的相机巡检状态快照写入 results，避免 draw_custom 再实时查相机状态导致视频流延迟错位
static void fill_camera_frame_state(axdl_results_t *results, int camera_id)
{
    if (!results) return;

    // 先清零，防止没有关联 camera 时残留旧状态
    results->frame_point_id = 0;
    results->frame_posture_completed = false;
    results->frame_light_flag = 0;
    results->frame_phase_ready_ms = 0;
    results->frame_capture_ts_ms = 0;
    results->frame_should_capture = 0;

    auto *cam = CameraController::getInstance()->getCamera(camera_id);
    if (!cam) return;

    results->frame_point_id = cam->now_point_id;
    results->frame_posture_completed = cam->posture_completed.load();
    results->frame_light_flag = cam->light_phase_changed ? 1 : 0;
    results->frame_phase_ready_ms = cam->phase_ready_ms.load();
    results->frame_capture_ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::system_clock::now().time_since_epoch()).count();
    results->frame_should_capture = cam->frame_should_capture.load();
}

// ---------- 点位前后对比（同光照↔同光照）小工具 ----------
namespace {
    // 基线图永久路径：/wt_tech/conf/baseline/<orga>/<camera>/point<id>_L<flag>.png
    static std::string make_baseline_path(const std::string& orga,
                                          const std::string& cam_name,
                                          int point_id, int light_flag)
    {
        char p[320] = {0};
        snprintf(p, sizeof(p),
            "/wt_tech/conf/baseline/%s/%s/point%d_L%d.png",
            orga.c_str(), cam_name.c_str(), point_id, light_flag);
        return p;
    }

    static void ensure_parent_dir(const std::string& path)
    {
        size_t pos = path.find_last_of('/');
        if (pos == std::string::npos) return;
        std::string dir = path.substr(0, pos);
        if (access(dir.c_str(), 0) != 0) {
            std::string cmd = "mkdir -p " + dir;
            int rc = system(cmd.c_str());
            (void)rc;
        }
    }

    // CLAHE 光照规范化（缓解同光照下的细微亮度漂移）
    static cv::Mat clahe_bgr(const cv::Mat& bgr)
    {
        cv::Mat lab; cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> ch; cv::split(lab, ch);
        auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(ch[0], ch[0]);
        cv::merge(ch, lab);
        cv::Mat out; cv::cvtColor(lab, out, cv::COLOR_Lab2BGR);
        return out;
    }

    // phaseCorrelate 平移对齐：抗云台机械重复定位的像素级漂移
    static cv::Mat align_translation(const cv::Mat& cur_gray, const cv::Mat& base_gray)
    {
        try {
            cv::Mat cF, bF;
            cur_gray.convertTo(cF, CV_32F);
            base_gray.convertTo(bF, CV_32F);
            cv::Point2d sh = cv::phaseCorrelate(cF, bF);
            if (std::abs(sh.x) > base_gray.cols * 0.1 ||
                std::abs(sh.y) > base_gray.rows * 0.1) {
                return cur_gray.clone();
            }
            cv::Mat warp = (cv::Mat_<float>(2, 3) <<
                1, 0, (float)sh.x, 0, 1, (float)sh.y);
            cv::Mat aligned;
            cv::warpAffine(cur_gray, aligned, warp, base_gray.size(),
                           cv::INTER_LINEAR, cv::BORDER_REPLICATE);
            return aligned;
        } catch (const cv::Exception&) {
            return cur_gray.clone();
        }
    }

    // 近似 SSIM 图（基于均值/方差，避免引入 contrib）
    static cv::Mat ssim_map(const cv::Mat& i1, const cv::Mat& i2)
    {
        const double C1 = 6.5025, C2 = 58.5225;
        cv::Mat I1, I2;
        i1.convertTo(I1, CV_32F);
        i2.convertTo(I2, CV_32F);
        cv::Mat I1_2 = I1.mul(I1), I2_2 = I2.mul(I2), I1_I2 = I1.mul(I2);
        cv::Mat mu1, mu2;
        cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
        cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);
        cv::Mat mu1_2 = mu1.mul(mu1), mu2_2 = mu2.mul(mu2), mu12 = mu1.mul(mu2);
        cv::Mat s1, s2, s12;
        cv::GaussianBlur(I1_2, s1, cv::Size(11, 11), 1.5); s1 -= mu1_2;
        cv::GaussianBlur(I2_2, s2, cv::Size(11, 11), 1.5); s2 -= mu2_2;
        cv::GaussianBlur(I1_I2, s12, cv::Size(11, 11), 1.5); s12 -= mu12;
        cv::Mat t1 = 2 * mu12 + C1, t2 = 2 * s12 + C2, t3 = t1.mul(t2);
        cv::Mat d = (mu1_2 + mu2_2 + C1).mul(s1 + s2 + C2);
        cv::Mat ssim; cv::divide(t3, d, ssim);
        return ssim;
    }

    struct DiffRegion { cv::Rect bbox; float score; };

    // 当前帧 vs 基线帧 → 差异区域（基线坐标系）
    static std::vector<DiffRegion> diff_against_baseline(
        const cv::Mat& cur_bgr, const cv::Mat& base_bgr)
    {
        std::vector<DiffRegion> out;
        if (cur_bgr.empty() || base_bgr.empty()) return out;

        const float SSIM_TH   = 0.45f; // 严格：改小  宽松：改大
        const float ABSD_RATIO = 0.30f; // 像素差 > 30% 才算
        const int   BLOCK     = 96;
        const int   MIN_AREA  = 2000; // 过滤小区域噪点

        cv::Mat cur = cur_bgr;
        if (cur.size() != base_bgr.size())
            cv::resize(cur, cur, base_bgr.size());

        cv::Mat curN  = clahe_bgr(cur);
        cv::Mat baseN = clahe_bgr(base_bgr);

        cv::Mat curG, baseG;
        cv::cvtColor(curN, curG, cv::COLOR_BGR2GRAY);
        cv::cvtColor(baseN, baseG, cv::COLOR_BGR2GRAY);
        curG = align_translation(curG, baseG);

        cv::Mat ssim = ssim_map(curG, baseG);
        cv::Mat absd; cv::absdiff(curG, baseG, absd);

        const int H = baseG.rows, W = baseG.cols;
        cv::Mat susp = cv::Mat::zeros(H, W, CV_8U);
        for (int y = 0; y < H; y += BLOCK) {
            for (int x = 0; x < W; x += BLOCK) {
                cv::Rect r(x, y, std::min(BLOCK, W - x), std::min(BLOCK, H - y));
                float s = (float)cv::mean(ssim(r))[0];
                float d = (float)cv::mean(absd(r))[0] / 255.f;
                if (s < SSIM_TH && d > ABSD_RATIO) susp(r).setTo(255);
            }
        }
        cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
        cv::morphologyEx(susp, susp, cv::MORPH_CLOSE, k);
        cv::morphologyEx(susp, susp, cv::MORPH_OPEN,  k);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(susp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto& c : contours) {
            cv::Rect b = cv::boundingRect(c);
            if (b.area() < MIN_AREA) continue;
            b &= cv::Rect(0, 0, W, H);
            if (b.area() <= 0) continue;
            float s = (float)cv::mean(ssim(b))[0];
            float d = (float)cv::mean(absd(b))[0] / 255.f;
            float score = std::max(0.f, std::min(1.f, (1.f - s) * 0.5f + d * 0.5f));
            out.push_back({b, score});
        }
        std::sort(out.begin(), out.end(),
                  [](const DiffRegion& a, const DiffRegion& b){ return a.score > b.score; });
        return out;
    }
}

// ---------- YOLO11-OBB helper functions ----------
namespace {
    static inline float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

    static void softmax_inplace(float* data, int n)
    {
        float m = data[0];
        for (int i = 1; i < n; ++i) m = std::max(m, data[i]);
        float sum = 0.f;
        for (int i = 0; i < n; ++i) { data[i] = std::exp(data[i] - m); sum += data[i]; }
        const float inv = 1.f / (sum + 1e-12f);
        for (int i = 0; i < n; ++i) data[i] *= inv;
    }

    // probiou (rotated NMS IoU)
    static inline void rbox_cov(float w, float h, float a,
                                float& A, float& B, float& C)
    {
        const float a_ = w * w / 12.f;
        const float b_ = h * h / 12.f;
        const float cos2 = std::cos(a); const float sin2 = std::sin(a);
        A = a_ * cos2 * cos2 + b_ * sin2 * sin2;
        B = a_ * sin2 * sin2 + b_ * cos2 * cos2;
        C = (a_ - b_) * cos2 * sin2;
    }

    struct OBBObject
    {
        cv::Rect2f rect;   // (cx, cy, w, h)
        float angle;       // radians
        float prob;
        int   label;
    };

    static float probiou(const OBBObject& p, const OBBObject& q, float eps = 1e-7f)
    {
        float a1, b1, c1; rbox_cov(p.rect.width, p.rect.height, p.angle, a1, b1, c1);
        float a2, b2, c2; rbox_cov(q.rect.width, q.rect.height, q.angle, a2, b2, c2);
        const float x1 = p.rect.x, y1 = p.rect.y;
        const float x2 = q.rect.x, y2 = q.rect.y;

        const float s  = (a1 + a2) * (b1 + b2) - (c1 + c2) * (c1 + c2);
        const float t1 = ((a1 + a2) * (y1 - y2) * (y1 - y2) +
                          (b1 + b2) * (x1 - x2) * (x1 - x2)) / (s + eps) * 0.25f;
        const float t2 = ((c1 + c2) * (x2 - x1) * (y1 - y2)) / (s + eps) * 0.5f;
        const float inner = std::max(a1 * b1 - c1 * c1, 0.f) *
                            std::max(a2 * b2 - c2 * c2, 0.f);
        const float t3 = std::log(s / (4.f * std::sqrt(inner) + eps) + eps) * 0.5f;
        float bd = t1 + t2 + t3;
        bd = std::max(std::min(bd, 100.f), eps);
        return 1.f - std::sqrt(1.f - std::exp(-bd) + eps);
    }

    static void nms_rotated(const std::vector<OBBObject>& in, std::vector<int>& picked,
                            float iou_thres, bool agnostic = false,
                            float max_wh = 7680.f)
    {
        picked.clear();
        std::vector<int> order(in.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](int a, int b) { return in[a].prob > in[b].prob; });

        std::vector<uint8_t> suppressed(in.size(), 0);
        for (size_t i = 0; i < order.size(); ++i)
        {
            int ia = order[i];
            if (suppressed[ia]) continue;
            picked.push_back(ia);

            OBBObject a = in[ia];
            if (!agnostic)
            {
                a.rect.x += a.label * max_wh;
                a.rect.y += a.label * max_wh;
            }

            for (size_t j = i + 1; j < order.size(); ++j)
            {
                int ib = order[j];
                if (suppressed[ib]) continue;
                OBBObject b = in[ib];
                if (!agnostic)
                {
                    b.rect.x += b.label * max_wh;
                    b.rect.y += b.label * max_wh;
                }
                if (probiou(a, b) >= iou_thres) suppressed[ib] = 1;
            }
        }
    }

    // Decode one scale for YOLO11-OBB
    static void decode_scale_yolo11_obb(const float* feat_box, const float* feat_cls, const float* feat_ang,
                                        int H, int W, int stride,
                                        int reg_max, int nc, int ne,
                                        float score_thres,
                                        std::vector<OBBObject>& out)
    {
        const float conf_raw = -std::log(1.f / score_thres - 1.f);
        const int   box_ch = 4 * reg_max;

        std::vector<float> bins(reg_max);

        for (int y = 0; y < H; ++y)
        {
            for (int x = 0; x < W; ++x)
            {
                const int idx = y * W + x;
                const float* pcls = feat_cls + idx * nc;

                int best = 0;
                float best_raw = pcls[0];
                for (int k = 1; k < nc; ++k)
                {
                    if (pcls[k] > best_raw) { best_raw = pcls[k]; best = k; }
                }
                if (best_raw < conf_raw) continue;

                const float* pbox = feat_box + idx * box_ch;
                float dist[4];
                for (int k = 0; k < 4; ++k)
                {
                    std::memcpy(bins.data(), pbox + k * reg_max, sizeof(float) * reg_max);
                    softmax_inplace(bins.data(), reg_max);
                    float acc = 0.f;
                    for (int b = 0; b < reg_max; ++b) acc += bins[b] * b;
                    dist[k] = acc;
                }

                const float raw_ang = feat_ang[idx * ne];
                const float angle = (sigmoid(raw_ang) - 0.25f) * float(M_PI);

                const float anchor_x = x + 0.5f;
                const float anchor_y = y + 0.5f;
                const float cosA = std::cos(angle), sinA = std::sin(angle);
                const float xf = (dist[2] - dist[0]) * 0.5f;
                const float yf = (dist[3] - dist[1]) * 0.5f;

                OBBObject obj;
                obj.rect.x      = (xf * cosA - yf * sinA + anchor_x) * stride;
                obj.rect.y      = (xf * sinA + yf * cosA + anchor_y) * stride;
                obj.rect.width  = (dist[0] + dist[2]) * stride;
                obj.rect.height = (dist[1] + dist[3]) * stride;
                obj.angle = angle;
                obj.prob  = sigmoid(best_raw);
                obj.label = best;
                out.push_back(obj);
            }
        }
    }

    // Letterbox inverse + angle regularize for YOLO11-OBB
    static void scale_back_yolo11_obb(std::vector<OBBObject>& objs, int letterbox_h, int letterbox_w,
                                       int src_h, int src_w)
    {
        const float r_h = (float)letterbox_h / src_h;
        const float r_w = (float)letterbox_w / src_w;
        const float scale = std::min(r_h, r_w);
        const int resized_w = int(scale * src_w);
        const int resized_h = int(scale * src_h);
        const int pad_w = (letterbox_w - resized_w) / 2;
        const int pad_h = (letterbox_h - resized_h) / 2;
        const float inv = 1.f / scale;
        const float pi_2 = float(M_PI_2);
        const float pi   = float(M_PI);

        for (auto& o : objs)
        {
            float w_ = std::max(o.rect.width, o.rect.height);
            float h_ = std::min(o.rect.width, o.rect.height);
            float a_ = std::fmod((o.rect.width > o.rect.height ? o.angle : o.angle + pi_2), pi);

            o.rect.x      = std::max(std::min((o.rect.x - pad_w) * inv, (float)src_w - 1), 0.f);
            o.rect.y      = std::max(std::min((o.rect.y - pad_h) * inv, (float)src_h - 1), 0.f);
            o.rect.width  = w_ * inv;
            o.rect.height = h_ * inv;
            o.angle       = a_;
        }
    }

    // Helper to identify output kind
    static int kind_of(const std::string& name)
    {
        if (name.rfind("box_", 0) == 0) return 0;
        if (name.rfind("cls_", 0) == 0) return 1;
        if (name.rfind("ang_", 0) == 0) return 2;
        return -1;
    }

    static int scale_idx_of(const std::string& name)
    {
        auto pos = name.find("_scale");
        if (pos == std::string::npos) return -1;
        return std::atoi(name.c_str() + pos + 6);
    }

    static int stride_of(const std::string& name)
    {
        auto pos = name.find("_stride");
        if (pos == std::string::npos) return -1;
        return std::atoi(name.c_str() + pos + 7);
    }

    // Convert OBBObject to detection::Object with vertices
    static void convert_obb_to_detection_object(const OBBObject& obb, detection::Object& obj)
    {
        obj.rect.x = obb.rect.x;
        obj.rect.y = obb.rect.y;
        obj.rect.width = obb.rect.width;
        obj.rect.height = obb.rect.height;
        obj.label = obb.label;
        obj.prob = obb.prob;
        obj.angle = obb.angle;

        // Calculate rotated rectangle vertices
        cv::RotatedRect rr(cv::Point2f(obb.rect.x, obb.rect.y),
                           cv::Size2f(obb.rect.width, obb.rect.height),
                           obb.angle * 180.f / float(M_PI));
        cv::Point2f pts[4];
        rr.points(pts);
        for (int k = 0; k < 4; ++k) {
            obj.obb_vertices[k].x = pts[k].x;
            obj.obb_vertices[k].y = pts[k].y;
        }
    }
}

int ax_model_damage::post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    // 缓存原图用于快照合成（将 NV12/RGB/BGR 转换为 BGR）
    if (pstFrame && pstFrame->pVir) {
        std::lock_guard<std::mutex> lk(m_frame_mutex);
        try {
            if (pstFrame->eDtype == axdl_color_space_nv12) {
                cv::Mat nv12(pstFrame->nHeight * 3 / 2, pstFrame->nWidth, CV_8UC1, pstFrame->pVir);
                cv::cvtColor(nv12, m_cached_frame_bgr, cv::COLOR_YUV2BGR_NV12);
            } else if (pstFrame->eDtype == axdl_color_space_rgb) {
                cv::Mat rgb(pstFrame->nHeight, pstFrame->nWidth, CV_8UC3, pstFrame->pVir);
                cv::cvtColor(rgb, m_cached_frame_bgr, cv::COLOR_RGB2BGR);
            } else if (pstFrame->eDtype == axdl_color_space_bgr) {
                cv::Mat bgr(pstFrame->nHeight, pstFrame->nWidth, CV_8UC3, pstFrame->pVir);
                m_cached_frame_bgr = bgr.clone();
            }
        } catch (...) {
            // 转换失败时保持旧缓存
        }
    }

    // 获取模型输出
    const ax_runner_tensor_t *pOutputsInfo = m_runner->get_outputs_ptr();
    int num_outputs = m_runner->get_num_outputs();

    // 检测输出格式：YOLO11-OBB 有多个输出（box, cls, ang 分离）
    bool is_yolo11_obb = false;
    if (num_outputs >= 3) {
        // 检查输出名称是否符合 YOLO11-OBB 格式
        int box_count = 0, cls_count = 0, ang_count = 0;
        for (int i = 0; i < num_outputs; ++i) {
            std::string name = pOutputsInfo[i].sName;
            int kind = kind_of(name);
            if (kind == 0) box_count++;
            else if (kind == 1) cls_count++;
            else if (kind == 2) ang_count++;
        }
        is_yolo11_obb = (box_count >= 1 && cls_count >= 1 && ang_count >= 1);
    }

    if (is_yolo11_obb) {
        // YOLO11-OBB 多输出解析逻辑
        // Group outputs by scale
        struct ScaleOutputs {
            int stride;
            int H, W;
            int box_ch, cls_ch, ang_ch;
            const float* box_ptr;
            const float* cls_ptr;
            const float* ang_ptr;
        };
        std::vector<ScaleOutputs> scales(3);
        for (auto& s : scales) { s.stride = -1; }

        for (int i = 0; i < num_outputs; ++i) {
            std::string name = pOutputsInfo[i].sName;
            int kind = kind_of(name);
            int sid  = scale_idx_of(name);
            int strd = stride_of(name);
            if (kind < 0 || sid < 0 || sid >= (int)scales.size() || strd < 0) continue;

            const auto& shape = pOutputsInfo[i].vShape;
            if (shape.size() != 4) continue;
            // NHWC: (1, H, W, C)
            int H = (int)shape[1], W = (int)shape[2], C = (int)shape[3];

            scales[sid].stride = strd;
            scales[sid].H = H;
            scales[sid].W = W;
            const float* ptr = (const float*)pOutputsInfo[i].pVirAddr;
            if      (kind == 0) { scales[sid].box_ch = C; scales[sid].box_ptr = ptr; }
            else if (kind == 1) { scales[sid].cls_ch = C; scales[sid].cls_ptr = ptr; }
            else                { scales[sid].ang_ch = C; scales[sid].ang_ptr = ptr; }
        }

        // Decode all scales
        std::vector<OBBObject> raw;
        int num_classes = CLASS_NUM;
        for (const auto& sc : scales) {
            if (sc.stride <= 0) continue;
            int reg_max = sc.box_ch / 4;
            decode_scale_yolo11_obb(sc.box_ptr, sc.cls_ptr, sc.ang_ptr, sc.H, sc.W, sc.stride,
                reg_max, sc.cls_ch, sc.ang_ch, PROB_THRESHOLD, raw);
        }

        // Letterbox inverse
        scale_back_yolo11_obb(raw, get_algo_height(), get_algo_width(), HEIGHT_DET_BBOX_RESTORE, WIDTH_DET_BBOX_RESTORE);

        // Apply rotated NMS
        std::vector<int> kept;
        nms_rotated(raw, kept, NMS_THRESHOLD, false);

        // Convert to detection::Object format
        std::vector<detection::Object> objects;
        objects.reserve(kept.size());
        for (int k : kept) {
            detection::Object obj;
            convert_obb_to_detection_object(raw[k], obj);
            objects.push_back(obj);
        }

        // Convert to axdl_results_t format
        results->nObjSize = MIN(objects.size(), SAMPLE_MAX_BBOX_COUNT);
        for (int i = 0; i < results->nObjSize; i++) {
            const detection::Object& obj = objects[i];

            // Set OBB vertices:
            results->mObjects[i].bHasBoxVertices = 1;
            for (int j = 0; j < 4; j++) {
                results->mObjects[i].bbox_vertices[j].x = obj.obb_vertices[j].x;
                results->mObjects[i].bbox_vertices[j].y = obj.obb_vertices[j].y;
            }

            // Set object name using OBB class names
            if (obj.label < (int)CLASS_NAMES.size()) {
                strncpy(results->mObjects[i].objname, CLASS_NAMES[obj.label].c_str(), sizeof(results->mObjects[i].objname) - 1);
                results->mObjects[i].objname[sizeof(results->mObjects[i].objname) - 1] = '\0';
            } else {
                strncpy(results->mObjects[i].objname, "unknown", sizeof(results->mObjects[i].objname) - 1);
                results->mObjects[i].objname[sizeof(results->mObjects[i].objname) - 1] = '\0';
            }
        }

        return 0;
    } else {
        // 原有的 YOLOv8-OBB 单输出解析逻辑
        WTALOGI("Using YOLOv8-OBB single-output parsing");

        const float *output_ptr = (float *)pOutputsInfo[0].pVirAddr;

        // 解析输出形状
        int num_anchors = 0;
        int num_classes = CLASS_NUM;

        if (pOutputsInfo[0].vShape.size() >= 3) {
            num_anchors = (int)pOutputsInfo[0].vShape[2];
            int channels = (int)pOutputsInfo[0].vShape[1];
        } else {
            WTALOGI("YOLOv8 OBB requires 3D output [batch, channels, anchors]");
            return -1;
        }

        if (num_anchors <= 0) {
            WTALOGI("Invalid YOLOv8 OBB output shape");
            return -1;
        }

        // Generate grid strides for multi-scale detection
        std::vector<int> strides = {8, 16, 32};
        std::vector<detection::GridAndStride> grid_strides;
        detection::generate_grids_and_stride(get_algo_width(), get_algo_height(), strides, grid_strides);

        // Generate OBB proposals using AXERA-style approach
        std::vector<detection::Object> proposals;
        detection::generate_proposals_yolov8_obb_native(grid_strides, output_ptr, PROB_THRESHOLD, proposals,
            get_algo_width(), get_algo_height(), num_classes);

        // Apply NMS and coordinate transformation
        std::vector<detection::Object> objects;
        detection::get_out_obb_bbox(proposals, objects, NMS_THRESHOLD, get_algo_height(), get_algo_width(),
                                    HEIGHT_DET_BBOX_RESTORE, WIDTH_DET_BBOX_RESTORE);

        // Convert to axdl_results_t format
        results->nObjSize = MIN(objects.size(), SAMPLE_MAX_BBOX_COUNT);
        for (int i = 0; i < results->nObjSize; i++) {
            const detection::Object& obj = objects[i];

            // Set OBB vertices:
            results->mObjects[i].bHasBoxVertices = 1;
            for (int j = 0; j < 4; j++) {
                results->mObjects[i].bbox_vertices[j].x = obj.obb_vertices[j].x;
                results->mObjects[i].bbox_vertices[j].y = obj.obb_vertices[j].y;
            }

            // Set object name using OBB class names
            if (obj.label < (int)CLASS_NAMES.size()) {
                strncpy(results->mObjects[i].objname, CLASS_NAMES[obj.label].c_str(), sizeof(results->mObjects[i].objname) - 1);
                results->mObjects[i].objname[sizeof(results->mObjects[i].objname) - 1] = '\0';
            } else {
                strncpy(results->mObjects[i].objname, "unknown", sizeof(results->mObjects[i].objname) - 1);
                results->mObjects[i].objname[sizeof(results->mObjects[i].objname) - 1] = '\0';
            }
        }

        return 0;
    }
}


// 在 BGR 图上绘制水印（时间、通道名、点位）
// 用于无差异存无框原图时保留水印；diff 对比仍使用未加水印的原图与 baseline 比较
void ax_model_damage::draw_watermark_bgr(cv::Mat &bgr, int cur_point, bool is_moving)
{
    if (bgr.empty()) return;

    auto *cam = CameraController::getInstance()->getCamera(camera_id);

    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int text_thickness = 2;
    int baseline = 0;

    // 时间字符串（左上角）
    time_t now_ts = time(nullptr) + 8 * 3600;
    tm *t_info = gmtime(&now_ts);
    char time_str[64] = {0};
    snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d:%02d",
             t_info->tm_year + 1900, t_info->tm_mon + 1, t_info->tm_mday,
             t_info->tm_hour, t_info->tm_min, t_info->tm_sec);
    cv::putText(bgr, time_str, cv::Point(10, 30), font_face, font_scale, cv::Scalar(139, 0, 0), text_thickness);

    // 通道名称/摄像机名称（左下角，使用预渲染的 FreeType 位图，BGRA alpha 合成到 BGR）
    if (!m_channel_name_bmp.empty()) {
        int text_x = 10;
        int text_y = bgr.rows - 30;
        int w = std::min(m_channel_name_bmp.cols, bgr.cols - text_x);
        int h = std::min(m_channel_name_bmp.rows, text_y);
        if (w > 0 && h > 0) {
            cv::Mat roi = bgr(cv::Rect(text_x, text_y - h, w, h));
            for (int y = 0; y < h; ++y) {
                const cv::Vec4b* src = m_channel_name_bmp.ptr<cv::Vec4b>(y);
                cv::Vec3b* dst = roi.ptr<cv::Vec3b>(y);
                for (int x = 0; x < w; ++x) {
                    uint8_t a = src[x][3];
                    if (!a) continue;
                    float fa = a / 255.f;
                    for (int c = 0; c < 3; ++c)
                        dst[x][c] = cv::saturate_cast<uint8_t>(src[x][c] * fa + dst[x][c] * (1 - fa));
                }
            }
        }
    }

    // 点位信息（正下方居中）：仅巡逻状态且到达点位时绘制
    if (cam && cam->is_patroling() && cur_point > 0) {
        char point_str[128] = {0};
        snprintf(point_str, sizeof(point_str), "Point: %d (%s)", cur_point,
                 is_moving ? "moving..." : "arrived");
        cv::Size point_size = cv::getTextSize(point_str, font_face, font_scale, text_thickness, &baseline);
        int text_x = (bgr.cols - point_size.width) / 2;
        int text_y = bgr.rows - baseline - 10;
        cv::putText(bgr, point_str, cv::Point(text_x, text_y + baseline), font_face, font_scale,
                    cv::Scalar(139, 0, 0), text_thickness);
    }
}

void ax_model_damage::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    auto *cam = CameraController::getInstance()->getCamera(camera_id);

    // 仅在到达点位且灯光相位就绪时绘制当前帧 AI 检测框；移动/回位/灯光切换中不绘制
    // 使用本帧自带的帧状态，避免实时查相机状态导致视频流延迟错位
    if (cam && results->frame_posture_completed && results->frame_phase_ready_ms > 0) {
        draw_bbox(image, results, fontscale, thickness, offset_x, offset_y);
    }

    /* ---------- 绘制时间和点位信息 ---------- */
    time_t now_ts = time(nullptr) + 8 * 3600;
    tm *t_info = gmtime(&now_ts);
    char time_str[64] = {0};
    snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d:%02d",
             t_info->tm_year + 1900, t_info->tm_mon + 1, t_info->tm_mday,
             t_info->tm_hour, t_info->tm_min, t_info->tm_sec);

    int cur_point = results->frame_point_id;
    char point_str[128] = {0};
    bool is_moving = false;
    if (cam && results->frame_posture_completed) {
        snprintf(point_str, sizeof(point_str), "Point: %d (arrived)", cur_point);
    } else {
        snprintf(point_str, sizeof(point_str), "Point: %d (moving...)", cur_point);
        is_moving = true;
    }

    int text_y = 30;
    int text_x = 10;
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int text_thickness = 2;
    int baseline = 0;

    // 画面左上角绘制时间文字（透明底色）
    cv::putText(image, time_str, cv::Point(text_x, text_y), font_face, font_scale, cv::Scalar(139, 0, 0, 255), text_thickness);

    // 在画面左下角绘制通道名称/摄像机名称（使用预渲染的 FreeType 位图）
    if (!m_channel_name_bmp.empty()) {
        text_x = 10;
        text_y = image.rows - 30;
        int w = std::min(m_channel_name_bmp.cols, image.cols - text_x);
        int h = std::min(m_channel_name_bmp.rows, text_y);
        if (w > 0 && h > 0) {
            cv::Mat roi = image(cv::Rect(text_x, text_y - h, w, h));
            // alpha 合成（image 是 BGRA）
            for (int y = 0; y < h; ++y) {
                const cv::Vec4b* src = m_channel_name_bmp.ptr<cv::Vec4b>(y);
                cv::Vec4b* dst = roi.ptr<cv::Vec4b>(y);
                for (int x = 0; x < w; ++x) {
                    uint8_t a = src[x][3];
                    if (!a) continue;
                    float fa = a / 255.f;
                    for (int c = 0; c < 3; ++c)
                        dst[x][c] = cv::saturate_cast<uint8_t>(src[x][c] * fa + dst[x][c] * (1 - fa));
                    dst[x][3] = std::max(dst[x][3], a);
                }
            }
        }
    }

    if (!cam || !cam->is_patroling() || cur_point <= 0) return; // 非巡逻状态或回位中不绘制以下内容

    // 画面正下方绘制点位文字背景及文字
    cv::Size point_size = cv::getTextSize(point_str, font_face, font_scale, text_thickness, &baseline);
    // 计算画面下方的居中位置
    int image_width = image.cols;
    text_x = (image_width - point_size.width) / 2;  // 居中
    text_y = image.rows - baseline - 10;  // 距离底部10像素

    // 计算呼吸灯系数 (0.2 ~ 1.0)
    double breath_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    double breath_factor = 0.2 + 0.8 * (std::sin(breath_time * 2.0 * M_PI / 2.0) + 1.0) / 2.0; // 2秒为一个周期

    // 动态计算颜色：移动时呼吸灯效果，到达时保持原色
    int point_color_val = is_moving ? static_cast<int>(139 * breath_factor) : 139;
    cv::Scalar point_color = cv::Scalar(point_color_val, 0, 0);

    cv::putText(image, point_str, cv::Point(text_x, text_y + baseline), font_face, font_scale, cv::Scalar(point_color[0], point_color[1], point_color[2], 255), text_thickness);

    // 灯光状态：使用本帧自带的帧状态，0=L0(开灯), 1=L1(关灯/低照)
    int cur_light_flag = results->frame_light_flag;
    int fired_key = cur_point * 10 + cur_light_flag;

    // 相位就绪：到位 + 灯光正确 + 巡检线程已 arm + 额外流延迟余量
    // 避免在 RTSP 解码缓冲中的旧帧上累积/拍照（例如灯光切换后还未实际呈现的画面）
    const long long STREAM_LATENCY_SETTLE_MS = 90; // 流延迟余量，可调
    // 相位就绪：使用本帧自带的 phase_ready_ms 和 capture_ts_ms，避免用当前系统时间判断旧帧
    long long phase_ready = results->frame_phase_ready_ms;
    long long now_ms = results->frame_capture_ts_ms;
    bool phase_settled = (phase_ready > 0) && (now_ms - phase_ready >= STREAM_LATENCY_SETTLE_MS);

    // ★ 每帧累积检测结果（只有相位真正就绪后才累积，避免灯光/移动过程中的误累积）
    if (results->frame_posture_completed && phase_settled && results->nObjSize > 0) {
        cam->accumulate_detection(results);
    }

    // 每个点位触发两次拍照（有灯照+无灯照）：
    // - 使用 photo_fired_keys 记录已拍照的 (point_id, light_flag) 组合
    // - key = point_id * 10 + light_flag，巡检开始时清空
    // - 直接使用 frame_should_capture 标记，由巡检线程控制
    if (results->frame_should_capture == 0) {
        return; // 巡检线程未标记拍照，跳过
    }

    // 根据 frame_should_capture 确定 cur_light_flag
    cur_light_flag = (results->frame_should_capture == 1) ? 0 : 1;
    fired_key = cur_point * 10 + cur_light_flag;

    // 去重检查
    if (cam->photo_fired_keys.count(fired_key)) {
        return;
    }

    WTALOGI("[draw_custom] 进入拍照逻辑: 点位[%d] L%d", cur_point, cur_light_flag);

    // 保存两份图片：
    // 1. 原图（不带检测框）→ _raw.png，用于 diff 对比
    // 2. 带框图（原图+叠加层）→ .png，用于展示告警
    cv::Mat raw_image;      // 不带框的原图（裁剪 letterbox 黑边后 resize）
    cv::Mat merged_image;   // 带框的合并图
    {
        std::lock_guard<std::mutex> lk(m_frame_mutex);
        if (!m_cached_frame_bgr.empty()) {
            // m_cached_frame_bgr 是 letterbox 填充后的图像（如 640×640 带黑边）
            // 需要裁剪掉黑边，只保留有效图像区域，再 resize 到 overlay 尺寸
            int src_h = m_cached_frame_bgr.rows;
            int src_w = m_cached_frame_bgr.cols;
            int dst_h = HEIGHT_DET_BBOX_RESTORE;
            int dst_w = WIDTH_DET_BBOX_RESTORE;

            // 计算 letterbox 的缩放比例和 padding
            float scale = std::min((float)src_w / dst_w, (float)src_h / dst_h);
            int new_w = (int)(dst_w * scale);
            int new_h = (int)(dst_h * scale);
            int pad_x = (src_w - new_w) / 2;
            int pad_y = (src_h - new_h) / 2;

            // 裁剪有效区域（去除黑边）
            cv::Rect valid_roi(pad_x, pad_y, new_w, new_h);
            valid_roi &= cv::Rect(0, 0, src_w, src_h); // 确保不越界
            cv::Mat cropped = m_cached_frame_bgr(valid_roi);

            // resize 到与 overlay (IVPS) 相同尺寸
            cv::Mat base_resized;
            cv::resize(cropped, base_resized, cv::Size(image.cols, image.rows));
            raw_image = base_resized.clone();

            // 将 resize 后的原图转换为 BGRA（4通道）
            cv::Mat base_bgra;
            cv::cvtColor(base_resized, base_bgra, cv::COLOR_BGR2BGRA);

            // Alpha 混合：将 RGBA 叠加层合并到原图上
            cv::Mat overlay_bgra;
            cv::cvtColor(image, overlay_bgra, cv::COLOR_RGBA2BGRA);

            // 逐像素 Alpha 混合
            merged_image = base_bgra.clone();
            for (int y = 0; y < merged_image.rows; ++y) {
                for (int x = 0; x < merged_image.cols; ++x) {
                    cv::Vec4b& dst = merged_image.at<cv::Vec4b>(y, x);
                    const cv::Vec4b& src = overlay_bgra.at<cv::Vec4b>(y, x);
                    float alpha = src[3] / 255.0f;
                    if (alpha > 0.01f) {
                        dst[0] = cv::saturate_cast<uchar>(src[0] * alpha + dst[0] * (1 - alpha));
                        dst[1] = cv::saturate_cast<uchar>(src[1] * alpha + dst[1] * (1 - alpha));
                        dst[2] = cv::saturate_cast<uchar>(src[2] * alpha + dst[2] * (1 - alpha));
                    }
                }
            }
            cv::cvtColor(merged_image, merged_image, cv::COLOR_BGRA2BGR);
        } else {
            cv::cvtColor(image, raw_image, cv::COLOR_RGBA2BGR);
            merged_image = raw_image.clone();
        }
    }

    // ★ diff 门控：当前图与基线对比，改变量达到阈值才执行模型识别（不产生独立 diff 告警，不绘制 diff 框）
    //   - 基线存在且改变量未达阈值 → 存无框原图，不识别不告警，直接返回
    //   - 基线存在且改变量达阈值 → 不绘制 diff 框，继续走模型累积框 + 告警
    //   - 基线不存在（未标定）→ 无法门控，照常存图告警（并由巡检结束后建基线）
    //   - 标定模式 → 跳过门控，强制存图并入队用于建/更新基线
    if (!cam->is_calibrating()) {
        std::string base_path = make_baseline_path(cam->orga_name, cam->getName(),
                                                   cur_point, cur_light_flag);
        cv::Mat base_bgr = cv::imread(base_path, cv::IMREAD_COLOR);
        if (!base_bgr.empty()) {
            auto diff_regions = diff_against_baseline(raw_image, base_bgr);
            if (diff_regions.empty()) {
                // 无差异 → 存无框原图（保留点位记录），不告警
                // 注意：diff 对比已用未加水印的 raw_image 完成，此处对副本加水印后再存，
                //       避免污染 baseline 对比（baseline 无水印）
                cv::Mat wm_image = raw_image.clone();
                draw_watermark_bgr(wm_image, cur_point, false);
                cam->captureSnapshot(wm_image, cur_point, cur_light_flag);
                cam->photo_fired_keys.insert(fired_key);
                cam->frame_should_capture.store(0); // 通知巡检线程本相位拍照完成
                if (cur_light_flag == 1) cam->light_phase_changed = false;
                WTALOGI("[damage] 点位[%d] L%d 与基线一致，存无框原图，不告警", cur_point, cur_light_flag);
                return;
            }
            // 有差异 → 不绘制 diff 标示框，仅作为触发模型识别/告警的门控
            WTALOGI("[damage] 点位[%d] L%d 与基线差异 %zu 处，触发模型识别与告警",
                    cur_point, cur_light_flag, diff_regions.size());
        }
    }

    // ★ 获取累积的检测结果（停留期间所有帧的合并）
    auto accumulated = cam->get_accumulated_objects();

    // ★ 在 merged_image 上绘制累积的检测框（确保所有检测到的损伤都显示在拍照图上）
    if (!accumulated.empty()) {
        for (const auto& obj : accumulated) {
            // 绘制检测框
            int x1 = (int)(obj.bbox.x * merged_image.cols);
            int y1 = (int)(obj.bbox.y * merged_image.rows);
            int x2 = (int)((obj.bbox.x + obj.bbox.w) * merged_image.cols);
            int y2 = (int)((obj.bbox.y + obj.bbox.h) * merged_image.rows);
            cv::rectangle(merged_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);

            // 绘制标签
            char label[128];
            snprintf(label, sizeof(label), "%s %.2f", obj.objname, obj.prob);
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            cv::rectangle(merged_image, cv::Point(x1, y1 - text_size.height - 4),
                          cv::Point(x1 + text_size.width, y1), cv::Scalar(0, 0, 255), -1);
            cv::putText(merged_image, label, cv::Point(x1, y1 - 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }

    // 保存带框图（用于展示告警），使用当前点位 cur_point 而非可能已变化的 now_point_id
    std::string saved_path = cam->captureSnapshot(merged_image, cur_point, cur_light_flag);

    // ★ 使用累积结果触发告警（而非仅当前帧）
    if (!accumulated.empty()) {
        std::set<std::string> seen_types;
        for (const auto& obj : accumulated) {
            if (obj.objname[0]) seen_types.insert(obj.objname);
        }
        // 兜底：objname 为空时退化为当前模型的 damage_type
        if (seen_types.empty() && !damage_type.empty()) {
            seen_types.insert(damage_type);
        }

        bool any_alarm = false;
        for (const auto& dt : seen_types) {
            if (CameraController::getInstance()->early_warning_process(camera_id, cur_point, cur_light_flag, dt)) {
                any_alarm = true;
                WTALOGI("[damage] 点位[%d] L%d 已拍照并告警: %s (损伤类型: %s, 累积检测数: %zu)",
                        cur_point, cur_light_flag, saved_path.c_str(), dt.c_str(), accumulated.size());
            }
        }
        if (any_alarm) {
            // 标记当前停留期间检出损伤：pipeline 状态机会在离开点位 3 秒后落盘 MP4
            cam->mark_damage_seen();
        }
    }

    // 点位前后对比（同光照↔同光照）：仅在此处入队，重型 OpenCV 计算延迟到巡检结束后统一执行。
    // 入队元数据：当前点位、当前灯光标志、原图（内存）和带框图路径
    if (!saved_path.empty() && !raw_image.empty()) {
        cam->enqueue_diff_task(cur_point, cur_light_flag, raw_image, saved_path);
    }

    // 标记该点位+灯光状态已处理
    cam->photo_fired_keys.insert(fired_key);
    cam->frame_should_capture.store(0); // 通知巡检线程本相位拍照完成

    // L1 拍照完成后消费掉标志（必须在拍照成功后才消费，避免提前消费导致状态错乱）
    if (cur_light_flag == 1) {
        cam->light_phase_changed = false;
    }

}

// ============================================================================
// 巡检/标定结束后的基线维护（diff 已前移到 draw_custom 拍照时门控，此处只建/更新基线）
//   - update_baseline=true（标定模式）：用不带框的原图覆盖更新基线
//   - update_baseline=false（巡检模式）：仅在基线缺失时补建，已存在则不动
// ============================================================================
void run_post_patrol_diff(Camera* cam, bool update_baseline)
{
    if (!cam) return;

    auto tasks = cam->drain_diff_queue();
    if (tasks.empty()) return;

    WTALOGI("[damage-diff] 摄像机[%s] 结束，开始维护 %zu 个点位基线",
            cam->getName().c_str(), tasks.size());

    int write_count = 0;
    for (const auto& t : tasks) {
        if (t.raw_image.empty()) {
            WTALOGI("[damage-diff] 原图为空，跳过点位[%d] L%d", t.point_id, t.light_flag);
            continue;
        }
        const cv::Mat& raw_bgr = t.raw_image;

        std::string base_path = make_baseline_path(cam->orga_name, cam->getName(),
                                                   t.point_id, t.light_flag);
        bool base_exists = (access(base_path.c_str(), 0) == 0);

        // 需要写基线的情形：标定模式（覆盖更新） 或 基线缺失（补建）
        if (update_baseline || !base_exists) {
            ensure_parent_dir(base_path);
            if (!cv::imwrite(base_path, raw_bgr)) {
                WTALOGI("[damage-diff] 摄像机[%d] 点位[%d] L%d 基线写入失败: %s",
                        cam->get_id(), t.point_id, t.light_flag, base_path.c_str());
            } else {
                ++write_count;
                WTALOGI("[damage-diff] 摄像机[%d] 点位[%d] L%d 基线%s: %s",
                        cam->get_id(), t.point_id, t.light_flag,
                        update_baseline ? "已更新" : "已补建", base_path.c_str());
            }
        }
    }

    WTALOGI("[damage-diff] 摄像机[%s] 基线维护结束，写入 %d 张",
            cam->getName().c_str(), write_count);
}

int ax_model_damage::sub_init(void *json_obj)
{
    // Extract damage type from configuration or model path
    auto jsondata = *(nlohmann::json *)json_obj;

    // 优先使用配置中显式指定的 DAMAGE_TYPE
    if (jsondata.contains("DAMAGE_TYPE")) {
        damage_type = jsondata["DAMAGE_TYPE"].get<std::string>();
    } else if (jsondata.contains("MODEL_PATH")) {
        // 从模型文件名中提取损伤类型（文件名去后缀即为损伤类型）
        std::string model_path = jsondata["MODEL_PATH"].get<std::string>();
        size_t last_slash = model_path.find_last_of("/\\");
        std::string filename = (last_slash != std::string::npos)
                               ? model_path.substr(last_slash + 1) : model_path;
        size_t last_dot = filename.find_last_of(".");
        if (last_dot != std::string::npos) {
            damage_type = filename.substr(0, last_dot);
        } else {
            damage_type = filename;
        }
    }

    WTALOGI("Damage type: '%s'", damage_type.c_str());
    return 0;
}

void ax_model_damage::set_channel_init_info(const std::string name, const int id)
{
    ax_model_base::set_channel_init_info(name, id);

    // 预渲染通道名称（使用 FreeType 支持中文）
    auto& ft = FreeTypeOverlay::instance();
    if (!ft.ready()) {
        ft.init("/wt_tech/conf/simsun.ttc", 20);
    }
    if (ft.ready() && !channel_name.empty()) {
        m_channel_name_bmp = ft.renderTextRGBA(channel_name, cv::Scalar(255, 0, 0, 255), 2);
        if (m_channel_name_bmp.empty()) {
            WTALOGI("[FreeType] render channel_name failed: '%s'", channel_name.c_str());
        } else {
            WTALOGI("[FreeType] channel_name ready: '%s' size=%dx%d",
                    channel_name.c_str(), m_channel_name_bmp.cols, m_channel_name_bmp.rows);
        }
    }
}

wt_damage_multi_model_recognize::wt_damage_multi_model_recognize()
{
    WTALOGI("Instance wt_damage_multi_model_recognize object");
}

wt_damage_multi_model_recognize::DamageModelInfo
wt_damage_multi_model_recognize::load_model_file(const std::string& dir, const std::string& fname)
{
    DamageModelInfo info;
    info.model = nullptr;

    // 文件名必须以 .axmodel 结尾
    if (fname.length() < 9 || fname.substr(fname.length() - 8) != ".axmodel") {
        WTALOGI("非法模型文件名(需以 .axmodel 结尾): %s", fname.c_str());
        return info;
    }

    // 损伤类型 = 文件名去掉 .axmodel 后缀
    std::string damage_type = fname.substr(0, fname.length() - 8);
    std::string model_path = dir + "/" + fname;
    std::string config_path = dir + "/" + damage_type + ".json";

    // 模型文件必须存在
    struct stat mst;
    if (stat(model_path.c_str(), &mst) != 0) {
        WTALOGI("模型文件不存在: %s", model_path.c_str());
        return info;
    }

    // 尝试加载独立配置文件，不存在则使用默认配置
    nlohmann::json model_json;
    std::ifstream config_file(config_path);
    if (config_file.good()) {
        try {
            model_json = nlohmann::json::parse(config_file);
            WTALOGI("加载模型独立配置: %s", config_path.c_str());
        } catch (const std::exception& e) {
            WTALOGI("解析配置失败 %s: %s", config_path.c_str(), e.what());
            config_file.close();
            return info;
        }
        config_file.close();
    } else {
        // 默认配置：CLASS_NUM=1，CLASS_NAMES=模型文件名（损伤类型）
        model_json["CLASS_NUM"] = 1;
        model_json["CLASS_NAMES"] = nlohmann::json::array({damage_type});
        model_json["PROB_THRESHOLD"] = 0.4;
        model_json["NMS_THRESHOLD"] = 0.45;
        WTALOGI("模型 '%s' 无独立配置，使用默认(CLASS_NUM=1, CLASS_NAMES=['%s'])", fname.c_str(), damage_type.c_str());
    }

    // 确保必要字段
    if (!model_json.contains("MODEL_TYPE")) {
        model_json["MODEL_TYPE"] = "MT_DAMAGE_MODEL";
    }
    model_json["MODEL_PATH"] = model_path;
    model_json["DAMAGE_TYPE"] = damage_type;

    // 创建模型实例
    std::string strModelType;
    int mt = get_model_type(&model_json, strModelType);
    std::shared_ptr<ax_model_base> model((ax_model_base *)OBJFactory::getInstance().getObjectByID(mt));
    if (!model) {
        WTALOGI("创建模型实例失败: %s", model_path.c_str());
        return info;
    }

    // 初始化模型
    int ret = model->init((void *)&model_json);
    if (ret != 0) {
        WTALOGI("初始化模型失败: %s, ret=%d", model_path.c_str(), ret);
        return info;
    }
    model->set_channel_init_info(channel_name, camera_id);

    info.damage_type_name = damage_type;
    info.model = model;
    return info;
}

int wt_damage_multi_model_recognize::scan_and_load_models(const std::string& position_dir)
{
    DIR *dir = opendir(position_dir.c_str());
    if (!dir) {
        WTALOGI("无法打开部位目录: %s", position_dir.c_str());
        return -1;
    }

    struct dirent *entry;
    int loaded_count = 0;
    while ((entry = readdir(dir)) != nullptr) {
        std::string fname = entry->d_name;
        // 只加载 .axmodel 文件（子目录如 specialized 会因扩展名不匹配被自然跳过）
        if (fname.length() < 9) continue; // 至少 "x.axmodel"
        if (fname.substr(fname.length() - 8) != ".axmodel") continue;

        DamageModelInfo info = load_model_file(position_dir, fname);
        if (!info.model) continue;

        m_damage_models.push_back(info);
        m_models.push_back(info.model); // 保持父类兼容

        loaded_count++;
        WTALOGI("加载通用模型: 部位='%s', 损伤类型='%s'",
                m_position_name.c_str(), info.damage_type_name.c_str());
    }
    closedir(dir);

    WTALOGI("部位 '%s' 共加载 %d 个通用损伤模型", m_position_name.c_str(), loaded_count);
    return 0;
}

int wt_damage_multi_model_recognize::load_specialized_models()
{
    auto *cam = CameraController::getInstance()->getCamera(camera_id);
    if (!cam) {
        WTALOGI("相机[%d] 不存在，跳过专用模型加载", camera_id);
        return 0;
    }

    // 收集本相机所有点位 models[] 的并集（去重），规范化为带 .axmodel 的文件名
    std::set<std::string> names;
    for (const auto& pos : cam->getPresetPositions()) {
        for (const auto& raw : pos.models) {
            if (raw.empty()) continue;
            std::string fname = raw;
            if (fname.length() < 8 || fname.substr(fname.length() - 8) != ".axmodel") {
                fname += ".axmodel";
            }
            names.insert(fname);
        }
    }
    if (names.empty()) {
        WTALOGI("相机[%d] 无点位配置专用模型", camera_id);
        return 0;
    }

    std::string specialized_dir = m_position_dir + "/specialized";
    struct stat st;
    if (stat(specialized_dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        WTALOGI("专用模型目录不存在: %s（%zu 个专用模型无法加载，仅运行通用模型）",
                specialized_dir.c_str(), names.size());
        return 0; // 不致命：通用模型仍可运行
    }

    int loaded = 0;
    for (const auto& fname : names) {
        if (m_specialized_models.count(fname)) continue;
        DamageModelInfo info = load_model_file(specialized_dir, fname);
        if (!info.model) {
            WTALOGI("专用模型加载失败或不存在: %s/%s", specialized_dir.c_str(), fname.c_str());
            continue;
        }
        m_specialized_models[fname] = info;
        m_models.push_back(info.model); // 保持父类兼容（deinit 时统一释放）
        loaded++;
        WTALOGI("加载专用模型: '%s' (损伤类型='%s')", fname.c_str(), info.damage_type_name.c_str());
    }
    WTALOGI("相机[%d] 专用模型加载完成: %d/%zu", camera_id, loaded, names.size());

    // 预计算 点位id -> 专用模型指针列表（供 inference O(logN) 查表，避免每帧扫描点位+字符串规范化）。
    // 此时 m_specialized_models 已全部加载完成，map 节点地址稳定，可安全存指针。
    for (const auto& pos : cam->getPresetPositions()) {
        std::vector<const DamageModelInfo*> vec;
        for (const auto& raw : pos.models) {
            if (raw.empty()) continue;
            std::string fname = raw;
            if (fname.length() < 8 || fname.substr(fname.length() - 8) != ".axmodel") {
                fname += ".axmodel";
            }
            auto it = m_specialized_models.find(fname);
            if (it != m_specialized_models.end() && it->second.model) {
                vec.push_back(&it->second);
            }
        }
        if (!vec.empty()) m_point_specialized[pos.id] = std::move(vec);
    }
    return 0;
}

int wt_damage_multi_model_recognize::init(void *json_obj)
{
    WTALOGI("初始化 damage 多模型（按部位目录扫描）...");
    auto jsondata = *(nlohmann::json *)json_obj;

    // 解析模型根目录
    if (jsondata.contains("MODEL_ROOT_DIR")) {
        m_model_root_dir = jsondata["MODEL_ROOT_DIR"].get<std::string>();
    } else {
        WTALOGI("配置中缺少 MODEL_ROOT_DIR");
        return -1;
    }

    // 部位名称：根据云台类型自动选择（big=大云台→outside，small=小云台→inside），
    // 无需在配置里冗余指定 POSITION。取不到相机时回退到配置中的 POSITION（可选，用于覆盖/调试）。
    std::string ptz_type;
    if (auto *cam = CameraController::getInstance()->getCamera(camera_id)) {
        ptz_type = cam->ptz_type;
    }
    if (ptz_type == "big") {
        m_position_name = "outside";
    } else if (ptz_type == "small") {
        m_position_name = "inside";
    } else if (jsondata.contains("POSITION")) {
        m_position_name = jsondata["POSITION"].get<std::string>();
        WTALOGI("相机[%d] 未取到云台类型，回退使用配置 POSITION=%s", camera_id, m_position_name.c_str());
    } else {
        WTALOGI("相机[%d] 无法确定类型：ptz_type 为空且配置缺少 POSITION", camera_id);
        return -1;
    }
    WTALOGI("相机[%d] 按云台类型[%s]选择: %s", camera_id, ptz_type.c_str(), m_position_name.c_str());

    // 直接访问指定部位的子目录
    std::string position_dir = m_model_root_dir + "/" + m_position_name;
    struct stat st;
    if (stat(position_dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        WTALOGI("部位目录不存在: %s", position_dir.c_str());
        return -1;
    }
    m_position_dir = position_dir; // 记录部位目录，specialized 子目录在其下

    // 加载该部位目录下的所有通用模型（每个点位都跑）
    int ret = scan_and_load_models(position_dir);
    if (ret != 0) {
        WTALOGI("部位 '%s' 模型加载失败", m_position_name.c_str());
        return -1;
    }

    // 预加载本相机各点位 rt.json models[] 引用的专用模型（在通用模型之上按点位额外叠加）
    load_specialized_models();

    // 输出加载摘要
    WTALOGI("=== 多模型加载摘要 ===");
    WTALOGI("  部位 '%s': %zu 个通用模型, %zu 个专用模型",
            m_position_name.c_str(), m_damage_models.size(), m_specialized_models.size());
    for (const auto& info : m_damage_models) {
        WTALOGI("    - 通用: '%s'", info.damage_type_name.c_str());
    }
    for (const auto& kv : m_specialized_models) {
        WTALOGI("    - 专用: '%s'", kv.first.c_str());
    }

    return 0;
}

int wt_damage_multi_model_recognize::inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    int result = 0;

    if (m_damage_models.empty() && m_specialized_models.empty()) {
        return 0;
    }

    // 先填充相机巡检状态快照，供 draw_custom 使用
    fill_camera_frame_state(results, camera_id);

    // ★ 相位门控：仅在到达点位且灯光相位就绪时执行推理；
    //   移动/回位/灯光切换中跳过，节省算力（此期间不产生检测结果，也不绘制/累积）
    auto *cam = CameraController::getInstance()->getCamera(camera_id);
    if (!cam || !cam->posture_completed || cam->phase_ready_ms.load() <= 0) {
        results->nObjSize = 0;
        return 0;
    }

    // 组装本次点位要跑的模型：通用模型（每个点位都跑） + 当前点位 rt.json 配置的专用模型
    std::vector<const DamageModelInfo*> run_models;
    run_models.reserve(m_damage_models.size() + 4);
    for (const auto& mi : m_damage_models) run_models.push_back(&mi);

    // 当前点位的专用模型：init 时已按 点位id 预计算好，这里直接查表（O(logN)，无字符串处理，
    // 也不读取可能被热加载改写的 preset_positions）。
    if (!m_point_specialized.empty()) {
        auto it = m_point_specialized.find(cam->now_point_id);
        if (it != m_point_specialized.end()) {
            for (const auto* mi : it->second) run_models.push_back(mi);
        }
    }

    if (run_models.empty()) {
        results->nObjSize = 0;
        return 0;
    }

    // 并行推理：每个模型在独立线程中执行
    struct ModelResult {
        axdl_results_t results;
        std::string damage_type;
        int ret;
    };

    std::vector<std::future<ModelResult>> futures; //
    futures.reserve(run_models.size());

    for (const auto* model_info : run_models) {
        futures.push_back(std::async(std::launch::async,
            [model_info, pstFrame, crop_resize_box]() -> ModelResult {
                ModelResult mr = {};
                mr.damage_type = model_info->damage_type_name;
                mr.ret = model_info->model->inference(pstFrame, crop_resize_box, &mr.results);
                return mr;
            }));
    }

    // 收集所有线程的推理结果并合并
    results->nObjSize = 0;
    for (auto& fut : futures) {
        ModelResult mr = fut.get();
        if (mr.ret != 0) {
            WTALOGI("模型 '%s' 推理失败, ret=%d", mr.damage_type.c_str(), mr.ret);
            result = mr.ret;
            continue;
        }

        // 将该模型检出的目标名称设为损伤类型（用于告警类型）
        for (int i = 0; i < (int)mr.results.nObjSize; i++) {
            strncpy(mr.results.mObjects[i].objname,
                    mr.damage_type.c_str(),
                    sizeof(mr.results.mObjects[i].objname) - 1);
            mr.results.mObjects[i].objname[sizeof(mr.results.mObjects[i].objname) - 1] = '\0';
        }

        // 合并当前模型的推理结果到主 results 中
        int space_left = SAMPLE_MAX_BBOX_COUNT - results->nObjSize;
        int to_copy = std::min((int)mr.results.nObjSize, space_left);
        if (to_copy > 0) {
            memcpy(&(results->mObjects[results->nObjSize]),
                   &(mr.results.mObjects[0]),
                   to_copy * sizeof(results->mObjects[0]));
            results->nObjSize += to_copy;
        }

        if (results->nObjSize >= SAMPLE_MAX_BBOX_COUNT) {
            WTALOGI("结果缓冲区已满，跳过剩余模型结果");
            break;
        }
    }

    return result;
}

void wt_damage_multi_model_recognize::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    this->m_damage_models.begin()->model->draw_results(image, results, fontscale, thickness, offset_x, offset_y);

    /*
    // 用任一已加载模型的绘制实现即可（优先通用模型，退化到专用模型）
    std::shared_ptr<ax_model_base> drawer;
    if (!m_damage_models.empty()) {
        drawer = m_damage_models.begin()->model;
    } else if (!m_specialized_models.empty()) {
        drawer = m_specialized_models.begin()->second.model;
    }
    if (drawer) {
        drawer->draw_results(image, results, fontscale, thickness, offset_x, offset_y);
    }
    */
}


