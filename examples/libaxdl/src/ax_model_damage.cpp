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

// ---------- 点位前后对比（同光照↔同光照）小工具 ----------
namespace {
    // 将 BGRA 文字位图 bmp 以 (x,y) 为左上角 alpha 合成到 dst 上（dst 可为 BGR 或 BGRA）。
    // 合成前先绘制白色不透明底框，保持"白底深色字"的可读性（与旧英文标签观感一致）。
    static void draw_label_bmp(cv::Mat& dst, const cv::Mat& bmp, int x, int y)
    {
        if (bmp.empty() || dst.empty()) return;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        int w = std::min(bmp.cols, dst.cols - x);
        int h = std::min(bmp.rows, dst.rows - y);
        if (w <= 0 || h <= 0) return;

        const int ch = dst.channels(); // 3=BGR, 4=BGRA
        // 白色底框
        cv::rectangle(dst, cv::Rect(x, y, w, h),
                      ch == 4 ? cv::Scalar(255, 255, 255, 255) : cv::Scalar(255, 255, 255), -1);

        for (int j = 0; j < h; ++j) {
            const cv::Vec4b* src = bmp.ptr<cv::Vec4b>(j);
            uint8_t* drow = dst.ptr<uint8_t>(y + j) + (size_t)x * ch;
            for (int i = 0; i < w; ++i) {
                uint8_t a = src[i][3];
                if (!a) continue;
                float fa = a / 255.f;
                uint8_t* p = drow + (size_t)i * ch;
                for (int c = 0; c < 3; ++c)
                    p[c] = cv::saturate_cast<uint8_t>(src[i][c] * fa + p[c] * (1 - fa));
                if (ch == 4) p[3] = std::max(p[3], a);
            }
        }
    }

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
    static cv::Mat align_translation(const cv::Mat& cur_gray, const cv::Mat& base_gray,
                                     cv::Point2d& out_shift, bool& out_ok)
    {
        out_ok = false;
        out_shift = cv::Point2d(0.0, 0.0);
        try {
            cv::Mat cF, bF;
            cur_gray.convertTo(cF, CV_32F);
            base_gray.convertTo(bF, CV_32F);
            cv::Point2d sh = cv::phaseCorrelate(cF, bF);
            out_shift = sh;
            if (std::abs(sh.x) > base_gray.cols * 0.1 ||
                std::abs(sh.y) > base_gray.rows * 0.1) {
                return cur_gray.clone();
            }
            out_ok = true;
            cv::Mat warp = (cv::Mat_<float>(2, 3) <<
                1.f, 0.f, (float)sh.x, 0.f, 1.f, (float)sh.y);
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

    // 几何对齐（旋转+平移，不含缩放）。
    // 由于此 OpenCV 构建精简（无 features2d/calib3d），改用多角度搜索：
    // 尝试若干旋转角度，对每个角度做 phaseCorrelate 平移对齐，选 SSIM 最高的组合。
    // 抗基线图与当前图之间的拍摄角度/旋转差异。
    // 失败时 ok=false，返回原图。
    static cv::Mat align_geometric(const cv::Mat& cur_bgr, const cv::Mat& base_bgr,
                                   cv::Mat& out_M, bool& out_ok, int& out_inliers)
    {
        out_ok = false;
        out_inliers = 0;
        out_M = cv::Mat();
        try {
            const int h = base_bgr.rows, w = base_bgr.cols;
            cv::Mat base_gray;
            cv::cvtColor(base_bgr, base_gray, cv::COLOR_BGR2GRAY);

            // 相机与基线最大 ±3° 角误差，故在 [-3, 3] 内以 0.5° 步进搜索
            const float angles[] = {-3.f, -2.5f, -2.f, -1.5f, -1.f, -0.5f, 0.f,
                                    0.5f, 1.f, 1.5f, 2.f, 2.5f, 3.f};
            const int n_angles = sizeof(angles) / sizeof(angles[0]);

            float best_score = -1.f;
            cv::Mat best_aligned;
            cv::Mat best_M;

            const cv::Point2f center(w * 0.5f, h * 0.5f);
            for (int i = 0; i < n_angles; ++i) {
                // 生成纯旋转矩阵（不缩放）
                cv::Mat rot = cv::getRotationMatrix2D(center, angles[i], 1.0);
                cv::Mat rotated;
                cv::warpAffine(cur_bgr, rotated, rot, base_bgr.size(),
                               cv::INTER_LINEAR, cv::BORDER_REPLICATE);

                // 转灰度后做平移对齐
                cv::Mat rot_gray;
                cv::cvtColor(rotated, rot_gray, cv::COLOR_BGR2GRAY);
                cv::Point2d shift;
                bool t_ok = false;
                cv::Mat aligned_gray = align_translation(rot_gray, base_gray, shift, t_ok);

                // 平移矩阵（在旋转矩阵基础上叠加平移）
                cv::Mat M = rot.clone();
                if (t_ok) {
                    M.at<float>(0, 2) += (float)shift.x;
                    M.at<float>(1, 2) += (float)shift.y;
                }

                // 用全图 SSIM 均值评估对齐质量
                cv::Mat ssim = ssim_map(aligned_gray, base_gray);
                float score = (float)cv::mean(ssim)[0];
                if (score > best_score) {
                    best_score = score;
                    best_aligned = rotated;
                    best_M = M;
                }
            }

            // 全图 SSIM 足够高才认为对齐成功（阈值可调整）
            if (best_score > 0.85f) {
                out_M = best_M;
                out_ok = true;
                out_inliers = (int)(best_score * 100);  // 用 SSIM 百分比近似表示质量
                return best_aligned;
            }
            return cur_bgr;
        } catch (const cv::Exception&) {
            return cur_bgr;
        }
    }

    struct DiffRegion {
        cv::Rect      bbox;   // 嵌套抑制/标签位置
        cv::RotatedRect rbox; // 倾斜框（cv::minAreaRect）
        float         score;
    };

    // diff 门控全局阈值（默认值与 Python diff_compare.py 对齐）
    struct DiffThresholds {
        float ssim_th    = 0.75f;
        float ssim_lo_th = 0.5f;  // 块内 SSIM 5% 低分位阈值，捕捉细线/小点（不被均值稀释）
        float absd_ratio = 0.005f;// 块内强变化像素占比阈值
        int   block      = 24;    // 分块尺寸
        int   min_area   = 20;    // 过滤小区域噪点
        int   pix_diff   = 15;    // 单像素强变化阈值（颜色/灰度）
        bool  use_clahe  = false; // 默认关闭 CLAHE
        bool  use_warp   = true;  // 默认开启几何对齐（旋转/角度配准）
    };
    static DiffThresholds g_diff_th;

    // 当前帧 vs 基线帧 → 差异区域（基线坐标系）
    static std::vector<DiffRegion> diff_against_baseline(const cv::Mat& cur_bgr, const cv::Mat& base_bgr)
    {
        std::vector<DiffRegion> out;
        if (cur_bgr.empty() || base_bgr.empty()) return out;

        const float SSIM_TH    = g_diff_th.ssim_th;
        const float SSIM_LO_TH = g_diff_th.ssim_lo_th;
        const float ABSD_RATIO = g_diff_th.absd_ratio;
        const int   BLOCK      = g_diff_th.block > 0 ? g_diff_th.block : 24;
        const int   MIN_AREA   = g_diff_th.min_area;
        const int   PIX_DIFF   = g_diff_th.pix_diff;

        cv::Mat cur = cur_bgr;
        if (cur.size() != base_bgr.size())
            cv::resize(cur, cur, base_bgr.size());

        const int H = base_bgr.rows, W = base_bgr.cols;
        // 有效区域掩膜：记录 cur 中经 warp 后仍是真实内容的像素（露出的边缘置 0，不参与比较）
        cv::Mat valid_mask = cv::Mat::ones(H, W, CV_8U);

        // 几何对齐（旋转/角度错位 → 先配准到基线坐标系，再走平移+分块）
        cv::Mat warp_M;
        bool warp_ok = false;
        int warp_inliers = 0;
        if (g_diff_th.use_warp) {
            cur = align_geometric(cur, base_bgr, warp_M, warp_ok, warp_inliers);
            if (warp_ok && !warp_M.empty()) {
                cv::Mat wm;
                cv::warpAffine(valid_mask, wm, warp_M, base_bgr.size(),
                               cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
                valid_mask = wm;
            }
        }

        // CLAHE 可开关（默认关闭，与 --no-clahe 对应）
        cv::Mat curN  = g_diff_th.use_clahe ? clahe_bgr(cur) : cur;
        cv::Mat baseN = g_diff_th.use_clahe ? clahe_bgr(base_bgr) : base_bgr;

        cv::Mat curG, baseG;
        cv::cvtColor(curN, curG, cv::COLOR_BGR2GRAY);
        cv::cvtColor(baseN, baseG, cv::COLOR_BGR2GRAY);

        // 平移对齐：基于 CLAHE 灰度求位移，并同步 warping 到彩色图
        cv::Point2d shift;
        bool aligned_ok = false;
        curG = align_translation(curG, baseG, shift, aligned_ok);
        if (aligned_ok && (std::abs(shift.x) > 0.0 || std::abs(shift.y) > 0.0)) {
            cv::Mat warp = (cv::Mat_<float>(2, 3) <<
                1.f, 0.f, (float)shift.x, 0.f, 1.f, (float)shift.y);
            cv::Mat aligned;
            cv::warpAffine(cur, aligned, warp, base_bgr.size(),
                           cv::INTER_LINEAR, cv::BORDER_REPLICATE);
            cur = aligned;
            // 平移同样会露出边缘，同步更新有效掩膜
            cv::warpAffine(valid_mask, valid_mask, warp, base_bgr.size(),
                           cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
        }

        // SSIM(结构, 基于 CLAHE 灰度) + 三通道最大差(颜色差)
        cv::Mat ssim = ssim_map(curG, baseG);
        cv::Mat diff_bgr;
        cv::absdiff(cur, base_bgr, diff_bgr);
        std::vector<cv::Mat> chs;
        cv::split(diff_bgr, chs);
        cv::Mat absd = cv::max(chs[0], cv::max(chs[1], chs[2]));

        // 屏蔽 warp 露出的无效边缘：无效区 SSIM 视为完全匹配、颜色差置 0，不参与差异判定
        cv::Mat invalid = (valid_mask == 0);
        if (cv::countNonZero(invalid) > 0) {
            ssim.setTo(1.0, invalid);
            absd.setTo(0, invalid);
        }

        // 强变化像素掩码：原始颜色/灰度最大差 > pix_diff
        cv::Mat strong;
        cv::threshold(absd, strong, (double)PIX_DIFF, 1.0, cv::THRESH_BINARY);

        cv::Mat susp = cv::Mat::zeros(H, W, CV_8U);
        for (int y = 0; y < H; y += BLOCK) {
            for (int x = 0; x < W; x += BLOCK) {
                cv::Rect r(x, y, std::min(BLOCK, W - x), std::min(BLOCK, H - y));
                float s = (float)cv::mean(ssim(r))[0];
                // 5% 低分位（比 min 抗噪）：细线/小点会被均值稀释，但逃不过低分位
                cv::Mat sb = ssim(r).clone(); // clone 保证连续
                std::vector<float> vals(sb.begin<float>(), sb.end<float>());
                size_t k = (size_t)(vals.size() * 0.05f);
                std::nth_element(vals.begin(), vals.begin() + k, vals.end());
                float s_lo = vals[k];
                float frac = (float)cv::mean(strong(r))[0];
                // OR 门控：均值结构差异 或 局部低分位差异（细线/小点）或 颜色/曝光差异
                if (s < SSIM_TH || s_lo < SSIM_LO_TH || frac > ABSD_RATIO) susp(r).setTo(255);
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
            float frac = (float)cv::mean(strong(b))[0];
            float d = (float)cv::mean(absd(b))[0] / 255.0f;
            float score = std::max(0.f, std::min(1.f,
                (1.f - s) * 0.4f + frac * 0.3f + d * 0.3f));
            cv::RotatedRect rbox = cv::minAreaRect(c);
            out.push_back({b, rbox, score});
        }
        std::sort(out.begin(), out.end(),
                  [](const DiffRegion& a, const DiffRegion& b){ return a.score > b.score; });

        // 抑制完全套在大框内的小框（小框 80% 以上被大框包含则丢弃）
        auto is_nested = [](const DiffRegion& inner, const DiffRegion& outer) {
            cv::Rect inter = inner.bbox & outer.bbox;
            if (inter.area() <= 0) return false;
            if (outer.bbox.area() <= inner.bbox.area()) return false;
            return inter.area() >= 0.8 * inner.bbox.area();
        };
        std::vector<DiffRegion> filtered;
        for (const auto& r : out) {
            bool nest = false;
            for (const auto& a : filtered) {
                if (is_nested(r, a)) { nest = true; break; }
            }
            if (!nest) filtered.push_back(r);
        }
        out.swap(filtered);
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

// 缓存原图用于快照合成/ diff 门控（将 NV12/RGB/BGR 转换为 BGR）。
// 抽成独立方法：跳过推理时也能由巡检调度器主动刷新缓存，避免拍照用到旧点位残帧。
void ax_model_damage::cache_source_frame(axdl_image_t *pstFrame)
{
    if (!pstFrame || !pstFrame->pVir) return;
    // 读取该帧所属相位（cache 发生在 inference 相位门控之后，帧确属当前相位）
    long long phase_ms = 0;
    auto *cam = CameraController::getInstance()->getCamera(camera_id);
    if (cam) phase_ms = cam->phase_ready_ms.load();

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
        m_cached_phase_ms = phase_ms; // 记录该帧所属相位，供拍照时严格校验
        // [DIAG] 节流日志：确认推理线程是否仍在刷新缓存及缓存到的相位
        {
            static int diag_cnt = 0;
            if (diag_cnt++ % 30 == 0) {
                WTALOGI("[DIAG cache] cam[%d] 刷新缓存 phase=%lld (第%d次)", camera_id, phase_ms, diag_cnt);
            }
        }
    } catch (...) {
        // 转换失败时保持旧缓存
    }
}

int ax_model_damage::post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    // 缓存原图用于快照合成（将 NV12/RGB/BGR 转换为 BGR）
    cache_source_frame(pstFrame);

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

            results->mObjects[i].prob  = obj.prob; // 写入置信度

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

            results->mObjects[i].prob  = obj.prob; // 写入置信度

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

// 预览检测框：在基类绘制风格基础上，标签改用 FreeType 预渲染的中文损伤名位图
// （不显示置信度；映射表缺失时回退英文原文）。隐藏基类同名函数。
void ax_model_damage::draw_bbox(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    int baseLine = 0;
    for (int i = 0; i < results->nObjSize; i++)
    {
        auto &o = results->mObjects[i];

        // 中文损伤名用 FreeType 预渲染位图；缺失映射时回退英文原文
        cv::Mat label_bmp = getDamageLabelBmp(o.objname);
        int label_w = label_bmp.empty() ? 0 : label_bmp.cols;
        int label_h = label_bmp.empty() ? 0 : label_bmp.rows;

        // 置信度（含 track_id）仍沿用原 cv::putText 方式绘制，紧跟中文名之后
        char conf_buf[64] = {0};
        if (o.prob > 0.f)
            snprintf(conf_buf, sizeof(conf_buf), " %.2f", o.prob);
        std::string conf_str = conf_buf;
        if (b_track)
            conf_str += " " + std::to_string(o.track_id);
        cv::Size conf_size = conf_str.empty()
            ? cv::Size(0, 0)
            : cv::getTextSize(conf_str, cv::FONT_HERSHEY_SIMPLEX, fontscale, thickness, &baseLine);

        int bar_h = std::max(label_h, conf_size.height + baseLine);
        int total_w = label_w + conf_size.width;

        int x, y;
        if (o.bHasBoxVertices)
        {
            for (int j = 0; j < 4; j++)
                cv::line(image,
                         cv::Point(o.bbox_vertices[j].x * image.cols + offset_x, o.bbox_vertices[j].y * image.rows + offset_y),
                         cv::Point(o.bbox_vertices[(j + 1) % 4].x * image.cols + offset_x, o.bbox_vertices[(j + 1) % 4].y * image.rows + offset_y),
                         cv::Scalar(128, 0, 0, 255), thickness * 2, 8, 0);
            x = o.bbox_vertices[0].x * image.cols + offset_x;
            y = o.bbox_vertices[0].y * image.rows + offset_y - bar_h;
        }
        else
        {
            cv::Rect rect(o.bbox.x * image.cols + offset_x, o.bbox.y * image.rows + offset_y,
                          o.bbox.w * image.cols, o.bbox.h * image.rows);
            cv::rectangle(image, rect, COCO_COLORS[o.label % COCO_COLORS.size()], thickness);
            x = rect.x;
            y = rect.y - bar_h;
        }

        if (y < 0) y = 0;
        if (x + total_w > image.cols) x = image.cols - total_w;
        if (x < 0) x = 0;

        // 中文损伤名位图（白底深色字）alpha 合成
        if (!label_bmp.empty())
            draw_label_bmp(image, label_bmp, x, y);

        // 置信度：白底黑字，紧跟在中文名右侧，垂直居中于标签条
        if (!conf_str.empty()) {
            int cx = x + label_w;
            cv::rectangle(image, cv::Rect(cx, y, conf_size.width, bar_h),
                          cv::Scalar(255, 255, 255, 255), -1);
            int ty = y + (bar_h + conf_size.height) / 2;
            cv::putText(image, conf_str, cv::Point(cx, ty), cv::FONT_HERSHEY_SIMPLEX, fontscale,
                        cv::Scalar(0, 0, 0, 255), thickness);
        }
    }
}

void ax_model_damage::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    auto *cam = CameraController::getInstance()->getCamera(camera_id);

    // === 小云台活体互检抓拍（检查方在自身流上抓两帧：被检查方转动前/后）===
    // inspect_peer 投递 peer_task；此处相位匹配后执行抓拍/diff，并通过 promise 回传结果。
    if (cam) {
        std::shared_ptr<Camera::PeerCaptureTask> task;
        {
            std::lock_guard<std::mutex> lk(cam->peer_task_mtx);
            task = cam->peer_task;
        }
        if (task) {
            if (task->phase > 0 && m_cached_phase_ms == task->phase && !m_cached_frame_bgr.empty()) {
                // 取当前相位帧，去 letterbox 黑边后缩放到叠加层尺寸
                cv::Mat raw_bgr;
                {
                    std::lock_guard<std::mutex> lk(m_frame_mutex);
                    int src_h = m_cached_frame_bgr.rows, src_w = m_cached_frame_bgr.cols;
                    int dst_h = HEIGHT_DET_BBOX_RESTORE, dst_w = WIDTH_DET_BBOX_RESTORE;
                    float scale = std::min((float)src_w / dst_w, (float)src_h / dst_h);
                    int new_w = (int)(dst_w * scale);
                    int new_h = (int)(dst_h * scale);
                    int pad_x = (src_w - new_w) / 2;
                    int pad_y = (src_h - new_h) / 2;
                    cv::Rect valid_roi(pad_x, pad_y, new_w, new_h);
                    valid_roi &= cv::Rect(0, 0, src_w, src_h);
                    cv::Mat cropped = m_cached_frame_bgr(valid_roi);
                    cv::resize(cropped, raw_bgr, cv::Size(image.cols, image.rows));
                }

                int peer_point = cam->find_peer_check_point_id();

                if (task->kind == 1) {
                    // 第一帧：被检查方转动前，仅缓存
                    cam->peer_frame1 = raw_bgr.clone();
                    task->prom.set_value(0);
                } else { // kind == 3
                    // 第二帧：被检查方转动后，与第一帧 diff，检出运动=正常
                    int result = 2; // 默认无运动=异常
                    cv::Mat show_bgr = raw_bgr.clone();
                    if (!cam->peer_frame1.empty()) {
                        auto regions = diff_against_baseline(raw_bgr, cam->peer_frame1);
                        if (!regions.empty()) {
                            result = 1; // 检出运动=正常
                            for (const auto& r : regions) {
                                cv::Rect b = r.bbox & cv::Rect(0, 0, show_bgr.cols, show_bgr.rows);
                                if (b.area() <= 0) continue;
                                cv::rectangle(show_bgr, b, cv::Scalar(0, 255, 0), 2);
                            }
                            cv::putText(show_bgr, "peer ALIVE (motion)", cv::Point(10, 30),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                            cam->captureSnapshot(show_bgr, peer_point > 0 ? peer_point : 0, 0);
                            WTALOGI("[peer] 摄像机[%d] 互检检出被检查方运动(区域数=%zu)，判正常",
                                    cam->get_id(), regions.size());
                        } else {
                            // 无运动：被检查方疑似脱落/卡死，告警
                            cv::putText(show_bgr, "peer NO-MOTION!", cv::Point(10, 30),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                            cam->captureSnapshot(show_bgr, peer_point > 0 ? peer_point : 0, 0);
                            if (CameraController::getInstance()->early_warning_process(
                                    cam->get_id(), peer_point > 0 ? peer_point : 0, 0, "互检异常")) {
                                WTALOGI("[peer] 摄像机[%d] 互检未检出被检查方运动，告警",
                                        cam->get_id());
                            }
                        }
                    } else {
                        WTALOGI("[peer] 摄像机[%d] 互检第一帧缺失，无法判定，判异常", cam->get_id());
                    }
                    cam->peer_frame1.release();
                    task->prom.set_value(result);
                }

                // 任务已完成，清空以免重复处理
                std::lock_guard<std::mutex> lk(cam->peer_task_mtx);
                if (cam->peer_task == task) cam->peer_task.reset();
            }
            // 互检抓拍期间跳过常规绘制，避免叠加异常
            return;
        }
    }

    // 仅在到达点位且灯光相位就绪时绘制当前帧 AI 检测框；移动/回位/灯光切换中不绘制
    if (cam && cam->posture_completed.load() && cam->phase_ready_ms.load() > 0) {
        draw_bbox(image, results, fontscale, thickness, offset_x, offset_y);
    }

    /* ---------- 绘制时间和点位信息 ---------- */
    time_t now_ts = time(nullptr) + 8 * 3600;
    tm *t_info = gmtime(&now_ts);
    char time_str[64] = {0};
    snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d:%02d",
             t_info->tm_year + 1900, t_info->tm_mon + 1, t_info->tm_mday,
             t_info->tm_hour, t_info->tm_min, t_info->tm_sec);

    int cur_point = cam ? cam->now_point_id : 0;

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

    bool is_moving = !cam->posture_completed.load();

    // 获取完整点位文字位图（FreeType 渲染，含编号+中文名+状态）
    cv::Mat point_bmp = getPointTextBmp(cur_point, is_moving);
    if (point_bmp.empty()) return;

    int bmp_w = point_bmp.cols;
    int bmp_h = point_bmp.rows;

    // 计算画面下方的居中位置
    int image_width = image.cols;
    text_x = (image_width - bmp_w) / 2;
    text_y = image.rows - bmp_h - 10;  // 距离底部10像素

    // 呼吸灯系数：移动时 0.2~1.0 呼吸，到达时固定 1.0
    double breath_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
    float breath_factor = is_moving
        ? (float)(0.2 + 0.8 * (std::sin(breath_time * 2.0 * M_PI / 2.0) + 1.0) / 2.0)
        : 1.0f;

    // alpha 合成到 BGRA
    int w = std::min(bmp_w, image_width - text_x);
    int h = std::min(bmp_h, image.rows - text_y);
    if (w > 0 && h > 0) {
        cv::Mat roi = image(cv::Rect(text_x, text_y, w, h));
        for (int y = 0; y < h; ++y) {
            const cv::Vec4b* src = point_bmp.ptr<cv::Vec4b>(y);
            cv::Vec4b* dst = roi.ptr<cv::Vec4b>(y);
            for (int x = 0; x < w; ++x) {
                uint8_t a = src[x][3];
                if (!a) continue;
                a = (uint8_t)(a * breath_factor);
                if (!a) continue;
                float fa = a / 255.f;
                for (int c = 0; c < 3; ++c)
                    dst[x][c] = cv::saturate_cast<uint8_t>(src[x][c] * fa + dst[x][c] * (1 - fa));
                dst[x][3] = std::max(dst[x][3], a);
            }
        }
    }

    // 灯光状态：0=L0(开灯), 1=L1(关灯/低照)
    int cur_light_flag = cam ? (cam->light_phase_changed ? 1 : 0) : 0;
    int fired_key = cur_point * 10 + cur_light_flag;

    // 相位就绪：到位 + 灯光正确 + 巡检线程已 arm + 额外流延迟余量
    // 避免在 RTSP 解码缓冲中的旧帧上累积/拍照（例如灯光切换后还未实际呈现的画面）
    const long long STREAM_LATENCY_SETTLE_MS = 40; // 流延迟余量，可调
    long long phase_ready = cam ? cam->phase_ready_ms.load() : 0;
    long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();

    bool phase_settled = (phase_ready > 0) && (now_ms - phase_ready >= STREAM_LATENCY_SETTLE_MS);

    // ★ 每帧累积检测结果（只有相位真正就绪后才累积，避免灯光/移动过程中的误累积）
    if (cam && cam->posture_completed.load() && phase_settled && results->nObjSize > 0) {
        cam->accumulate_detection(results);
    }

    // 每个点位触发两次拍照（有灯照+无灯照）：
    // - 使用 photo_fired_keys 记录已拍照的 (point_id, light_flag) 组合
    // - key = point_id * 10 + light_flag，巡检开始时清空
    // - 直接读取 cam->frame_should_capture（atomic，跨线程安全，由巡检线程设置）
    int should_capture = cam ? cam->frame_should_capture.load() : 0;
    if (should_capture == 0) {
        return; // 巡检线程未标记拍照，跳过
    }

    // ★ 严格相位绑定：缓存帧必须来自当前相位，否则说明推理线程尚未把当前相位的帧
    //   刷新进 m_cached_frame_bgr，此时拍照会拍到上一相位的滞后帧（表现为 L1 与
    //   下一点位 L0 画面相同）。不依赖时间余量，直接用相位身份匹配：不一致则放弃本次，等下一帧。
    long long cur_phase = cam ? cam->phase_ready_ms.load() : 0;
    {
        std::lock_guard<std::mutex> lk(m_frame_mutex);
        if (cur_phase == 0 || m_cached_frame_bgr.empty() || m_cached_phase_ms != cur_phase) {
            // [DIAG] 节流日志：拍照被标记(should_capture!=0)但相位绑定不通过，打印具体原因
            static int diag_cnt = 0;
            if (diag_cnt++ % 30 == 0) {
                WTALOGI("[DIAG mismatch] cam[%d] 点位[%d] should=%d 相位失配: cur_phase=%lld cached_phase=%lld 差=%lld empty=%d (第%d次)",
                        camera_id, cur_point, should_capture, cur_phase, m_cached_phase_ms,
                        cur_phase - m_cached_phase_ms, (int)m_cached_frame_bgr.empty(), diag_cnt);
            }
            return; // 缓存帧不属于当前相位，等推理线程刷新后再拍
        }
    }

    // 根据 frame_should_capture 确定 cur_light_flag
    cur_light_flag = (should_capture == 1) ? 0 : 1;
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

    // 临界区仅负责从共享缓存帧裁剪黑边并 resize 出一份独立的原图（base_resized），
    // 随后的 cvtColor / Alpha 混合全部移出锁，避免长时间占用 m_frame_mutex
    // 阻塞每帧刷新缓存的 cache_source_frame（推理线程），这是"到点位拍照瞬间卡顿"的关键。
    cv::Mat base_resized;
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

            // resize 到与 overlay (IVPS) 相同尺寸（resize 输出为独立内存，锁外可安全使用）
            cv::resize(cropped, base_resized, cv::Size(image.cols, image.rows));
        }
    }

    if (!base_resized.empty()) {
        raw_image = base_resized; // resize 输出已是独立数据

        // 将 RGBA 叠加层拆出 BGR 与 alpha 通道
        cv::Mat overlay_bgra;
        cv::cvtColor(image, overlay_bgra, cv::COLOR_RGBA2BGRA);
        std::vector<cv::Mat> ch;
        cv::split(overlay_bgra, ch); // ch[0..2]=BGR, ch[3]=alpha

        cv::Mat alpha1;
        ch[3].convertTo(alpha1, CV_32FC1, 1.0 / 255.0); // 归一化 alpha ∈ [0,1]
        cv::Mat alpha3;
        cv::Mat alphas[] = { alpha1, alpha1, alpha1 };
        cv::merge(alphas, 3, alpha3);

        cv::Mat overlay_bgr, overlay_f, base_f;
        cv::merge(std::vector<cv::Mat>{ ch[0], ch[1], ch[2] }, overlay_bgr);
        overlay_bgr.convertTo(overlay_f, CV_32FC3);
        base_resized.convertTo(base_f, CV_32FC3);

        // out = overlay*alpha + base*(1-alpha)，向量化替代逐像素循环（快 1~2 个数量级）
        cv::Mat out_f = overlay_f.mul(alpha3) + base_f.mul(cv::Scalar::all(1.0) - alpha3);
        out_f.convertTo(merged_image, CV_8UC3);
    } else {
        cv::cvtColor(image, raw_image, cv::COLOR_RGBA2BGR);
        merged_image = raw_image.clone();
    }

    // ★ 差异 diff 与模型并行：此处只处理模型检测结果与告警；
    //   diff 比对已解耦到巡检结束后的 run_post_patrol_diff 统一执行并单独告警。

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

            // 中文损伤名用 FreeType 位图；缺失映射时回退英文原文
            cv::Mat label_bmp = getDamageLabelBmp(obj.objname);
            int label_w = label_bmp.empty() ? 0 : label_bmp.cols;
            int label_h = label_bmp.empty() ? 0 : label_bmp.rows;

            // 置信度仍沿用原 cv::putText 方式（红底白字），紧跟中文名之后
            char conf_buf[32] = {0};
            snprintf(conf_buf, sizeof(conf_buf), " %.2f", obj.prob);
            std::string conf_str = conf_buf;
            int baseline = 0;
            cv::Size conf_size = cv::getTextSize(conf_str, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

            int bar_h = std::max(label_h, conf_size.height + baseline);
            int total_w = label_w + conf_size.width;

            int ly = y1 - bar_h;
            if (ly < 0) ly = 0;
            int lx = x1;
            if (lx + total_w > merged_image.cols) lx = merged_image.cols - total_w;
            if (lx < 0) lx = 0;

            if (!label_bmp.empty())
                draw_label_bmp(merged_image, label_bmp, lx, ly);

            // 置信度：红底白字，紧跟中文名右侧，垂直居中于标签条
            int cx = lx + label_w;
            cv::rectangle(merged_image, cv::Rect(cx, ly, conf_size.width, bar_h),
                          cv::Scalar(0, 0, 255), -1);
            int ty = ly + (bar_h + conf_size.height) / 2;
            cv::putText(merged_image, conf_str, cv::Point(cx, ty),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }

    // 保存带框图（用于展示告警），使用当前点位 cur_point 而非可能已变化的 now_point_id
    std::string saved_path = cam->captureSnapshot(merged_image, cur_point, cur_light_flag);

    // ★ 使用累积结果触发模型告警（diff 差异告警由巡检结束后 run_post_patrol_diff 单独生成）
    if (!accumulated.empty()) {
        // 按损伤类型记录最大置信度，用于告警字段
        std::map<std::string, float> type_conf;
        for (const auto& obj : accumulated) {
            if (obj.objname[0]) {
                auto it = type_conf.find(obj.objname);
                if (it == type_conf.end() || obj.prob > it->second)
                    type_conf[obj.objname] = obj.prob;
            }
        }
        // 兜底：objname 为空时退化为当前模型的 damage_type
        if (type_conf.empty() && !damage_type.empty()) {
            type_conf[damage_type] = 0.f;
        }

        bool any_new_alarm = false;
        for (const auto& kv : type_conf) {
            const std::string& dt = kv.first;
            float conf = kv.second;
            if (CameraController::getInstance()->early_warning_process(camera_id, cur_point, cur_light_flag, dt, conf)) {
                any_new_alarm = true;
                WTALOGI("[damage] 点位[%d] L%d 已拍照并告警: %s (损伤类型: %s, 置信度: %.2f, 累积检测数: %zu)",
                        cur_point, cur_light_flag, saved_path.c_str(), dt.c_str(), conf, accumulated.size());
            }
        }

        // 冷却期内(同点位同类型)不再告警，对应损伤片段也不落盘：
        // 仅当本次存在过冷却期的新告警时才标记 damage_seen 以生成现场片段。
        if (any_new_alarm) {
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
// 巡检/标定结束后统一处理（差异 diff 与模型并行：diff 比对集中在此处执行）
//   - update_baseline=true（标定模式）：用不带框的原图覆盖更新基线，不做 diff 告警
//   - update_baseline=false（巡检模式）：
//       · 基线已存在 → 与基线做 diff 比对，检出差异则叠加差异框并生成"差异变化"告警
//       · 基线缺失   → 补建基线（首轮无可比对基线，不告警）
// ============================================================================
void run_post_patrol_diff(Camera* cam, bool update_baseline)
{
    if (!cam) return;

    auto tasks = cam->drain_diff_queue();
    if (tasks.empty()) return;

    WTALOGI("[damage-diff] 摄像机[%s] 结束，开始处理 %zu 个点位（%s）",
            cam->getName().c_str(), tasks.size(),
            update_baseline ? "标定:更新基线" : "巡检:diff比对+基线补建");

    int write_count = 0;
    int alarm_count = 0;
    for (const auto& t : tasks) {
        if (t.raw_image.empty()) {
            WTALOGI("[damage-diff] 原图为空，跳过点位[%d] L%d", t.point_id, t.light_flag);
            continue;
        }
        const cv::Mat& raw_bgr = t.raw_image;

        std::string base_path = make_baseline_path(cam->orga_name, cam->getName(), t.point_id, t.light_flag);
        bool base_exists = (access(base_path.c_str(), 0) == 0);

        // ★ 巡检模式且已有基线 → 统一 diff 比对，检出差异则单独告警（与模型告警并列）
        if (!update_baseline && base_exists) {
            cv::Mat base_bgr = cv::imread(base_path, cv::IMREAD_COLOR);
            if (!base_bgr.empty()) {
                auto regions = diff_against_baseline(raw_bgr, base_bgr);
                if (!regions.empty()) {
                    // 在展示图（已含模型框）上叠加差异框；无展示图则退化为原图
                    cv::Mat show = t.display_path.empty() ? cv::Mat()
                                                          : cv::imread(t.display_path, cv::IMREAD_COLOR);
                    if (show.empty()) show = raw_bgr.clone();
                    // regions 基于基线坐标系，按比例缩放到展示图尺寸
                    float sx = base_bgr.cols > 0 ? (float)show.cols / base_bgr.cols : 1.f;
                    float sy = base_bgr.rows > 0 ? (float)show.rows / base_bgr.rows : 1.f;
                    for (const auto& r : regions) {
                        cv::Rect b((int)(r.bbox.x * sx), (int)(r.bbox.y * sy),
                                   (int)(r.bbox.width * sx), (int)(r.bbox.height * sy));
                        b &= cv::Rect(0, 0, show.cols, show.rows);
                        if (b.area() <= 0) continue;
                        // 倾斜框贴合轮廓
                        cv::RotatedRect rb = r.rbox;
                        rb.center.x *= sx;
                        rb.center.y *= sy;
                        rb.size.width  *= sx;
                        rb.size.height *= sy;
                        cv::Point2f pts2f[4];
                        rb.points(pts2f);
                        cv::Point pts[4];
                        for (int k = 0; k < 4; ++k) pts[k] = cv::Point((int)pts2f[k].x, (int)pts2f[k].y);
                        std::vector<std::vector<cv::Point>> poly =
                            { std::vector<cv::Point>(pts, pts + 4) };
                        cv::polylines(show, poly, true, cv::Scalar(0, 255, 255), 2);
                        char lbl[64];
                        snprintf(lbl, sizeof(lbl), "diff %.2f", r.score);
                        cv::putText(show, lbl, cv::Point(b.x, std::max(0, b.y - 4)), cv::FONT_HERSHEY_SIMPLEX,
                                    0.5, cv::Scalar(0, 255, 255), 1);
                    }
                    // 覆盖保存展示图并写回 pic_filename（captureSnapshot 命名与 display_path 一致）
                    cam->captureSnapshot(show, t.point_id, t.light_flag);
                    if (CameraController::getInstance()->early_warning_process(cam->get_id(), t.point_id,
                        t.light_flag, "差异变化"))
                    {
                        ++alarm_count;
                        WTALOGI("[damage-diff] 摄像机[%d] 点位[%d] L%d 差异告警，差异区域数=%zu",
                                cam->get_id(), t.point_id, t.light_flag, regions.size());
                    }
                }
            }
        }

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

    WTALOGI("[damage-diff] 摄像机[%s] 处理结束，基线写入 %d 张，差异告警 %d 条",
            cam->getName().c_str(), write_count, alarm_count);
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

cv::Mat ax_model_damage::getPointTextBmp(int point_id, bool is_moving)
{
    auto key = std::make_pair(point_id, is_moving);
    auto it = m_point_text_bmp_cache.find(key);
    if (it != m_point_text_bmp_cache.end()) {
        return it->second;
    }

    auto *cam = CameraController::getInstance()->getCamera(camera_id);
    if (!cam) return cv::Mat();

    std::string point_name;
    for (const auto& pos : cam->getPresetPositions()) {
        if (pos.id == point_id) {
            point_name = pos.name;
            break;
        }
    }

    cv::Mat bmp;
    if (!point_name.empty()) {
        char text[256];
        snprintf(text, sizeof(text), "Point: %d %s (%s)", point_id, point_name.c_str(),
                 is_moving ? "moving..." : "arrived");
        auto& ft = FreeTypeOverlay::instance();
        if (!ft.ready()) {
            ft.init("/wt_tech/conf/simsun.ttc", 20);
        }
        if (ft.ready()) {
            bmp = ft.renderTextRGBA(text, cv::Scalar(139, 0, 0, 255), 2);
            if (bmp.empty()) {
                WTALOGI("[FreeType] render point_text failed: '%s'", text);
            }
        }
    }

    m_point_text_bmp_cache[key] = bmp;
    return bmp;
}

cv::Mat ax_model_damage::getDamageLabelBmp(const std::string &objname)
{
    // 跨实例共享：映射表（英文键->中文名）与已渲染中文位图缓存，惰性加载一次。
    static std::mutex s_mtx;
    static bool s_map_loaded = false;
    static std::map<std::string, std::string> s_name_map;   // 英文键 -> 中文名
    static std::map<std::string, cv::Mat>     s_bmp_cache;  // 英文键 -> BGRA 位图

    if (objname.empty()) return cv::Mat();

    std::lock_guard<std::mutex> lk(s_mtx);

    // 首次调用加载映射表 /wt_tech/conf/damage_names.txt
    if (!s_map_loaded) {
        s_map_loaded = true;
        std::ifstream fin("/wt_tech/conf/damage_names.txt");
        if (fin.is_open()) {
            std::string line;
            while (std::getline(fin, line)) {
                // 去除尾部回车（兼容 CRLF）
                while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
                    line.pop_back();
                // 跳过空行与注释
                size_t s = line.find_first_not_of(" \t");
                if (s == std::string::npos || line[s] == '#') continue;
                // 第一个空白分隔符前为英文键，之后（去首尾空白）为中文名
                size_t sep = line.find_first_of(" \t", s);
                if (sep == std::string::npos) continue;
                std::string key = line.substr(s, sep - s);
                size_t vs = line.find_first_not_of(" \t", sep);
                if (vs == std::string::npos) continue;
                size_t ve = line.find_last_not_of(" \t");
                std::string val = line.substr(vs, ve - vs + 1);
                if (!key.empty() && !val.empty()) s_name_map[key] = val;
            }
            WTALOGI("[FreeType] 损伤中文映射表加载完成: %zu 条", s_name_map.size());
        } else {
            WTALOGI("[FreeType] 未找到损伤中文映射表 /wt_tech/conf/damage_names.txt, 标签回退英文原文");
        }
    }

    // 命中位图缓存直接返回
    auto cit = s_bmp_cache.find(objname);
    if (cit != s_bmp_cache.end()) return cit->second;

    // 查中文名，缺失则以英文原文兜底（占位）
    auto nit = s_name_map.find(objname);
    const std::string& text = (nit != s_name_map.end()) ? nit->second : objname;

    cv::Mat bmp;
    auto& ft = FreeTypeOverlay::instance();
    if (!ft.ready()) {
        ft.init("/wt_tech/conf/simsun.ttc", 20);
    }
    if (ft.ready()) {
        // 深色字（白底），关闭置信度显示，仅渲染中文名
        bmp = ft.renderTextRGBA(text, cv::Scalar(0, 0, 0, 255), 2);
        if (bmp.empty()) {
            WTALOGI("[FreeType] render damage label failed: '%s'", text.c_str());
        }
    }

    s_bmp_cache[objname] = bmp; // 即使为空也缓存，避免反复尝试
    return bmp;
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

    // diff 门控全局阈值（可选，缺省用默认值）。放在 wt_rtsp.json，全局生效。
    if (jsondata.contains("DIFF_SSIM_TH"))    g_diff_th.ssim_th    = jsondata["DIFF_SSIM_TH"].get<float>();
    if (jsondata.contains("DIFF_SSIM_LO_TH")) g_diff_th.ssim_lo_th = jsondata["DIFF_SSIM_LO_TH"].get<float>();
    if (jsondata.contains("DIFF_ABSD_RATIO")) g_diff_th.absd_ratio = jsondata["DIFF_ABSD_RATIO"].get<float>();
    if (jsondata.contains("DIFF_BLOCK"))      g_diff_th.block      = jsondata["DIFF_BLOCK"].get<int>();
    if (jsondata.contains("DIFF_MIN_AREA"))   g_diff_th.min_area   = jsondata["DIFF_MIN_AREA"].get<int>();
    if (jsondata.contains("DIFF_PIX_DIFF"))   g_diff_th.pix_diff   = jsondata["DIFF_PIX_DIFF"].get<int>();
    WTALOGI("diff 阈值: SSIM_TH=%.3f SSIM_LO_TH=%.3f ABSD_RATIO=%.3f BLOCK=%d MIN_AREA=%d PIX_DIFF=%d",
            g_diff_th.ssim_th, g_diff_th.ssim_lo_th, g_diff_th.absd_ratio, g_diff_th.block,
            g_diff_th.min_area, g_diff_th.pix_diff);

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

    // 只对第一个模型实例预渲染点位文字（draw_custom 只用第一个模型）
    if (!m_damage_models.empty()) {
        auto *first_model = dynamic_cast<ax_model_damage *>(m_damage_models.begin()->model.get());
        if (first_model) {
            auto& ft = FreeTypeOverlay::instance();
            if (!ft.ready()) {
                ft.init("/wt_tech/conf/simsun.ttc", 20);
            }
            auto *cam = CameraController::getInstance()->getCamera(camera_id);
            if (cam && ft.ready()) {
                for (const auto& pos : cam->getPresetPositions()) {
                    if (pos.name.empty()) continue;
                    for (bool moving : {false, true}) {
                        char text[256];
                        snprintf(text, sizeof(text), "Point: %d %s (%s)", pos.id, pos.name.c_str(),
                                 moving ? "moving..." : "arrived");
                        cv::Mat bmp = ft.renderTextRGBA(text, cv::Scalar(139, 0, 0, 255), 2);
                        if (bmp.empty()) {
                            WTALOGI("[FreeType] render point_text failed: '%s'", text);
                        } else {
                            WTALOGI("[FreeType] point_text ready: '%s' size=%dx%d", text, bmp.cols, bmp.rows);
                        }
                        first_model->m_point_text_bmp_cache[{pos.id, moving}] = bmp;
                    }
                }
            }
        }
    }

    return 0;
}

int wt_damage_multi_model_recognize::inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    int result = 0;

    // [DIAG] 节流日志：确认推理线程是否仍被调用（判断 IVPS 是否断供 / 推理是否停摆）
    {
        static int diag_cnt = 0;
        if (diag_cnt++ % 30 == 0) {
            auto *dcam = CameraController::getInstance()->getCamera(camera_id);
            long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            WTALOGI("[DIAG infer] cam[%d] inference被调用 posture=%d phase_ready=%lld now-phase=%lld (第%d次)",
                    camera_id, dcam ? (int)dcam->posture_completed.load() : -1,
                    dcam ? dcam->phase_ready_ms.load() : -1,
                    dcam ? (now_ms - dcam->phase_ready_ms.load()) : -1, diag_cnt);
        }
    }

    if (m_damage_models.empty() && m_specialized_models.empty()) {
        return 0;
    }

    // ★ 相位门控：仅在到达点位且灯光相位就绪时执行推理；
    //   移动/回位/灯光切换中跳过，节省算力（此期间不产生检测结果，也不绘制/累积）
    auto *cam = CameraController::getInstance()->getCamera(camera_id);
    if (!cam || !cam->posture_completed || cam->phase_ready_ms.load() <= 0) {
        results->nObjSize = 0;
        return 0;
    }

    long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

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

    // ★ 差异 diff 与模型并行：无论有无差异都执行模型推理；
    //   diff 比对不再前置门控，而是在巡检结束后由 run_post_patrol_diff 统一处理并告警。
    //   等待流延迟余量，避免在 RTSP 缓冲中的旧灯光帧上做推理（与拍照 phase_settled 一致）。
    const long long STREAM_LATENCY_SETTLE_MS = 40;
    if (now_ms - cam->phase_ready_ms.load() < STREAM_LATENCY_SETTLE_MS) {
        // [DIAG] 节流日志：被 90ms 稳定门控挡下（若持续刷此日志说明 phase_ready 被反复刷新）
        static int diag_cnt = 0;
        if (diag_cnt++ % 30 == 0) {
            WTALOGI("[DIAG settle] cam[%d] 未过稳定门控 now-phase=%lld < %lld (第%d次)",
                    camera_id, now_ms - cam->phase_ready_ms.load(), STREAM_LATENCY_SETTLE_MS, diag_cnt);
        }
        results->nObjSize = 0;
        return 0; // 相位尚未稳定，暂不推理（下一帧再判）
    }

    // 计时：仅统计通过门控后真正执行模型推理的帧耗时
    auto t_infer_start = std::chrono::steady_clock::now();

    // 并行推理：每个模型在独立线程中执行
    struct ModelResult {
        axdl_results_t results;
        std::string damage_type;
        int ret;
    };

    std::vector<std::future<ModelResult>> futures;
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
            strncpy(mr.results.mObjects[i].objname, mr.damage_type.c_str(),
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

    {
        auto t_infer_end = std::chrono::steady_clock::now();
        auto infer_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_infer_end - t_infer_start).count();
        static int infer_log_cnt = 0;
        if (infer_log_cnt++ % 30 == 0) {
            WTALOGI("[DIAG infer-time] cam[%d] 模型数=%d 检出=%d 耗时=%lldms (第%d次)",
                    camera_id, (int)run_models.size(), (int)results->nObjSize, infer_ms, infer_log_cnt);
        }
    }

    return result;
}

void wt_damage_multi_model_recognize::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    if (m_damage_models.empty()) return;
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


