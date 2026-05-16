#include "ax_model_damage.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>  // 添加此行以使用 std::accumulate
#include <algorithm>
#include <unistd.h>
#include <ctime>
#include "../../camera/camera_controller.hpp"
#include "../../utilities/json.hpp"

//int ax_model_damage::inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
//{
//    this->ax_model_single_base_t::inference(pstFrame, crop_resize_box, results); // 调用基类推理函数 后续重写推理逻辑:使用多模型

//    return 0;
//}

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
                strcpy(results->mObjects[i].objname, CLASS_NAMES[obj.label].c_str());
            } else {
                strcpy(results->mObjects[i].objname, "unknown");
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
                strcpy(results->mObjects[i].objname, CLASS_NAMES[obj.label].c_str());
            } else {
                strcpy(results->mObjects[i].objname, "unknown");
            }
        }

        return 0;
    }
}


void ax_model_damage::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    draw_bbox(image, results, fontscale, thickness, offset_x, offset_y);

    auto *cam = CameraController::getInstance()->getCamera(camera_id);
    if (!cam || !cam->is_patroling()) return;

    int cur_point = cam->now_point_id;

    // 每个点位仅触发一次拍照+告警：point_id 变化才视为"新点位"
    static std::map<int, int> last_fired_point; // camera_id -> 上次已触发的点位
    auto it = last_fired_point.find(camera_id);
    if (it != last_fired_point.end() && it->second == cur_point) {
        return; // 同点位已触发过，跳过
    }

    if (!cam->posture_completed)
        return; //未到点位，跳过

    // 生成路径（沿用 record_ffmpeg_pipe_jpg 的目录方案）
    time_t now = time(nullptr) + 8 * 3600;
    tm *t = gmtime(&now);
    char ymd[16] = {0};
    snprintf(ymd, sizeof(ymd), "%04d%02d%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

    char dirname[160] = {0};
    snprintf(dirname, sizeof(dirname), "/wt_tech/data/F02/%s/%s_%d/image", ymd, ymd, t->tm_hour);
    if (access(dirname, 0) != 0) {
        char mk[256] = {0};
        snprintf(mk, sizeof(mk), "mkdir -p %s", dirname);
        system(mk);
    }

    char filepath[320] = {0};
    snprintf(filepath, sizeof(filepath), "%s/%s_%02d_%s_%d.jpg", dirname, ymd, t->tm_hour, channel_name.c_str(), cur_point);

    std::vector<int> jpg_params = { cv::IMWRITE_JPEG_QUALITY, 90 };
    if (!cv::imwrite(filepath, image, jpg_params)) {
        WTALOGI("[damage] 点位[%d] cv::imwrite 失败: %s", cur_point, filepath);
    }

    // 写回 pipeline，让 generateAlarm 拿到本张快照路径
    if (auto *pipe = cam->get_pipeline()) {
        strncpy(pipe->pic_filename, filepath, sizeof(pipe->pic_filename) - 1);
        pipe->pic_filename[sizeof(pipe->pic_filename) - 1] = '\0';
    }

    // 触发新增点位告警
    if (results->nObjSize > 0)
        CameraController::getInstance()->early_warning_process(camera_id);

    // 标记该点位已处理
    last_fired_point[camera_id] = cur_point;
    WTALOGI("[damage] 点位[%d] 已拍照并告警: %s", cur_point, filepath);
}

int ax_model_damage::sub_init(void *json_obj)
{
    // Extract model type from configuration (called after base init)
    auto jsondata = *(nlohmann::json *)json_obj;
    if (jsondata.contains("MODEL_PATH")) {
        std::string model_path = jsondata["MODEL_PATH"];
        model_type = extract_type_from_filename(model_path);
        WTALOGI("Model type extracted: '%s' from path '%s'", model_type.c_str(), model_path.c_str());
    }

    return 0;
}

std::string ax_model_damage::extract_type_from_filename(const std::string& model_path)
{
    // Extract filename from path
    std::string filename = model_path;
    size_t last_slash = model_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        filename = model_path.substr(last_slash + 1);
    }

    // Remove file extension
    size_t last_dot = filename.find_last_of(".");
    if (last_dot != std::string::npos) {
        filename = filename.substr(0, last_dot);
    }

    // Look for type prefix pattern (e.g., "damage_model_A1", "damage_model_B2")
    // Expected format: <base_name>_<type><index>
    size_t last_underscore = filename.find_last_of("_");
    if (last_underscore != std::string::npos && last_underscore < filename.length() - 1) {
        std::string suffix = filename.substr(last_underscore + 1);

        // Check if suffix starts with a letter (type) followed by digits
        if (!suffix.empty() && isalpha(suffix[0])) {
            // Extract the type letter(s) before first digit
            std::string type_prefix;
            for (char c : suffix) {
                if (isalpha(c)) {
                    type_prefix += c;
                } else {
                    break; // Stop at first digit
                }
            }

            if (!type_prefix.empty()) {
                WTALOGI("Extracted model type '%s' from filename '%s'", type_prefix.c_str(), model_path.c_str());
                return type_prefix;
            }
        }
    }

    // Fallback: try to find any uppercase letter sequence that could be a type
    for (size_t i = 0; i < filename.length(); i++) {
        if (isupper(filename[i])) {
            size_t start = i;
            while (i < filename.length() && isupper(filename[i])) {
                i++;
            }
            std::string potential_type = filename.substr(start, i - start);
            if (potential_type.length() >= 1 && potential_type.length() <= 3) {
                WTALOGI("Extracted model type '%s' from filename '%s' (fallback method)", potential_type.c_str(), model_path.c_str());
                return potential_type;
            }
        }
    }

    // Default type if no pattern matches
    WTALOGI("Could not extract type from filename '%s', using default type 'default'", model_path.c_str());
    return "default";
}

wt_damage_multi_model_recognize::wt_damage_multi_model_recognize()
{
    WTALOGI("Instance wt_damage_multi_model_recognize object");
}

int wt_damage_multi_model_recognize::init(void *json_obj)
{
    // Parse JSON configuration first to get model types
    auto jsondata = *(nlohmann::json *)json_obj;

    // Parse model list with type information
    if (jsondata.contains("MODEL_LIST")) {
        nlohmann::json json_model_list = jsondata["MODEL_LIST"];

        for (int i = 0; i < json_model_list.size(); i++) {
            auto& model_json = json_model_list[i];

            // Create model instance
            std::string strModelType;
            int mt = get_model_type(&model_json, strModelType);
            std::shared_ptr<ax_model_base> model((ax_model_base *)OBJFactory::getInstance().getObjectByID(mt));

            if (!model) {
                return -1;
            }

            // Initialize the model
            int ret = model->init((void *)&model_json);
            if (ret != 0) {
                WTALOGI("Failed to initialize model, ret=%d", ret);
                return ret;
            }

            // Get model type from the model instance (for ax_model_damage)
            std::string actual_model_type = "default";
            ax_model_damage* damage_model = dynamic_cast<ax_model_damage*>(model.get());
            if (damage_model) {
                actual_model_type = damage_model->get_model_type();
            }

            // Add model to type group
            m_model_types[actual_model_type].type_name = actual_model_type;
            m_model_types[actual_model_type].models.push_back(model);
            m_models.push_back(model); // Keep parent class vector for compatibility

            WTALOGI("Added model to type '%s': model index %d", actual_model_type.c_str(), i);
        }
    }

    // Parse point prefix to model type mappings
    if (jsondata.contains("POINT_PREFIX_MAPPINGS")) {
        nlohmann::json json_mappings = jsondata["POINT_PREFIX_MAPPINGS"];

        for (auto& mapping : json_mappings.items()) {
            std::string point_prefix = mapping.key();
            std::string model_type = mapping.value();

            // Validate model type exists
            if (m_model_types.find(model_type) != m_model_types.end()) {
                m_point_prefix_to_model_type[point_prefix] = model_type;
                WTALOGI("Added point prefix mapping: '%s' -> model type '%s'",
                        point_prefix.c_str(), model_type.c_str());
            } else {
                WTALOGI("Model type '%s' not found for point prefix '%s'",
                        model_type.c_str(), point_prefix.c_str());
            }
        }
    } else {
        WTALOGI("No POINT_PREFIX_MAPPINGS found in configuration");
    }

    // Log model type summary
    for (const auto& type_pair : m_model_types) {
        WTALOGI("Model type '%s' has %zu models",
                type_pair.first.c_str(), type_pair.second.models.size());
    }

    return 0;
}

int wt_damage_multi_model_recognize::add_model_type_mapping(const std::string& point_prefix, const std::string& model_type)
{
    // Validate model type exists
    if (m_model_types.find(model_type) != m_model_types.end()) {
        m_point_prefix_to_model_type[point_prefix] = model_type;
        WTALOGI("Added point prefix mapping: '%s' -> model type '%s'",
                point_prefix.c_str(), model_type.c_str());
        return 0;
    } else {
        WTALOGI("Model type '%s' not found for point prefix '%s'",
                model_type.c_str(), point_prefix.c_str());
        return -1;
    }
}

std::string wt_damage_multi_model_recognize::get_current_point_prefix()
{
    // Get current point information from camera controller
    CameraController* controller = CameraController::getInstance();
    auto camera = controller->getCamera(this->camera_id);

    int current_point_id = camera->now_point_id;

    // Find preset position by ID
    for (const auto& pos : camera->getPresetPositions()) {
        if (pos.id == current_point_id) {
            return pos.name; // Return full point name for prefix matching
        }
    }

    return ""; // Return empty string if not found
}

std::string wt_damage_multi_model_recognize::find_model_type_for_point(const std::string& point_name)
{
    // Find matching model type by point name prefix
    for (const auto& mapping : m_point_prefix_to_model_type) {
        const std::string& prefix = mapping.first;
        const std::string& model_type = mapping.second;

        // Check if point name starts with the prefix
        if (point_name.find(prefix) == 0) {
            WTALOGI("Found matching model type: point '%s' matches prefix '%s' -> model type '%s'",
                    point_name.c_str(), prefix.c_str(), model_type.c_str());
            return model_type;
        }
    }

    // If no specific match found, return first available model type as fallback
    if (!m_model_types.empty()) {
        std::string fallback_type = m_model_types.begin()->first;
        WTALOGI("No specific model type match for point '%s', using fallback type '%s'", point_name.c_str(),
            fallback_type.c_str());
        return fallback_type;
    }

    WTALOGI("No model types available for point '%s'", point_name.c_str());
    return "";
}

std::vector<std::shared_ptr<ax_model_base>> wt_damage_multi_model_recognize::get_models_by_type(const std::string& model_type)
{
    auto it = m_model_types.find(model_type);
    if (it != m_model_types.end()) {
        return it->second.models;
    }

    WTALOGI("Model type '%s' not found, returning empty vector", model_type.c_str());
    return std::vector<std::shared_ptr<ax_model_base>>();
}

int wt_damage_multi_model_recognize::inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    // Get current point name prefix
    std::string current_point_name = get_current_point_prefix();

    std::vector<std::shared_ptr<ax_model_base>> models_to_run;
    std::string model_type;
    int result = 0;
    if (current_point_name.empty()) {
        WTALOGI("Could not determine current point name！");
    } else {
        // Find appropriate model type for current point
        model_type = find_model_type_for_point(current_point_name);

        if (model_type.empty()) {
            WTALOGI("No suitable model type found for point '%s'", current_point_name.c_str());
        }

        // 获取所有匹配类型的模型
        models_to_run = get_models_by_type(model_type);
    }

    // 如果没有对应类型的模型，则遍历使用所有模型
    if (models_to_run.empty()) {
        WTALOGI("未找到类型为 '%s' 的模型，应当遍历使用所有模型", model_type.c_str());
        models_to_run = m_models;
    } else {
        WTALOGI("Running inference for point '%s' with %zu models of type '%s'",
            current_point_name.c_str(), models_to_run.size(), model_type.c_str());
    }

    results->nObjSize = 0;
    for (auto& model : models_to_run) {
        axdl_results_t temp_results = {};
        int model_result = model->inference(pstFrame, crop_resize_box, &temp_results);
        if (model_result != 0) {
            WTALOGI("Model inference failed, result=%d", model_result);
            result = model_result;
            continue;
        }

        // 合并当前模型的推理结果到主results中
        int space_left = SAMPLE_MAX_BBOX_COUNT - results->nObjSize;
        int to_copy = std::min((int)temp_results.nObjSize, space_left);
        if (to_copy > 0) {
            memcpy(&(results->mObjects[results->nObjSize]), &(temp_results.mObjects[0]), to_copy * sizeof(results->mObjects[0]));
            results->nObjSize += to_copy;
        }

        if (results->nObjSize >= SAMPLE_MAX_BBOX_COUNT) {
            WTALOGI("Results buffer full, skipping remaining models");
            break;
        }
    }

    return result;

}
