#include "ax_model_damage.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>  // 添加此行以使用 std::accumulate
#include <algorithm>
#include "../../camera/camera_controller.hpp"

//int ax_model_damage::inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
//{
//    this->ax_model_single_base_t::inference(pstFrame, crop_resize_box, results); // 调用基类推理函数 后续重写推理逻辑:使用多模型

//    return 0;
//}

int ax_model_damage::post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    // 获取模型输出
    const ax_runner_tensor_t *pOutputsInfo = m_runner->get_outputs_ptr();
    const float *output_ptr = (float *)pOutputsInfo[0].pVirAddr;

    // 解析输出形状
    int num_anchors = 0;
    int num_classes = CLASS_NUM; // 使用类成员变量

    if (pOutputsInfo[0].vShape.size() == 3) {
        // YOLOv8 OBB format: [batch=1, channels=5+classes, anchors=8400]
        num_anchors = (int)pOutputsInfo[0].vShape[2];  // 8400
        int channels = (int)pOutputsInfo[0].vShape[1];  // 5 + num_classes
    } else {
        ALOGE("YOLOv8 OBB requires 3D output [batch, channels, anchors]");
        return -1;
    }

    if (num_anchors <= 0) {
        ALOGE("Invalid YOLOv8 OBB output shape");
        return -1;
    }

    // Generate grid strides for multi-scale detection
    std::vector<int> strides = {8, 16, 32};
    std::vector<detection::GridAndStride> grid_strides;
    detection::generate_grids_and_stride(get_algo_width(), get_algo_height(), strides, grid_strides);

    // Generate OBB proposals using AXERA-style approach
    std::vector<detection::Object> proposals;
    detection::generate_proposals_yolov8_obb_xyxyxyxy(grid_strides, output_ptr, PROB_THRESHOLD, proposals,
        get_algo_width(), get_algo_height(), num_classes);

    // Apply NMS and coordinate transformation
    std::vector<detection::Object> objects;
    detection::get_out_obb_bbox(proposals, objects, NMS_THRESHOLD, get_algo_width(), get_algo_height(),
                                pstFrame->nHeight, pstFrame->nWidth);

    // Convert to axdl_results_t format
    results->nObjSize = MIN(objects.size(), SAMPLE_MAX_BBOX_COUNT);
    for (int i = 0; i < results->nObjSize; i++) {
        const detection::Object& obj = objects[i];

        // Set axis-aligned bounding box for compatibility
        results->mObjects[i].bbox.x = obj.rect.x;
        results->mObjects[i].bbox.y = obj.rect.y;
        results->mObjects[i].bbox.w = obj.rect.width;
        results->mObjects[i].bbox.h = obj.rect.height;
        results->mObjects[i].label = obj.label;
        results->mObjects[i].prob = obj.prob;
        /* ↑ 似乎是冗余的 */

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


void ax_model_damage::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    // Draw OBB polygons for each detected object
    for (int i = 0; i < results->nObjSize; i++) {
        if (results->mObjects[i].bHasBoxVertices) {
            cv::Point pts[4];
            for (int j = 0; j < 4; j++) {
                pts[j] = cv::Point(
                    results->mObjects[i].bbox_vertices[j].x * image.cols + offset_x,
                    results->mObjects[i].bbox_vertices[j].y * image.rows + offset_y
                );
            }

            // Draw rotated rectangle/polygon
            for (int j = 0; j < 4; j++) {
                cv::line(image, pts[j], pts[(j + 1) % 4],
                        cv::Scalar(0, 255, 0), thickness, 8, 0);
            }

            // Draw label with class name and confidence
            std::string label_str = std::string(results->mObjects[i].objname) + " " +
                                   std::to_string(static_cast<int>(results->mObjects[i].prob * 100)) + "%";
            cv::putText(image, label_str, pts[0], cv::FONT_HERSHEY_SIMPLEX,
                       fontscale, cv::Scalar(0, 255, 0), thickness);
        }
    }
}

//void ax_model_damage::draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness)
//{
//    // Custom OSD drawing for OBB - use polygon drawing
//    //for (int i = 0; i < results->nObjSize; i++) {
//    //    if (results->mObjects[i].bHasBoxVertices) {
//    //        if (results->bObjTrack) {
//    //            m_drawers[chn].add_polygon(results->mObjects[i].bbox_vertices, 4,
//    //                COCO_COLORS_ARGB[results->mObjects[i].track_id % COCO_COLORS_ARGB.size()], thickness);
//    //        } else {
//    //            m_drawers[chn].add_polygon(results->mObjects[i].bbox_vertices, 4,
//    //                COCO_COLORS_ARGB[results->mObjects[i].label % COCO_COLORS_ARGB.size()], thickness);
//    //        }

//    //        // Add label
//    //        if (results->bObjTrack && b_draw_obj_name) {
//    //            m_drawers[chn].add_text(
//    //                std::string(results->mObjects[i].objname) + " " + std::to_string(results->mObjects[i].track_id),
//    //                results->mObjects[i].bbox_vertices[0],
//    //                {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
//    //        } else if (b_draw_obj_name) {
//    //            m_drawers[chn].add_text(results->mObjects[i].objname,
//    //                results->mObjects[i].bbox_vertices[0],
//    //                {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
//    //        }
//
//            //生成告警 调用camera_Controller
//            CameraController::getInstance()->early_warning_process(chn/2);
//        }
//    }
//}

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
                WTALOGE("Failed to create model instance for type '%s'", model_type.c_str());
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

    // Try to get current camera from channel name or other means
    // For now, we'll use the channel name to identify the camera
    std::string channel_name = get_channel_name();

    // Find camera by channel name
    for (auto camera : controller->getAllCameras()) {
        if (camera->getName() == channel_name) {
            int current_point_id = camera->now_point_id;

            // Find preset position by ID
            for (const auto& pos : camera->getPresetPositions()) {
                if (pos.id == current_point_id) {
                    WTALOGI("Current point: ID=%d, Name='%s'", current_point_id, pos.name.c_str());
                    return pos.name; // Return full point name for prefix matching
                }
            }
            break;
        }
    }

    WTALOGI("Could not find current point information for channel '%s'", channel_name.c_str());
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
        WTALOGI("No specific model type match for point '%s', using fallback type '%s'",
                point_name.c_str(), fallback_type.c_str());
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

    if (current_point_name.empty()) {
        WTALOGI("Could not determine current point name, skipping inference");
        return -1;
    }

    // Find appropriate model type for current point
    std::string model_type = find_model_type_for_point(current_point_name);

    if (model_type.empty()) {
        WTALOGI("No suitable model type found for point '%s'", current_point_name.c_str());
        return -1;
    }

    // Get all models of the matched type
    std::vector<std::shared_ptr<ax_model_base>> models_to_run = get_models_by_type(model_type);

    if (models_to_run.empty()) {
        WTALOGI("No models found for type '%s'", model_type.c_str());
        return -1;
    }

    // Run inference with all models of the matched type
    WTALOGI("Running inference for point '%s' with %zu models of type '%s'",
            current_point_name.c_str(), models_to_run.size(), model_type.c_str());

    int result = 0;
    for (auto& model : models_to_run) {
        int model_result = model->inference(pstFrame, crop_resize_box, results);
        if (model_result != 0) {
            WTALOGI("Model inference failed for type '%s', result=%d", model_type.c_str(), model_result);
            result = model_result; // Return last error, but continue with other models
        }
    }

    return result;
}
