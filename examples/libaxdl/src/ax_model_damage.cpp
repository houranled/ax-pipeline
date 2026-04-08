#include "ax_model_damage.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>  // 添加此行以使用 std::accumulate
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

int wt_damage_multi_model_recognize::inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    // 推理时根据当前摄像头是哪个点位来选用模型进行推理
    WTALOGI("推理时根据当前摄像头是哪个点位来选用模型进行推理");

    auto camera = CameraController::getInstance()->getCamera(1); // 获取当前摄像头信息
    camera->now_point_id;

    std::string camera_name("");

    // 根据摄像头xxx选择模型
    int model_index;
    if (camera->now_point_id == 1) {
        model_index = 1;  // 特殊摄像头使用专用模型
    } else {
        model_index = 0;  // 其他摄像头使用通用模型
    }

    // 使用选定的模型进行推理
    m_models[model_index].get()->inference(pstFrame, crop_resize_box, results);


    return 0;
}
