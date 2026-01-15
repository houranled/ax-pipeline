#include "ax_model_damage.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>

// YOLO11 OBB implementation
struct OBBResult {
    float x, y, w, h, angle;
    float score;
    int label;
};

static void obb_decode(const float* output, int num_classes, int num_anchors,
                      std::vector<OBBResult>& results, float conf_threshold) {
    results.clear();

    // YOLO11 OBB output format: [x, y, w, h, angle, class_scores...]
    for (int i = 0; i < num_anchors; ++i) {
        const float* ptr = output + i * (5 + num_classes);

        float x = ptr[0];
        float y = ptr[1];
        float w = ptr[2];
        float h = ptr[3];
        float angle = ptr[4];

        // Find max class score
        float max_score = 0;
        int max_class = 0;
        for (int j = 0; j < num_classes; ++j) {
            if (ptr[5 + j] > max_score) {
                max_score = ptr[5 + j];
                max_class = j;
            }
        }

        if (max_score >= conf_threshold) {
            OBBResult result;
            result.x = x;
            result.y = y;
            result.w = w;
            result.h = h;
            result.angle = angle;
            result.score = max_score;
            result.label = max_class;
            results.push_back(result);
        }
    }
}

static void obb_to_vertices(const OBBResult& obb, axdl_point_t vertices[4]) {
    float cos_a = cos(obb.angle);
    float sin_a = sin(obb.angle);

    // Half dimensions
    float hw = obb.w * 0.5f;
    float hh = obb.h * 0.5f;

    // Calculate rotated corners
    float corners[4][2] = {
        {-hw, -hh},  // top-left
        { hw, -hh},  // top-right
        { hw,  hh},  // bottom-right
        {-hw,  hh}   // bottom-left
    };

    for (int i = 0; i < 4; ++i) {
        vertices[i].x = obb.x + corners[i][0] * cos_a - corners[i][1] * sin_a;
        vertices[i].y = obb.y + corners[i][0] * sin_a + corners[i][1] * cos_a;
    }
}

static float obb_iou(const OBBResult& a, const OBBResult& b) {
    // Simplified IoU calculation for OBB - using axis-aligned bbox approximation
    float a_x1 = a.x - a.w * 0.5f;
    float a_y1 = a.y - a.h * 0.5f;
    float a_x2 = a.x + a.w * 0.5f;
    float a_y2 = a.y + a.h * 0.5f;

    float b_x1 = b.x - b.w * 0.5f;
    float b_y1 = b.y - b.h * 0.5f;
    float b_x2 = b.x + b.w * 0.5f;
    float b_y2 = b.y + b.h * 0.5f;

    float inter_x1 = std::max(a_x1, b_x1);
    float inter_y1 = std::max(a_y1, b_y1);
    float inter_x2 = std::min(a_x2, b_x2);
    float inter_y2 = std::min(a_y2, b_y2);

    if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) {
        return 0.0f;
    }

    float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    float a_area = a.w * a.h;
    float b_area = b.w * b.h;
    float union_area = a_area + b_area - inter_area;

    return inter_area / union_area;
}

static void obb_nms(std::vector<OBBResult>& results, float nms_threshold) {
    std::sort(results.begin(), results.end(),
              [](const OBBResult& a, const OBBResult& b) {
                  return a.score > b.score;
              });

    std::vector<bool> suppressed(results.size(), false);

    for (size_t i = 0; i < results.size(); ++i) {
        if (suppressed[i]) continue;

        for (size_t j = i + 1; j < results.size(); ++j) {
            if (suppressed[j]) continue;

            if (obb_iou(results[i], results[j]) > nms_threshold) {
                suppressed[j] = true;
            }
        }
    }

    // Remove suppressed results
    results.erase(std::remove_if(results.begin(), results.end(),
                                [&](const OBBResult& r) {
                                    return suppressed[&r - &results[0]];
                                }), results.end());
}

int ax_model_damage::post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    int nOutputSize = m_runner->get_num_outputs();
    if (nOutputSize < 1) {
        ALOGE("YOLO11 OBB requires at least 1 output");
        return -1;
    }

    const ax_runner_tensor_t *pOutputsInfo = m_runner->get_outputs_ptr();
    const float *output_ptr = (float *)pOutputsInfo[0].pVirAddr;

    // 从output vShape计算锚点的数量
    // YOLO11 OBB output format: [batch, anchors, 5+classes]
    int num_anchors = pOutputsInfo[0].vShape[1];  // 锚点个数TODO

    // 解码OBB结果
    std::vector<OBBResult> obb_results;
    obb_decode(output_ptr, CLASS_NUM, num_anchors, obb_results, PROB_THRESHOLD);

    // 应用NMS
    obb_nms(obb_results, NMS_THRESHOLD);

    // 转换为 axdl_results_t 格式
    results->nObjSize = MIN(obb_results.size(), SAMPLE_MAX_BBOX_COUNT);
    for (int i = 0; i < results->nObjSize; i++) {
        const OBBResult& obb = obb_results[i];

        // 将OBB转换为顶点
        axdl_point_t vertices[4];
        obb_to_vertices(obb, vertices);

        // 计算轴对齐的边界框以确保兼容性
        float min_x = vertices[0].x, max_x = vertices[0].x;
        float min_y = vertices[0].y, max_y = vertices[0].y;
        for (int j = 1; j < 4; ++j) {
            min_x = std::min(min_x, vertices[j].x);
            max_x = std::max(max_x, vertices[j].x);
            min_y = std::min(min_y, vertices[j].y);
            max_y = std::max(max_y, vertices[j].y);
        }

        results->mObjects[i].bbox.x = min_x;
        results->mObjects[i].bbox.y = min_y;
        results->mObjects[i].bbox.w = max_x - min_x;
        results->mObjects[i].bbox.h = max_y - min_y;
        results->mObjects[i].label = obb.label;
        results->mObjects[i].prob = obb.score;

        // 设置 OBB 顶点
        results->mObjects[i].bHasBoxVertices = 1;
        for (int j = 0; j < 4; ++j) {
            results->mObjects[i].bbox_vertices[j].x = vertices[j].x;
            results->mObjects[i].bbox_vertices[j].y = vertices[j].y;
        }

        // 设置对象名称
        if (obb.label < (int)CLASS_NAMES.size()) {
            strcpy(results->mObjects[i].objname, CLASS_NAMES[obb.label].c_str());
        } else {
            strcpy(results->mObjects[i].objname, "unknown");
        }
    }

    return 0;
}

void ax_model_damage::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    // Custom drawing for OBB - draw rotated rectangles
    for (int i = 0; i < results->nObjSize; i++) {
        if (results->mObjects[i].bHasBoxVertices) {
            cv::Point pts[4];
            for (int j = 0; j < 4; j++) {
                pts[j] = cv::Point(
                    results->mObjects[i].bbox_vertices[j].x * image.cols + offset_x,
                    results->mObjects[i].bbox_vertices[j].y * image.rows + offset_y
                );
            }

            // Draw rotated rectangle
            for (int j = 0; j < 4; j++) {
                cv::line(image, pts[j], pts[(j + 1) % 4],
                        cv::Scalar(0, 255, 0), thickness, 8, 0);
            }

            // Draw label
            std::string label_str = std::string(results->mObjects[i].objname) + " " +
                                   std::to_string(static_cast<int>(results->mObjects[i].prob * 100)) + "%";
            cv::putText(image, label_str, pts[0], cv::FONT_HERSHEY_SIMPLEX,
                       fontscale, cv::Scalar(0, 255, 0), thickness);
        }
    }
}

void ax_model_damage::draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness)
{
    // Custom OSD drawing for OBB - use polygon drawing
    for (int i = 0; i < results->nObjSize; i++) {
        if (results->mObjects[i].bHasBoxVertices) {
            if (results->bObjTrack) {
                m_drawers[chn].add_polygon(results->mObjects[i].bbox_vertices, 4,
                    COCO_COLORS_ARGB[results->mObjects[i].track_id % COCO_COLORS_ARGB.size()], thickness);
            } else {
                m_drawers[chn].add_polygon(results->mObjects[i].bbox_vertices, 4,
                    COCO_COLORS_ARGB[results->mObjects[i].label % COCO_COLORS_ARGB.size()], thickness);
            }

            // Add label
            if (results->bObjTrack && b_draw_obj_name) {
                m_drawers[chn].add_text(
                    std::string(results->mObjects[i].objname) + " " + std::to_string(results->mObjects[i].track_id),
                    results->mObjects[i].bbox_vertices[0],
                    {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
            } else if (b_draw_obj_name) {
                m_drawers[chn].add_text(results->mObjects[i].objname,
                    results->mObjects[i].bbox_vertices[0],
                    {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
            }
        }
    }
}
