#include "ax_model_damage.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>

// YOLO11 OBB implementation - 支持[cx, cy, w, h, angle]格式转换为4个顶点坐标
struct OBBResult {
    float cx, cy, w, h, angle;  // 中心点、宽高、角度
    float score;
    int label;
};

// 将[cx, cy, w, h, angle]转换为4个顶点坐标
static void obb_to_vertices(const OBBResult& obb, axdl_point_t vertices[4]) {
    float cos_a = cos(obb.angle);
    float sin_a = sin(obb.angle);

    // 计算4个顶点（相对于中心点）
    float dx1 = obb.w * 0.5f * cos_a - obb.h * 0.5f * sin_a;
    float dy1 = obb.w * 0.5f * sin_a + obb.h * 0.5f * cos_a;
    float dx2 = obb.w * 0.5f * cos_a + obb.h * 0.5f * sin_a;
    float dy2 = obb.w * 0.5f * sin_a - obb.h * 0.5f * cos_a;

    // 4个顶点坐标（顺时针）
    vertices[0].x = obb.cx - dx1;  // 左上
    vertices[0].y = obb.cy - dy1;
    vertices[1].x = obb.cx + dx2;  // 右上
    vertices[1].y = obb.cy + dy2;
    vertices[2].x = obb.cx + dx1;  // 右下
    vertices[2].y = obb.cy + dy1;
    vertices[3].x = obb.cx - dx2;  // 左下
    vertices[3].y = obb.cy - dy2;
}

static void obb_decode_channel_first(const float* output, int num_classes, int num_anchors,
                      std::vector<OBBResult>& results, float conf_threshold) {
    results.clear();

    for (int i = 0; i < num_anchors; ++i) {
        // 读取OBB参数: [cx, cy, w, h, angle]
        float cx = output[0 * num_anchors + i];
        float cy = output[1 * num_anchors + i];
        float w = output[2 * num_anchors + i];
        float h = output[3 * num_anchors + i];
        float angle = output[4 * num_anchors + i];

        float max_score = 0;
        int max_class = 0;
        for (int j = 0; j < num_classes; ++j) {
            float score = output[(5 + j) * num_anchors + i];
            if (score > max_score) {
                max_score = score;
                max_class = j;
            }
        }

        if (max_score >= conf_threshold) {
            OBBResult result;
            result.cx = cx;
            result.cy = cy;
            result.w = w;
            result.h = h;
            result.angle = angle;
            result.score = max_score;
            result.label = max_class;
            results.push_back(result);
        }
    }
}

static float obb_iou(const OBBResult& a, const OBBResult& b) {
    // 计算两个OBB的轴对齐边界框用于快速IoU计算
    float a_x1 = a.cx - a.w * 0.5f;
    float a_y1 = a.cy - a.h * 0.5f;
    float a_x2 = a.cx + a.w * 0.5f;
    float a_y2 = a.cy + a.h * 0.5f;

    float b_x1 = b.cx - b.w * 0.5f;
    float b_y1 = b.cy - b.h * 0.5f;
    float b_x2 = b.cx + b.w * 0.5f;
    float b_y2 = b.cy + b.h * 0.5f;

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
    // YOLO11 OBB output format: [batch, channels, anchors] - [1, 20, 8400]
    int num_anchors = 0;
    int num_classes;  // 类别数
    bool channel_first = true;  // YOLO11 OBB使用channel_first格式

/*    printf("YOLO11 OBB output shape: [%d, %d, %d]\n",
           (int)pOutputsInfo[0].vShape[0],
           (int)pOutputsInfo[0].vShape[1],
           (int)pOutputsInfo[0].vShape[2]);
*/

    if (pOutputsInfo[0].vShape.size() == 3) {
        // YOLO11 OBB格式: [batch=1, channels=20, anchors=8400]
        num_anchors = (int)pOutputsInfo[0].vShape[2];  // 8400
        int channels = (int)pOutputsInfo[0].vShape[1];  // 20
        num_classes = channels - YOLO11_OBB_PARAMS;  // 20 - 5 = 15个类别

        // printf("YOLO11 OBB: detected %d anchors, %d classes (from %d channels)\n", num_anchors, num_classes, channels);
    } else {
        ALOGE("YOLO11 OBB requires 3D output [batch, channels, anchors]");
        return -1;
    }

    if (num_anchors <= 0 || num_classes <= 0) {
        ALOGE("Invalid YOLO11 OBB output shape");
        return -1;
    }

    // 解码OBB结果
    std::vector<OBBResult> obb_results;
    if (channel_first) {
        // YOLO11 OBB: [batch, channels, anchors] 格式，跳过batch维度
        const float* batch_data = output_ptr + pOutputsInfo[0].vShape[1] * pOutputsInfo[0].vShape[2] * 0; // batch=0
        obb_decode_channel_first(batch_data, num_classes, num_anchors, obb_results, PROB_THRESHOLD);
    } else {
        ALOGE("YOLO11 OBB expects channel_first format");
        return -1;
    }

    // printf("YOLO11 OBB: decoded %d candidates before NMS\n", (int)obb_results.size());

    // 应用NMS
    obb_nms(obb_results, NMS_THRESHOLD);

	// printf("YOLO11 OBB: %d objects after NMS\n", (int)obb_results.size());

    // 转换为 axdl_results_t 格式
    results->nObjSize = MIN(obb_results.size(), SAMPLE_MAX_BBOX_COUNT);
    for (int i = 0; i < results->nObjSize; i++) {
        const OBBResult& obb = obb_results[i];

        // 将OBB转换为顶点
        axdl_point_t vertices[4];
        obb_to_vertices(obb, vertices);

        // 添加调试信息
		/*
        printf("OBB %d: cx=%.3f, cy=%.3f, w=%.3f, h=%.3f, angle=%.3f, score=%.3f\n",
               i, obb.cx, obb.cy, obb.w, obb.h, obb.angle, obb.score);
        printf("Vertices: [(%.3f,%.3f), (%.3f,%.3f), (%.3f,%.3f), (%.3f,%.3f)]\n",
               vertices[0].x, vertices[0].y, vertices[1].x, vertices[1].y,
               vertices[2].x, vertices[2].y, vertices[3].x, vertices[3].y);
		*/
        // 计算轴对齐的边界框以确保兼容性
        float min_x = obb.cx - obb.w * 0.5f;
        float max_x = obb.cx + obb.w * 0.5f;
        float min_y = obb.cy - obb.h * 0.5f;
        float max_y = obb.cy + obb.h * 0.5f;

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
