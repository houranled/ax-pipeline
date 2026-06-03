#pragma once
#include "ax_model_det.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"
#include "../../alarm/alarm.hpp"
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include <mutex>
#include "base/detection.hpp"
#include "../include/ax_model_base.hpp"

struct ScaleOutputs
{
    int stride;
    int H, W;
    int box_ch, cls_ch, ang_ch;
    const float* box_ptr;
    const float* cls_ptr;
    const float* ang_ptr;
};

class ax_model_damage : public ax_model_yolov8_native
{
public:

    ax_model_damage()
    {
        WTALOGI("实例化一个ax_model_damage对象");
        model_type = "default";
    }

    // Override sub_init to extract model type from filename (called after base init)
    int sub_init(void *json_obj) override;

    // Get model type
    std::string get_model_type() const { return model_type; }

protected:
    // 在这里添加自定义属性

    int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    //void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;

private:
    //AlarmGenerator m_alarm_generator; //告警检测器
    std::string model_type; // 模型类型（A、B、C等）

    // 缓存原图用于快照合成（在 post_process 中更新）
    cv::Mat m_cached_frame_bgr;
    std::mutex m_frame_mutex;

    // Helper method to extract type from filename
    std::string extract_type_from_filename(const std::string& model_path);

};
REGISTER(MT_DAMAGE_MODEL, ax_model_damage)

class wt_damage_multi_model_recognize : public wt_model_multi_base_t {
public:
    wt_damage_multi_model_recognize();

    // Add model type to point prefix mapping
    int add_model_type_mapping(const std::string& point_prefix, const std::string& model_type);

    // Override inference to select models based on point name prefix type
    int inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;

    // Override init to parse model type mappings
    int init(void *json_obj) override;

    void *GetRunnerHandle() override {return NULL;}

private:
    // Structure to hold model type information
    struct ModelTypeInfo {
        std::string type_name;
        std::vector<std::shared_ptr<ax_model_base>> models;
    };

    // Map from model type to model info
    std::map<std::string, ModelTypeInfo> m_model_types;

    // Map from point name prefix to model type
    std::map<std::string, std::string> m_point_prefix_to_model_type;

    // Helper method to get current point name prefix
    std::string get_current_point_prefix();

    // Helper method to find matching model type for point prefix
    std::string find_model_type_for_point(const std::string& point_name);

    // Helper method to get all models of a specific type
    std::vector<std::shared_ptr<ax_model_base>> get_models_by_type(const std::string& model_type);

    };
REGISTER(WT_DAMAGE_MULTI_MODEL_RECOGNIZE, wt_damage_multi_model_recognize)