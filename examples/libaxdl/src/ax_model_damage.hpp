#pragma once
#include "ax_model_det.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"
#include "../../alarm/alarm.hpp"
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include <thread>
#include <future>
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
        damage_type = "unknown";
    }

    // Override sub_init to extract damage type from filename (called after base init)
    int sub_init(void *json_obj) override;

    // 设置通道信息后预渲染中文水印
    void set_channel_init_info(const std::string name, const int id) override;

    // Get damage type (损伤类型，即模型文件名去后缀)
    std::string get_damage_type() const { return damage_type; }

protected:
    // 在这里添加自定义属性

    int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    //void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;

private:
    std::string damage_type; // 损伤类型（模型文件名，如"裂缝"、"腐蚀"等）

    // 缓存原图用于快照合成（在 post_process 中更新）
    cv::Mat m_cached_frame_bgr;
    std::mutex m_frame_mutex;

    // 缓存通道名称位图（FreeType 预渲染）
    cv::Mat m_channel_name_bmp;

};
REGISTER(MT_DAMAGE_MODEL, ax_model_damage)

class wt_damage_multi_model_recognize : public wt_model_multi_base_t {
public:
    wt_damage_multi_model_recognize();

    // Override inference to select models based on point name's position keyword
    int inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;

    // Override init to scan model directories by position
    int init(void *json_obj) override;

    void *GetRunnerHandle() override {return NULL;}

    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;

private:
    // 单个损伤模型信息
    struct DamageModelInfo {
        std::string damage_type_name;  // 损伤类型（模型文件名去后缀，如"裂缝"）
        std::shared_ptr<ax_model_base> model;
    };

    // 当前部位下所有损伤模型
    std::vector<DamageModelInfo> m_damage_models;

    // 模型根目录
    std::string m_model_root_dir;

    // 部署时指定的部位名称（对应子目录名）
    std::string m_position_name;

    // 扫描指定目录下的所有 .axmodel 文件并加载
    int scan_and_load_models(const std::string& position_dir);
};
REGISTER(WT_DAMAGE_MULTI_MODEL_RECOGNIZE, wt_damage_multi_model_recognize)