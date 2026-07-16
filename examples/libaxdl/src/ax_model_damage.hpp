#pragma once
#include "ax_model_det.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"
#include "../../alarm/alarm.hpp"
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <set>
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

    // 刷新原图缓存（供拍照与巡检结束后 diff 比对复用）
    void cache_source_frame(axdl_image_t *pstFrame);

protected:
    // 在这里添加自定义属性
    int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    //void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;

    // 在 BGR 图上绘制水印（时间、通道名、点位），用于无差异存无框原图时保留水印
    void draw_watermark_bgr(cv::Mat &bgr, int cur_point, bool is_moving);

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

    // 当前部位下所有通用损伤模型（big=outside/ small=inside 目录，每个点位都跑）
    std::vector<DamageModelInfo> m_damage_models;

    // 专用模型缓存：文件名(含扩展名) -> 模型信息。
    // init 时从本相机各点位 rt.json models[] 的并集预加载，位于 <position_dir>/specialized。
    std::map<std::string, DamageModelInfo> m_specialized_models;

    // 点位id -> 该点位专用模型指针列表（指向 m_specialized_models 中的条目，map 节点地址稳定）。
    // init 时预计算，inference 时直接查表，避免每帧线性扫描点位+字符串规范化，
    // 且不再在推理热路径读取可能被热加载改写的 preset_positions（消除潜在数据竞争）。
    std::map<int, std::vector<const DamageModelInfo*>> m_point_specialized;

    // 模型根目录
    std::string m_model_root_dir;

    // 部署时指定的部位名称（对应子目录名）
    std::string m_position_name;

    // 当前部位目录（m_model_root_dir/m_position_name），specialized 子目录在其下
    std::string m_position_dir;

    // 扫描指定目录下的所有 .axmodel 文件并加载（通用模型）
    int scan_and_load_models(const std::string& position_dir);

    // 加载单个 .axmodel：dir 为所在目录（用于查找同名 .json 配置），fname 为文件名(含.axmodel)。
    // 成功返回 model 非空的 DamageModelInfo，失败返回的 info.model 为 nullptr。
    DamageModelInfo load_model_file(const std::string& dir, const std::string& fname);

    // 预加载本相机所有点位 models[] 引用的专用模型（位于 <position_dir>/specialized）
    int load_specialized_models();
};
REGISTER(WT_DAMAGE_MULTI_MODEL_RECOGNIZE, wt_damage_multi_model_recognize)