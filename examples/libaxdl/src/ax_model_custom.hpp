#pragma once
#include "../include/ax_model_base.hpp"
#include "ax_model_det.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"

#include <atomic>
#include <thread>
#include <mutex>

class ax_model_custom : public ax_model_yolov8_native
{
public:

    ax_model_custom() {
        //加载配置
        load_config();

        if (!export_thread_running.exchange(true)) {
            export_thread = std::thread(export_amplitude);
        }
    }

    ~ax_model_custom() {
        // 只有最后一个实例销毁时才停止线程
        if (export_thread_running.exchange(false)) {
            if (export_thread.joinable()) {
                export_thread.join();
            }
        }
    }

protected:
    // 在这里添加自定义属性

    int preprocess(axdl_image_t *srcFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override; // 推理前
    //int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override; // 推理后归一化前
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;
    void process_texts(axdl_results_t *results, int &chn, int d, float fontscale) override;
    static void export_amplitude();
    void load_config();

private:

    // 为每个通道创建独立的振幅相关数据存储
    struct ChannelAmplitudeData {
        float origin_x_no_uniform = 167; //原始x像素坐标未归一化 单位像素
        float origin_x=0.133f; //原始x像素坐标归一化形式 范围为[0,1]
        float X = 2.3f;    //扇叶左半边真实长度 单位m
        float Y = 0;       //扇叶于下方平板标定距离 单位m
        float occlusion_pixel_height = 190; //画面遮挡部分的高度差190 单位像素
        float f = 0.0028;   //焦距 单位m  焦距28mm
        float size_per_pixel = 0.0000001f; //摄像传感器像素大小 单位 m/像素

        std::vector<float> amplitude_datas;
        std::chrono::steady_clock::time_point last_save_time;
        float amplitude_now = 0;
        float amp_max_positive = 0;
        float amp_max_negative = 0;
        axdl_point_t max_positive_point_pos;
        axdl_point_t max_negative_point_pos;
    };

    static std::map<std::string, ChannelAmplitudeData> channel_amplitude_map;  // 通道name到振幅数据的映射

    static std::mutex amplitude_mutex;
    static std::thread export_thread;
    static std::atomic<bool> export_thread_running;

    // 添加静态成员变量存储carNo
    static std::string car_no;


};
REGISTER(MT_CUSTOM_MODEL, ax_model_custom)