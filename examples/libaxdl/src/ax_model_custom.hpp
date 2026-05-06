#pragma once
#include "../include/ax_model_base.hpp"
#include "ax_model_det.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"

#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <sstream>

class ax_model_custom : public ax_model_yolov8_native
{
public:

    ax_model_custom()
    {
        // 为每个实例创建自己的线程
        export_thread = std::thread([this]() { this->export_amplitude(); });
    }

    ~ax_model_custom()
    {
        // 每个实例销毁时停止自己的线程
        if (export_thread.joinable()) {
            export_thread.join();
        }
    }

protected:
    // 在这里添加自定义属性

    int preprocess(axdl_image_t *srcFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override; // 推理前
    //int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override; // 推理后归一化前
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;
    void process_texts(axdl_results_t *results, int &chn, int d, float fontscale) override;
    void export_amplitude();
    void load_config();
    void set_channel_init_info(std::string name, const int id) override;

private:
    // 通道相关数据
    struct ChannelAmplitudeData {
        float origin_x_no_uniform = 167; //原始x像素坐标未归一化 单位像素
        float X = 2.3f;    //扇叶左半边真实长度 单位m
        float Y = 0.8f;       //扇叶于下方平板标定距离 单位m

        float occlusion_pixel_height = 0; //画面遮挡部分的高度差
        float f = 0.0028;   //焦距 单位m  焦距28mm
        float size_per_pixel = 0.0000001f; //摄像传感器像素大小 单位 m/像素

        std::vector<float> amplitude_datas;
        std::chrono::steady_clock::time_point last_save_time;
        float amplitude_now = 0;
        float amp_max_positive = 0;
        float amp_max_negative = 0;
        axdl_point_t max_positive_point_pos;
        axdl_point_t max_negative_point_pos;

        std::chrono::system_clock::time_point last_sample_time; // 最后一次采样时间
    } channel_amplitude_data;

    std::thread export_thread;

    // 添加静态成员变量存储carNo
    static std::string car_no;

    // ========== 人脸检测录像相关成员 ==========
    struct FaceRecordData {
        bool is_recording = false;           // 是否正在录制
        std::chrono::system_clock::time_point last_face_detect_time;  // 最后一次检测到人脸的时间
        std::chrono::system_clock::time_point record_start_time;      // 录制开始时间
        int face_detect_count = 0;           // 当前录制周期内检测到人脸的帧数
    } face_record_data;

    // 录制参数配置
    static constexpr int FACE_DETECT_THRESHOLD = 3;           // 连续检测到3帧人脸才开始录制（防抖）
    static constexpr int RECORD_MAX_DURATION_SECONDS = 60;      // 最大录制时长60秒
    static constexpr int RECORD_POST_FACE_SECONDS = 5;        // 人脸消失后继续录制5秒
    static constexpr int RECORD_MIN_DURATION_SECONDS = 10;    // 最小录制时长10秒

    void check_and_trigger_face_recording(axdl_results_t *results);
    void stop_face_recording();
    std::string generate_face_record_filename();
    void trigger_camera_record(bool start);

};
REGISTER(MT_CUSTOM_MODEL, ax_model_custom)