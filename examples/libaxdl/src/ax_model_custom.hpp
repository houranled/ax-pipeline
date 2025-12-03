#pragma once
#include "../include/ax_model_base.hpp"
#include "ax_model_det.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"

class ax_model_custom : public ax_model_yolov8_native
{
public:

    ax_model_custom() 
    {
        //tan_xita =  (pixel_height / 2 - occlusion_pixel_height) * size_per_pixel / f;
        //0.577f; //暂时写死为30°的tan值
    }
protected:
    // 在这里添加自定义属性 

    //int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;
    void process_texts(axdl_results_t *results, int &chn, int d, float fontscale) override;
    void save_amplitude_to_csv();
    
    float pixel_height = 768; //传感器画面原始高度 单位像素
    float occlusion_pixel_height = 190; //画面遮挡部分的高度差190 单位像素
    float origin_x=0.333f; //原始x像素坐标归一化形式 范围为[0,1]
    float f = 0.005;   //焦距 单位m  焦距5mm
    float X = 2.5f;    //扇叶左半边真实长度 单位m 
    //float tan_xita=0;

    float amplitude = 0; //当前帧检测后计算出的振幅  单位m
    float size_per_pixel = 0.0000001; //像素大小 单位 m/像素

private:
    std::vector<float> amplitude_datas;  // 存储振幅数据
    std::chrono::steady_clock::time_point last_save_time;  // 上次保存时
    
};
REGISTER(MT_CUSTOM_MODEL, ax_model_custom)