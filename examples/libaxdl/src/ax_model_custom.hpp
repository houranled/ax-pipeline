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
    }
protected:
    // 在这里添加自定义属性 

    //int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;
    void process_texts(axdl_results_t *results, int &chn, int d, float fontscale) override;
    void save_amplitude_to_csv();
    

    float occlusion_pixel_height = 190; //画面遮挡部分的高度差190 单位像素
    float origin_x=0.133f; //原始x像素坐标归一化形式 范围为[0,1]
    float f = 0.0028;   //焦距 单位m  焦距28mm
    float X = 0.25f;    //扇叶左半边真实长度 单位m
    float size_per_pixel = 0.0000001f; //摄像传感器像素大小 单位 m/像素

    float amplitude_now = 0; //当前帧检测后计算出的振幅  单位m
    float amp_max_positive =0; //振幅正向最大值
    float amp_max_negative =0; //振幅负向最大值

    axdl_point_t max_positive_point_pos;
    axdl_point_t max_negative_point_pos;

private:
    std::vector<float> amplitude_datas;  // 存储振幅数据
    std::chrono::steady_clock::time_point last_save_time;  // 上次保存时
    
};
REGISTER(MT_CUSTOM_MODEL, ax_model_custom)