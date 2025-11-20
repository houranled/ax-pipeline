#pragma once
#include "../include/ax_model_base.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"

class ax_model_custom : public ax_model_single_base_t
{
public:

    ax_model_custom() 
    {
        algo_width = get_algo_width();
        tan_xita =  (pixel_height / 2 - occlusion_pixel_height) * size_per_pixel / f;
        //0.577f; //暂时写死为30°的tan值
    }
protected:
    // 在这里添加自定义属性 

    int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;
    void process_texts(axdl_object_t& obj, int chn,  float fontscale) override;
    
    float pixel_height = 0; //传感器画面真实高度 单位像素
    float occlusion_pixel_height = 0; //画面遮挡部分的高度 单位像素        
    float origin_x; //原始x像素坐标 单位像素
    float f = 0.005;   //焦距 单位m  焦距5mm
    float X = 2.5f;    //扇叶左半边真实长度 单位m 
    float tan_xita;  //tan(画面上沿视线与摄像头主视线的垂直夹角) 即 原镜头中间水平线到画面上沿的距离长度÷焦距 是个比例值

    float amplitude = 0; //当前帧检测后计算出的振幅  单位m
    float size_per_pixel = 0.0000001; //像素大小 单位 m/像素
    float algo_width; 
    
};
REGISTER(MT_CUSTOM_MODEL, ax_model_custom)