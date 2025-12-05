#include "ax_model_custom.hpp"
#include "../../utilities/sample_log.h"

#include <fstream>
#include <iomanip>


void ax_model_custom::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    /*
     * 用户可以在这里，对后处理结果进行绘制，此函数可以使用opencv，绘制到 cv::Mat &image 中。
     * 注意：
     * 1、cv::Mat &image 是一个BGRA的四通道图像
     * 2、请不要对 cv::Mat &image 进行除了绘制图案之外的其他修改，如 resize、cvtColor 等
     */
    draw_bbox(image, results, fontscale, thickness, offset_x, offset_y);
}

void ax_model_custom::draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness)
{
    /*
     * 用户可以在这里，对后处理结果进行绘制。此处使用的是原生的api进行绘制，效率会比opencv快，但是具有一定的限制性
     * 注意：
     * 1、使用 m_drawers[chn] 的接口进行绘制，如果 m_drawers[chn] 不能满足绘制需求，则无法绘制
     * 2、详情可以参考 examples/libaxdl/include/ax_osd_drawer.hpp 定义的结构体
     */
    axdl_point_t pos = {origin_x, occlusion_pixel_height/m_drawers[chn].get_height()};
    m_drawers[chn].add_point(&pos, {255, 0, 0, 255}, 6);

    draw_bbox(chn, results, fontscale, thickness);
}

void ax_model_custom::process_texts(axdl_results_t *results, int &chn, int d, float fontscale)
{    
    /* 计算振幅
     * 根据x =fX/Z (透视原理. XYZ是监测视点在物理世界的坐标, xy是其在画面屏幕上的坐标)可以得出:  
     * 1、 x' = f*X/Z'  →  Z' = fX/x'       # 第0帧的景深 = 焦距 * 扇叶宽度 / 帧0识别框左上角横坐标
     * 2、 x'' = f*X/Z'' → Z'' = fX/x''      # 第n帧的景深 = 焦距 * 扇叶宽度 / 帧n识别框左上角横坐标
     * 已知x' x'' f  X  ,求出了 Z' 和 Z''两个景深.
     * 然后计算景深差 △Z = Z'' - Z'   # 景深差 = 第n帧的景深 - 第0帧的景深    
     * 需要提前知道摄像头上沿视线与摄像头主视线的垂直夹角,
     * 然后通过  tanθ= △Y /△Z    →   △Y = △Z * tan仰角得出△Y, 即振幅.
    */
   auto image_width = m_drawers[chn].get_width();   //用来反归一化，即像平面的像素宽度， 单位为像素
   auto &obj = results->mObjects[d];

   //tan(画面上沿视线与摄像头主视线的垂直夹角) 即 原镜头中间水平线到画面上沿的距离长度÷焦距 是个比例值
   auto tan_xita = (m_drawers[chn].get_height() - occlusion_pixel_height) /  m_drawers[chn].get_height(); 
   
   if (0.5f == obj.bbox.x || 0.5f == origin_x) {
       amplitude_now = 0;
   } else {
       amplitude_now/*振幅*/= f * X /(image_width * size_per_pixel) * (1/(0.5f - obj.bbox.x)  - 1/(0.5f - origin_x)) /*△Z*/
         * tan_xita /*tan仰角*/;
   }


   if (amplitude_now > 0 && amplitude_now > amp_max_positive) {
        amp_max_positive = amplitude_now;
        max_positive_point_pos = {obj.bbox.x, obj.bbox.y};
   } else if (amplitude_now < 0 && amplitude_now < amp_max_negative) {
        amp_max_negative =  amplitude_now;
        max_positive_point_pos = {obj.bbox.x, obj.bbox.y};
   }
   if (amp_max_positive)
        m_drawers[chn].add_point(&max_positive_point_pos, {0, 127, 255, 0}, 6);

   if (amp_max_negative)   
        m_drawers[chn].add_point(&max_negative_point_pos, {0, 127, 0, 255}, 6);

   amplitude_datas.push_back(amplitude_now);
   // 检查是否需要保存数据（每秒保存一次）
   auto current_time = std::chrono::steady_clock::now();
   if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_save_time).count() >= 1) {
        // 写入csv   
        //   save_amplitude_to_csv();
       
        // 清空数据
        amplitude_datas.clear();

        last_save_time = current_time;
   }
    
    m_drawers[chn].add_text(std::string(obj.objname) + ":" + std::to_string(amplitude_now),
        {obj.bbox.x, obj.bbox.y},
        {UCHAR_MAX, 0, 0, 0}, fontscale, 2);

    m_drawers[chn].add_text(std::string("positive max") + ":" + std::to_string(amp_max_positive),
        {0.3, 0},
        {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
    m_drawers[chn].add_text(std::string("negative max") + ":" + std::to_string(amp_max_negative),
        {0.5, 0},
        {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
}

void ax_model_custom::save_amplitude_to_csv() {
    std::ofstream csv_file;
    csv_file.open("amplitude_data.csv", std::ios::app);  // 以追加模式打开文件
    
    // 写入时间戳
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    csv_file << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << " ";
    
    // 写入振幅数据
    for (size_t i = 0; i < amplitude_datas.size(); ++i) {
        csv_file << amplitude_datas[i];
        if (i < amplitude_datas.size() - 1) {
            csv_file << ",";  // 不是最后一个数据时添加逗号
        } else {
            csv_file << std::endl;  // 最后一个数据时添加换行符
        }
    }    
    
    csv_file.close();
}

