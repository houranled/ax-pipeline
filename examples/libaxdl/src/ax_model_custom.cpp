#include "ax_model_custom.hpp"
#include "../../utilities/sample_log.h"

#include <fstream>
#include <iomanip>
#include "../../utilities/json.hpp"

#include "../../camera/camera_controller.hpp"
#include <time.h>


// 初始化静态成员
std::string ax_model_custom::car_no = "";

int ax_model_custom::preprocess(axdl_image_t *srcFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    // 在图像上方绘制红色遮罩框
    if (srcFrame && srcFrame->pVir) {
        // 根据图像格式直接操作
        if (srcFrame->eDtype == axdl_color_space_nv12) {
            // NV12格式直接操作YUV分量
            int width = srcFrame->nWidth;
            int height = channel_amplitude_data.occlusion_pixel_height; // 遮罩高度

            // 直接操作Y分量，设置为暗色
            unsigned char* y_plane = (unsigned char*)srcFrame->pVir;
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    y_plane[y * width + x] = 76; // Y=76对应较暗的颜色
                }
            }

            // 操作UV分量，设置为红色
            unsigned char* uv_plane = (unsigned char*)srcFrame->pVir + width * srcFrame->nHeight;
            for (int y = 0; y < height / 2; y++) {
                for (int x = 0; x < width; x += 2) {
                    uv_plane[y * width + x] = 85; // U=85
                    uv_plane[y * width + x + 1] = 255; // V=255
                }
            }
        } else if (srcFrame->eDtype == axdl_color_space_bgr || srcFrame->eDtype == axdl_color_space_rgb) {
            // BGR或RGB格式，创建cv::Mat对象但不复制数据
            cv::Mat image(srcFrame->nHeight, srcFrame->nWidth, CV_8UC3, srcFrame->pVir);

            // 绘制红色遮罩框
            cv::Rect red_mask(0, 0, image.cols, channel_amplitude_data.occlusion_pixel_height);

            // 直接在image上绘制，避免创建ROI
            if (srcFrame->eDtype == axdl_color_space_rgb) {
                cv::rectangle(image, red_mask, cv::Scalar(255, 0, 0), -1); // RGB格式下的红色，填充矩形
            } else {
                cv::rectangle(image, red_mask, cv::Scalar(0, 0, 255), -1); // BGR格式下的红色，填充矩形
            }

            // 如果是RGB格式，转换回RGB
            if (srcFrame->eDtype == axdl_color_space_rgb) {
                cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
            }
        }
    }
    auto ret = this->ax_model_single_base_t::preprocess(srcFrame, crop_resize_box, results); // 保留原有逻辑
    return ret;
}

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

    //auto camera =  CameraController::getInstance()->getCamera(chn/2); // 获取相机

    auto& cad = channel_amplitude_data;

    axdl_point_t pos = {cad.origin_x_no_uniform/m_drawers[chn].get_width(), cad.occlusion_pixel_height/m_drawers[chn].get_height()};
    m_drawers[chn].add_point(&pos, {255, 0, 0, 255}, 6);  //添加初始位置坐标

    //视频左上角绘制车牌号水印 carNo 与时间戳
    if (car_no != "") {
        m_drawers[chn].add_text(car_no, {0, 0}, {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
    }
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);

    std::tm tm_now;
    localtime_r(&now_time, &tm_now);  // Linux平台

    std::ostringstream time_stream;
    time_stream << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");

    // 在车牌号下方显示时间戳
    m_drawers[chn].add_text(time_stream.str(), {0, 0.05}, {UCHAR_MAX, 0, 0, 0}, fontscale, 2);

    // 绘制遮罩(矩形)
    axdl_bbox_t mask = {0, 0, 1, cad.occlusion_pixel_height/m_drawers[chn].get_height()};
    m_drawers[chn].add_rect(&mask, {0, 0, 0, 255}, 1);

    draw_bbox(chn, results, fontscale, thickness);
}

void ax_model_custom::process_texts(axdl_results_t *results, int &chn, int d, float fontscale)
{
    /* 计算振幅△Y:
     * 根据x =fX/Z (透视原理. XYZ是监测视点在物理世界的坐标, xy是其在画面屏幕上的坐标)可以得出:
     * 1、 x' = f*X/Z'  →  Z' = fX/x'       # 第0帧的景深 = 焦距 * 扇叶宽度 / 帧0识别框左上角横坐标
     * 2、 x'' = f*X/Z'' → Z'' = fX/x''      # 第n帧的景深 = 焦距 * 扇叶宽度 / 帧n识别框左上角横坐标
     * 已知x' x'' f  X  ,求出了 Z' 和 Z''两个景深.
     * 然后计算△Z景深差： △Z = Z'' - Z'   # 景深差 = 第n帧的景深 - 第0帧的景深
     * 需要提前知道摄像头上沿视线与摄像头主视线的垂直夹角,
     * 然后通过  tanθ= △Y /△Z    →   △Y = △Z * tan仰角得出△Y, 即振幅.
    */
    auto image_width = m_drawers[chn].get_width();   //用来反归一化，即像平面的像素宽度， 单位为像素
    auto &obj = results->mObjects[d];

    auto& cad = channel_amplitude_data;  //cad: channel amplitude data

    //tan(画面上沿视线与摄像头主视线的垂直夹角) 即 镜头中间水平线到画面上沿的距离长度÷焦距 是个比例值
    auto tanAngle = (m_drawers[chn].get_height()/2.0f - cad.occlusion_pixel_height) /  (cad.f/cad.size_per_pixel);

    if (0.5f == obj.bbox.x || 0.5f == cad.origin_x_no_uniform/m_drawers[chn].get_width()) {
        cad.amplitude_now = 0;
    } else {
        cad.amplitude_now/*振幅*/= cad.f * cad.X /(image_width * cad.size_per_pixel) * (1/(0.5f - obj.bbox.x)
                - 1/(0.5f - cad.origin_x_no_uniform/m_drawers[chn].get_width())) /*△Z*/
                * tanAngle /*tan仰角*/;
    }

    auto distance_now = std::round((cad.amplitude_now+cad.Y) * 100000) / 100000; //保留小数点后5位并加上初始距离形成绝对距离

    if (cad.amplitude_now > 0 && cad.amplitude_now > cad.amp_max_positive) {
        cad.amp_max_positive = cad.amplitude_now;
        cad.max_positive_point_pos = {obj.bbox.x, obj.bbox.y};
    } else if (cad.amplitude_now < 0 && cad.amplitude_now < cad.amp_max_negative) {
        cad.amp_max_negative =  cad.amplitude_now;
        cad.max_positive_point_pos = {obj.bbox.x, obj.bbox.y};
    }
    if (cad.amp_max_positive)
        m_drawers[chn].add_point(&cad.max_positive_point_pos, {0, 127, 255, 0}, 6);

    if (cad.amp_max_negative)
        m_drawers[chn].add_point(&cad.max_negative_point_pos, {0, 127, 0, 255}, 6);

    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);

    // 检查是否需要采样（100ms = 10Hz）
    if (cad.last_sample_time.time_since_epoch().count() == 0) {
        cad.last_sample_time = now;
    }
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - cad.last_sample_time).count(); // 计算时间差
    if (elapsed_ms >= 90) { // 10Hz采样
        cad.amplitude_datas.push_back(distance_now); // 将距离数据存储到数组中
        cad.last_sample_time = now; // 更新最后采样时间
    }

    float actual_fontscale = 1;
    m_drawers[chn].add_text(std::string(obj.objname) + ":" + std::to_string(distance_now),
        {obj.bbox.x, obj.bbox.y},
        {UCHAR_MAX, 0, 0, 0}, fontscale, 2);

    m_drawers[chn].add_text(std::string("positive max") + ":" + std::to_string(cad.amp_max_positive),
        {0.3, 0},
        {UCHAR_MAX, 0, 0, 0}, actual_fontscale, 2);
    m_drawers[chn].add_text(std::string("negative max") + ":" + std::to_string(cad.amp_max_negative),
        {0.5, 0},
        {UCHAR_MAX, 0, 0, 0}, actual_fontscale, 2);
}

void ax_model_custom::export_amplitude()
{
    while (export_thread_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒输出一次

        // 写入时间戳
        auto now = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::to_time_t(now);

        std::tm tm_now;
        localtime_r(&now_time, &tm_now);  // Linux平台

        std::ostringstream time_stream;
        time_stream << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");

        // 收集数据并输出JSON
        // 从channel_amplitude_map获取数据

        auto& datas = channel_amplitude_data.amplitude_datas;
        nlohmann::json output_json; // 创建一个空的JSON对象
        if (!channel_amplitude_data.amplitude_datas.empty()) {
            std::vector<std::string> formatted_amplitudes;
            for (float amplitude : datas) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(5) << amplitude;
                formatted_amplitudes.push_back(oss.str());
            }
            output_json[channel_name] = formatted_amplitudes;
            output_json["time"] = time_stream.str(); // 添加时间戳
            datas.clear();
        }

        if (!output_json.empty()) {
            std::cout << output_json.dump() << std::endl;
        }
    }

}

void ax_model_custom::load_config()
{
    try {
        // 打开配置文件
        std::ifstream config_file("/wt_tech/conf/rt.json");
        if (!config_file.is_open()) {
            WTALOGI("Failed to open config file: /wt_tech/conf/rt.json");
            return;
        }

        // 解析 JSON
        nlohmann::json config;
        config_file >> config;

        // 获取carNo字段
        if (config.contains("carNo")) {
            car_no = config["carNo"].get<std::string>();
            WTALOGI("Loaded carNo from config: %s", car_no.c_str());
        } else {
            WTALOGI("carNo not found in config file");
        }

        if (config.contains("chl_list")) {
            for (const auto& one_chl_config : config["chl_list"]) {
                if (one_chl_config["type"] != "Webcam")
                    continue;

                auto chal_name = one_chl_config.value("name", "");
                if (chal_name != channel_name) //只读取属于自身通道的配置
                    continue;

                if (one_chl_config.contains("calibration")) {
                    std::string Y = one_chl_config.value("calibration", "");
                    channel_amplitude_data.Y = std::stof(Y); // 将字符串转换为浮点数
                }

                if (one_chl_config.contains("point")) {
                    auto point = one_chl_config["point"];
                    channel_amplitude_data.origin_x_no_uniform = point.value("x", 0.0f);

                    channel_amplitude_data.occlusion_pixel_height = point.value("y", 0.0f); // 这个能通过标定来生成
                }

                if (one_chl_config.contains("width")) {
                    channel_amplitude_data.X = std::stof(one_chl_config["width"].get<std::string>()) / 2;
                }
            }
        }

    } catch (const std::exception& e) {
        WTALOGI("Error loading config: %s", e.what());
    }
}

void ax_model_custom::set_channel_name_init(const std::string name)
{
    ax_model_base::set_channel_name_init(name);
    load_config(); // 加载配置

    if (channel_amplitude_data.occlusion_pixel_height == 0) {
        // tc通道的occlusion_pixel_heigh默认值置为390; 其它通道的设置为190
        if (channel_name == "tc") {
            channel_amplitude_data.occlusion_pixel_height = 390.0f;
        } else {
            channel_amplitude_data.occlusion_pixel_height = 190.0f;
        }
    }
}
