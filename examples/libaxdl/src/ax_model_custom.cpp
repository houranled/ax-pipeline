#include "ax_model_custom.hpp"
#include "../../utilities/sample_log.h"

#include <fstream>
#include <iomanip>
#include "../../utilities/json.hpp"

#include "../../camera/camera_controller.hpp"
#include <time.h>


// 初始化静态成员
std::mutex ax_model_custom::amplitude_mutex;
std::thread ax_model_custom::export_thread;
std::atomic<bool> ax_model_custom::export_thread_running(false);
std::map<std::string, ax_model_custom::ChannelAmplitudeData> ax_model_custom::channel_amplitude_map;

std::string ax_model_custom::car_no = "";
float ax_model_custom::ChannelAmplitudeData::Y = 0;

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

    auto camera =  CameraController::getInstance()->getCamera(chn/2); // 获取相机
    auto name = camera->getName();

    auto& cad = channel_amplitude_map[name];

    axdl_point_t pos = {cad.origin_x, cad.occlusion_pixel_height/m_drawers[chn].get_height()};
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

    auto name = CameraController::getInstance()->getCamera(chn/2)->getName();
    auto& cad = channel_amplitude_map[name];

    //tan(画面上沿视线与摄像头主视线的垂直夹角) 即 原镜头中间水平线到画面上沿的距离长度÷焦距 是个比例值
    auto tan_xita = (m_drawers[chn].get_height() - cad.occlusion_pixel_height) /  m_drawers[chn].get_height();

    if (0.5f == obj.bbox.x || 0.5f == cad.origin_x) {
        cad.amplitude_now = 0;
    } else {
        cad.amplitude_now/*振幅*/= cad.f * cad.X /(image_width * cad.size_per_pixel) * (1/(0.5f - obj.bbox.x)  - 1/(0.5f - cad.origin_x)) /*△Z*/
         * tan_xita /*tan仰角*/;
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

    cad.amplitude_datas.push_back(distance_now); // 将距离数据存储到数组中

    m_drawers[chn].add_text(std::string(obj.objname) + ":" + std::to_string(distance_now),
        {obj.bbox.x, obj.bbox.y},
        {UCHAR_MAX, 0, 0, 0}, fontscale, 2);

    m_drawers[chn].add_text(std::string("positive max") + ":" + std::to_string(cad.amp_max_positive),
        {0.3, 0},
        {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
    m_drawers[chn].add_text(std::string("negative max") + ":" + std::to_string(cad.amp_max_negative),
        {0.5, 0},
        {UCHAR_MAX, 0, 0, 0}, fontscale, 2);
}

void ax_model_custom::export_amplitude()
{
    nlohmann::json output_json;

    while (export_thread_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒输出一次

        // 写入时间戳
        auto now = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::to_time_t(now);

        std::tm tm_now;
        localtime_r(&now_time, &tm_now);  // Linux平台

        std::ostringstream time_stream;
        time_stream << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");

        std::lock_guard<std::mutex> lock(amplitude_mutex);
        // 收集数据并输出JSON
        // 从channel_amplitude_map获取数据
        for (auto& [name, data] : channel_amplitude_map) {
            if (!data.amplitude_datas.empty()) {
                std::vector<std::string> formatted_amplitudes;
                for (float amplitude : data.amplitude_datas) {
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(5) << amplitude;
                    formatted_amplitudes.push_back(oss.str());
                }
                output_json[name] = formatted_amplitudes;
                data.amplitude_datas.clear();
            }
        }

        output_json["time"] = time_stream.str(); // 添加时间戳

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
            ALOGE("Failed to open config file: /wt_tech/conf/rt.json");
            return;
        }

        // 解析 JSON
        nlohmann::json config;
        config_file >> config;

        // 获取carNo字段
        if (config.contains("carNo")) {
            car_no = config["carNo"].get<std::string>();
            ALOGI("Loaded carNo from config: %s", car_no.c_str());
        } else {
            ALOGW("carNo not found in config file");
        }

        if (config.contains("calibration")) {
            ax_model_custom::ChannelAmplitudeData::Y =  std::stof(config["calibration"].get<std::string>());
        }
    } catch (const std::exception& e) {
        ALOGE("Error loading config: %s", e.what());
    }
}
