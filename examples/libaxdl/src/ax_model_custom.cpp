#include "ax_model_custom.hpp"
#include "../../utilities/sample_log.h"

#include <fstream>
#include <iomanip>
#include "../../utilities/json.hpp"

#include "../../camera/camera_controller.hpp"
#include <time.h>
#include <sys/stat.h>
#include "ax_model_base.hpp"
#include "ax_freetype_overlay.hpp"


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
            auto real_pixel = srcFrame->nHeight - 140*2; //上下有各140像素的黑色填充:letterbox
            cv::Rect red_mask(0, 0, image.cols, channel_amplitude_data.occlusion_pixel_height*real_pixel/1080 + 140); // 遮罩高度

            // 直接在image上绘制，避免创建ROI
            if (srcFrame->eDtype == axdl_color_space_rgb) {
                cv::rectangle(image, red_mask, cv::Scalar(255, 0, 0), -1); // RGB格式下的红色，填充矩形
            } else {
                cv::rectangle(image, red_mask, cv::Scalar(0, 0, 255), -1); // BGR格式下的红色，填充矩形
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

     /* 计算振幅△Y:
     * 根据x =fX/Z (透视原理. XYZ是监测视点在物理世界的坐标, xy是其在画面屏幕上的坐标)可以得出:
     * 1、 x' = f*X/Z'  →  Z' = fX/x'       # 第0帧的景深 = 焦距 * 扇叶宽度 / 帧0识别框左上角横坐标
     * 2、 x'' = f*X/Z'' → Z'' = fX/x''      # 第n帧的景深 = 焦距 * 扇叶宽度 / 帧n识别框左上角横坐标
     * 已知x' x'' f  X  ,求出了 Z' 和 Z''两个景深.
     * 然后计算△Z景深差： △Z = Z'' - Z'   # 景深差 = 第n帧的景深 - 第0帧的景深
     * 需要提前知道摄像头上沿视线与摄像头主视线的垂直夹角,
     * 然后通过  tanθ= △Y /△Z    →   △Y = △Z * tan仰角得出△Y, 即振幅.
    */

    // 检查人脸检测并触发录像
    check_and_trigger_face_recording(results);

     // 获取图像尺寸
    int image_width = image.cols;
    int image_height = image.rows;

    auto now = std::chrono::system_clock::now();
    auto &obj = results->mObjects[0];
    auto& cad = channel_amplitude_data;  //cad: channel amplitude data
    if (results->nObjSize > 0 && strcmp(obj.objname, "fan") == 0) {

        //tan(画面上沿视线与摄像头主视线的垂直夹角) 即 镜头中间水平线到画面上沿的距离长度÷焦距 是个比例值
        auto tanAngle = (image_height/2.0f - cad.occlusion_pixel_height) / (cad.f/cad.size_per_pixel);

        if (0.5f == obj.bbox.x || 0.5f == cad.origin_x_no_uniform/image_width) {
            cad.amplitude_now = 0;
        } else {
            cad.amplitude_now/*振幅*/= cad.f * cad.X /(image_width * cad.size_per_pixel) *
                (1/(0.5f - obj.bbox.x) - 1/(0.5f - cad.origin_x_no_uniform/image_width)) /*△Z*/
                * tanAngle /*tan仰角*/;
        }

        cad.amplitude_now = std::round(cad.amplitude_now * 100000) / 100000; // 保留小数点后5位并加上初始距离形成绝对距离
        auto distance_now = cad.amplitude_now + cad.Y; // 距离 = 振幅 + 初始距离

        if (cad.amplitude_now > 0 && cad.amplitude_now > cad.amp_max_positive) {
            cad.amp_max_positive = cad.amplitude_now;
            cad.max_positive_point_pos = {obj.bbox.x, obj.bbox.y};
        } else if (cad.amplitude_now < 0 && cad.amplitude_now < cad.amp_max_negative) {
            cad.amp_max_negative = cad.amplitude_now;
            cad.max_negative_point_pos = {obj.bbox.x, obj.bbox.y};
        }


        // 时间采样逻辑
        if (cad.last_sample_time.time_since_epoch().count() == 0) {
            cad.last_sample_time = now;
        }

        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - cad.last_sample_time).count();
        if (elapsed_ms >= 90) { // 10Hz采样
            cad.amplitude_datas.push_back(distance_now);
            cad.last_sample_time = now;
        }

        // 使用OpenCV绘制目标点内容
        cv::Point obj_pt(static_cast<int>(obj.bbox.x * image_width),
                        static_cast<int>(obj.bbox.y * image_height + 3));
        std::string obj_text = std::to_string(distance_now);
        cv::circle(image, obj_pt, 2, cv::Scalar(255, 0, 0, 0), 2);
        cv::putText(image, obj_text, obj_pt, cv::FONT_HERSHEY_SIMPLEX, fontscale,
                   cv::Scalar(255, 255, 255, UCHAR_MAX), 2);
    }

    cv::Point pos_text_pt(static_cast<int>(0.3 * image_width), 20);
    std::string pos_text = "positive max:" + std::to_string(cad.amp_max_positive);
    cv::putText(image, pos_text, pos_text_pt, cv::FONT_HERSHEY_SIMPLEX, 1.0,
               cv::Scalar(255, 0, 0, UCHAR_MAX), 2);

    cv::Point neg_text_pt(static_cast<int>(0.5 * image_width), 20);
    std::string neg_text = "negative max:" + std::to_string(cad.amp_max_negative);
    cv::putText(image, neg_text, neg_text_pt, cv::FONT_HERSHEY_SIMPLEX, 1.0,
               cv::Scalar(255, 0, 0, UCHAR_MAX), 2);


    // 添加初始位置坐标 (橙色点)
    int point_x = static_cast<int>(cad.origin_x_no_uniform);
    int point_y = static_cast<int>(cad.occlusion_pixel_height);
    cv::circle(image, cv::Point(point_x, point_y), 2, cv::Scalar(255, 0, 255, 255), -1);

    // 视频左上角绘制车牌号 + 通道名水印（启动时已用 FreeType 预渲染）
    int text_y = 30;  // 起始Y位置
    if (!m_static_label_bmp.empty()) {
        int w = std::min(m_static_label_bmp.cols, image.cols - 10);
        int h = std::min(m_static_label_bmp.rows, image.rows - 10);
        if (w > 0 && h > 0) {
            cv::Mat roi = image(cv::Rect(10, 10, w, h));
            // alpha 合成（image 是 BGRA）
            for (int y = 0; y < h; ++y) {
                const cv::Vec4b* src = m_static_label_bmp.ptr<cv::Vec4b>(y);
                cv::Vec4b* dst = roi.ptr<cv::Vec4b>(y);
                for (int x = 0; x < w; ++x) {
                    uint8_t a = src[x][3];
                    if (!a) continue;
                    float fa = a / 255.f;
                    for (int c = 0; c < 3; ++c)
                        dst[x][c] = cv::saturate_cast<uint8_t>(src[x][c] * fa + dst[x][c] * (1 - fa));
                    dst[x][3] = std::max(dst[x][3], a);
                }
            }
            text_y = 10 + h + 10;
        }
    }

    // 获取当前时间
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&now_time, &tm_now);

    std::ostringstream time_stream;
    time_stream << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");

    // 在车牌号下方显示时间戳
    cv::putText(image, time_stream.str(), cv::Point(10, text_y+20),
               cv::FONT_HERSHEY_SIMPLEX, fontscale * 2, cv::Scalar(255, 255, 255, 255), thickness * 2);

    // 绘制遮罩框
    int mask_height = static_cast<int>(cad.occlusion_pixel_height);
    cv::Rect mask_rect(0, 0, image_width, mask_height);
    cv::rectangle(image, mask_rect, cv::Scalar(255, 0, 0, 255), 1);

    wt_amp_draw_face_bbox(image, results, fontscale, thickness, offset_x, offset_y);
}

void ax_model_custom::wt_amp_draw_face_bbox(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{
    int x, y;
    cv::Size label_size;
    int baseLine = 0;

    for (int i = 0; i < results->nObjSize; i++)
    {
        if (strcmp(results->mObjects[i].objname, "face") != 0)
            continue;

        cv::Rect rect(results->mObjects[i].bbox.x * image.cols + offset_x,
                      results->mObjects[i].bbox.y * image.rows + offset_y,
                      results->mObjects[i].bbox.w * image.cols,
                      results->mObjects[i].bbox.h * image.rows);
        std::string label_str = results->mObjects[i].objname;
        if (b_track)
        {
            label_str += " " + std::to_string(results->mObjects[i].track_id);
        }

        label_size = cv::getTextSize(label_str, cv::FONT_HERSHEY_SIMPLEX, fontscale, thickness, &baseLine);
        if (results->mObjects[i].bHasBoxVertices)
        {
            cv::line(image,
                     cv::Point(results->mObjects[i].bbox_vertices[0].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[0].y * image.rows + offset_y),
                     cv::Point(results->mObjects[i].bbox_vertices[1].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[1].y * image.rows + offset_y),
                     cv::Scalar(128, 0, 0, 255), thickness * 2, 8, 0);
            cv::line(image,
                     cv::Point(results->mObjects[i].bbox_vertices[1].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[1].y * image.rows + offset_y),
                     cv::Point(results->mObjects[i].bbox_vertices[2].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[2].y * image.rows + offset_y),
                     cv::Scalar(128, 0, 0, 255), thickness * 2, 8, 0);
            cv::line(image,
                     cv::Point(results->mObjects[i].bbox_vertices[2].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[2].y * image.rows + offset_y),
                     cv::Point(results->mObjects[i].bbox_vertices[3].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[3].y * image.rows + offset_y),
                     cv::Scalar(128, 0, 0, 255), thickness * 2, 8, 0);
            cv::line(image,
                     cv::Point(results->mObjects[i].bbox_vertices[3].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[3].y * image.rows + offset_y),
                     cv::Point(results->mObjects[i].bbox_vertices[0].x * image.cols + offset_x, results->mObjects[i].bbox_vertices[0].y * image.rows + offset_y),
                     cv::Scalar(128, 0, 0, 255), thickness * 2, 8, 0);

            x = results->mObjects[i].bbox_vertices[0].x * image.cols + offset_x;
            y = results->mObjects[i].bbox_vertices[0].y * image.rows + offset_y - label_size.height - baseLine;
        }
        else
        {
            cv::rectangle(image, rect, COCO_COLORS[results->mObjects[i].label % COCO_COLORS.size()], thickness);
            x = rect.x;
            y = rect.y - label_size.height - baseLine;
        }

        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255, 255), -1);

        cv::putText(image, label_str, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, fontscale,
                    cv::Scalar(0, 0, 0, 255), thickness);
    }
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

    // 视频左上角绘制车牌号 + 通道名水印（启动时已用 FreeType 预渲染）
    if (!m_static_label_bmp.empty()) {
        m_drawers[chn].add_bitmap({0, 0}, m_static_label_bmp);
    }
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);

    std::tm tm_now;
    localtime_r(&now_time, &tm_now);  // Linux平台

    std::ostringstream time_stream;
    time_stream << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");

    // 在车牌号下方显示时间戳（位置根据预渲染位图高度动态计算，避免重叠）
    float ts_y = m_static_label_bmp.empty() ? 0.0f
                                            : float(m_static_label_bmp.rows) / m_drawers[chn].get_height();
    m_drawers[chn].add_text(time_stream.str(), {0, ts_y}, {UCHAR_MAX, 0, 0, 0}, fontscale, 2);

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

    cad.amplitude_now = std::round(cad.amplitude_now * 100000) / 100000; // 保留小数点后5位并加上初始距离形成绝对距离
    auto distance_now = cad.amplitude_now + cad.Y; // 距离 = 振幅 + 初始距离

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
    while (true) {
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

void ax_model_custom::set_channel_init_info(const std::string name, const int id)
{
    ax_model_base::set_channel_init_info(name, id);
    load_config(); // 加载配置

    if (channel_amplitude_data.occlusion_pixel_height == 0) {
        // tc通道的occlusion_pixel_heigh默认值置为390; 其它通道的设置为190
        if (channel_name == "tc") {
            channel_amplitude_data.occlusion_pixel_height = 390.0f;
        } else {
            channel_amplitude_data.occlusion_pixel_height = 190.0f;
        }
    }

    // 启动时一次性 FreeType 渲染中文水印（车牌号 + 通道名）
    build_static_label();
}

void ax_model_custom::build_static_label()
{
    auto& ft = FreeTypeOverlay::instance();
    if (!ft.ready()) {
        // 默认字体路径，板上提供
        ft.init("/wt_tech/conf/simsun.ttc", 20);
    }
    if (!ft.ready()) {
        WTALOGI("[FreeType] not ready, static label skipped (chn=%s)", channel_name.c_str());
        m_static_label_bmp.release();
        return;
    }

    std::string text;
    if (!car_no.empty())    text += car_no;
    if (!channel_name.empty()) {
        if (!text.empty()) text += "  ";
        text += channel_name;
    }
    if (text.empty()) {
        m_static_label_bmp.release();
        return;
    }

    // 白色文字，透明背景
    m_static_label_bmp = ft.renderTextRGBA(text, cv::Scalar(255, 255, 255, 255), 2);
    if (m_static_label_bmp.empty()) {
        WTALOGI("[FreeType] renderTextRGBA empty, text='%s'", text.c_str());
    } else {
        WTALOGI("[FreeType] static label ready: chn=%s text='%s' size=%dx%d",
                channel_name.c_str(), text.c_str(),
                m_static_label_bmp.cols, m_static_label_bmp.rows);
    }
}

void ax_model_custom::check_and_trigger_face_recording(axdl_results_t *results)
{
    bool face_detected = false;

    // 检查检测结果中是否包含人脸(face类别)
    for (int i = 0; i < results->nObjSize; i++) {
        auto &obj = results->mObjects[i];
        // person 通过 objname 判断
        if (std::string(obj.objname).find("face") != std::string::npos) {
            face_detected = true;
            break;
        }
    }

    auto now = std::chrono::system_clock::now();

    if (face_detected) {
        face_record_data.last_face_detect_time = now;
        face_record_data.face_detect_count++;

        // 如果未在录制状态，且连续检测到足够次数的人脸，开始录制
        if (!face_record_data.is_recording && face_record_data.face_detect_count >= FACE_DETECT_THRESHOLD)
        {
            face_record_data.is_recording = true;
            face_record_data.record_start_time = now;
            face_record_data.face_detect_count = 0;

            // 触发录制
            trigger_camera_record(true);
        }
    } else {
        // 未检测到人脸，重置连续检测计数
        face_record_data.face_detect_count = 0;
    }

    // 检查是否需要停止录制
    if (face_record_data.is_recording) {
        auto record_duration = std::chrono::duration_cast<std::chrono::seconds>(
            now - face_record_data.record_start_time).count();

        auto time_since_last_face = std::chrono::duration_cast<std::chrono::seconds>(
            now - face_record_data.last_face_detect_time).count();

        bool should_stop = false;
        std::string stop_reason;

        // 停止条件1：超过最大录制时长
        if (record_duration >= RECORD_MAX_DURATION_SECONDS) {
            should_stop = true;
            stop_reason = "达到最大录制时长(" + std::to_string(RECORD_MAX_DURATION_SECONDS) + "秒)";
        }
        // 停止条件2：人脸消失超过指定时间，且已录制超过最小时长
        else if (time_since_last_face >= RECORD_POST_FACE_SECONDS &&
                 record_duration >= RECORD_MIN_DURATION_SECONDS) {
            should_stop = true;
            stop_reason = "人脸消失超过" + std::to_string(RECORD_POST_FACE_SECONDS) + "秒";
        }

        if (should_stop) {
            stop_face_recording();
            WTALOGI("[FaceRecord] 通道[%s] 停止录制，原因: %s，录制时长: %ld秒",
                    channel_name.c_str(),
                    stop_reason.c_str(), record_duration);
        }
    }
}

void ax_model_custom::stop_face_recording()
{
    if (face_record_data.is_recording) {
        face_record_data.is_recording = false;
        face_record_data.face_detect_count = 0;
        trigger_camera_record(false);
    }
}

std::string ax_model_custom::generate_face_record_filename()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&time_t, &tm_now);

    std::ostringstream filename;
    // 格式: 通道名_年_月_日_时_分_秒.mp4
    filename << channel_name << "_" << std::put_time(&tm_now, "%Y_%m_%d_%H_%M_%S") << ".mp4";

    return filename.str();
}

void ax_model_custom::trigger_camera_record(bool start)
{
    // 通过 CameraController 获取对应通道的 Camera 对象
    CameraController* controller = CameraController::getInstance();
    if (!controller) {
        WTALOGI("[FaceRecord] CameraController 实例为空");
        return;
    }

    auto camera = controller->getCamera(camera_id);
    if (camera) {
        pipeline_t* pipeline = camera->get_pipeline();
        if (!pipeline) {
            WTALOGI("[FaceRecord] 通道[%s] pipeline 为空", channel_name.c_str());
            return;
        }

        if (start) {
            // 生成并设置文件名
            auto fpath = camera->generateCustomVideoPath(Camera::VideoPathType::PERSON);

            // 开始录制
            camera->start_record_video();
            WTALOGI("[FaceRecord] 通道[%s] 开始录制，路径: %s", channel_name.c_str(), fpath.c_str());
        } else {
            // 停止录制 - 直接设置 IsRecordVideo = false
            // h265_save_func 会检测到变化并关闭 ffmpeg 管道
            pipeline->IsRecordVideo = false;
        }
    }
}