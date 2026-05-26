#pragma once
#include "opencv2/opencv.hpp"
#include <string>
#include <mutex>

// 基于原生 libfreetype 的最小文字 -> RGBA 位图渲染器（单例）。
// 仅用于启动时一次性预渲染中文水印（车牌号/通道名），生命周期内重复使用。
class FreeTypeOverlay
{
public:
    static FreeTypeOverlay& instance();

    // 加载字体；可重复调用（仅首次生效）。返回是否就绪。
    bool init(const std::string& font_path, int pixel_height = 36, int face_index = 0);

    bool ready() const { return m_ready; }

    // 将 UTF-8 文本渲染为紧凑的 RGBA(BGRA in memory) cv::Mat。
    // - 透明背景；前景为 bgra（B,G,R,A），alpha 由字形覆盖度乘以 bgra[3] 得到。
    // - padding：四周像素留白。
    // 返回空 Mat 表示失败。
    cv::Mat renderTextRGBA(const std::string& utf8,
                           cv::Scalar bgra = cv::Scalar(255, 255, 255, 255),
                           int padding = 2);

    ~FreeTypeOverlay();

private:
    FreeTypeOverlay() = default;
    FreeTypeOverlay(const FreeTypeOverlay&) = delete;
    FreeTypeOverlay& operator=(const FreeTypeOverlay&) = delete;

    // UTF-8 -> 一组 UCS4/UTF-32 码点
    static void utf8_to_codepoints(const std::string& s, std::vector<uint32_t>& out);

    void* m_library = nullptr; // FT_Library
    void* m_face    = nullptr; // FT_Face
    int   m_pixel_height = 36;
    bool  m_ready = false;
    std::mutex m_mtx;
};
