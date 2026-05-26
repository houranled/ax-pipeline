#include "ax_freetype_overlay.hpp"
#include "../../utilities/sample_log.h"

#include <ft2build.h>
#include FT_FREETYPE_H

FreeTypeOverlay& FreeTypeOverlay::instance()
{
    static FreeTypeOverlay s;
    return s;
}

FreeTypeOverlay::~FreeTypeOverlay()
{
    if (m_face)    FT_Done_Face((FT_Face)m_face);
    if (m_library) FT_Done_FreeType((FT_Library)m_library);
}

bool FreeTypeOverlay::init(const std::string& font_path, int pixel_height, int face_index)
{
    std::lock_guard<std::mutex> lk(m_mtx);
    if (m_ready) return true;

    FT_Library lib = nullptr;
    if (FT_Init_FreeType(&lib)) {
        ALOGE("FreeTypeOverlay: FT_Init_FreeType failed");
        return false;
    }
    FT_Face face = nullptr;
    if (FT_New_Face(lib, font_path.c_str(), face_index, &face)) {
        ALOGE("FreeTypeOverlay: FT_New_Face failed: %s", font_path.c_str());
        FT_Done_FreeType(lib);
        return false;
    }
    if (FT_Set_Pixel_Sizes(face, 0, pixel_height)) {
        ALOGE("FreeTypeOverlay: FT_Set_Pixel_Sizes failed");
        FT_Done_Face(face);
        FT_Done_FreeType(lib);
        return false;
    }

    m_library = lib;
    m_face = face;
    m_pixel_height = pixel_height;
    m_ready = true;
    return true;
}

void FreeTypeOverlay::utf8_to_codepoints(const std::string& s, std::vector<uint32_t>& out)
{
    out.clear();
    out.reserve(s.size());
    const unsigned char* p = (const unsigned char*)s.data();
    const unsigned char* end = p + s.size();
    while (p < end) {
        uint32_t cp = 0;
        unsigned char c = *p;
        int extra = 0;
        if (c < 0x80)               { cp = c;          extra = 0; }
        else if ((c & 0xE0) == 0xC0){ cp = c & 0x1F;   extra = 1; }
        else if ((c & 0xF0) == 0xE0){ cp = c & 0x0F;   extra = 2; }
        else if ((c & 0xF8) == 0xF0){ cp = c & 0x07;   extra = 3; }
        else { ++p; continue; } // 非法首字节，跳过
        ++p;
        for (int i = 0; i < extra && p < end; ++i, ++p) {
            if ((*p & 0xC0) != 0x80) { cp = 0; break; }
            cp = (cp << 6) | (*p & 0x3F);
        }
        if (cp) out.push_back(cp);
    }
}

cv::Mat FreeTypeOverlay::renderTextRGBA(const std::string& utf8, cv::Scalar bgra, int padding)
{
    std::lock_guard<std::mutex> lk(m_mtx);
    if (!m_ready || utf8.empty()) return cv::Mat();

    FT_Face face = (FT_Face)m_face;

    std::vector<uint32_t> cps;
    utf8_to_codepoints(utf8, cps);
    if (cps.empty()) return cv::Mat();

    // 第一遍：度量整体宽高
    int pen_x = 0;
    int min_top = INT_MAX, max_bot = INT_MIN;
    struct GMetric {
        int advance;
        int bitmap_left;
        int bitmap_top;
        int bitmap_w;
        int bitmap_h;
        std::vector<unsigned char> buf; // 灰度 alpha
    };
    std::vector<GMetric> metrics;
    metrics.reserve(cps.size());

    for (uint32_t cp : cps) {
        if (FT_Load_Char(face, cp, FT_LOAD_RENDER)) {
            metrics.push_back({0, 0, 0, 0, 0, {}});
            continue;
        }
        FT_GlyphSlot g = face->glyph;
        GMetric m;
        m.advance     = (int)(g->advance.x >> 6);
        m.bitmap_left = g->bitmap_left;
        m.bitmap_top  = g->bitmap_top;
        m.bitmap_w    = (int)g->bitmap.width;
        m.bitmap_h    = (int)g->bitmap.rows;
        m.buf.assign(g->bitmap.buffer,
                     g->bitmap.buffer + (size_t)m.bitmap_w * m.bitmap_h);
        metrics.push_back(std::move(m));

        int top = -metrics.back().bitmap_top;                     // 相对基线 y(顶端)
        int bot = top + metrics.back().bitmap_h;                  // 相对基线 y(底端)
        if (top < min_top) min_top = top;
        if (bot > max_bot) max_bot = bot;
        pen_x += metrics.back().advance;
    }
    if (pen_x <= 0 || min_top == INT_MAX) return cv::Mat();

    int W = pen_x + padding * 2;
    int H = (max_bot - min_top) + padding * 2;
    int baseline_y = padding - min_top; // 基线在画布中的 y 坐标

    cv::Mat canvas(H, W, CV_8UC4, cv::Scalar(0, 0, 0, 0));

    int pen = padding;
    uint8_t bB = (uint8_t)cv::saturate_cast<uint8_t>(bgra[0]);
    uint8_t bG = (uint8_t)cv::saturate_cast<uint8_t>(bgra[1]);
    uint8_t bR = (uint8_t)cv::saturate_cast<uint8_t>(bgra[2]);
    float fgA  = (float)cv::saturate_cast<uint8_t>(bgra[3]) / 255.f;

    for (auto& m : metrics) {
        if (m.bitmap_w > 0 && m.bitmap_h > 0) {
            int x0 = pen + m.bitmap_left;
            int y0 = baseline_y - m.bitmap_top;
            for (int j = 0; j < m.bitmap_h; ++j) {
                int yy = y0 + j;
                if (yy < 0 || yy >= H) continue;
                cv::Vec4b* row = canvas.ptr<cv::Vec4b>(yy);
                const unsigned char* src = m.buf.data() + j * m.bitmap_w;
                for (int i = 0; i < m.bitmap_w; ++i) {
                    int xx = x0 + i;
                    if (xx < 0 || xx >= W) continue;
                    uint8_t a = (uint8_t)(src[i] * fgA);
                    if (!a) continue;
                    // 简单覆盖（透明背景下等价 alpha 合成）
                    if (a > row[xx][3]) {
                        row[xx][0] = bB;
                        row[xx][1] = bG;
                        row[xx][2] = bR;
                        row[xx][3] = a;
                    }
                }
            }
        }
        pen += m.advance;
    }

    return canvas;
}
