#include "ax_model_damage.hpp"


int ax_model_damage::post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results)
{
    return 0;
}

void ax_model_damage::draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y)
{

}

void ax_model_damage::draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness)
{

}
