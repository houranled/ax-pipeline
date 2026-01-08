#include "ax_model_damage.hpp"


void ax_model_damage::draw_custom(cv::Mat &image, axdl_results_t *results,
    float fontscale, int thickness, int offset_x, int offset_y)
{


}

void ax_model_damage::draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness)
{
    /*
    * 获取当前摄像头所对点位以及相应的初始健康状态下的物体对象照片(基准照片)
    *
    * 发生识别到对象时触发告警(ax-pipele原有的推理识别)  ||   当前图像与基准图片差异达到一定程度时告警(自定义)
    *
    */

    //CameraController::get_instance();
}
