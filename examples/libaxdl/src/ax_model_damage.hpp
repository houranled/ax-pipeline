#pragma once
#include "ax_model_det.hpp"
#include "utilities/object_register.hpp"
#include "../../utilities/ringbuffer.hpp"
#include "../../alarm/alarm.hpp"
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "base/detection.hpp"

class ax_model_damage : public ax_model_yolov8_native
{
public:

    ax_model_damage()
    {
        WTALOGI("实例化一个ax_model_damage对象");
    }

    //virtual int inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;

protected:
    // 在这里添加自定义属性

    int post_process(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
    void draw_custom(cv::Mat &image, axdl_results_t *results, float fontscale, int thickness, int offset_x, int offset_y) override;
    //void draw_custom(int chn, axdl_results_t *results, float fontscale, int thickness) override;

private:
    //AlarmGenerator m_alarm_generator; //告警检测器

};


class wt_damage_multi_model_recognize : public wt_model_multi_base_t {
public:
    // 推理
    virtual int inference(axdl_image_t *pstFrame, axdl_bbox_t *crop_resize_box, axdl_results_t *results) override;
};
REGISTER(WT_DAMAGE_MULTI_MODEL_RECOGNIZE, wt_damage_multi_model_recognize)