#include "alarm.hpp"
#include <ctime>

#include "../camera/camera_controller.hpp"
#include "../examples/utilities/json.hpp"
#include "../examples/utilities/sample_log.h"

uint32_t AlarmManager::cooldown = 5;  // 默认冷却 5 秒

bool AlarmManager::isAlarmTriggered(AlarmType type, const std::string &message, int cameraId, float confidence)
{
    return false;
}

bool AlarmManager::generateAlarm(AlarmType type, const std::string& message, float confidence, Camera *camera)
{
    if (camera == nullptr || !camera->is_patroling() || !camera->posture_completed)
        return false;

    Alarm alarm;
    alarm.cameraId = camera->get_id();
    WTALOGI("为摄像头id[%d]生成告警!", alarm.cameraId);

    alarm.point_id = camera->now_point_id;
    alarm.type = type;
    alarm.timestamp = time(nullptr);  // 获取当前时间戳
    alarm.confidence = confidence;
    alarm.picPath = camera->get_pic_path();

    // 使用互斥锁保护队列操作
    std::lock_guard<std::mutex> lock(queueMutex);
    alarm_map_datas[alarm.cameraId].push(alarm);

    // 通知等待的线程有新的告警
    queueCondition.notify_one();
    return true;
}


std::string AlarmManager::output_alarms(int camera_id)
{
    std::unique_lock<std::mutex> lock(queueMutex);

    // 创建JSON对象
    nlohmann::json result;

    result["warnings"] = nlohmann::json::array();

    // 处理所有通道的告警
    if (camera_id > 0) {
        // 处理指定通道的告警
        if (alarm_map_datas.find(camera_id) != alarm_map_datas.end()) {
            while (!alarm_map_datas[camera_id].empty()) {
                Alarm alarm = alarm_map_datas[camera_id].front();
                alarm_map_datas[camera_id].pop();

                nlohmann::json warning;
                warning["camera_id"] = alarm.cameraId;
                warning["point_id"] = alarm.point_id;
                warning["type"] = static_cast<int>(alarm.type);
                warning["image"] = alarm.picPath.empty() ? "" : alarm.picPath;

                result["warnings"].push_back(warning);
            }
        }
    } else {
        // 处理所有通道的告警
        for (auto& pair : alarm_map_datas) {
            int current_camera_id = pair.first;
            while (!alarm_map_datas[current_camera_id].empty()) {
                Alarm alarm = alarm_map_datas[current_camera_id].front();
                alarm_map_datas[current_camera_id].pop();

                nlohmann::json one_warning;
                one_warning["camera_id"] = alarm.cameraId;
                one_warning["point_id"] = alarm.point_id;
                one_warning["type"] = static_cast<int>(alarm.type);
                one_warning["image"] = alarm.picPath.empty() ? "" : alarm.picPath;

                result["warnings"].push_back(one_warning);
            }
        }
    }

    lock.unlock();

    auto res_str = result.dump();
    WTALOGI("返回告警: %s", res_str.c_str());

    // 返回JSON字符串
    return res_str;
}
