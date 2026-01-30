#include "alarm.hpp"
#include <ctime>

#include "../camera/camera_controller.hpp"
#include "../examples/utilities/json.hpp"

uint32_t AlarmManager::cooldown = 0;

bool AlarmManager::isAlarmTriggered(AlarmType type, const std::string &message, int cameraId, float confidence)
{
    return false;
}

Alarm AlarmManager::generateAlarm(AlarmType type, const std::string& message, float confidence, Camera *camera)
{
    Alarm alarm;
    alarm.cameraId = camera->get_id();
    alarm.point_id = camera->now_point_id;
    alarm.type = type;
    alarm.timestamp = time(nullptr);  // 获取当前时间戳
    alarm.confidence = confidence;

    // 使用互斥锁保护队列操作
    std::lock_guard<std::mutex> lock(queueMutex);
    alarmMapDatas[alarm.cameraId].push(alarm);

    // 通知等待的线程有新的告警
    queueCondition.notify_one();
    return alarm;
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
        if (alarmMapDatas.find(camera_id) != alarmMapDatas.end()) {
            while (!alarmMapDatas[camera_id].empty()) {
                Alarm alarm = alarmMapDatas[camera_id].front();
                alarmMapDatas[camera_id].pop();

                nlohmann::json warning;
                warning["point_id"] = alarm.point_id;
                warning["type"] = alarm.type;
                warning["image"] = alarm.picPath.empty() ? "" : alarm.picPath;

                result["warnings"].push_back(warning);
            }
        }
    } else {
        // 处理所有通道的告警
        for (auto& pair : alarmMapDatas) {
            int current_camera_id = pair.first;
            while (!alarmMapDatas[current_camera_id].empty()) {
                Alarm alarm = alarmMapDatas[current_camera_id].front();
                alarmMapDatas[current_camera_id].pop();

                nlohmann::json one_warning;
                one_warning["point_id"] = alarm.point_id;
                one_warning["type"] = alarm.type;
                one_warning["image"] = alarm.picPath.empty() ? "" : alarm.picPath;

                result["warnings"].push_back(one_warning);
            }
        }
    }

    lock.unlock();

    // 返回JSON字符串
    return result.dump();
}
