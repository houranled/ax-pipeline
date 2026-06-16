#include "alarm.hpp"
#include <ctime>

#include "../examples/utilities/json.hpp"
#include "../examples/utilities/sample_log.h"
#include "../camera/camera_controller.hpp"

uint32_t AlarmManager::cooldown = 5;  // 默认冷却 5 秒

bool AlarmManager::isAlarmTriggered(AlarmType type, const std::string &message, int cameraId, float confidence)
{
    return false;
}

bool AlarmManager::generateAlarm(AlarmType type, const std::string& message, float confidence, Camera *camera, int point_id, int light_flag)
{
    if (camera == nullptr || !camera->is_patroling() || !camera->posture_completed)
        return false;

    int cameraId = camera->get_id();

    // 使用互斥锁保护队列操作
    std::lock_guard<std::mutex> lock(queueMutex);

    // ★ 入队前去重：检查是否已存在相同 camera_id + point_id + light_flag 的告警
    auto& q = alarm_map_datas[cameraId];
    std::queue<Alarm> temp_q;
    bool exists = false;
    while (!q.empty()) {
        Alarm& a = q.front();
        if (a.point_id == point_id && a.light_flag == light_flag) {
            exists = true;
        }
        temp_q.push(std::move(a));
        q.pop();
    }
    alarm_map_datas[cameraId] = std::move(temp_q);

    if (exists) {
        WTALOGI("摄像头[%d] 点位[%d] L%d 告警已存在，跳过重复入队", cameraId, point_id, light_flag);
        return false;
    }

    Alarm alarm;
    alarm.cameraId = cameraId;
    alarm.channel_name = camera->getName();
    alarm.point_id = point_id;
    alarm.light_flag = light_flag;
    alarm.type = type;
    alarm.timestamp = time(nullptr);
    alarm.confidence = confidence;
    alarm.picPath = camera->get_pic_path();

    alarm_map_datas[cameraId].push(alarm);
    WTALOGI("为摄像头[%d] 点位[%d] L%d 生成告警!", cameraId, point_id, light_flag);

    queueCondition.notify_one();
    return true;
}

bool AlarmManager::generateDiffAlarm(Camera *camera, int point_id, int light_flag, const std::string& pic_path)
{
    if (camera == nullptr || !camera->is_patroling())
        return false;

    int cameraId = camera->get_id();

    std::lock_guard<std::mutex> lock(queueMutex);

    // ★ 入队前去重：检查是否已存在相同 camera_id + point_id + light_flag 的告警
    auto& q = alarm_map_datas[cameraId];
    std::queue<Alarm> temp_q;
    bool exists = false;
    while (!q.empty()) {
        Alarm& a = q.front();
        if (a.point_id == point_id && a.light_flag == light_flag) {
            exists = true;
        }
        temp_q.push(std::move(a));
        q.pop();
    }
    alarm_map_datas[cameraId] = std::move(temp_q);

    if (exists) {
        WTALOGI("摄像头[%d] 点位[%d] L%d 差异告警已存在，跳过重复入队", cameraId, point_id, light_flag);
        return false;
    }

    Alarm alarm;
    alarm.cameraId = cameraId;
    alarm.channel_name = camera->getName();
    alarm.point_id = point_id;
    alarm.light_flag = light_flag;
    alarm.type = AlarmType::LINE_CROSSING;
    alarm.timestamp = time(nullptr);
    alarm.confidence = 1.0f;
    alarm.picPath = pic_path;

    alarm_map_datas[cameraId].push(alarm);
    WTALOGI("为摄像头[%d] 点位[%d] L%d 生成差异对比告警!", cameraId, point_id, light_flag);

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
                warning["name"] = alarm.channel_name;
                warning["point_id"] = alarm.point_id;
                warning["type"] = alarm.damage_type;
                warning["damage_type"] = static_cast<int>(alarm.type);
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
                one_warning["name"] = alarm.channel_name;
                one_warning["point_id"] = alarm.point_id;
                one_warning["type"] = alarm.damage_type;
                one_warning["damage_type"] = static_cast<int>(alarm.type);
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
