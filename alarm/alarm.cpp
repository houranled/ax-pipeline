#include "alarm.hpp"
#include <ctime>

#include "../examples/utilities/json.hpp"
#include "../examples/utilities/sample_log.h"
#include "../camera/camera_controller.hpp"

uint32_t AlarmManager::cooldown = 2;  // 默认冷却 5 小时

bool AlarmManager::isAlarmTriggered(AlarmType type, const std::string &message, int cameraId, float confidence)
{
    return false;
}

bool AlarmManager::generateAlarm(AlarmType type, const std::string& message, float confidence, Camera *camera, int point_id, int light_flag)
{
    if (camera == nullptr)
        return false;

    int cameraId = camera->get_id();

    // 使用互斥锁保护队列操作
    std::lock_guard<std::mutex> lock(queueMutex);

    // ★ 入队前去重：检查是否已存在相同 camera_id + point_id + light_flag + 损伤类型(type字符串) 的告警
    //   注意：仅按 (point_id, light_flag) 去重会导致同点位检出多种损伤时只保留第一种，
    //   因此把 message（即 alarm.type）也纳入判定，不同损伤类型可并存。
    auto& q = alarm_map_datas[cameraId];
    std::queue<Alarm> temp_q;
    bool exists = false;
    while (!q.empty()) {
        Alarm& a = q.front();
        if (a.point_id == point_id && a.light_flag == light_flag && a.type == message) {
            exists = true;
        }
        temp_q.push(std::move(a));
        q.pop();
    }
    alarm_map_datas[cameraId] = std::move(temp_q);

    if (exists) {
        WTALOGI("摄像头[%d] 点位[%d] L%d 损伤类型[%s] 告警已存在，跳过重复入队", cameraId, point_id, light_flag, message.c_str());
        return false;
    }

    // 冷却：同(相机,点位,损伤类型)首次告警后 cooldown 小时内不再告警（对应损伤片段也不生成）
    std::string cd_key = std::to_string(cameraId) + "_" + std::to_string(point_id) + "_" + message;
    time_t now = time(nullptr);
    auto cd_it = last_alarm_time.find(cd_key);
    if (cd_it != last_alarm_time.end() && (now - cd_it->second) < (time_t)cooldown * 3600) {
        // WTALOGI("摄像头[%d] 点位[%d] 损伤类型[%s] 处于冷却期(%u小时内)，跳过告警", cameraId, point_id, message.c_str(), cooldown);
        return false;
    }
    last_alarm_time[cd_key] = now;

    Alarm alarm;
    alarm.cameraId = cameraId;
    alarm.channel_name = camera->getName();
    alarm.point_id = point_id;
    alarm.light_flag = light_flag;
    alarm.damage_type = type;
    alarm.type = message;  // 使用 message 参数作为损伤类型名称
    alarm.timestamp = now;
    alarm.confidence = confidence;
    alarm.picPath = camera->get_pic_path();

    alarm_map_datas[cameraId].push(alarm);
    WTALOGI("为摄像头[%d] 点位[%d] L%d 生成告警!", cameraId, point_id, light_flag);

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
                warning["type"] = alarm.type;
                warning["damage_type"] = static_cast<int>(alarm.damage_type);
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
                one_warning["type"] = alarm.type;
                one_warning["damage_type"] = static_cast<int>(alarm.damage_type);
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
