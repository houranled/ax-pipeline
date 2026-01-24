#include "alarm.hpp"
#include <ctime>

AlarmGenerator::AlarmGenerator() {

}

AlarmGenerator::~AlarmGenerator() {

}


bool AlarmGenerator::isAlarmTriggered(AlarmType type, const std::string &message, int cameraId, float confidence)
{
    return false;
}

Alarm AlarmGenerator::generateAlarm(AlarmType type, const std::string& message, int cameraId, float confidence) {
    Alarm alarm;
    alarm.type = type;
    alarm.message = message;
    alarm.cameraId = cameraId;
    alarm.timestamp = time(nullptr);  // 获取当前时间戳
    alarm.confidence = confidence;

    // 通知所有监听器
    for (auto listener : m_listeners) {
        listener(alarm);
    }

    return alarm;
}

void AlarmGenerator::addAlarmListener(void (*listener)(const Alarm&)) {
    m_listeners.push_back(listener);
}

void AlarmGenerator::look_a_alarm()
{

}
