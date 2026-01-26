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

    // 使用互斥锁保护队列操作
    std::lock_guard<std::mutex> lock(queueMutex);
    alarmQueue.push(alarm);

    // 通知等待的线程有新的告警
    queueCondition.notify_one();
    return alarm;
}


size_t AlarmGenerator::getAlarmCount()
{
    std::lock_guard<std::mutex> lock(queueMutex);
    return alarmQueue.size();
}

void AlarmGenerator::look_a_alarm()
{
    std::unique_lock<std::mutex> lock(queueMutex);

    // 等待直到队列不为空
    queueCondition.wait(lock, [this](){ return !alarmQueue.empty(); });

    // 获取队列中的第一个告警（先进先出）
    Alarm alarm = alarmQueue.front();
    alarmQueue.pop();

    lock.unlock();

    // 在这里处理获取到的告警
    // 例如：记录日志、触发警报通知等
}
