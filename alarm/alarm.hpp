#ifndef _ALARM_H_
#define _ALARM_H_

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>

// 告警类型枚举
enum class AlarmType {
    MOTION_DETECTED,    // 运动检测告警
    OBJECT_DETECTED,    // 物体检测告警
    AREA_INTRUSION,     // 区域入侵告警
    LINE_CROSSING,      // 越线检测告警
    UNKNOWN             // 未知类型
};

// 告警结构体
struct Alarm {
    AlarmType type;         // 告警类型
    std::string message;    // 告警消息
    int cameraId;           // 摄像头ID
    time_t timestamp;       // 时间戳
    float confidence;       // 置信度
};

// 告警生成器类
class AlarmGenerator {
public:
    AlarmGenerator();
    ~AlarmGenerator();

    //检测是否触发告警
    bool isAlarmTriggered(AlarmType type, const std::string& message, int cameraId, float confidence);

    // 生成告警
    Alarm generateAlarm(AlarmType type, const std::string& message, int cameraId, float confidence);

    // 消费者获取每个告警
    void look_a_alarm();
    
    // 获取队列中的告警数量
    size_t getAlarmCount();

private:
    std::queue<Alarm> alarmQueue;  // 告警队列
    std::mutex queueMutex;         // 队列互斥锁
    std::condition_variable queueCondition;  // 队列条件变量
};

#endif // ALARM_GENERATOR_H
