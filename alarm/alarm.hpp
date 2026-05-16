#ifndef _ALARM_H_
#define _ALARM_H_

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <map>


// 添加 Camera 类的前置声明
class Camera;

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
    int cameraId;           // 摄像头ID
    int point_id;           // 告警点id
    AlarmType type;         // 告警类型
    std::string message;    // 告警消息
    time_t timestamp;       // 时间戳
    float confidence;       // 置信度
    std::string picPath;         // 告警图片存储路径
};

// 告警生成器类
class AlarmManager {
public:
    AlarmManager() {};
    ~AlarmManager() {};

    //检测是否触发告警
    bool isAlarmTriggered(AlarmType type, const std::string& message, int cameraId, float confidence);


    bool generateAlarm(AlarmType type, const std::string& message, float confidence, Camera *camera); // 生成告警
    std::string output_alarms(int camera_id); // 输出对应id的摄像机的告警信息

    static uint32_t cooldown;  // 同(相机,类型)告警冷却时间(秒)

private:
    std::map<int, std::queue<Alarm>> alarm_map_datas;  // 按通道ID分别存储的告警队列
    std::mutex queueMutex;         // 队列互斥锁
    std::condition_variable queueCondition;  // 队列条件变量
};

#endif // ALARM_GENERATOR_H
