#ifndef _CAMERA_CONTROLLER_H_
#define _CAMERA_CONTROLLER_H_

#include <thread>
#include <chrono>
#include <curl/curl.h>
#include <map>
#include <vector>
#include <string>
#include <modbus/modbus.h>
#include <ctime>
#include <fstream>
#include <sys/stat.h>

#define CONFIG_FILE_PATH "/wt_tech/conf/rt.json"

#define MODBUSPTZ  0x4450  //云台modbus参数起始地址

class CameraController;

class Camera {
    friend CameraController;    // 允许CameraController类访问Camera类的私有成员
public:
    struct PresetPosition {
        int id;
        std::string name;
        int distance;
        int duration; // 时间间隔
        int rotation_x;
        int rotation_y;
        int zoom;
        int focus;
        int brightness;

    };

    Camera();
    ~Camera();
    int start(); // 启动相机执行点位循环遍历任务
    int pause(); // 暂停相机任务
    int set_ptz(int horizontal, int vertical, int brightness);
    int set_brighten(int brightness); //单独控制补光灯
    int set_zoom_and_focus(int zoom, int focus);
    int fetch_remote_status();
    int get_id();
    bool is_patroling() const; // 获取是否在巡逻中
    void set_patroling(bool patroling); // 设置是否在巡逻中
    int add_preset_position(PresetPosition pos); // 添加单个点位到点位集合中
    int capture_image_for_reference(); // 拍摄标定图像

private:
    int id;
    std::string ip;  // 相机ip地址
    std::string ptz_ip; //云台ip地址
    int rotation_x=0;  // 当前水平角度
    int rotation_y=0; // 当前垂直角度
    int zoom=0; // 当前焦距
    int focus=0;   // 当前聚焦
    int brightness=0; // 当前补光灯亮度

    bool running;
    bool patrolling = false;  // 是否在巡逻中
    int now_point_id; // 当前所在点位id
    std::vector<PresetPosition> preset_positions; // 点位信息集

    // 图像拍摄相关成员
    static std::string capture_path; // 图像保存路径

    CURL *curl_handle;  // 持久化的curl句柄
    modbus_t *modbus_ctx = nullptr;

    int patrol_with_calibration_loop(bool is_calibrate);  // 摄像机巡检(可伴随标定)
    bool is_posture_completed(); // 判断是否到达指定位置

};

class CameraController {
public:
    CameraController();
    ~CameraController();

    int start();
    // 关闭处理线程
    int stop();


private:
    std::map<int, Camera*> cameras; // 相机id与相机对象的映射
    std::thread input_thread;
    bool running;
    static CURL *curl_handle;  // 持久化的curl句柄用于通过http与摄像机通信并控制

    static int cooldown; //告警冷却时间 单位小时.

    // a function for executing in new thread to receive the input read from std-io
    int receive_input_loop();

    int patrol(); // 巡逻
    int calibrate(int camera_id); // 标定

    int load_config_from_file(const std::string& config_file_path); //根据配置文件信息自构建所有相机对象

};

#endif // _CAMERA_CONTROLLER_H_