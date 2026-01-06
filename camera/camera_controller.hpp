#ifndef _CAMERA_CONTROLLER_H_
#define _CAMERA_CONTROLLER_H_

#include <thread>
#include <curl/curl.h>
#include <map>
#include <vector>
#include <string>
#include <modbus/modbus.h>

#define CONFIG_FILE_PATH ""

class CameraController;

class Camera {
    friend CameraController;    // 允许CameraController类访问Camera类的私有成员
public:
    struct PresetPosition {
        std::string id;
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
    int set_ptz(int horizontal, int vertical);
    int set_zoom(int zoom);
    int set_brighten(int brightness); //设置亮度
    int fetch_remote_status();
    int get_id();
    bool is_patroling() const; // 获取是否在巡逻中
    void set_patroling(bool patroling); // 设置是否在巡逻中
    std::string get_mode() const;
    int add_preset_position(PresetPosition pos); // 添加单个点位到点位集合中

private:
    int id;
    std::string ip;  // 相机ip地址
    std::string ptz_ip;
    int rotation_x;  // 当前水平角度
    int rotation_y; // 当前垂直角度
    int zoom; // 当前焦距
    int focus;   // 当前聚焦
    int brightness; // 当前补光灯亮度
    int camera_id;
    bool running;
    bool patroling;  // 是否在巡逻中
    std::vector<PresetPosition> preset_positions; // 点位信息集

    std::thread position_thread; // 命令控制相机姿态线程

    CURL *curl_handle;  // 持久化的curl句柄
    modbus_t *modbus_ctx = nullptr;

    // 摄像机巡检线程
    int position_patrol_loop();
    bool is_on_the_place(); // 判断是否到达指定位置

};

class CameraController {
public:
    CameraController();
    ~CameraController();

    int start();
    // 关闭处理线程
    int stop();



private:
    std::map<int, Camera*> cameras;
    std::thread input_thread;
    int camera_id_;
    bool running;
    static CURL *curl_handle;  // 持久化的curl句柄用于通过http与摄像机通信并控制

    // a function for executing in new thread to receive the input read from std-io
    int receive_input_loop();

    int load_config_from_file(const std::string& config_file_path); //根据配置文件信息自构建所有相机对象

};

#endif // _CAMERA_CONTROLLER_H_