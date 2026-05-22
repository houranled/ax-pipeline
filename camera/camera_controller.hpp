#ifndef _CAMERA_CONTROLLER_H_
#define _CAMERA_CONTROLLER_H_

#include <thread>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <modbus/modbus.h>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include "../alarm/alarm.hpp"
#include <curl/curl.h>
#include <functional>

#include "../examples/common/common_pipeline/common_pipeline.h"

#include "../examples/libaxdl/include/ax_model_base.hpp"

#define CONFIG_FILE_PATH "/wt_tech/conf/rt.json"


class Camera; // 声明Camera类，以便在CameraController类中使用

class CameraController {
public:
    // 单例模式获取实例
    static CameraController* getInstance() {
        static CameraController instance;
        return &instance;
    }

    // 禁止拷贝和赋值
    CameraController(const CameraController&) = delete;
    CameraController& operator=(const CameraController&) = delete;

    int start();
    int stop(); // 关闭处理线程
    void early_warning_process(int camera_id); // 对摄像机id为camera_id触发预警
    Camera* getCamera(int camera_id); // 根据id获取相机对象
    std::vector<Camera*> getAllCameras(); // 获取所有摄像机实例
    void forEachCamera(std::function<void(Camera*)> func)
    {
        for (const auto& pair : cameras) {
            func(pair.second);
        }
    }

    void addCamera(int id, std::string channel_name, std::string rtsp_url); // 添加相机
    void remove_all_cameras(); // 移除所有相机对象

private:
    // 将构造函数设为私有
    CameraController();
    ~CameraController();

    std::map<int, Camera*> cameras; // 相机id与相机对象的映射
    std::thread input_thread;
    bool running;
    static CURL *curl_handle;  // 持久化的curl句柄用于通过http与摄像机通信并控制

    AlarmManager alarm_manager; // 告警生成器
    ax_model_base* _model;

    bool is_patrolling = false; // 是否存在巡逻中的摄像通道

    // a function for executing in new thread to receive the input read from std-io
    int receive_input_loop();
    int all_cameras_patrol(); // 巡逻
    int calibrate(int camera_id); // 标定
    int load_config_from_file(const std::string& config_file_path); //根据配置文件信息自构建所有相机对象

};

class Camera {
    #define MODBUSPTZ  0x4450  //云台modbus参数起始地址
    #define MODBUSPORT 8802    // 默认Modbus TCP 端口为 502 改为8802
    #define MODBUSWIPER 0x4458 // 雨刮器
    #define MODBUSSYS   0x4461 // 系统状态查看寄存器
    #define MODBUSPSENT 0x4466 // 光敏值地址
    #define MODBUSPTHRESHOLD  0x4468 // 光敏阈值读写地址

    friend CameraController;  // 允许CameraController类访问Camera类的私有成员
    friend AlarmManager;      // 允许AlarmManager类访问Camera类的私有成员

public:
    enum class VideoPathType {
        PERSON,  // 人物识别视频路径
        VIDEO    // 普通视频路径
    };

    struct PresetPosition {
        int id;
        std::string name;
        int distance;
        int duration; // 时间间隔
        int web_rotation_x;
        int web_rotation_y;
        int zoom;
        int focus;
        int brightness;
    };

    std::string ip;  // 相机ip地址
    std::string ptz_ip; // 云台ip地址
    int now_point_id=0; // 当前所在点位id
    bool posture_completed = true; // 是否到达指定位置
    std::string orga_name; //风场名称
    char pic_dirname[160] = {0}; //巡检保存的图片路径

    int web_rotation_x=0;  // 当前水平角度
    int rotation_x = 0;

    int web_rotation_y=0; // 当前垂直角度
    int rotation_y=0;

    Camera();
    ~Camera();
    int start(); // 启动相机
    int pause(); // 暂停相机任务
    int set_ptz(int horizontal, int vertical, int brightness);
    int set_wiper(int _switch); // 单独控制雨刷
    int set_brighten(int brightness); //单独控制补光灯
    int getPhotosensitive();
    int setphotosensitiveThreshold(int threshold);

    int set_zoom_and_focus(int zoom, int focus);
    int fetch_remote_status();
    int get_id() const;
    bool is_patroling() const; // 获取是否在巡逻中
    void finish_patrolling(); // 告知巡逻结束
    int add_preset_position(PresetPosition pos); // 添加单个点位到点位集合中

    bool start_record_video(); // 录制视频
    bool stop_record_video();  //结束录制并存储视频文件
    bool start_take_a_picture(int kind); // 拍照
    /**
     * @brief 保存当前帧为图片，并更新 pipeline 的图片路径
     * @param image 要保存的 OpenCV 图像
     * @param channel_name 通道名称
     * @param point_id 点位ID
     * @return 保存的图片完整路径，失败返回空字符串
     */
    std::string captureSnapshot(const cv::Mat& image);


    std::string getName(); // 获取相机名称
    std::string get_pic_path() const; // 获取当前录制图片路径
    std::string get_camera_rtsp_url();
    std::string generateCustomVideoPath(VideoPathType); // 生成自定义视频路径

    // 获取相机rtsp url
    void set_camera_rtsp_url(const std::string& url); // 设置相机rtsp url
    void connectPipes(pipeline_t *pipe1, pipeline_t *pipe2);
    pipeline_t* get_pipeline() const; // 获取绑定的pipeline

    const std::vector<PresetPosition>& getPresetPositions() const {
        return preset_positions;
    }

private:
    int id;
    std::string name; // 相机名称
    std::string camera_rtsp_url; // 相机rtsp地址

    int zoom=0; // 当前焦距
    int focus=0;   // 当前聚焦
    int brightness=0; // 当前补光灯亮度
    int wiper_switch=0; // 雨刷开关
    int photosensitive=0; // 光敏值
    int photosensitiveThreshold=0; // 光敏值阈值

    bool running;
    bool patrolling = false;  // 是否在巡逻中

    std::vector<PresetPosition> preset_positions; // 点位信息集

    // 图像拍摄相关成员
    pipeline_t *m_pipeline;

    CURL *curl_handle;  // 持久化的curl句柄
    modbus_t *modbus_ctx = nullptr;

    int patrol_with_calibration_loop(bool is_calibrate, bool enable_light=true);  // 摄像机巡检(可伴随标定)
    void update_posture_state(int x, int y); // 判断是否到达指定位置
    void setPipe(pipeline_t * pipe); // 绑定pipeline
    bool connect_modbus(); // 重连modbus

};

#endif // _CAMERA_CONTROLLER_H_