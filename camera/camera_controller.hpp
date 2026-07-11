#ifndef _CAMERA_CONTROLLER_H_
#define _CAMERA_CONTROLLER_H_

#include <thread>
#include <chrono>
#include <map>
#include <set>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include "../alarm/alarm.hpp"
#include <curl/curl.h>
#include <functional>
#include "../examples/common/common_pipeline/common_pipeline.h"
#include "../examples/libaxdl/include/ax_model_base.hpp"
#include <modbus/modbus.h>

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
    bool early_warning_process(int camera_id, int point_id, int light_flag, const std::string& damage_type = ""); // 对摄像机id为camera_id触发预警，返回是否真正生成告警
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
    int reload_config(); // 热加载配置（点位+基础字段，不含RTSP重连）

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
    time_t last_config_mtime = 0; // 配置文件最后修改时间，用于热加载判断

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
        PERSON,       // 人物识别视频路径
        VIDEO,        // 普通巡检视频路径
        DAMAGE_CLIP   // 损伤目标点位 ±停留段 视频路径
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
    std::string ptz_type = "big";
    int now_point_id=0; // 当前所在点位id
    std::atomic<bool> posture_completed{true}; // 是否到达指定位置
    bool light_phase_changed = false; // 灯光状态变更标志，用于同一点位触发两次拍照
    // 相位就绪时刻(ms since epoch)：到位且灯光稳定/armed后由巡检线程置为当前时刻；
    // 移动/回位时置 0。draw_custom 据此 + 流延迟余量判定，只有真正就绪后才累积/拍照，
    // 避免把移动过程、灯光切换过程或视频流缓冲中的旧帧误当作当前点位画面。
    std::atomic<long long> phase_ready_ms{0};
    std::atomic<int> frame_should_capture{0}; // 0=不拍照, 1=L0, 2=L1，由巡检线程设置
    std::set<int> photo_fired_keys; // 已拍照点位+灯光状态（key = point_id * 10 + light_flag）

    // 累积检测结果：停留期间所有帧的检测结果合并，拍照时使用
    std::mutex accumulated_mutex;
    std::vector<axdl_object_t> accumulated_objects; // 累积的检测目标
    void accumulate_detection(const axdl_results_t* results); // 累积当前帧检测结果
    void clear_accumulated_detections(); // 清空累积结果（点位切换时调用）
    std::vector<axdl_object_t> get_accumulated_objects(); // 获取累积结果的拷贝

    int lamp_toggle = 0;        // 主副灯切换标志（均衡寿命）
    int last_brightness = 0;    // 上次亮度值（用于检测开灯时机）
    std::string orga_name; //风场名称
    char pic_dirname[160] = {0}; //巡检保存的图片路径

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
    bool is_calibrating() const { return calibrating.load(); } // 是否处于标定模式
    void finish_patrolling(); // 告知巡逻结束
    // 请求中断当前巡检/标定循环：循环会在最近的可打断点退出
    void request_stop_patrol() {
        {
            std::lock_guard<std::mutex> lk(stop_mtx);
            stop_requested.store(true);
        }
        stop_cv.notify_all(); // 立刻唤醒所有正在 interruptible_sleep_ms 中的等待
    }
    bool is_stop_requested() const { return stop_requested.load(); }
    int add_preset_position(PresetPosition pos); // 添加单个点位到点位集合中

    bool start_record_video(); // 录制视频
    bool stop_record_video();  //结束录制并存储视频文件
    bool start_take_a_picture(int kind); // 拍照
    /**
     * @brief 保存当前帧为图片，并更新 pipeline 的图片路径
     * @param image 要保存的 OpenCV 图像
     * @param point_id 点位ID（避免使用可能已变化的 now_point_id）
     * @param light_flag 灯光状态：0=无灯照，1=有灯照
     * @return 保存的图片完整路径，失败返回空字符串
     */
    std::string captureSnapshot(const cv::Mat& image, int point_id, int light_flag = -1);


    std::string getName(); // 获取相机名称
    std::string get_pic_path() const; // 获取当前录制图片路径
    std::string get_camera_rtsp_url();
    std::string generateCustomVideoPath(VideoPathType); // 生成自定义视频路径

    // ===== 损伤片段独立录像（与巡检主录像并行） =====
    // 由损伤检测模型在检出损伤目标时调用，标记当前停留期间存在损伤
    void mark_damage_seen();
    // 进入"到位"：把 3 秒预录缓冲固化为片段起点，开始累积停留期间帧
    void on_arrived_at_point();
    // 进入"离开"：再录 3 秒后由 pipeline 状态机自动落盘（如果期间检出过损伤）
    void on_leaving_point();

    // 设置相机rtsp url
    void set_camera_rtsp_url(const std::string& url);
    void connectPipes(pipeline_t *pipe1, pipeline_t *pipe2);
    pipeline_t* get_pipeline() const; // 获取绑定的pipeline

    const std::vector<PresetPosition>& getPresetPositions() const {
        return preset_positions;
    }

    // 当前补光灯是否开启（brightness>0）。供"点位前后对比"算法判定 light_flag 用。
    bool is_light_on() const { return brightness > 0; }

    // ---- 点位前后对比（同光照↔同光照）：巡检结束后批量处理 ----
    struct PendingDiffTask {
        int         point_id;       // 点位号
        int         light_flag;     // 0=未开灯, 1=开灯
        cv::Mat     raw_image;      // 不带检测框的原图（内存中，用于 diff 对比）
        std::string display_path;   // 带检测框的图片路径（用于展示告警）
    };
    // 在 draw_custom 拍照成功后入队（不做重型计算，仅排队元数据）
    void enqueue_diff_task(int point_id, int light_flag, const cv::Mat& raw_image, const std::string& display_path);
    // 取走并清空当前队列；由 patrol_with_calibration_loop 末尾的批量处理消费
    std::vector<PendingDiffTask> drain_diff_queue();

private:
    int id;
    std::string name; // 相机名称
    std::string camera_rtsp_url; // 相机rtsp地址
    int web_rotation_x=0;  // 当前水平角度
    int rotation_x = 0;

    int web_rotation_y=0; // 当前垂直角度
    int rotation_y=0;

    int zoom=0; // 当前焦距
    int focus=0;   // 当前聚焦
    int brightness=0; // 当前补光灯亮度
    int wiper_switch=0; // 雨刷开关
    int photosensitive=0; // 光敏值
    int photosensitiveThreshold=0; // 光敏值阈值

    bool running;
    bool patrolling = false;  // 是否在巡逻中
    std::atomic<bool> calibrating{false}; // 是否处于标定模式（标定时跳过 diff 门控，强制建/更新基线）
    time_t patrol_start_time = 0; // 巡检开始时间戳（东八区已修正），用于本轮巡检所有路径统一时间基准
    std::atomic<bool> stop_requested{false}; // 收到停止巡检指令
    std::mutex              stop_mtx;        // 配合 stop_cv 使用
    std::condition_variable stop_cv;         // 用于立刻唤醒巡检中各处 sleep

    std::vector<PresetPosition> preset_positions; // 点位信息集

    // 图像拍摄相关成员
    pipeline_t *m_pipeline;

    CURL *curl_handle;  // 持久化的curl句柄
    modbus_t *modbus_ctx = nullptr;

    // === Modbus 线程安全 & 后台重连 ===
    // modbus_ctx 会被多条线程访问（姿态轮询线程、JSON 命令线程、fetch_remote_status
    // 线程、connect_modbus 内部的 close/free），必须整体互斥；且 connect_modbus 可能
    // 从锁内部再次调用，所以用 recursive_mutex。
    std::recursive_mutex m_modbus_mtx;                 // 保护 modbus_ctx 的所有读/写/IO
    std::atomic<bool>    m_modbus_broken{false};       // 业务函数发现 modbus IO 失败时置位
    std::atomic<bool>    m_modbus_reconnect_exit{false};
    std::thread          m_modbus_reconnect_thread;    // 后台重连线程（异步 + 冷却）
    void _modbus_reconnect_loop();

    // 姿态轮询防重入：避免同一 Camera 起多条 update_posture_state 轮询线程
    std::atomic<bool>    m_posture_polling{false};

    int patrol_with_calibration_loop(bool is_calibrate);  // 摄像机巡检(可伴随标定)
    // 可被 stop_requested 打断的等待；返回 true 表示等满 ms，false 表示被打断
    bool interruptible_sleep_ms(int ms);
    void update_posture_state(int x, int y); // 判断是否到达指定位置
    // 把点位竖直角度 web_rotation_y 归一化到 (-180, 180]（例如 320 → -40），
    // 再按 ptz_type 对应的硬件可动范围 clamp：
    //   big:   [-40, 90]  俯下 90° + 上仰 40°
    //   small: [-90, 90]  俯下 90° + 上仰 90°
    // 归一化后仍越界的（例如 -160）clamp 到最近边界。
    void clamp_preset_y(PresetPosition& pos) const;
    // 把任意竖直角度归一化到 (-180,180] 并按 ptz_type 硬件范围 clamp，返回可下发角度
    int clamp_y_angle(int y) const;
    void setPipe(pipeline_t * pipe); // 绑定pipeline
    bool connect_modbus(); // 重连modbus

    // 点位前后对比的待处理队列（每轮巡检独立累积，巡检结束后被 drain）
    std::vector<PendingDiffTask> m_diff_queue;
    std::mutex                   m_diff_queue_mtx;
};

#endif // _CAMERA_CONTROLLER_H_