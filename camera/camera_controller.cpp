#include "camera_controller.hpp"
#include <iostream>
#include <algorithm>
#include <future>
#include <tuple>
#include <cstdio>
#include <cstdlib>
#include <curl/curl.h>
#include <curl/easy.h>
#include "../examples/utilities/json.hpp"
#include <fstream>
#include "../examples/utilities/sample_log.h"
#include <unistd.h>
#include <pthread.h>
#include <deque>
#include <condition_variable>

// 巡检结束后的"点位前后对比"批量处理函数（实现位于 examples/libaxdl/src/ax_model_damage.cpp）
// 这里前置声明，链接期解析。把重型 OpenCV 计算从渲染热路径移到巡检结束后统一执行。
// update_baseline: true=标定模式更新基线，false=巡检模式不更新基线
void run_post_patrol_diff(class Camera* cam, bool update_baseline);

// ============================ 异步存图 ============================
// 快照的 JPEG/PNG 编码与磁盘写入较重（PNG 尤甚），若在渲染线程（draw_custom）中同步执行，
// 会在"到点位拍照"的瞬间卡住画面输出，表现为丢帧、左上角时间戳跳变。
// 这里用单个后台线程串行消费存图任务：渲染线程只负责把图入队后立即返回。
// FIFO 顺序保证同一路径"先拍照图、后 diff 覆盖图"的写入次序正确；析构时写完剩余任务，不丢图。
namespace {
struct SnapshotJob {
    cv::Mat          image;
    std::string      path;
    std::vector<int> params;
};

class AsyncImageWriter {
public:
    static AsyncImageWriter& instance() {
        // 单例永不析构。同时避免 std::thread 在 joinable 状态被析构
        // 触发 std::terminate。进程退出时后台线程由 OS 回收。
        static AsyncImageWriter* w = new AsyncImageWriter();
        return *w;
    }

    void enqueue(cv::Mat image, std::string path, std::vector<int> params) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!stop_) {
                jobs_.push_back({std::move(image), std::move(path), std::move(params)});
                cv_.notify_one();
                return;
            }
        }
        // 已停机：退化为同步落盘，避免丢图（此路径极少触发）
        if (!cv::imwrite(path, image, params)) {
            WTALOGI("[Camera] 同步存图失败: %s", path.c_str());
        }
    }

    // 阻塞直到队列清空且当前任务写完（供巡检结束后确保所有快照已落盘）
    void flush() {
        std::unique_lock<std::mutex> lk(mtx_);
        done_cv_.wait(lk, [this]{ return jobs_.empty() && !busy_; });
    }

    // 写完剩余任务后停止后台线程（幂等）。供进程退出前显式调用。
    void shutdown() {
        {
            std::unique_lock<std::mutex> lk(mtx_);
            if (stop_) return;
            done_cv_.wait(lk, [this]{ return jobs_.empty() && !busy_; });
            stop_ = true;
        }
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

private:
    AsyncImageWriter() : worker_([this]{ run(); }) {}
    ~AsyncImageWriter() = default;  // 对象被泄漏，析构不会执行；停机请显式调用 shutdown()

    void run() {
        for (;;) {
            SnapshotJob job;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait(lk, [this]{ return stop_ || !jobs_.empty(); });
                if (jobs_.empty()) { if (stop_) return; else continue; }
                job = std::move(jobs_.front());
                jobs_.pop_front();
                busy_ = true;
            }
            if (!cv::imwrite(job.path, job.image, job.params)) {
                WTALOGI("[Camera] 异步存图失败: %s", job.path.c_str());
            }
            {
                std::lock_guard<std::mutex> lk(mtx_);
                busy_ = false;
            }
            done_cv_.notify_all();
        }
    }

    std::deque<SnapshotJob>  jobs_;
    std::mutex               mtx_;
    std::condition_variable  cv_;
    std::condition_variable  done_cv_;
    bool                     stop_ = false;
    bool                     busy_ = false;
    std::thread              worker_;
};
} // namespace

// 供本文件巡检结束流程调用：等待所有异步快照落盘
static void flush_pending_snapshots() { AsyncImageWriter::instance().flush(); }

// 解析点位专用模型字段 pointModels：逗号分隔的模型名字符串（如 "a,b,c"），
// 按逗号拆分并去除每项首尾空白后追加到 out（空项忽略）。
static void parse_point_models(const std::string& csv, std::vector<std::string>& out)
{
    size_t start = 0;
    while (start <= csv.size()) {
        size_t comma = csv.find(',', start);
        std::string item = (comma == std::string::npos)
                               ? csv.substr(start)
                               : csv.substr(start, comma - start);
        size_t b = item.find_first_not_of(" \t\r\n");
        size_t e = item.find_last_not_of(" \t\r\n");
        if (b != std::string::npos) out.push_back(item.substr(b, e - b + 1));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
}


void CameraController::addCamera(int id, std::string channel_name, std::string rtsp_url)
{
    auto camera = new Camera; // 添加一个默认相机
    camera->id = id;
    camera->name = channel_name;
    camera->set_camera_rtsp_url(rtsp_url);
    cameras[id] = camera; // 添加到相机列表
}

CameraController::CameraController()
{
    WTALOGI("创建摄像头控制器实例.");

    // 初始化libcurl全局环境
    CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
    //根据json配置文件创建对应的相机实例
    load_config_from_file(CONFIG_FILE_PATH);
}

CameraController::~CameraController()
{
    // 清理libcurl全局环境
    curl_global_cleanup();
    stop(); // 停止接收输入循环
}

/*
* 专用的工作线程处理命令执行
* 通过标准输入与输出进行对上层应用的数据传输
*/
int CameraController::receive_input_loop() {
    while (running) {
        std::string json_str;

        // 读取标准输入，接收命令以解析
        std::string input;
        int brace_count = 0;
        while (std::getline(std::cin, input)) {
            for (char c : input) {
                if (c == '{') brace_count++;
                if (c == '}') brace_count--;
            }
            json_str += input + "\n";
            if (brace_count == 0 && json_str.size()>2 ) break; // JSON 闭合
        }

        // getline 因 EOF/错误退出且没读到有效内容：不要拿空串去 parse（会抛
        // "unexpected end of input"），更要避免在 stdin 关闭后空转刷屏。
        if (json_str.find_first_not_of(" \t\r\n") == std::string::npos) {
            if (std::cin.eof() || std::cin.fail()) {
                std::cin.clear();                 // 复位流状态以便后续可继续读取
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 防忙等
            }
            continue;
        }

        nlohmann::json json_request; //请求
        try {
            // 解析JSON字符串(处理JSON命令)
            json_request = nlohmann::json::parse(json_str);
        } catch (const nlohmann::json::parse_error& e) {
            WTALOGI("%s json parse error%s",json_str.c_str(),e.what());
            continue;
        }

        nlohmann::json response;  //响应
        // 处理JSON命令
        int camera_id = -1;
        if (json_request.contains("id")) {
            camera_id = std::stoi(json_request["id"].get<std::string>());
        }

        std::string reqId;
        if (json_request.contains("reqId")) {
            reqId = json_request.value("reqId", "");
        }
        response["reqId"] = reqId;

        std::string cmd = json_request.value("cmd", "");
        response["cmd"] = cmd;

        Camera* camera = NULL;
        if (camera_id > 0) {
            auto it = cameras.find(camera_id);
            if (it != cameras.end()) {
                camera = it->second;
            }
        }

        WTALOGI("接收到命令:[%s],相机id:[%d]\n", json_str.c_str(), camera_id);
        if (cmd == "get") {
            if (camera_id > 0 && camera != NULL) {
                response["data"] = nlohmann::json::array();
                //以json组装一台相机的内部状态信息
                auto& resp_first_camera_data = response["data"][0];
                resp_first_camera_data["id"] = camera_id;
                camera->fetch_remote_status();
                resp_first_camera_data["rotationx"] = camera->web_rotation_x;
                resp_first_camera_data["rotationy"] = camera->web_rotation_y;
                resp_first_camera_data["zoom"] = camera->zoom;
                resp_first_camera_data["focus"] = camera->focus;
                resp_first_camera_data["brightness"] = camera->brightness;
                resp_first_camera_data["point_id"] = camera->now_point_id;
                resp_first_camera_data["patrol"] = camera->patrolling;
                resp_first_camera_data["wiper"] = std::to_string(camera->wiper_switch); // 雨刷开关
                resp_first_camera_data["photosensitive"] = camera->photosensitive; // 光敏亮度
                resp_first_camera_data["photosensitiveThreshold"] = camera->photosensitiveThreshold; // 光敏阈值
            } else if (camera_id > 0) {
                response["msg"] = "该相机不存在!";
                response["status"] = 500;
            } else if (camera_id <= 0) { // 所有摄像机信息
                int index = 0;
                response["data"] = nlohmann::json::array();
                for (auto& pair : cameras) {
                    auto& camera = pair.second;
                    camera->fetch_remote_status();
                    // 以json组装每个相机的内部状态信息
                    auto& resp_one_camera_data = response["data"][index];
                    resp_one_camera_data["id"] = camera->id;
                    resp_one_camera_data["rotationx"] = camera->web_rotation_x;
                    resp_one_camera_data["rotationy"] = camera->web_rotation_y;
                    resp_one_camera_data["zoom"] = camera->zoom;
                    resp_one_camera_data["focus"] = camera->focus;
                    resp_one_camera_data["brightness"] = camera->brightness;
                    resp_one_camera_data["point_id"] = camera->now_point_id;
                    resp_one_camera_data["patrol"] = camera->patrolling;
                    resp_one_camera_data["wiper"] = std::to_string(camera->wiper_switch); // 雨刷开关
                    resp_one_camera_data["photosensitive"] = camera->photosensitive; // 光敏
                    resp_one_camera_data["photosensitiveThreshold"] = camera->photosensitiveThreshold; // 光敏阈值
                }
            }
        } else if (cmd == "set" || cmd == "add") {
            if (camera_id > 0 && camera != NULL) { //指定了相机
                if (camera->is_patroling()) { // 是巡检模式,不允许设置
                    response["status"] = 500;
                    response["msg"] = "摄像头处于巡逻模式，无法操作.待巡逻结束后再试...";
                    goto Finish;
                }

                bool has_data = json_request.contains("data");
                if (!has_data) {
                    response["status"] = 500;
                    response["msg"] = "The JSON key 'data' is missing or empty";
                    goto Finish;
                }
                nlohmann::json data = json_request["data"];
                bool has_rotatex = data.contains("rotatex");
                bool has_rotatey =  data.contains("rotatey");
                bool has_zoom =  data.contains("zoom");
                bool has_focus = data.contains("focus");
                bool has_brightness = data.contains("brightness");
                bool has_wiper = data.contains("wiper");
                bool has_photosensitiveThreshold = data.contains("photosensitiveThreshold");

                int origin_rotatex, origin_rotatey, origin_zoom, origin_focus, origin_brightness; //前端原有值
                int photosensitiveThreshold = -1;
                if (cmd == "set") {
                    origin_rotatex = 0;
                    origin_rotatey = 0;
                    origin_zoom = 0;
                    origin_focus = 0;
                    origin_brightness = 0;
                } else { //add
                    origin_rotatex = camera->web_rotation_x;
                    origin_rotatey = camera->web_rotation_y;
                    origin_zoom = camera->zoom;
                    origin_focus = camera->focus;
                    origin_brightness = camera->brightness;
                }

                int x=-1, y=-1, zoom=-1, focus=-1,brightness=-1; //后端最终值 -1表示此次不需要更改该值
                int wiper_switch = -1; // -1表示此次不需要更改该值
                int new_rotatex=0,new_rotatey=0,new_zoom=0,new_focus=0,new_brightness=0;

                if (has_rotatex) {
                    new_rotatex = data["rotatex"]; //前端角度变化量x
                    camera->web_rotation_x = (origin_rotatex + new_rotatex)%360; // 更新前端值x
                    x = (360+camera->web_rotation_x)%360 * 100;  // 映射后端值x
                }

                if (has_rotatey) {
                    new_rotatey = data["rotatey"]; // 前端角度变化量y
                    // 归一化并按云台硬件可动范围 clamp（大云台俯下90/上仰40），避免下发到达不了的角度
                    camera->web_rotation_y = camera->clamp_y_angle(origin_rotatey + new_rotatey); // 更新前端值y
                    y = (360+camera->web_rotation_y)%360 * 100;  // 映射后端值y
                }

                if (has_zoom) {
                    new_zoom = data["zoom"];
                    zoom = origin_zoom + new_zoom;
                }
                if (has_focus) {
                    new_focus = data["focus"];
                    focus = origin_focus + new_focus;
                }
                if (has_brightness) {
                    new_brightness = data["brightness"];
                    brightness = origin_brightness + new_brightness;
                    if (brightness<0) {
                        brightness = 0;
                    } else if (brightness>1000) {
                        brightness = 1000;
                    }
                }
                if (has_wiper) {
                    wiper_switch = std::stoi(data["wiper"].get<std::string>());
                }

                std::string err_msg = "";
                bool has_error = false;

                if (camera->set_ptz(x, y, brightness) < 0) {
                    err_msg = "对云台的操作失败!";
                    has_error = true;
                }

                if (camera->set_zoom_and_focus(zoom, focus) < 0) {
                    err_msg += "对摄像机操作失败!";
                    has_error = true;
                }

                if (wiper_switch != -1) {
                    if (camera->set_wiper(wiper_switch)<0) {
                        err_msg += "对雨刷的操作失败!";
                        has_error = true;
                    }
                }

                if (has_photosensitiveThreshold) {
                    photosensitiveThreshold = data["photosensitiveThreshold"];
                    if (camera->setphotosensitiveThreshold(photosensitiveThreshold)<0) {
                        err_msg += "对云台操作失败!";
                        has_error = true;
                    }
                }

                response["msg"] = err_msg;

                if (has_error == true)
                    response["status"] = 500;
                else
                    response["status"] = 200;

            } else  { // 未指定摄像头 直接报错
                response["status"] = 500;
                response["msg"] = "未指定摄像头";
            }
        } else if (cmd == "action") {//执行一次巡检
            // 快速返回，在后台线程中执行巡检
            std::string currentReqId = reqId; // 捕获当前的reqId值
            int currentCameraId = camera_id; // 捕获当前的camera_id值

            if (is_patrolling == true) {
                response["status"] = 500;
                response["msg"] = "存在正进行的巡检任务的摄像云台，请稍后再执行...";
                goto Finish;
            }

            // 根据是否指定了有效的摄像机ID返回不同的消息
            if (camera_id > 0 && cameras.find(camera_id) != cameras.end()) {
                response["msg"] = "相机[" + std::to_string(camera_id) + "] 巡检于后台开始运行...";
            } else {
                response["msg"] = "所有摄像机的巡检任务于后台开始运行...";
            }
            response["status"] = "start";

            is_patrolling = true; // 设置巡检状态为true
            // 创建后台线程执行巡检任务
            std::thread patrol_thread([this, currentReqId, currentCameraId]() { // 值捕获方式
                // 如果指定了有效的摄像机ID，则只巡检该摄像机；否则巡检所有摄像机
                if (currentCameraId > 0 && cameras.find(currentCameraId) != cameras.end()) {

                    Camera* camera = getCamera(currentCameraId);
                    // 一次巡检：每个点位内部分为无灯照和有灯照两阶段拍照
                    camera->patrol_with_calibration_loop(false);
                    auto warnstr = alarm_manager.output_alarms(currentCameraId);

                    // 指定摄像通道巡检完成后构造JSON响应
                    nlohmann::json result;
                    result["status"] = "end";
                    // 将warnings字符串解析为JSON对象，然后提取其中的warnings数组
                    nlohmann::json warn_json = nlohmann::json::parse(warnstr);
                    result["warnings"] = warn_json["warnings"];
                    result["msg"] = "相机[" + std::to_string(currentCameraId) + "] 巡检任务结束";
                    result["cmd"] = "action";
                    result["reqId"] = currentReqId; // 添加reqId到响应中
                    std::cout << result.dump() << std::endl;
                } else {  // 如果没有指定有效的摄像机ID，则巡检所有摄像机
                    auto res = all_cameras_patrol();
                    auto warnstr = alarm_manager.output_alarms(currentCameraId); //批量检查告警

                    // 全部摄像通道巡检完成后构造JSON响应
                    nlohmann::json result;
                    result["status"] = "end";
                    // 将warnings字符串解析为JSON对象，然后提取其中的warnings数组
                    nlohmann::json warn_json = nlohmann::json::parse(warnstr);
                    result["warnings"] = warn_json["warnings"];
                    // 取第一个非空 pic_dirname：首台相机可能未巡检（无 pipeline 被跳过），其 pic_dirname 为空。
                    // 路径按风场/日期组织、不区分摄像头，任一已巡检相机的目录即可代表本轮。
                    std::string pic_path;
                    for (auto& kv : cameras) {
                        if (kv.second->pic_dirname[0] != '\0') {
                            pic_path = kv.second->pic_dirname;
                            break;
                        }
                    }
                    result["path"] = pic_path;
                    if (res) {
                        result["msg"] = "巡检过程可能受限，请检查设备连接";
                    } else {
                        result["msg"] = "所有摄像头的巡检任务结束";
                    }

                    result["cmd"] = "action";
                    result["reqId"] = currentReqId; // 添加reqId到响应中
                    std::cout << result.dump() << std::endl;
                }
                is_patrolling = false; // 巡检结束后，将巡检状态设置为false
            });
            patrol_thread.detach();

        } else if (cmd == "stopwhen") { // 停止当前正在进行的巡检/标定
            if (is_patrolling == false) {
                response["status"] = 200;
                response["msg"] = "当前无正在进行的巡检任务";
            } else if (camera_id > 0) {
                // 停止指定摄像机
                if (camera != NULL) {
                    camera->request_stop_patrol();
                    response["status"] = 200;
                    response["msg"] = "已请求停止相机[" + std::to_string(camera_id) + "]的巡检任务";
                } else {
                    response["status"] = 500;
                    response["msg"] = "该相机不存在!";
                }
            } else {
                // 未指定相机：停止所有正在巡检的相机
                for (auto& pair : cameras) {
                    if (pair.second) pair.second->request_stop_patrol();
                }
                response["status"] = 200;
                response["msg"] = "已请求停止所有摄像机的巡检任务";
            }
        } else if (cmd == "calibrate") { //执行一次标定
            /* 标定时，会执行一次巡检; 快速返回，在后台线程中执行标定  */
            std::string currentReqId = reqId; // 捕获当前的reqId值
            int currentCameraId = camera_id; // 捕获当前的camera_id值

            // 巡检/标定互斥：is_patroling() 在巡检或标定期间均为 true，
            // 目标相机正忙则拒绝本次标定，避免同一相机并发运行两个 patrol_with_calibration_loop。
            if (currentCameraId > 0 && cameras.find(currentCameraId) != cameras.end()) { // 指定相机
                if (getCamera(currentCameraId)->is_patroling()) {
                    response["status"] = 500;
                    response["msg"] = "相机[" + std::to_string(currentCameraId) + "]正在巡检/标定中，无法开始标定";
                    goto Finish;
                }
            } else { // 未指定相机（标定全部）：任一相机在巡检/标定则拒绝
                if (is_patrolling) {
                    response["status"] = 500;
                    response["msg"] = "存在正在巡检/标定的相机，无法开始标定";
                    goto Finish;
                }
            }

            // 可选的点位ID：指定后仅标定该摄像头的该点位（转到点位拍摄 L0/L1 基线）。
            // 兼容顶层 point_id 与 data.point_id，支持数字或字符串。-1 表示标定全部点位。
            int currentPointId = -1;

            auto parse_point_id = [](const nlohmann::json& j) -> int {
                if (!j.is_null()) {
                    if (j.is_number_integer()) return j.get<int>();
                    if (j.is_string()) {
                        try { return std::stoi(j.get<std::string>()); } catch (...) {}
                    }
                }
                return -1;
            };

            if (json_request.contains("point")) {
                currentPointId = parse_point_id(json_request["point"]);
            }

            // 指定点位标定必须同时指定有效摄像头
            if (currentPointId > 0 && !(camera_id > 0 && cameras.find(camera_id) != cameras.end())) {
                response["status"] = 500;
                response["msg"] = "指定点位标定必须同时指定有效的摄像头id";
                goto Finish;
            }

            // 根据是否指定了有效的摄像机ID返回不同的消息
            if (camera_id > 0 && cameras.find(camera_id) != cameras.end()) {
                if (currentPointId > 0) {
                    response["msg"] = "Camera " + std::to_string(camera_id) + " point " + std::to_string(currentPointId) + " calibration started in background...";
                } else {
                    response["msg"] = "Camera " + std::to_string(camera_id) + " calibration started in background...";
                }
            } else {
                response["msg"] = "All cameras calibration are starting in background...";
            }
            response["status"] = 200;
            is_patrolling = true; // 标定占用全局状态，使 action/stopwhen 能正确感知标定进行中

            // 创建后台线程执行标定任务
            try {
                std::thread calibrate_thread([this, currentReqId, currentCameraId, currentPointId]() {
                    // 如果指定了有效的摄像机ID，则只标定该摄像机；否则标定所有摄像机
                    if (currentCameraId > 0 && cameras.find(currentCameraId) != cameras.end()) {
                        auto res = calibrate(currentCameraId, currentPointId); // 标定指定摄像机（可选指定点位）
                        // 标定完成后构造JSON响应
                        nlohmann::json result;
                        result["status"] = 200;
                        if (currentPointId > 0) {
                            result["message"] = "Camera " + std::to_string(currentCameraId) + " point " + std::to_string(currentPointId) + " calibration completed successfully";
                        } else {
                            result["message"] = "Camera " + std::to_string(currentCameraId) + " calibration completed successfully";
                        }
                        result["cmd"] = "calibrate";
                        result["reqId"] = currentReqId; // 添加reqId到响应中
                        std::cout << result.dump() << std::endl;
                    } else {
                        // 如果没有指定有效的摄像机ID，则标定所有摄像机
                        auto res = calibrate(-1); // -1表示标定所有摄像机
                        // 标定完成后构造JSON响应
                        nlohmann::json result;
                        result["status"] = 200;
                        result["message"] = "All cameras calibration completed successfully";
                        result["cmd"] = "calibrate";
                        result["reqId"] = currentReqId; // 添加reqId到响应中
                        std::cout << result.dump() << std::endl;
                    }
                    is_patrolling = false; // 标定结束释放全局状态
                });
                calibrate_thread.detach();
            } catch (const std::system_error& e) {
                response["status"] = 500;
                response["msg"] = "Failed to create calibration thread: " + std::string(e.what());
                ALOGE("RThread creation failed: %s", e.what());
                is_patrolling = false; // 线程创建失败也要释放状态
            }
        } else { //unknown action
            response["status"] = "fail";
            response["message"] = "unknown cmd.";
        }
Finish:
        WTALOGI("指令返回内容: %s", response.dump().c_str());
        std::cout << response.dump() << std::endl;
        WTALOGI("继续等待接收下一条指令...");
    }

    return 0;
}

int CameraController::all_cameras_patrol()
{
    reload_config(); // 巡检前热加载配置（点位+基础字段）

    // 统一本轮巡检时间基准：所有相机共用同一时间戳，避免各相机启动时刻恰好跨分钟而落到不同目录。
    time_t unified_start = time(nullptr);

    // 区分大云台/小云台；小云台才需要组内互检与轮流监视
    std::vector<Camera*> big_cams;
    std::vector<Camera*> small_cams;
    for (auto& pair : cameras) {
        if (!pair.second->m_pipeline)
            continue;
        if (pair.second->ptz_type == "small")
            small_cams.push_back(pair.second);
        else
            big_cams.push_back(pair.second);
    }

    // 小云台按 ptz_ip 排序后两两相邻成组
    auto ip_to_tuple = [](const std::string& ip) {
        int a = 0, b = 0, c = 0, d = 0;
        sscanf(ip.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d);
        return std::make_tuple(a, b, c, d);
    };

    std::sort(small_cams.begin(), small_cams.end(),
              [&](Camera* x, Camera* y) { return ip_to_tuple(x->ptz_ip) < ip_to_tuple(y->ptz_ip); });

    // 大云台并行巡检
    std::vector<std::future<int>> big_futures;
    for (auto* cam : big_cams) {
        big_futures.push_back(std::async(std::launch::async, [cam, unified_start]() {
            return cam->patrol_with_calibration_loop(false, unified_start);
        }));
    }

    // 小云台按组成对并行：先互检，再交替巡检/监视
    std::vector<std::future<int>> small_futures;
    for (size_t i = 0; i + 1 < small_cams.size(); i += 2) {
        Camera* a = small_cams[i];
        Camera* b = small_cams[i + 1];
        small_futures.push_back(std::async(std::launch::async, [a, b, unified_start]() -> int {
            // 1) 轮流互检：a 作检查方检视 b，再 b 作检查方检视 a
            int ra = a->inspect_peer(b, unified_start);
            if (ra != 0) {
                WTALOGI("小云台组 %s(检查) / %s(被检查) 互检未通过（%d），跳过本组巡检",
                        a->getName().c_str(), b->getName().c_str(), ra);
                return 1;
            }
            int rb = b->inspect_peer(a, unified_start);  // b检查a
            if (rb != 0) {
                WTALOGI("小云台组 %s(检查) / %s(被检查) 互检未通过（%d），跳过本组巡检",
                        b->getName().c_str(), a->getName().c_str(), rb);
                return 1;
            }

            // 2) 第一轮：a 巡检，b 定检
            std::atomic<bool> a_patrol_done(false);
            std::future<int> f_patrol = std::async(std::launch::async, [a, unified_start, &a_patrol_done]() {
                int r = a->patrol_with_calibration_loop(false, unified_start);
                a_patrol_done.store(true);
                return r;
            });
            std::future<int> f_monitor = std::async(std::launch::async, [b, unified_start, &a_patrol_done]() {
                return b->monitor_at_point(a_patrol_done, unified_start);
            });
            int r1 = f_patrol.get();
            int r2 = f_monitor.get();
            if (r1 != 0 || r2 != 0) {
                WTALOGI("小云台组 %s / %s 第一轮巡检/监视异常（%d/%d）",a->getName().c_str(), b->getName().c_str(), r1, r2);
                return 1;
            }

            // 3) 第二轮：b 巡检，a 定检
            std::atomic<bool> b_patrol_done(false);
            f_patrol = std::async(std::launch::async, [b, unified_start, &b_patrol_done]() {
                int r = b->patrol_with_calibration_loop(false, unified_start);
                b_patrol_done.store(true);
                return r;
            });
            f_monitor = std::async(std::launch::async, [a, unified_start, &b_patrol_done]() {
                return a->monitor_at_point(b_patrol_done, unified_start);
            });
            r1 = f_patrol.get();
            r2 = f_monitor.get();
            if (r1 != 0 || r2 != 0) {
                WTALOGI("小云台组 %s / %s 第二轮巡检/监视异常（%d/%d）",
                        a->getName().c_str(), b->getName().c_str(), r1, r2);
                return 1;
            }
            return 0;
        }));
    }

    // 若小云台数量为奇数，最后一台单独按普通巡检处理
    if (small_cams.size() % 2 == 1) {
        Camera* last = small_cams.back();
        big_futures.push_back(std::async(std::launch::async, [last, unified_start]() {
            return last->patrol_with_calibration_loop(false, unified_start);
        }));
    }

    // 等待所有任务完成并汇总结果
    int res = 0;
    for (auto& f : big_futures)    if (f.get() != 0) res = 1;
    for (auto& f : small_futures)  if (f.get() != 0) res = 1;

    return res;
}

int CameraController::calibrate(int camera_id, int target_point_id) //return 0正常  非0异常
{
    reload_config(); // 标定前热加载配置（点位+基础字段）

    int res = 0;
    // 如果指定了有效的摄像机ID，则只标定该摄像机；否则标定所有摄像机
    if (camera_id > 0 && cameras.find(camera_id) != cameras.end()) {
        Camera* camera = getCamera(camera_id);
        // target_point_id>0 时仅标定指定点位；否则标定该相机全部点位
        res =  camera->patrol_with_calibration_loop(true, 0, target_point_id);
    } else {
        // 遍历所有摄像机，设置标定模式并启动标定（指定点位仅对单相机生效，此处忽略）
        for (auto& pair : cameras) {
            res |= pair.second->patrol_with_calibration_loop(true);
        }
    }
    return res;
}

int CameraController::load_config_from_file(const std::string& config_file_path)
{
    try {
        // 读取配置文件
        std::ifstream config_file(config_file_path);
        if (!config_file.is_open()) {
            std::cerr << "Failed to open config file: " << config_file_path << std::endl;
            return -1;
        }

        // 解析 JSON...
        nlohmann::json config;
        config_file >> config;

        if (config.contains("cooldown")){
            alarm_manager.cooldown = config["cooldown"]; //单位 小时
        }

        std::string orga_name="";
        if (config.contains("org_name")) {
            orga_name = config["org_name"];
        }

        // mp4文件模拟URL
        std::ifstream test_config_file("/wt_tech/app/ax-pipeline/config/wt_rtsp.json");
        if (!test_config_file.is_open()) {
            std::cerr << "Failed to open config file: " << std::endl;
            return -1;
        }
        nlohmann::json test_config;
        test_config_file >> test_config;
        int camera_test_id = 1;
        if (test_config.contains("RTSP_LIST") && test_config["RTSP_LIST"].is_array()) {
            for (const auto& url_file : test_config["RTSP_LIST"]) {
                Camera* camera = new Camera();
                camera->set_camera_rtsp_url(url_file);
                int camera_test_id2 = 1;
                for (const auto& camera_config : config["chl_list"]) {
                    auto type = camera_config["type"];
                    auto enable = camera_config["enable"];
                    if (type != "Webcam" || enable != "1") {//该通道不是相机或使能关闭 跳过该设备解析
                        WTALOGI("不是相机或使能关闭 跳过该设备解析");
                        continue;
                    }
                    if (camera_test_id == camera_test_id2) {
                        camera->id = std::stoi(camera_config["id"].get<std::string>());
                        camera->name = camera_config["name"];
                        if (camera_config.contains("ip")) {
                            camera->ip = camera_config["ip"];
                        }
                    }
                    camera_test_id2++;
                }
                cameras[camera->id] = camera; // 将相机实例添加到相机列表中
                camera_test_id++;
            }

        } else if (config.contains("chl_list") && config["chl_list"].is_array()) { // 遍历相机列表
            for (const auto& camera_config : config["chl_list"]) {
                auto type = camera_config["type"];
                auto enable = camera_config["enable"];
                if (type != "Webcam" || enable != "1") {//该通道不是相机或使能关闭 跳过该设备解析
                    WTALOGI("不是相机或使能关闭 跳过该设备解析");
                    continue;
                }

                // 创建相机实例
                Camera* camera = new Camera();

                // 设置相机基本信息
                camera->id = std::stoi(camera_config["id"].get<std::string>());

                camera->name = camera_config["name"];

                camera->orga_name = orga_name;

                // 云台大小类型（big/small），影响点位 y 值 clamp 范围
                if (config.contains("ptz_type")) {
                    camera->ptz_type = config["ptz_type"];
                }

                // 小云台互检/监视点位配置
                if (camera_config.contains("peer_check_point_id")) {
                    camera->peer_check_point_id = std::stoi(camera_config["peer_check_point_id"].get<std::string>());
                }
                if (camera_config.contains("monitor_point_id")) {
                    camera->monitor_point_id = std::stoi(camera_config["monitor_point_id"].get<std::string>());
                }

                // 设置 PTZ IP
                if (camera_config.contains("ptz_ip")) {
                    camera->ptz_ip = camera_config["ptz_ip"];
                }

                if (camera_config.contains("ip")) {
                    camera->ip = camera_config["ip"];
                    std::string camera_rtsp_url = "rtsp://admin@" + camera->ip + ":8554/onvif1";
                    if (camera->ptz_ip.empty()) {
                        camera_rtsp_url = "rtsp://admin@" + camera->ip + "/channel=1&stream=0.sdp?";
                    }
                    camera->set_camera_rtsp_url(camera_rtsp_url);
                }

                // 从配置文件中读取并加载点位列表
                if (camera_config.contains("points") && camera_config["points"].is_array()) {
                    std::vector<Camera::PresetPosition> preset_positions;
                    for (const auto& point : camera_config["points"]) {
                        Camera::PresetPosition pos;
                        pos.id = std::stoi(point["id"].get<std::string>());
                        pos.name = point["name"];
                        pos.distance = point["dist"];
                        pos.duration = point["duration"];
                        pos.web_rotation_x = point["rotatex"];
                        pos.web_rotation_y = point["rotatey"];
                        pos.zoom = point["zoom"];
                        pos.focus = point["focus"];
                        pos.brightness = point["brightness"];
                        // 该点位专用模型（可选）：pointModels 为逗号分隔的模型名字符串，在通用模型之上额外叠加
                        if (point.contains("pointModels") && point["pointModels"].is_string()) {
                            parse_point_models(point["pointModels"].get<std::string>(), pos.models);
                        }
                        camera->add_preset_position(pos); // 设置点位信息
                    }
                }

                // 将相机添加到控制器
                cameras[camera->id] = camera;
                WTALOGI("相机[id:%d]构造完成:ip:%s, ptz_ip:%s, 点位数量为:%d",camera->id, camera->ip.c_str(),
                  camera->ptz_ip.c_str(), camera->preset_positions.size());
            }
        }
        WTALOGI("完成配置加载!");

        // 记录初始 mtime，用于后续热加载判断
        struct stat st;
        if (stat(config_file_path.c_str(), &st) == 0) {
            last_config_mtime = st.st_mtime;
        }
        return 0;
    } catch (const std::exception& e) {
        WTALOGI("Error loading config:%s",  e.what());
        return -1;
    }
}

int CameraController::reload_config()
{
    // 通过 mtime 判断配置是否变化，未变化则跳过
    struct stat st;
    if (stat(CONFIG_FILE_PATH, &st) != 0) {
        WTALOGI("热加载: 无法获取配置文件状态: %s", CONFIG_FILE_PATH);
        return -1;
    }
    if (st.st_mtime == last_config_mtime) {
        return 0; // 文件未变化
    }

    try {
        std::ifstream config_file(CONFIG_FILE_PATH);
        if (!config_file.is_open()) {
            WTALOGI("热加载: 打开配置文件失败");
            return -1;
        }

        nlohmann::json config;
        config_file >> config;

        // 全局字段：告警冷却时间
        if (config.contains("cooldown")) {
            alarm_manager.cooldown = config["cooldown"];
        }

        // 全局字段：风场名称
        std::string orga_name = "";
        if (config.contains("org_name")) {
            orga_name = config["org_name"];
        }

        if (!config.contains("chl_list") || !config["chl_list"].is_array()) {
            WTALOGI("热加载: chl_list 不存在或非数组");
            return -1;
        }

        int updated_count = 0;
        for (const auto& camera_config : config["chl_list"]) {
            auto type = camera_config["type"];
            auto enable = camera_config["enable"];
            if (type != "Webcam" || enable != "1") {
                continue;
            }

            int cam_id = std::stoi(camera_config["id"].get<std::string>());
            auto it = cameras.find(cam_id);
            if (it == cameras.end()) {
                WTALOGI("热加载: 相机[%d]未在初始配置中，跳过", cam_id);
                continue;
            }
            Camera* camera = it->second;

            // 巡检中的相机跳过，避免数据竞争
            if (camera->is_patroling()) {
                WTALOGI("热加载: 相机[%d]巡检中，跳过点位热加载", cam_id);
                continue;
            }

            // 更新基础字段
            camera->name = camera_config["name"];
            camera->orga_name = orga_name;
            if (camera_config.contains("ptz_ip")) {
                camera->ptz_ip = camera_config["ptz_ip"];
            }
            if (config.contains("ptz_type")) {
                camera->ptz_type = config["ptz_type"];
            }

            // 检测 IP 变化（本方案不重连 RTSP，仅记录日志提示）
            if (camera_config.contains("ip")) {
                std::string new_ip = camera_config["ip"];
                if (new_ip != camera->ip) {
                    WTALOGI("热加载: 相机[%d] IP 变化 %s -> %s，需要重启生效",
                            cam_id, camera->ip.c_str(), new_ip.c_str());
                }
            }

            // 重建点位列表
            std::vector<Camera::PresetPosition> new_positions;
            if (camera_config.contains("points") && camera_config["points"].is_array()) {
                for (const auto& point : camera_config["points"]) {
                    Camera::PresetPosition pos;
                    pos.id = std::stoi(point["id"].get<std::string>());
                    pos.name = point["name"];
                    pos.distance = point["dist"];
                    pos.duration = point["duration"];
                    pos.web_rotation_x = point["rotatex"];
                    pos.web_rotation_y = point["rotatey"];
                    pos.zoom = point["zoom"];
                    pos.focus = point["focus"];
                    pos.brightness = point["brightness"];
                    // 该点位专用模型（可选）：pointModels 为逗号分隔的模型名字符串，在通用模型之上额外叠加
                    if (point.contains("pointModels") && point["pointModels"].is_string()) {
                        parse_point_models(point["pointModels"].get<std::string>(), pos.models);
                    }
                    camera->clamp_preset_y(pos);
                    new_positions.push_back(pos);
                }
            }
            camera->preset_positions = std::move(new_positions);

            WTALOGI("热加载: 相机[%d]更新完成, 点位数量:%d",
                    cam_id, (int)camera->preset_positions.size());
            updated_count++;
        }

        last_config_mtime = st.st_mtime;
        WTALOGI("热加载完成: 共更新 %d 个相机", updated_count);
        return 0;
    } catch (const std::exception& e) {
        WTALOGI("热加载失败: %s", e.what());
        return -1;
    }
}

void CameraController::remove_all_cameras()
{
    for (auto& camera : cameras) {
        delete camera.second;
    }
    cameras.clear();
}

int CameraController::start()
{
    WTALOGI("启动相机控制器...");
    running = true; // 标记控制器为运行状态

    // 创建线程并行启动所有摄像机
    std::vector<std::thread> start_threads;
    for (auto& camera : cameras) {
        start_threads.emplace_back([camera = camera.second]() {
            if (camera)
                camera->start(); // 启动相机
        });
    }

    // 等待所有相机启动完成
    for (auto& thread : start_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    // 创建并启动一个新线程来执行receive_input函数
    input_thread = std::thread(&CameraController::receive_input_loop, this);

    return 0;
}

int CameraController::stop()
{
    running = false; // 标记控制器为停止状态

    if (input_thread.joinable()) {
        pthread_cancel(reinterpret_cast<pthread_t>(input_thread.native_handle()));
        input_thread.join();
    }
    // 停止所有摄像机
    for (auto camera : cameras) {
        camera.second->pause(); // 停止相机
    }
    return 0;
}

bool CameraController::early_warning_process(int camera_id, int point_id, int light_flag, const std::string& damage_type, float confidence)
{
    auto camera = getCamera(camera_id);
    return alarm_manager.generateAlarm(AlarmType::LINE_CROSSING, damage_type, confidence, camera, point_id, light_flag);
}

Camera *CameraController::getCamera(int camera_id)
{
    auto it = cameras.find(camera_id);
    if (it != cameras.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<Camera*> CameraController::getAllCameras()
{
    std::vector<Camera*> camera_list;
    for (const auto& pair : cameras) {
        camera_list.push_back(pair.second);
    }
    return camera_list;
}

/* ================================== */

Camera::Camera()
{
    WTALOGI("相机构建中...");

    curl_handle = curl_easy_init();
    // 设置请求基本通用选项
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 5L);  // 5秒超时
    curl_easy_setopt(curl_handle, CURLOPT_FOLLOWLOCATION, 1L);  // 跟随重定向

    // 设置Basic认证
    std::string auth = "admin:12345";
    curl_easy_setopt(curl_handle, CURLOPT_USERPWD, auth.c_str());
    curl_easy_setopt(curl_handle, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
}

Camera::~Camera()
{
    // 通知并 join 后台 modbus 重连线程，避免线程访问已析构的 modbus_ctx / this
    m_modbus_reconnect_exit.store(true);
    if (m_modbus_reconnect_thread.joinable()) {
        m_modbus_reconnect_thread.join();
    }
    // 关闭 modbus_ctx（此时其他线程已停）
    {
        std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
        if (modbus_ctx) {
            modbus_close(modbus_ctx);
            modbus_free(modbus_ctx);
            modbus_ctx = nullptr;
        }
    }

    if (curl_handle) {
        curl_easy_cleanup(curl_handle);
        curl_handle = nullptr;
    }
}

int Camera::start()
{
    WTALOGI("摄像机[%d]启动...ip:%s, 点位数：%d", id, ip.c_str(), this->preset_positions.size());
    running = true;
    auto res_code = 1;

    auto rc = connect_modbus();  //云台连接

    // 启动后台 modbus 重连线程：业务函数只置 m_modbus_broken 标志，重连由此线程异步处理
    if (!m_modbus_reconnect_thread.joinable() && !ptz_ip.empty()) {
        m_modbus_reconnect_exit.store(false);
        m_modbus_reconnect_thread = std::thread(&Camera::_modbus_reconnect_loop, this);
    }

    fetch_remote_status();

    if (rc == false)     {
        res_code = 0;
    }
    return res_code;
}

int Camera::pause()
{
    running = false;

    return 0;
}

bool Camera::connect_modbus()
{
    if (ptz_ip.empty())
        return false;

    // 整个 connect_modbus 期间独占 modbus_ctx，避免其他线程正在 IO 时被 close/free
    std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);

    uint32_t to_sec = 10; // 超时秒数

    // 清理旧连接
    if (modbus_ctx != nullptr) {
        WTALOGI("将重连接云台[%s]...", ptz_ip.c_str());
        modbus_close(modbus_ctx);
        modbus_free(modbus_ctx);
    } else {
        WTALOGI("将连接云台[%s](modbus)...", ptz_ip.c_str());
        to_sec = 5; // 超时秒数
    }

    // 初始化Modbus 连接
    modbus_ctx = nullptr;
    modbus_ctx = modbus_new_tcp(ptz_ip.c_str(), MODBUSPORT);
    if (!modbus_ctx) {
        WTALOGI("连接云台[%s]失败！", ptz_ip.c_str());
        modbus_ctx = nullptr;
    } else {
        // 设置 Modbus 超时时间
        modbus_set_response_timeout(modbus_ctx, to_sec, 0);
        modbus_set_slave(modbus_ctx, 1);

        // 连接到 Modbus 服务器
        if (modbus_connect(modbus_ctx) == -1) {
            WTALOGI("连接云台[%s]失败！,原因:%s", ptz_ip.c_str(), modbus_strerror(errno) );
            modbus_free(modbus_ctx);
            modbus_ctx = nullptr;
        } else {
            // 设置 Modbus 从站 ID（根据实际情况调整）
            //modbus_set_slave(modbus_ctx, 1);
            m_modbus_broken.store(false); // 重连成功，清除损坏标志
            return true;
        }
    }

    return false;
}

// 后台 modbus 重连线程：观察 m_modbus_broken 标志，异步重连并带冷却，
// 避免业务函数（如 update_posture_state / fetch_remote_status）在 200ms 轮询里
// 同步调 connect_modbus 阻塞整个循环。
void Camera::_modbus_reconnect_loop()
{
    WTALOGI("摄像机[%d] modbus 后台重连线程启动", id);
    while (!m_modbus_reconnect_exit.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (m_modbus_reconnect_exit.load()) break;

        if (!m_modbus_broken.load()) continue;

        WTALOGI("摄像机[%d] 检测到 modbus 断开标志，尝试后台重连...", id);
        bool ok = connect_modbus();
        if (ok) {
            WTALOGI("摄像机[%d] modbus 后台重连成功", id);
        } else {
            // 失败：叠加 4s 冷却，避免高频重连
            for (int i = 0; i < 4 && !m_modbus_reconnect_exit.load(); i++) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    WTALOGI("摄像机[%d] modbus 后台重连线程退出", id);
}

int Camera::get_id() const
{
    return id;
}

bool Camera::is_patroling() const
{
    //WTALOGI("查看是否处于巡检状态:%s", patrolling? "是":"否");
    return patrolling;
}

void Camera::finish_patrolling()
{
    patrolling = false;
    calibrating.store(false); // 退出巡逻/标定，清除标定标志
    WTALOGI("摄像机[%d]完成巡逻!", this->id);
}

// 计算两个检测框的 IoU
static float compute_iou(const axdl_bbox_t& a, const axdl_bbox_t& b)
{
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.w, b.x + b.w);
    float y2 = std::min(a.y + a.h, b.y + b.h);
    float inter = std::max(0.f, x2 - x1) * std::max(0.f, y2 - y1);
    float area_a = a.w * a.h;
    float area_b = b.w * b.h;
    return inter / (area_a + area_b - inter + 1e-6f);
}

void Camera::accumulate_detection(const axdl_results_t* results)
{
    if (!results || results->nObjSize <= 0) return;

    std::lock_guard<std::mutex> lk(accumulated_mutex);
    for (int i = 0; i < results->nObjSize; i++) {
        const axdl_object_t& obj = results->mObjects[i];

        // 检查是否与已有目标重复（同类型 + IoU > 0.5）
        bool merged = false;
        for (auto& existing : accumulated_objects) {
            if (strcmp(existing.objname, obj.objname) == 0 &&
                compute_iou(existing.bbox, obj.bbox) > 0.5f) {
                // 保留置信度更高的
                if (obj.prob > existing.prob) {
                    existing = obj;
                }
                merged = true;
                break;
            }
        }
        if (!merged && accumulated_objects.size() < SAMPLE_MAX_BBOX_COUNT) {
            accumulated_objects.push_back(obj);
        }
    }
}

void Camera::clear_accumulated_detections()
{
    std::lock_guard<std::mutex> lk(accumulated_mutex);
    accumulated_objects.clear();
}

std::vector<axdl_object_t> Camera::get_accumulated_objects()
{
    std::lock_guard<std::mutex> lk(accumulated_mutex);
    return accumulated_objects; // 返回拷贝
}

int Camera::clamp_y_angle(int orig) const
{
    // 硬件可动范围（前端约定：负=上仰，正=俯下）
    //   big:   俯下 90 + 上仰 40 → y ∈ [-40, 90]
    //   small: 俯下 90 + 上仰 90 → y ∈ [-90, 90]（默认）
    int lo = (ptz_type == "big") ? -40 : -90;
    int hi = 90;

    // 前端可能传任意角度值（例如 320 实际等价 -40，即上仰 40°）。
    // 先归一化到 (-180, 180]，让同一姿态只有一种数值表达；再按硬件可动范围 clamp。
    int y = orig % 360;       // 归一化到 (-360, 360)
    if (y > 180)        y -= 360;  // (180, 360)   -> (-180, 0)
    else if (y < -180) y += 360;  // (-360, -180] -> (0, 180]

    if (y < lo)      y = lo;
    else if (y > hi) y = hi;
    return y;
}

void Camera::clamp_preset_y(Camera::PresetPosition& pos) const
{
    int orig = pos.web_rotation_y;
    int adjusted = clamp_y_angle(orig);

    if (adjusted != orig) {
        int lo = (ptz_type == "big") ? -40 : -90;
        WTALOGI("摄像机[%d] 点位[%d] y=%d 归一化并 clamp 到 %s 云台范围[%d,%d] -> %d",
                id, pos.id, orig, ptz_type.c_str(), lo, 90, adjusted);
    }
    pos.web_rotation_y = adjusted;
}

int Camera::add_preset_position(Camera::PresetPosition pos)
{
    clamp_preset_y(pos);
    this->preset_positions.push_back(pos);
    return 0;
}

void Camera::setPipe(pipeline_t * pipe)
{
    WTALOGI("摄像机[%d]绑定pipeline[%d]",id, pipe->pipeid);
    m_pipeline = pipe;
    snprintf(m_pipeline->channel_name, sizeof(m_pipeline->channel_name), "%s", name.c_str()); // 设置通道名称
}

bool Camera::start_record_video()
{
    WTALOGI("摄像机[%d]启动录像", id);

    // 此处在 IsRecordVideo=true 之前重置，VENC 线程尚未进入录制分支，无竞态。
    m_pipeline->max_memory_limit = 0;

    m_pipeline->IsRecordVideo = true; // 标识开始录像
    return false;
}

bool Camera::stop_record_video()
{
    m_pipeline->IsRecordVideo = false; // 停止录像标志
    WTALOGI("摄像机[%d]停止持续录像", id);
    return false;
}

// ===== 损伤片段独立录像：转发到 pipeline 状态机 =====
// 这些符号定义在 sample_multi_demux_ivps_npu_multi_rtsp.cpp，由链接器解析
extern void damage_pipeline_on_arrived(pipeline_t* pipe, const char* clip_filename);
extern void damage_pipeline_on_leaving(pipeline_t* pipe);
extern void damage_pipeline_mark_seen(pipeline_t* pipe);

void Camera::mark_damage_seen()
{
    if (!m_pipeline) return;
    damage_pipeline_mark_seen(m_pipeline);
}

void Camera::on_arrived_at_point()
{
    if (!m_pipeline) return;
    // 生成损伤片段输出文件路径，写入 pipeline，再驱动状态机切到 STAYING
    std::string clip_path = generateCustomVideoPath(VideoPathType::DAMAGE_CLIP);
    if (clip_path.empty()) {
        WTALOGI("[Camera-%d] 生成损伤片段路径失败，跳过本点位损伤录像", id);
        return;
    }
    damage_pipeline_on_arrived(m_pipeline, clip_path.c_str());
}

void Camera::on_leaving_point()
{
    if (!m_pipeline) return;
    damage_pipeline_on_leaving(m_pipeline);
}

//0:不拍 1：标定 2：巡检
bool Camera::start_take_a_picture(int kind)
{
    m_pipeline->whatPicture = kind; // 标识拍照
    return false;
}

void Camera::prepare_snapshot_dir()
{
    // 使用巡检开始时冻结的时间戳（系统时间已是东八区），保证整轮巡检所有快照在同一分钟目录下
    time_t base_time = patrol_start_time > 0 ? patrol_start_time : time(nullptr);
    struct tm tmbuf;
    localtime_r(&base_time, &tmbuf);  // 线程安全：使用私有 tm 缓冲区
    char ymd[16] = {0};
    snprintf(ymd, sizeof(ymd), "%04d%02d%02d", tmbuf.tm_year + 1900, tmbuf.tm_mon + 1, tmbuf.tm_mday);

    snprintf(pic_dirname, sizeof(pic_dirname), "/wt_tech/data/%s/%s/%s_%02d%02d/image",
        orga_name.c_str(), ymd, ymd, tmbuf.tm_hour, tmbuf.tm_min);
    if (access(pic_dirname, 0) != 0) {
        char mk[256] = {0};
        snprintf(mk, sizeof(mk), "mkdir -p %s", pic_dirname);
        system(mk);
    }
}

std::string Camera::captureSnapshot(const cv::Mat& image, int point_id, int light_flag)
{
    // 使用巡检开始时冻结的时间戳（系统时间已是东八区），文件名与目录保持同一时间基准
    time_t base_time = patrol_start_time > 0 ? patrol_start_time : time(nullptr);
    struct tm tmbuf;
    localtime_r(&base_time, &tmbuf);  // 线程安全：使用私有 tm 缓冲区，避免 localtime 静态缓冲区被并发覆盖
    tm *t = &tmbuf;
    char ymd[16] = {0};
    snprintf(ymd, sizeof(ymd), "%04d%02d%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

    char filepath[320] = {0};
    // 文件名加入灯光状态 L0（无灯照）或 L1（有灯照），使用传入的 point_id 而非 now_point_id
    if (light_flag >= 0) {
        snprintf(filepath, sizeof(filepath), "%s/%s_%02d%02d_%s_%s_%d_L%d.png", pic_dirname, ymd, t->tm_hour, t->tm_min,
            orga_name.c_str(), name.c_str(), point_id, light_flag);
    } else {
        snprintf(filepath, sizeof(filepath), "%s/%s_%02d%02d_%s_%s_%d.png", pic_dirname, ymd, t->tm_hour, t->tm_min,
            orga_name.c_str(), name.c_str(), point_id);
    }

    // 根据文件扩展名选择正确的压缩参数
    std::vector<int> params;
    size_t len = strlen(filepath);
    if (len >= 4 && filepath[len - 4] == '.' && filepath[len - 3] == 'p' && filepath[len - 2] == 'n' && filepath[len - 1] == 'g') {
        // PNG 文件使用 PNG 压缩参数
        params = { cv::IMWRITE_PNG_COMPRESSION, 1 };  // 压缩级别 0-9，9 为最高压缩
    } else {
        // JPEG 文件使用 JPEG 质量参数
        params = { cv::IMWRITE_JPEG_QUALITY, 90 };
    }
    // 编码+写盘交给后台线程异步执行，避免阻塞渲染线程（draw_custom）造成拍照瞬间丢帧。
    // clone 一份图交给队列，调用方随后可安全复用/释放 image。文件路径此处即确定并返回，
    // 不依赖写盘完成；FIFO 队列保证同路径先后写入次序（拍照图 → diff 覆盖图）。
    AsyncImageWriter::instance().enqueue(image.clone(), filepath, params);

    // 写回 pipeline，让 generateAlarm 拿到本张快照路径
    if (auto *pipe = get_pipeline()) {
        strncpy(pipe->pic_filename, filepath, sizeof(pipe->pic_filename) - 1);
        pipe->pic_filename[sizeof(pipe->pic_filename) - 1] = '\0';
    }

    return filepath;
}

std::string Camera::getName()
{
    return name;
}

void Camera::enqueue_diff_task(int point_id, int light_flag, const cv::Mat& raw_image, const std::string& display_path)
{
    if (raw_image.empty() || display_path.empty()) return;
    std::lock_guard<std::mutex> lk(m_diff_queue_mtx);
    m_diff_queue.push_back({point_id, light_flag, raw_image.clone(), display_path});
}

std::vector<Camera::PendingDiffTask> Camera::drain_diff_queue()
{
    std::lock_guard<std::mutex> lk(m_diff_queue_mtx);
    std::vector<PendingDiffTask> tmp;
    tmp.swap(m_diff_queue);
    return tmp;
}

void Camera::connectPipes(pipeline_t *pipe0, pipeline_t *pipe1)
{
    this->setPipe(pipe0);
    // 设置两个管道为同一个相机指针
    pipe0->m_pcamera = this;
    pipe1->m_pcamera = this;
}

std::string Camera::get_pic_path() const
{
    return this->m_pipeline->pic_filename; // 返回图片路径
}

pipeline_t* Camera::get_pipeline() const
{
    return this->m_pipeline; // 返回pipeline指针
}

std::string Camera::get_camera_rtsp_url()
{
    return camera_rtsp_url;
}

std::string Camera::generateCustomVideoPath(VideoPathType type= VideoPathType::VIDEO)
{
    // 优先使用巡检开始时冻结的时间戳（系统时间已是东八区），保证整轮巡检所有视频/封面/快照同目录
    time_t timeReal;
    if (patrol_start_time > 0) {
        timeReal = patrol_start_time;
    } else {
        time(&timeReal);  // 系统时间已是东八区
    }
    struct tm tmbuf;
    localtime_r(&timeReal, &tmbuf);  // 线程安全：使用私有 tm 缓冲区，避免 localtime 静态缓冲区被并发覆盖
    tm *t = &tmbuf;

    char dateStr[16] = {0};
    snprintf(dateStr, sizeof(dateStr), "%04d%02d%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

    // 生成目录路径
    std::string dirname;
    char filename[256] = {0};

    // 根据类型选择不同的基础路径
    std::string base_path;
    if (type == VideoPathType::PERSON) {
        base_path = "/wt_tech/data/video2/person";
        dirname = base_path;

        // 生成文件名 - 增加到秒级
        snprintf(filename, sizeof(filename), "%s/%s-%d-%02d-%02d_%02d%02d%02d.mp4", dirname.c_str(), name.c_str(),
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

    } else if (type == VideoPathType::DAMAGE_CLIP) {
        // 损伤片段视频：按 风场/年月日/小时/damage 分目录，文件名与快照前缀一致（不带开关灯标识）
        base_path = "/wt_tech/data/";
        char hourMinStr[8] = {0};
        snprintf(hourMinStr, sizeof(hourMinStr), "%02d%02d", t->tm_hour, t->tm_min);
        dirname = base_path + orga_name.c_str() + "/" + std::string(dateStr) + "/" + std::string(dateStr) + "_"
            + hourMinStr + "/damage";

        snprintf(filename, sizeof(filename), "%s/%s_%02d%02d_%s_%s_%d.mp4", dirname.c_str(),
            dateStr, t->tm_hour, t->tm_min, orga_name.c_str(), name.c_str(), now_point_id);
    } else {
        base_path = "/wt_tech/data/";
        char hourMinStr[8] = {0};
        snprintf(hourMinStr, sizeof(hourMinStr), "%02d%02d", t->tm_hour, t->tm_min);
        dirname = base_path + orga_name.c_str() + "/" + std::string(dateStr) + "/" + std::string(dateStr) + "_"
            + hourMinStr + "/video";

        // 生成文件名
        snprintf(filename, sizeof(filename), "%s/%d-%02d-%02d_%02d%02d_%s_%s.mp4", dirname.c_str(), t->tm_year + 1900,
            t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, orga_name.c_str(), name.c_str());
    }

    // 仅巡检主录像和人物视频写回 video_filename；损伤片段路径由调用方单独保存到 damage_clip_filename
    if (type != VideoPathType::DAMAGE_CLIP) {
        strncpy(m_pipeline->video_filename, filename, sizeof(this->m_pipeline->video_filename)-1);
        m_pipeline->video_filename[sizeof(m_pipeline->video_filename)-1] = '\0';
    }

    // 创建目录（用 std::string 拼接命令，避免长路径被固定缓冲截断）
    if (access(dirname.c_str(), 0) != 0) {
        std::string cmd = "mkdir -p " + dirname;
        system(cmd.c_str());
    }

    return filename;
}


void Camera::set_camera_rtsp_url(const std::string& url)
{
    camera_rtsp_url = url;
}

//return 0表示正常 非0表示异常
int Camera::patrol_with_calibration_loop(bool is_calibrate, time_t start_time, int target_point_id)
{
    patrolling = true; // 标识进入巡逻模式
    calibrating.store(is_calibrate); // 标定模式标志：标定时 draw_custom 跳过 diff 门控强制建/更新基线
    stop_requested.store(false); // 每轮巡检开始时清掉上一次的停止请求
    photo_fired_keys.clear(); // 清空拍照去重状态，避免跨轮次误判
    phase_infer_decision.clear(); // 清空每相位 diff 推理门控决策，避免跨轮次复用
    // 冻结本轮巡检时间戳（系统时间已是东八区），保证所有路径在同一分钟目录下。
    // 多相机并行巡检由 all_cameras_patrol 传入统一 start_time，避免各相机跨分钟落到不同目录。
    patrol_start_time = start_time > 0 ? start_time : time(nullptr);
    prepare_snapshot_dir(); // 巡检开始即固定本轮图片目录
    WTALOGI("开始巡逻...");

    int res = 0;
    std::vector<Camera::PresetPosition>::iterator first_pos = preset_positions.begin();

    // 巡检模式：开始持续录像
    if (!is_calibrate) {
        //这里设置存储的文件路径
        generateCustomVideoPath();
        start_record_video();
    }

    if (this->modbus_ctx == NULL) {
        WTALOGI("Modbus连接失败，无法进行巡检！");
        res = 1;
        sleep(5);
        goto END;
    }

    /* 这里实现点位切换逻辑
     * 例如：遍历预设点位列表，定期切换到下一个点位
     * 每个点位分为无灯照和有灯照两种方式拍快照和分析损伤告警
     */
    now_point_id = 1; // 当前所在的预置点位ID记录
    for(auto position = first_pos; position != preset_positions.end(); ++position, now_point_id++) {
        if (stop_requested.load()) {
            WTALOGI("摄像机[%d] 收到停止指令，中断巡检循环", id);
            res = 2;
            break;
        }
        // 指定点位标定：仅处理目标点位，其余跳过（不移动云台、不拍照）
        if (target_point_id > 0 && now_point_id != target_point_id) {
            continue;
        }
        this->m_pipeline->point_id = now_point_id; // 设置当前点位ID
        posture_completed = false;
        phase_ready_ms.store(0); // 移动开始，未就绪
        frame_should_capture.store(0); // 重置拍照标记

        /* 前端值映射为后端值 */
        auto px = (360+position->web_rotation_x)%360 * 100;
        auto py = (360+position->web_rotation_y)%360 * 100;

        // 清空上一个点位的累积检测结果
        clear_accumulated_detections();

        // 第一阶段：开灯拍照L0
        set_ptz(px, py, position->brightness);

        web_rotation_x = position->web_rotation_x;
        web_rotation_y = position->web_rotation_y;

        set_zoom_and_focus(position->zoom, position->focus); // 设置缩放级别

        //轮询等待姿态完成或超时
        auto start_time = std::chrono::steady_clock::now();

        // set_ptz 已启动后台线程更新姿态状态，此处只需等待 posture_completed 或超时
        while (!posture_completed && !stop_requested.load() && std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start_time).count() < 30) { // 30秒内必需转完
            interruptible_sleep_ms(200); // stop 时会被 notify 立刻唤醒
        }
        if (stop_requested.load()) {
            WTALOGI("摄像机[%d] 姿态等待期间收到停止指令", id);
            res = 2;
            break;
        }

        const char* status = posture_completed ? "已" : "无法";
        WTALOGI("摄像机[%d]状态%s切换到点位[%d]", id, status, now_point_id);
        if (!posture_completed)
            res = 1;

        if (!posture_completed) {
            // 未到位，跳过本点位
        } else if (is_calibrate) { //标定模式且到位成功
            // L0 开灯标定拍照
            if (!interruptible_sleep_ms(2500)) break; // 等待灯点亮稳定
            phase_ready_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch()).count());
            frame_should_capture.store(1); // 标记该帧应该拍 L0
            start_take_a_picture(1); //标定拍 L0
            // 等待 L0 拍照完成
            int wait_ms = 0;
            while (frame_should_capture.load() != 0 && !stop_requested.load() && wait_ms < 5000) {
                interruptible_sleep_ms(50);
                wait_ms += 50;
            }
            if (stop_requested.load()) break;

            // L1 关灯标定拍照
            set_brighten(0); // 关灯
            if (!interruptible_sleep_ms(2500)) break; // 等待灯熄灭稳定
            clear_accumulated_detections(); // L1 阶段重新累积检测结果
            phase_ready_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch()).count());
            light_phase_changed = true; // 切换到 L1 阶段
            frame_should_capture.store(2); // 标记该帧应该拍 L1
            start_take_a_picture(1); //标定拍 L1
            // 等待 L1 拍照完成后再重置
            wait_ms = 0;
            while (frame_should_capture.load() != 0 && !stop_requested.load() && wait_ms < 5000) {
                interruptible_sleep_ms(50);
                wait_ms += 50;
            }
            light_phase_changed = false; // 拍完后再重置
        } else { //巡航模式且到位成功

            // ★ 云台真正到位后再进入"到位"状态：预录缓冲最近1秒即"到位前1秒"
            // 避免把上个点位到当前点位的云台移动过程录入损伤片段
            on_arrived_at_point();

            // 记录到位时刻，用于补足剩余停留时间
            auto point_start = std::chrono::steady_clock::now();

            // 先等待画面稳定，再设置拍照条件（避免在画面未稳定时触发拍照）
            // 确保从 phase_ready_ms 设置到推理开始有足够时间差
            // 避免推理开始时刻离就绪时刻太近导致 phase_settled 判断失败
            if (!interruptible_sleep_ms(800)) break;

            // 标记 L0 相位就绪：此后 draw_custom 等待流延迟余量即可开始累积/拍照
            phase_ready_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch()).count());

            frame_should_capture.store(1); // 标记该帧应该拍 L0
            light_phase_changed = false;
            WTALOGI("摄像机[%d] 点位[%d] 开灯拍摄 L0", id, now_point_id);

            // 有灯照拍照：等待推理线程设置 frame_should_capture = 0
            // 开灯(L0)占 3/4，关灯(L1)占 1/4
            int l0_duration = std::max(1, position->duration * 3 / 4);
            int l1_duration = std::max(1, position->duration - l0_duration);
            int wait_ms = 0;
            const int l0_wait_ms = l0_duration * 1000;
            while (frame_should_capture.load() != 0 && !stop_requested.load()) {
                interruptible_sleep_ms(50); // stop 时会被 notify 立刻唤醒
                wait_ms += 50;
            }

            if (stop_requested.load()) break;

            if (wait_ms < l0_wait_ms) {
                // L0 拍完后继续等待剩余时间，确保有灯照阶段占满 3/4 duration 秒
                if (!interruptible_sleep_ms(l0_wait_ms - wait_ms)) break;
            }

            // 第二阶段：关灯拍照（L1 = 无灯照）
            set_brighten(position->brightness/4); // 接近关灯状态

            clear_accumulated_detections(); // L1 阶段重新累积检测结果（必须在 phase_ready_ms 之前）

            // 标记 L1 相位就绪
            phase_ready_ms.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch()).count());

            // 请求关灯后 稍微等待800ms
            wait_ms = 0;
            interruptible_sleep_ms(800);
            wait_ms += 800;

            frame_should_capture.store(2); // 标记该帧应该拍 L1
            light_phase_changed = true; // 灯已关闭，允许 draw_custom 触发无灯照拍照
            WTALOGI("摄像机[%d] 点位[%d] 关灯拍摄 L1", id, now_point_id);

            // 无灯照拍照：等待绘制线程设置 frame_should_capture = 0
            const int l1_wait_ms = l1_duration * 1000;
            while (frame_should_capture.load() != 0 && !stop_requested.load()) {  // stop 时会被 notify 立刻唤醒
                interruptible_sleep_ms(50);
                wait_ms += 50;
            }

            if (stop_requested.load()) break;

            if (wait_ms < l1_wait_ms) {
                // L1 拍完后继续等待剩余时间，确保无灯照阶段占满 1/4 duration 秒
                if (!interruptible_sleep_ms(l1_wait_ms - wait_ms)) break;
            }

            // 补足剩余停留时间，确保录像达到配置的 duration 秒
            int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - point_start).count();
            int remaining_ms = position->duration * 1000 - elapsed_ms;
            if (remaining_ms > 0) {
                WTALOGI("摄像机[%d] 点位[%d] 补足剩余停留时间 %d ms", id, now_point_id, remaining_ms);
                if (!interruptible_sleep_ms(remaining_ms)) break;
            }

            // ★ 每个点位结束时进入"离开"状态，触发该点位的损伤视频落盘
            on_leaving_point();
        }

        if (is_calibrate)
            if (!interruptible_sleep_ms(2000)) break; //进行标定的话可以快速切换点位
    } //point finished

    // 兜底：若因 stop 中途 break 跳过了点位末尾的 on_leaving_point，
    // 此处再触发一次，确保进行中的损伤片段（DC_STAYING）能进入 POST 并落盘。
    // on_leaving_point 幂等：仅当状态为 DC_STAYING 时才生效。
    if (!is_calibrate) {
        on_leaving_point();
    }

    // 所有点位巡检完成：立即关闭巡逻状态，释放 NPU/CPU 给后续差异对比算法
    // （ai_inference_func 通过 is_patroling() 判断是否跑模型）
    finish_patrolling();

    if (stop_requested.load()) {
        WTALOGI("摄像机[%d] 巡检被停止指令中断，跳过回位并关闭补光灯", id);
        set_brighten(0); // 中断时主动关灯，避免补光灯持续点亮
        res = 2;
    }

    // 完成最后一个点位， 回到首个点位， 并关闭灯（被停止则跳过回位）
    if (stop_requested.load()) {
        // 被停止：保持当前位置，前面已 set_brighten(0)，这里不做回位也不延时
    } else if (first_pos != preset_positions.end()) {
        now_point_id = 0; // 设为0表示回位中，OSD不显示点位信息
        this->m_pipeline->point_id = 0;

        posture_completed = false; // 标记未到位，防止回位过程中触发拍照覆盖已有图片
        phase_ready_ms.store(0);   // 回位中，未就绪
        frame_should_capture.store(0); // 重置拍照标记
        auto px = (360 + first_pos->web_rotation_x)%360 * 100;
        auto py = (360 + first_pos->web_rotation_y)%360 * 100;
        set_ptz(px, py, 0); // 回位时关闭灯光
        web_rotation_x = first_pos->web_rotation_x;
        web_rotation_y = first_pos->web_rotation_y;
        set_zoom_and_focus(first_pos->zoom, first_pos->focus); // 设置缩放级别

        WTALOGI("摄像机[%d]巡检结束，回到起始位置并关闭灯光", id);
    } else {
        interruptible_sleep_ms(5000);  // 没有点位，模拟等待 5 秒（可被 stop 打断）
    }

END:
    // 结束录像
    if (!is_calibrate) {
        stop_record_video();
    }

    // 异常路径兜底：goto END 跳转可能未经过上面的 finish_patrolling()，此处保证状态归位
    if (patrolling) {
        finish_patrolling();
    }

    // 巡检结束后批量做点位前后对比（同光照↔同光照），
    // 命中差异即生成告警 + 标注对比图，避免占用 OSD/录像热路径
    // 标定模式(is_calibrate=true)时更新基线，巡检模式时不更新
    // 注意：被 stop 中断时跳过该重型计算，保证停止指令尽快完成
    if (!stop_requested.load()) {
        run_post_patrol_diff(this, is_calibrate);
    } else {
        WTALOGI("摄像机[%d] 被停止中断，跳过 run_post_patrol_diff 以尽快退出", id);
        drain_diff_queue(); // 仍然清空队列，避免下轮残留
    }

    // 等待本轮异步快照全部落盘，保证下游（上传/展示）能读到完整文件
    flush_pending_snapshots();

    return res;
}

int Camera::find_peer_check_point_id() const
{
    if (peer_check_point_id > 0) return peer_check_point_id;
    for (const auto& p : preset_positions) {
        if (p.name.find("互检") != std::string::npos) return p.id;
    }
    return -1;
}

int Camera::move_to_random_posture()
{
    if (modbus_ctx == nullptr) {
        WTALOGI("摄像机[%d] Modbus 未连接，无法转随机姿态", id);
        return 1;
    }
    // 以当前姿态为基准，取一个足够可见但受限的随机偏移，保证物理上确实发生转动
    auto rand_delta = []() {
        int mag = 5 + (std::rand() % 16);        // 5~20 度
        return (std::rand() % 2 == 0) ? mag : -mag;
    };
    int nx = ((web_rotation_x + rand_delta()) % 360 + 360) % 360;
    int ny = web_rotation_y + rand_delta();
    if (ny < 0)  ny = 0;
    if (ny > 85) ny = 85;   // 垂直角度限幅，避免越界
    int rand_bright = 200;  //

    posture_completed = false;
    auto px = (360 + nx) % 360 * 100;
    auto py = (360 + ny) % 360 * 100;
    set_ptz(px, py, rand_bright);
    web_rotation_x = nx;
    web_rotation_y = ny;

    auto wait_start = std::chrono::steady_clock::now();
    while (!posture_completed && !stop_requested.load() &&
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - wait_start).count() < 30) {
        interruptible_sleep_ms(200);
    }
    if (stop_requested.load()) return 2;
    if (!posture_completed) {
        WTALOGI("摄像机[%d] 被检查方转随机姿态超时", id);
        return 1;
    }
    WTALOGI("摄像机[%d] 被检查方已转到随机姿态(x=%d,y=%d,亮度=%d)", id, nx, ny, rand_bright);
    return 0;
}

int Camera::inspect_peer(Camera* checked, time_t start_time)
{
    // this = 检查方，checked = 被检查方
    if (modbus_ctx == nullptr) {
        WTALOGI("摄像机[%d] Modbus 未连接，无法互检", id);
        return 1;
    }
    if (checked == nullptr) return 1;

    int point_id = find_peer_check_point_id();
    if (point_id < 0) {
        WTALOGI("摄像机[%d] 未配置互检点（peer_check_point_id 或 name 含'互检'）", id);
        return 1;
    }
    auto it = std::find_if(preset_positions.begin(), preset_positions.end(),
                           [point_id](const PresetPosition& p){ return p.id == point_id; });
    if (it == preset_positions.end()) {
        WTALOGI("摄像机[%d] 互检点 %d 不在预置点位列表中", id, point_id);
        return 1;
    }

    // 初始化本轮时间/目录，captureSnapshot 需要
    stop_requested.store(false);
    patrol_start_time = start_time;
    prepare_snapshot_dir();

    // 1) 检查方转向互检点，观察被检查方
    now_point_id = point_id;
    posture_completed = false;
    phase_ready_ms.store(0);
    frame_should_capture.store(0);
    photo_fired_keys.clear();
    {
        auto px = (360 + it->web_rotation_x) % 360 * 100;
        auto py = (360 + it->web_rotation_y) % 360 * 100;
        set_ptz(px, py, it->brightness);
        web_rotation_x = it->web_rotation_x;
        web_rotation_y = it->web_rotation_y;
        set_zoom_and_focus(it->zoom, it->focus);
    }

    auto wait_start = std::chrono::steady_clock::now();
    while (!posture_completed && !stop_requested.load() &&
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - wait_start).count() < 30) {
        interruptible_sleep_ms(200);
    }
    if (stop_requested.load()) { now_point_id = 0; return 2; }
    if (!posture_completed) {
        WTALOGI("摄像机[%d] 互检转点超时", id);
        now_point_id = 0;
        return 1;
    }

    // 2) 检查方就位后抓第一帧（被检查方转动前）
    auto request_peer_capture = [this](int req_state, int done_state) -> bool {
        long long phase = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        {
            std::lock_guard<std::mutex> lk(peer_capture_mtx);
            peer_capture_state.store(req_state);
            peer_capture_phase.store(phase);
        }
        phase_ready_ms.store(phase);
        std::unique_lock<std::mutex> lk(peer_capture_mtx);
        peer_capture_cv.wait_for(lk, std::chrono::milliseconds(10000),
            [this, done_state]{ return peer_capture_state.load() == done_state || stop_requested.load(); });
        return peer_capture_state.load() == done_state;
    };

    if (!request_peer_capture(1, 2)) {
        WTALOGI("摄像机[%d] 互检第一帧抓拍超时", id);
        peer_capture_state.store(0);
        phase_ready_ms.store(0);
        now_point_id = 0;
        return 1;
    }

    // 3) 令被检查方转到随机姿态
    int mret = checked->move_to_random_posture();
    if (mret == 2 || stop_requested.load()) {
        peer_capture_state.store(0);
        phase_ready_ms.store(0);
        now_point_id = 0;
        return 2;
    }
    if (mret != 0) {
        // 被检查方无法响应转动指令，直接判异常
        WTALOGI("摄像机[%d] 被检查方[%d] 未能转到随机姿态，判异常", id, checked->get_id());
        peer_capture_state.store(0);
        phase_ready_ms.store(0);
        now_point_id = 0;
        return 1;
    }

    // 稳定等待，确保流缓冲刷新到被检查方新姿态
    if (!interruptible_sleep_ms(800)) { now_point_id = 0; return 2; }

    // 4) 抓第二帧并与第一帧 diff（检出运动=正常）
    if (!request_peer_capture(3, 4)) {
        WTALOGI("摄像机[%d] 互检第二帧抓拍超时", id);
        peer_capture_state.store(0);
        phase_ready_ms.store(0);
        now_point_id = 0;
        return 1;
    }

    int result = peer_capture_result;
    peer_capture_state.store(0);
    phase_ready_ms.store(0);
    now_point_id = 0;

    if (result == 2) {
        WTALOGI("摄像机[%d] 互检：被检查方[%d] 转动前后无明显变化，疑似脱落/卡死，判异常",
                id, checked->get_id());
        return 1;
    }
    WTALOGI("摄像机[%d] 互检通过：检出被检查方[%d]运动", id, checked->get_id());
    return 0;
}

int Camera::monitor_at_point(const std::atomic<bool>& peer_done, time_t start_time)
{
    if (modbus_ctx == nullptr) {
        WTALOGI("摄像机[%d] Modbus 未连接，无法监视", id);
        return 1;
    }
    auto it = std::find_if(preset_positions.begin(), preset_positions.end(),
                           [this](const PresetPosition& p){ return p.id == monitor_point_id; });
    if (it == preset_positions.end()) {
        WTALOGI("摄像机[%d] 监视点 %d 不在预置点位列表中", id, monitor_point_id);
        return 1;
    }

    // 进入巡逻/录像状态
    stop_requested.store(false);
    patrolling = true;
    calibrating.store(false);
    photo_fired_keys.clear();
    phase_infer_decision.clear();
    patrol_start_time = start_time;
    prepare_snapshot_dir();
    generateCustomVideoPath();
    start_record_video();

    now_point_id = monitor_point_id;
    posture_completed = false;
    phase_ready_ms.store(0);
    frame_should_capture.store(0);
    auto px = (360 + it->web_rotation_x) % 360 * 100;
    auto py = (360 + it->web_rotation_y) % 360 * 100;
    set_ptz(px, py, it->brightness);
    web_rotation_x = it->web_rotation_x;
    web_rotation_y = it->web_rotation_y;
    set_zoom_and_focus(it->zoom, it->focus);

    auto wait_start = std::chrono::steady_clock::now();
    while (!posture_completed && !stop_requested.load() &&
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - wait_start).count() < 30) {
        interruptible_sleep_ms(200);
    }
    if (stop_requested.load()) {
        finish_patrolling();
        return 2;
    }
    if (!posture_completed) {
        finish_patrolling();
        return 1;
    }

    on_arrived_at_point();
    if (!interruptible_sleep_ms(800)) {
        set_brighten(0);
        on_leaving_point();
        finish_patrolling();
        return 2;
    }

    long long phase = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    phase_ready_ms.store(phase);
    frame_should_capture.store(1); // L0 抓拍
    light_phase_changed = false;
    WTALOGI("摄像机[%d] 监视点 %d 开灯拍摄 L0", id, monitor_point_id);

    int wait_ms = 0;
    while (frame_should_capture.load() != 0 && !stop_requested.load() && wait_ms < 5000) {
        interruptible_sleep_ms(50);
        wait_ms += 50;
    }

    if (stop_requested.load()) {
        set_brighten(0);
        on_leaving_point();
        finish_patrolling();
        return 2;
    }

    // 持续监视，直到同伴巡检完成
    while (!peer_done.load() && !stop_requested.load()) {
        interruptible_sleep_ms(200);
    }

    set_brighten(0);
    on_leaving_point();

    // 立即处理监视点 diff（不等到下一轮巡检）
    run_post_patrol_diff(this, false);

    finish_patrolling();
    return 0;
}

bool Camera::interruptible_sleep_ms(int ms)
{
    // 使用 condition_variable，stop 时由 notify_all 立刻唤醒，几乎无延迟
    // 返回 true 表示等满 ms（未被中断），false 表示被提前唤醒（已请求停止）
    if (ms <= 0) return !stop_requested.load();
    std::unique_lock<std::mutex> lk(stop_mtx);
    // 谓词为 true（已请求停止）则立刻返回 true，wait_for 整体返回 true => 被中断
    bool stopped = stop_cv.wait_for(lk, std::chrono::milliseconds(ms),
                                    [this]{ return stop_requested.load(); });
    return !stopped;
}

void Camera::update_posture_state(int x, int y)
{
    // 调用modbus请求（整个流程持锁：write + sleep + read 必须是原子事务，
    // 否则另一线程可能插入自己的 write/read 导致响应错位）
    std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
    if (modbus_ctx == nullptr)
        return;


    // 先通过写命令发送读取请求
    uint16_t regs[2];
    regs[0] = 1;
    regs[1] = 1;
    int rc = modbus_write_registers(modbus_ctx, 0x445C, 2, regs);

    std::this_thread::sleep_for(std::chrono::duration<double>(0.3));

    // 读取保持寄存器
    regs[0] = 0;
    regs[1] = 0;
    rc = modbus_read_registers(modbus_ctx, 0x445A, 2, regs);
    if (rc == -1) {
        WTALOGI("摄像机[%d] read position failed: %s", id, modbus_strerror(errno));
        m_modbus_broken.store(true); // 交给后台线程异步重连，本调用立即返回
    } else {
        // 环形距离比较（处理 0°/360° 边界情况，如 -180° 与 179°）
        int full_circle = 36000; // 360° * 100
        int dx = std::abs((int)regs[1] - x);
        if (dx > full_circle / 2) dx = full_circle - dx;
        int dy = std::abs((int)regs[0] - y);
        if (dy > full_circle / 2) dy = full_circle - dy;

        if (dx <= 150 && dy <= 150)  // 容差 1.5°
            posture_completed = true;
        else
            posture_completed = false;
    }

    return ;
}

int Camera::set_ptz(int horizontal, int vertical, int brightness)
{
    WTALOGI("设置摄像头[云台ip %s]姿态：水平:%d,垂直:%d,亮度:%d",this->ptz_ip.c_str(), horizontal, vertical, brightness);

    if (horizontal==-1 && vertical==-1 && brightness==-1)
        return 0; // 忽略控制请求，不进行任何操作

    // 写 PTZ 寄存器（持锁）
    {
        std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
        if (modbus_ctx == nullptr)
            return -1;

        // 准备要写入的寄存器值
        uint16_t regs[2];
        regs[0] = static_cast<uint16_t>(vertical == 9000 ? 8999 : vertical);   // 垂直角度  TODO临时bug修复
        regs[1] = static_cast<uint16_t>(horizontal); // 水平角度

        int rc = modbus_write_registers(modbus_ctx, MODBUSPTZ, 2, regs); // 全部写入寄存器
        if (rc == -1) {
            WTALOGI("Failed to write PTZ registers: %s", modbus_strerror(errno));
            m_modbus_broken.store(true); // 交给后台重连
        }
    }

    // 启动后台线程持续读取云台实际角度并更新状态，直到到位或超时
    // ★ 防重入：如果已经有一条轮询线程在跑，就不再开新的，避免多线程同时用 modbus_ctx
    if ((horizontal != -1 || vertical != -1) && !m_posture_polling.exchange(true)) {
        std::thread([this, horizontal, vertical]() {
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start).count() < 30) {
                this->update_posture_state(horizontal, vertical);
                if (this->posture_completed)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            m_posture_polling.store(false); // 释放锁位
        }).detach();
    } else if (horizontal != -1 || vertical != -1) {
        WTALOGI("摄像机[%d] 已有姿态轮询线程在跑，跳过重复启动", id);
    }

    this->set_brighten(brightness); //亮度

    return 0;
}

int Camera::set_wiper(int _switch)
{
    std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
    if (modbus_ctx == nullptr)
        return -1;

    // 准备要写入的寄存器值（假设开关寄存器地址为0x4451）
    uint16_t wiper_reg = static_cast<uint16_t>(_switch);

    // 写入保持寄存器
    int rc = modbus_write_registers(modbus_ctx, MODBUSWIPER, 1, &wiper_reg);
    if (rc == -1) {
        WTALOGI("Failed to write wiper register: %s", modbus_strerror(errno));
        m_modbus_broken.store(true);
        return -1;
    }

    this->wiper_switch = _switch; // 更新内部状态
    return 1;
}

int Camera::set_brighten(int brightness)
{
    if (brightness==-1)
        return 0;

    WTALOGI("相机[%d] 设置亮度:%d", id, brightness);

    // 使用Modbus设置亮度
    std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
    if (modbus_ctx == nullptr) {
        std::cerr << "Modbus context not initialized" << std::endl;
        return -1;
    }

    // 更新内部状态
    this->brightness = brightness;

    // 阶梯式分配：0-500 单灯递增，500-1000 主灯满+副灯递增
    // 前端亮度 0-1000 映射到灯珠功率 0-750（灯珠满功率上限 750，1000 功率过高不可用）
    // 只在开灯时切换主副灯，避免频繁闪烁
    if (last_brightness == 0 && brightness > 0) {
        lamp_toggle = 1 - lamp_toggle; // 仅在开灯时切换
    }
    last_brightness = brightness;

    int lamp_main = 0, lamp_sub = 0;
    if (brightness <= 500) {
        lamp_main = brightness * 3 / 2;  // 主灯 0-750
        lamp_sub = 0;                    // 副灯不亮
    } else {
        lamp_main = 750;                          // 主灯满(750)
        lamp_sub = (brightness - 500) * 3 / 2;    // 副灯 0-750
    }

    // 根据 toggle 决定哪个灯做主灯
    uint16_t reg_A = static_cast<uint16_t>(lamp_toggle ? lamp_sub : lamp_main);
    uint16_t reg_B = static_cast<uint16_t>(lamp_toggle ? lamp_main : lamp_sub);

    // 分别写入两个灯的寄存器
    int rc = modbus_write_register(modbus_ctx, 0x44A5, reg_A);  // 白光灯1
    rc = modbus_write_register(modbus_ctx, 0x44A6, reg_B);      // 白光灯2

    int rc2 = modbus_write_register(modbus_ctx, 0x4469, reg_A + reg_B); // 光敏控制开关的红外灯（总亮度）

    if (rc == -1 || rc2 == -1)  {
        WTALOGI("Failed to write brighten register(%d)|(%d): %s", rc, rc2, modbus_strerror(errno));
        m_modbus_broken.store(true);
        return -1;
    }

    return 0;
}

// 获取云台设备上的光敏亮度值和阈值
int Camera::getPhotosensitive()
{
    std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
    if (modbus_ctx == nullptr)
        return -1;

    // 读取保持寄存器
    uint16_t regs[1];
    int rc = modbus_read_registers(modbus_ctx, MODBUSPSENT, 1, regs);
    if (rc == -1) {
        WTALOGI("Failed to read photosensitive register");
        m_modbus_broken.store(true);
    } else {
        photosensitive = regs[0]; // 更新内部状态（避免失败时读到脏 regs）
    }

    regs[0] = 0;
    rc = modbus_read_registers(modbus_ctx, MODBUSPTHRESHOLD, 1, regs);
    if (rc == -1) {
        WTALOGI("Failed to read photosensitive threshold register");
        m_modbus_broken.store(true);
    } else {
        photosensitiveThreshold = regs[0]; // 更新内部状态
    }

    return 1;
}

int Camera::setphotosensitiveThreshold(int threshold)
{
    if (threshold == -1)
        return 0;

    std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
    if (modbus_ctx == nullptr)
        return -1;

    uint16_t reg = 0;
    reg = threshold;
    int rc = modbus_write_register(modbus_ctx, MODBUSPTHRESHOLD, reg);
    if (rc == -1) {
        WTALOGI("Failed to write photosensitive threshold register");
        m_modbus_broken.store(true);
        return -1;
    }
    photosensitiveThreshold = threshold;

    return 1;
}

// 回调函数，用于处理HTTP响应
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t totalSize = size * nmemb;
    ((std::string*)userp)->append((char*)contents, totalSize);
    return totalSize;
}

int Camera::set_zoom_and_focus(int zoom, int focus)
{
    //调用libcurl库
    // 构造包含缩放参数的URL
    std::string url = "http://" + ip + "/cgi-bin/param.cgi?action=update&group=CAMPOS&channel=0";

    bool todo = false;
    // 仅当zoom > -1时添加zoom参数
    if (zoom > -1) {
        url += "&CAMPOS.zoompos=" + std::to_string(zoom);
        todo = true;
    }

    // 仅当focus > -1时添加focus参数
    if (focus > -1) {
        // 0~80 段不生效，统一 clamp 到 80；>=80 保持原值（避免整体 +80 顶穿上限）
        url += "&CAMPOS.focuspos=" + std::to_string(focus < 80 ? 80 : focus);
        todo = true;
    }

    if (!todo)
        return 0;

    WTALOGI("摄像头[%d]设置镜头url串: %s", id, url.c_str());

    std::lock_guard<std::mutex> curl_lk(m_curl_mtx);
    // 设置请求URL
    curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());

    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback);

    std::string response_data;
    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &response_data);

    // 执行请求
    CURLcode res = curl_easy_perform(curl_handle);
    if(res != CURLE_OK) {
        return -1;
    }

    // 更新成员变量
    if (zoom > -1) this->zoom = zoom;
    if (focus > -1) this->focus = focus;

    return 0;
}


int Camera::fetch_remote_status()
{
    int res1 = 0;
    std::thread th1([this, &res1](){
        bool error = false;
        std::string url = "http://" + ip + "/cgi-bin/param.cgi?action=list&group=CAMPOS&channel=0";
        std::lock_guard<std::mutex> curl_lk(m_curl_mtx);
        curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());

        // 创建一个缓冲区来存储响应数据
        std::string response;
        curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &response);

        CURLcode res = curl_easy_perform(curl_handle);
        if(res == CURLE_OK) {
            // 检查错误码
            size_t err_no_pos = response.find("root.ERR.no=");
            if (err_no_pos != std::string::npos) {
                size_t err_no_end = response.find("\n", err_no_pos);
                std::string err_no_str = response.substr(err_no_pos + 12, err_no_end - err_no_pos - 12);
                int err_no = std::stoi(err_no_str);

                if (err_no != 0) {
                    // 获取错误描述
                    size_t err_des_pos = response.find("root.ERR.des=");
                    if (err_des_pos != std::string::npos) {
                        size_t err_des_end = response.find("\n", err_des_pos);
                        std::string err_des = response.substr(err_des_pos + 13, err_des_end - err_des_pos - 13);
                    }
                    error = true;
                }
            }

            if (!error) {
                // 解析缩放位置
                size_t zoompos_pos = response.find("root.CAMPOS.zoompos=");
                if (zoompos_pos != std::string::npos) {
                    size_t zoompos_end = response.find("\n", zoompos_pos);
                    std::string zoompos_str = response.substr(zoompos_pos + 19, zoompos_end - zoompos_pos - 19);


                    zoompos_str.erase(std::remove_if(zoompos_str.begin(), zoompos_str.end(),
                        [](char c) { return c == '=' || isspace(c); }), zoompos_str.end());

                    try {
                        if (!zoompos_str.empty()) {
                            zoom = std::stoi(zoompos_str);
                        }
                    } catch (const std::exception& e) {
                        WTALOGI("Failed to parse zoom position: %s, error: %s", zoompos_str.c_str(), e.what());
                    }
                }

                // 解析焦距位置
                size_t focuspos_pos = response.find("root.CAMPOS.focuspos=");
                if (focuspos_pos != std::string::npos) {
                    size_t focuspos_end = response.find("\n", focuspos_pos);
                    std::string focuspos_str = response.substr(focuspos_pos + 20, focuspos_end - focuspos_pos - 20);

                    // 焦距位置可以存储在类成员变量
                    focuspos_str.erase(std::remove_if(focuspos_str.begin(), focuspos_str.end(),
                        [](char c) { return c == '=' || isspace(c); }), focuspos_str.end());

                    try {
                        if (!focuspos_str.empty()) {
                            focus = std::stoi(focuspos_str);
                        }
                    } catch (const std::exception& e) {
                        WTALOGI("Failed to parse focus position: %s, error: %s", focuspos_str.c_str(), e.what());
                    }
                }

                WTALOGI("摄像机[%d]同步当前镜头状态: zoom=%d, focus =%d",id, zoom, focus);
            }
        } else {
            res1 = -1;
        }
    }); // 创建一个线程来执行curl_easy_perform

    int res2 = 0;
    std::thread th2([this,&res2](){
        // 整个 modbus 会话持锁：这里连着多次读寄存器（姿态/灯/雨刮/光敏），
        // 一次锁到底既保证事务不被打断，也避免和其他线程交织。
        std::lock_guard<std::recursive_mutex> lk(m_modbus_mtx);
        if (modbus_ctx == nullptr) { res2 = -1; return; }

        //调用modbus获取云台姿态
        uint16_t regs[2];
        regs[0] = 1;
        regs[1] = 1;
        int rc = modbus_write_registers(modbus_ctx, 0x445C, 2, regs);

        std::this_thread::sleep_for(std::chrono::duration<double>(0.6));

        regs[0] = 0;
        regs[1] = 0;
        rc = modbus_read_registers(modbus_ctx, 0x445A, 2, regs);
        if (rc == -1) {
            WTALOGI("摄像机[%d] read position failed: %s", id, modbus_strerror(errno));
            m_modbus_broken.store(true); // 交给后台线程重连，本处立即返回失败
            res2 = -1;
        } else {
            rotation_y = regs[0];
            rotation_x = regs[1];

            /* 后端值映射为前端值 */

            if (0<=rotation_x && rotation_x<=18000) {
                web_rotation_x = rotation_x/100;
            } else if (18000 <rotation_x && rotation_x<=36000) {
                web_rotation_x =  (rotation_x - 36000)/100;
            }

            if (0<=rotation_y && rotation_y<=18000) {
                web_rotation_y = rotation_y/100;
            } else if (18000 < rotation_y && rotation_y<=36000) {
                web_rotation_y = (rotation_y - 36000)/100;
            }
            web_rotation_y = clamp_y_angle(web_rotation_y);
            WTALOGI("摄像机[%d]同步当前姿态: x=%d, y=%d",id, web_rotation_x, web_rotation_y);
        }

        rc = modbus_read_registers(modbus_ctx, 0x44A5, 2, regs); // 读取补光灯亮度（灯A + 灯B）
        if (rc == -1) {
            res2 = -1;
        } else {
            // 逆运算：根据两灯亮度还原 brightness（灯珠 0-750 还原前端 0-1000）
            // 阶梯式分配：0-500 单灯，500-1000 双灯
            int lamp_A = regs[0];  // 灯A 0-750
            int lamp_B = regs[1];  // 灯B 0-750
            int lamp_main = std::max(lamp_A, lamp_B);
            int lamp_sub = std::min(lamp_A, lamp_B);
            if (lamp_sub == 0) {
                // 单灯模式：brightness = lamp_main * 2 / 3
                this->brightness = lamp_main * 2 / 3;
            } else {
                // 双灯模式：brightness = 500 + lamp_sub * 2 / 3
                this->brightness = 500 + lamp_sub * 2 / 3;
            }
        }

        rc = modbus_read_registers(modbus_ctx, MODBUSSYS, 1, regs);
        if (rc == -1) {
            res2 = -1;
        } else {
            (regs[0] & 1<<8) ? this->wiper_switch = true : this->wiper_switch = false; //第9bit位是雨刮器开关状态
        }

        getPhotosensitive();

    });

    // 等待两个线程完成
    th1.join(), th2.join();

    if (res1 == -1 || res2 == -1)
        return -1;
    else
        return 0;
}
