#include "camera_controller.hpp"
#include <iostream>
#include <curl/curl.h>
#include <curl/easy.h>
#include "../examples/utilities/json.hpp"
#include <fstream>
#include "../examples/utilities/sample_log.h"
#include <unistd.h>
#include <pthread.h>

// 巡检结束后的"点位前后对比"批量处理函数（实现位于 examples/libaxdl/src/ax_model_damage.cpp）
// 这里前置声明，链接期解析。把重型 OpenCV 计算从渲染热路径移到巡检结束后统一执行。
// update_baseline: true=标定模式更新基线，false=巡检模式不更新基线
void run_post_patrol_diff(class Camera* cam, bool update_baseline);


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
            camera = cameras[camera_id];
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
                resp_first_camera_data["photosensitive"] = camera->photosensitive;
                resp_first_camera_data["photosensitiveThreshold"] = camera->photosensitiveThreshold;
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
                    camera->web_rotation_y = (origin_rotatey + new_rotatey)%360; // 更新前端值y
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
                    result["path"] = cameras.size()>0 ? cameras.begin()->second->pic_dirname: "";
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

            // 根据是否指定了有效的摄像机ID返回不同的消息
            if (camera_id > 0 && cameras.find(camera_id) != cameras.end()) {
                response["msg"] = "Camera " + std::to_string(camera_id) + " calibration started in background...";
            } else {
                response["msg"] = "All cameras calibration are starting in background...";
            }
            response["status"] = 200;

            // 创建后台线程执行标定任务
            try {
                std::thread calibrate_thread([this, currentReqId, currentCameraId]() {
                    // 如果指定了有效的摄像机ID，则只标定该摄像机；否则标定所有摄像机
                    if (currentCameraId > 0 && cameras.find(currentCameraId) != cameras.end()) {
                        auto res = calibrate(currentCameraId); // 标定指定摄像机
                        // 标定完成后构造JSON响应
                        nlohmann::json result;
                        result["status"] = 200;
                        result["message"] = "Camera " + std::to_string(currentCameraId) + " calibration completed successfully";
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
                });
                calibrate_thread.detach();
            } catch (const std::system_error& e) {
                response["status"] = 500;
                response["msg"] = "Failed to create calibration thread: " + std::string(e.what());
                ALOGE("RThread creation failed: %s", e.what());
            }
        }
        /* else if (cmd == "photograph") { //拍照指令
            if (camera_id > 0 && camera != NULL) {
                camera->capture_reference_image(); // 拍照
            } else if (camera_id<=0){ // (camera == NULL) && (camera_id<=0) // 所有相机拍照
                for (auto& pair : cameras) {
                    auto& camera = pair.second;
                    camera->capture_reference_image(); // 无论是否巡逻模式下都允许外部指令操作拍照
                }
            }

        } */
       else { //unknown action
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

    // 遍历所有摄像机，设置不标定模式并启动巡逻
    std::vector<std::thread> threads;
    bool res = false;
    for (auto& pair : cameras) {
        // 为每个摄像机创建一个线程执行patrol_with_calibration_loop
        if (!pair.second->m_pipeline)
            continue;

        threads.emplace_back([camera = pair.second, &res]() {
            // 一次巡检：每个点位内部分为无灯照和有灯照两阶段拍照
            res = camera->patrol_with_calibration_loop(false);
        });
    }

    // 等待所有线程完成
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    return res;
}

int CameraController::calibrate(int camera_id) //return 0正常  非0异常
{
    reload_config(); // 标定前热加载配置（点位+基础字段）

    int res = 0;
    // 如果指定了有效的摄像机ID，则只标定该摄像机；否则标定所有摄像机
    if (camera_id > 0 && cameras.find(camera_id) != cameras.end()) {
        Camera* camera = getCamera(camera_id);
        res =  camera->patrol_with_calibration_loop(true);
    } else {
        // 遍历所有摄像机，设置标定模式并启动标定
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

bool CameraController::early_warning_process(int camera_id, int point_id, int light_flag, const std::string& damage_type)
{
    auto camera = getCamera(camera_id);
    return alarm_manager.generateAlarm(AlarmType::LINE_CROSSING, damage_type, 1, camera, point_id, light_flag);
}

bool CameraController::diff_warning_process(int camera_id, int point_id, int light_flag, const std::string& pic_path)
{
    auto camera = getCamera(camera_id);
    return alarm_manager.generateDiffAlarm(camera, point_id, light_flag, pic_path);
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
    WTALOGI("摄像机[%d]完成巡逻!", this->id);
}

int Camera::add_preset_position(Camera::PresetPosition pos)
{
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

std::string Camera::captureSnapshot(const cv::Mat& image, int point_id, int light_flag)
{
    // 使用巡检开始时冻结的时间戳，保证整轮巡检所有快照在同一分钟目录下
    time_t base_time = patrol_start_time > 0 ? patrol_start_time : (time(nullptr) + 8 * 3600);
    tm *t = gmtime(&base_time);
    char ymd[16] = {0};
    snprintf(ymd, sizeof(ymd), "%04d%02d%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

    snprintf(pic_dirname, sizeof(pic_dirname), "/wt_tech/data/%s/%s/%s_%02d%02d/image", orga_name.c_str(), ymd, ymd, t->tm_hour, t->tm_min);
    if (access(pic_dirname, 0) != 0) {
        char mk[256] = {0};
        snprintf(mk, sizeof(mk), "mkdir -p %s", pic_dirname);
        system(mk);
    }

    char filepath[320] = {0};
    // 文件名加入灯光状态 L0（无灯照）或 L1（有灯照），使用传入的 point_id 而非 now_point_id
    if (light_flag >= 0) {
        snprintf(filepath, sizeof(filepath), "%s/%s_%02d%02d_%s_%s_%d_L%d.png", pic_dirname, ymd, t->tm_hour, t->tm_min,
            orga_name.c_str(), name.c_str(), point_id, light_flag);
    } else {
        snprintf(filepath, sizeof(filepath), "%s/%s_%02d%02d_%s_%s_%d.png", pic_dirname, ymd, t->tm_hour, t->tm_min,
            orga_name.c_str(), name.c_str(), point_id);
    }

    std::vector<int> jpg_params = { cv::IMWRITE_JPEG_QUALITY, 90 };
    if (!cv::imwrite(filepath, image, jpg_params)) {
        WTALOGI("[Camera] 点位[%d] cv::imwrite 失败: %s", now_point_id, filepath);
        return "";
    }

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
    // 优先使用巡检开始时冻结的时间戳，保证整轮巡检所有视频/封面/快照同目录
    time_t timeReal;
    if (patrol_start_time > 0) {
        timeReal = patrol_start_time;
    } else {
        time(&timeReal);
        timeReal = timeReal + 8 * 3600; // 转换为东八区时间
    }
    tm *t = gmtime(&timeReal);

    char dateStr[16] = {0};
    sprintf(dateStr, "%04d%02d%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

    // 生成目录路径
    std::string dirname;
    char filename[256] = {0};

    // 根据类型选择不同的基础路径
    std::string base_path;
    if (type == VideoPathType::PERSON) {
        base_path = "/wt_tech/data/video2/person";
        dirname = base_path;

        // 生成文件名 - 增加到秒级
        sprintf(filename, "%s/%s-%d-%02d-%02d_%02d%02d%02d.mp4", dirname.c_str(), name.c_str(),
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

    } else if (type == VideoPathType::DAMAGE_CLIP) {
        // 损伤片段视频：按 风场/年月日/小时/damage 分目录，文件名与快照前缀一致（不带开关灯标识）
        base_path = "/wt_tech/data/";
        char hourMinStr[8] = {0};
        sprintf(hourMinStr, "%02d%02d", t->tm_hour, t->tm_min);
        dirname = base_path + orga_name.c_str() + "/" + std::string(dateStr) + "/" + std::string(dateStr) + "_"
            + hourMinStr + "/damage";

        sprintf(filename, "%s/%s_%02d%02d_%s_%s_%d.mp4", dirname.c_str(),
            dateStr, t->tm_hour, t->tm_min, orga_name.c_str(), name.c_str(), now_point_id);
    } else {
        base_path = "/wt_tech/data/";
        char hourMinStr[8] = {0};
        sprintf(hourMinStr, "%02d%02d", t->tm_hour, t->tm_min);
        dirname = base_path + orga_name.c_str() + "/" + std::string(dateStr) + "/" + std::string(dateStr) + "_"
            + hourMinStr + "/video";

        // 生成文件名
        sprintf(filename, "%s/%d-%02d-%02d_%02d%02d_%s_%s.mp4", dirname.c_str(), t->tm_year + 1900,
            t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, orga_name.c_str(), name.c_str());
    }

    // 仅巡检主录像和人物视频写回 video_filename；损伤片段路径由调用方单独保存到 damage_clip_filename
    if (type != VideoPathType::DAMAGE_CLIP) {
        strncpy(m_pipeline->video_filename, filename, sizeof(this->m_pipeline->video_filename)-1);
        m_pipeline->video_filename[sizeof(m_pipeline->video_filename)-1] = '\0';
    }

    // 创建目录
    if (access(dirname.c_str(), 0) != 0) {
        char cmd[256] = {0};
        sprintf(cmd, "mkdir -p %s", dirname.c_str());
        system(cmd);
    }

    return filename;
}


void Camera::set_camera_rtsp_url(const std::string& url)
{
    camera_rtsp_url = url;
}

int Camera::patrol_with_calibration_loop(bool is_calibrate) //return 0表示正常 非0表示异常
{
    patrolling = true; // 标识进入巡逻模式
    stop_requested.store(false); // 每轮巡检开始时清掉上一次的停止请求
    photo_fired_keys.clear(); // 清空拍照去重状态，避免跨轮次误判
    // 冻结本轮巡检时间戳（东八区已修正），保证所有路径在同一分钟目录下
    patrol_start_time = time(nullptr) + 8 * 3600;
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
        this->m_pipeline->point_id = now_point_id; // 设置当前点位ID
        posture_completed = false;

        /* 前端值映射为后端值 */
        auto px = (360+position->web_rotation_x)%360 * 100;
        auto py = (360+position->web_rotation_y)%360 * 100;

        // ★ 点位移动开始时即进入"到位"状态，开始累积帧（包含完整开关灯画面）
        if (!is_calibrate) {
            on_arrived_at_point();
        }

        // 第一阶段：开灯拍照L0
        set_ptz(px, py, position->brightness);

        web_rotation_x = position->web_rotation_x;
        web_rotation_y = position->web_rotation_y;

        set_zoom_and_focus(position->zoom, position->focus); // 设置缩放级别

        //轮询等待姿态完成或超时12秒
        auto start_time = std::chrono::steady_clock::now();

        // set_ptz 已启动后台线程更新姿态状态，此处只需等待 posture_completed 或超时
        while (!posture_completed && !stop_requested.load() && std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start_time).count() < 12) { //12秒内必需转完
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
            start_take_a_picture(1); //标定拍 L0

            // L1 关灯标定拍照
            set_brighten(0); // 关灯
            if (!interruptible_sleep_ms(2500)) break; // 等待灯熄灭稳定
            light_phase_changed = true; // 切换到 L1 阶段
            start_take_a_picture(1); //标定拍 L1
            light_phase_changed = false; // 重置
        } else { //巡航模式且到位成功

            // 记录到位时刻，用于补足剩余停留时间
            auto point_start = std::chrono::steady_clock::now();

            // 先等待画面稳定，再设置拍照条件（避免在画面未稳定时触发拍照）
            if (!interruptible_sleep_ms(2500)) break; // 等待灯点亮稳定 + 补偿画面延迟

            // 设置拍照条件，允许 draw_custom 触发 L0 拍照
            photo_captured.store(false);
            light_phase_changed = false;
            WTALOGI("摄像机[%d] 点位[%d] 开灯拍摄 L0", id, now_point_id);

            // 有灯照拍照：等待 draw_custom 触发拍照完成，拍完后继续等满 duration/2 秒
            int half_duration = std::max(1, position->duration / 2);
            int wait_ms = 0;
            const int max_wait_ms = half_duration * 1000;
            while (!photo_captured.load() && !stop_requested.load() && wait_ms < max_wait_ms) {
                interruptible_sleep_ms(50); // stop 时会被 notify 立刻唤醒
                wait_ms += 50;
            }
            if (stop_requested.load()) break;
            if (!photo_captured.load()) {
                WTALOGI("摄像机[%d] 点位[%d] L0 拍照超时", id, now_point_id);
            } else if (wait_ms < max_wait_ms) {
                // L0 拍完后继续等待剩余时间，确保有灯照阶段占满 duration/2 秒
                if (!interruptible_sleep_ms(max_wait_ms - wait_ms)) break;
            }

            // 第二阶段：关灯拍照（L1 = 无灯照）
            set_brighten(position->brightness/4); // 接近关灯状态
            if (!interruptible_sleep_ms(2500)) break; // 等待灯熄灭稳定 + 补偿画面延迟
            photo_captured.store(false); // 重置拍照标志
            light_phase_changed = true; // 灯已关闭，允许 draw_custom 触发无灯照拍照
            WTALOGI("摄像机[%d] 点位[%d] 关灯拍摄 L1", id, now_point_id);

            // 无灯照拍照：等待 draw_custom 触发拍照完成，拍完后继续等满 duration/2 秒
            wait_ms = 0;
            while (!photo_captured.load() && !stop_requested.load() && wait_ms < max_wait_ms) {
                interruptible_sleep_ms(50); // stop 时会被 notify 立刻唤醒
                wait_ms += 50;
            }
            if (stop_requested.load()) break;
            if (!photo_captured.load()) {
                WTALOGI("摄像机[%d] 点位[%d] L1 拍照超时", id, now_point_id);
            } else if (wait_ms < max_wait_ms) {
                // L1 拍完后继续等待剩余时间，确保无灯照阶段占满 duration/2 秒
                if (!interruptible_sleep_ms(max_wait_ms - wait_ms)) break;
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

    return res;
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
                       std::chrono::steady_clock::now() - start).count() < 12) {
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
    // 只在开灯时切换主副灯，避免频繁闪烁
    if (last_brightness == 0 && brightness > 0) {
        lamp_toggle = 1 - lamp_toggle; // 仅在开灯时切换
    }
    last_brightness = brightness;

    int lamp_main = 0, lamp_sub = 0;
    if (brightness <= 500) {
        lamp_main = brightness * 2;  // 主灯 0-1000
        lamp_sub = 0;                // 副灯不亮
    } else {
        lamp_main = 1000;                      // 主灯满
        lamp_sub = (brightness - 500) * 2;    // 副灯 0-1000
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
    // 设置请求URL
    curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());
    // 向curl设置请求URL
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
            WTALOGI("摄像机[%d]同步当前姿态: x=%d, y=%d",id, web_rotation_x, web_rotation_y);
        }

        rc = modbus_read_registers(modbus_ctx, 0x44A5, 2, regs); // 读取补光灯亮度（灯A + 灯B）
        if (rc == -1) {
            res2 = -1;
        } else {
            // 逆运算：根据两灯亮度还原 brightness
            // 阶梯式分配：0-500 单灯，500-1000 双灯
            int lamp_A = regs[0];  // 灯A 0-1000
            int lamp_B = regs[1];  // 灯B 0-1000
            int lamp_main = std::max(lamp_A, lamp_B);
            int lamp_sub = std::min(lamp_A, lamp_B);
            if (lamp_sub == 0) {
                // 单灯模式：brightness = lamp_main / 2
                this->brightness = lamp_main / 2;
            } else {
                // 双灯模式：brightness = 500 + lamp_sub / 2
                this->brightness = 500 + lamp_sub / 2;
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