#include "camera_controller.hpp"
#include <iostream>
#include <curl/curl.h>
#include <curl/easy.h>
#include "../examples/utilities/json.hpp"
#include <fstream>
#include "../examples/utilities/sample_log.h"

std::string Camera::capture_path ="/wt_tech/conf/img/"; // 图像保存路径初始化

CameraController::CameraController()
{
    // 初始化libcurl全局环境
    CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
    //根据json配置文件创建对应的相机实例
    load_config_from_file(CONFIG_FILE_PATH);
}

CameraController::~CameraController()
{
    // 清理libcurl全局环境
    curl_global_cleanup();
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
            if (brace_count == 0) break; // JSON 闭合
        }

        nlohmann::json json_request; //请求
        try {
            // 解析JSON字符串(处理JSON命令)
            json_request = nlohmann::json::parse(json_str);
        } catch (const nlohmann::json::parse_error& e) {
            ALOGE("json parse error%s", e.what());
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

        if (cmd == "get") {
            if (camera_id > 0 && camera != NULL) {
                response["data"] = nlohmann::json::array();
                //以json组装一台相机的内部状态信息
                auto& resp_first_camera_data = response["data"][0];
                resp_first_camera_data["id"] = camera_id;
                resp_first_camera_data["rotationx"] = camera->rotation_x;
                resp_first_camera_data["rotationy"] = camera->rotation_y;
                resp_first_camera_data["zoom"] = camera->zoom;
                resp_first_camera_data["focus"] = camera->focus;
                resp_first_camera_data["brightness"] = camera->brightness;
                resp_first_camera_data["point_id"] = camera->now_point_id;
                resp_first_camera_data["patrol"] = camera->patrolling;
            } else if (camera_id > 0) {
                response["msg"] = "该相机不存在";
                response["status"] = 500;
            } else if (camera_id <= 0) { // 所有摄像机信息
                int index = 0;
                response["data"] = nlohmann::json::array();
                for (auto& pair : cameras) {
                    auto& camera = pair.second;
                    // 以json组装每个相机的内部状态信息
                    auto& resp_one_camera_data = response["data"][index];
                    resp_one_camera_data["id"] = camera->id;
                    resp_one_camera_data["rotationx"] = camera->rotation_x;
                    resp_one_camera_data["rotationy"] = camera->rotation_y;
                    resp_one_camera_data["zoom"] = camera->zoom;
                    resp_one_camera_data["focus"] = camera->focus;
                    resp_one_camera_data["brightness"] = camera->brightness;
                    resp_one_camera_data["point_id"] = camera->now_point_id;
                    resp_one_camera_data["patrol"] = camera->patrolling;
                }
            }
        } else if (cmd == "set" || cmd == "add") {
            if (camera_id > 0 && camera != NULL) { //指定了相机
                if (camera->is_patroling()) { // 是巡检模式,不允许设置
                    response["status"] = 500;
                    response["msg"] = "Camera is in patrol mode, can't set parameters";
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

                int origin_rotatex, origin_rotatey, origin_zoom, origin_focus, origin_brightness;
                if (cmd == "set") {
                    origin_rotatex = 0;
                    origin_rotatey = 0;
                    origin_zoom = 0;
                    origin_focus = 0;
                    origin_brightness = 0;
                } else { //add
                    origin_rotatex = camera->rotation_x;
                    origin_rotatey = camera->rotation_y;
                    origin_zoom = camera->zoom;
                    origin_focus = camera->focus;
                    origin_brightness = camera->brightness;
                }

                int x=-1, y=-1, zoom=-1, focus=-1,brightness=-1; //最终值 -1表示此次不需要更改该值

                int new_rotatex,new_rotatey,new_zoom,new_focus,new_brightness=0;
                if (has_rotatex) {
                    new_rotatex = data["rotatex"];
                    x = (origin_rotatex + new_rotatex%360)%360;
                    x<0? x+=360 : x;
                }
                if (has_rotatey) {
                    new_rotatey = data["rotatey"];
                    y = (origin_rotatey + new_rotatey%360)%360;
                    y<0? y+=360 : y;
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

            // 根据是否指定了有效的摄像机ID返回不同的消息
            if (camera_id > 0 && cameras.find(camera_id) != cameras.end()) {
                response["msg"] = "Camera " + std::to_string(camera_id) + " patrol started in background...";
            } else {
                response["msg"] = "All cameras patrol started in background...";
            }
            response["status"] = "start";

            // 创建后台线程执行巡检任务
            try {
                std::thread patrol_thread([this, currentReqId, currentCameraId]() {
                    // 如果指定了有效的摄像机ID，则只巡检该摄像机；否则巡检所有摄像机
                    if (currentCameraId > 0 && cameras.find(currentCameraId) != cameras.end()) {
                        Camera* camera = cameras[currentCameraId];
                        camera->patrol_with_calibration_loop(false);
                        // 巡检完成后构造JSON响应
                        nlohmann::json result;
                        result["status"] = "end";
                        result["msg"] = "Camera " + std::to_string(currentCameraId) + " patrol has completed successfully";
                        result["cmd"] = "action";
                        result["reqId"] = currentReqId; // 添加reqId到响应中
                        std::cout << result.dump() << std::endl;
                    } else {
                        // 如果没有指定有效的摄像机ID，则巡检所有摄像机
                        all_cameras_patrol();
                        // 巡检完成后构造JSON响应
                        nlohmann::json result;
                        result["status"] = "end";
                        result["msg"] = "All cameras patrol has completed successfully";
                        result["cmd"] = "action";
                        result["reqId"] = currentReqId; // 添加reqId到响应中
                        std::cout << result.dump() << std::endl;
                    }
                });
                patrol_thread.detach();
            } catch (const std::system_error& e) {
                response["status"] = 500;
                response["msg"] = "Failed to create patrol thread: " + std::string(e.what());
                ALOGE("RThread creation failed: %s", e.what());
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
                        calibrate(currentCameraId); // 标定指定摄像机
                        // 标定完成后构造JSON响应
                        nlohmann::json result;
                        result["status"] = 200;
                        result["message"] = "Camera " + std::to_string(currentCameraId) + " calibration completed successfully";
                        result["cmd"] = "calibrate";
                        result["reqId"] = currentReqId; // 添加reqId到响应中
                        std::cout << result.dump() << std::endl;
                    } else {
                        // 如果没有指定有效的摄像机ID，则标定所有摄像机
                        calibrate(-1); // -1表示标定所有摄像机
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
        std::cout << response.dump() << std::endl;
    }

    return 0;
}

int CameraController::all_cameras_patrol()
{
    // 遍历所有摄像机，设置不标定模式并启动巡逻
    for (auto& pair : cameras) {
        pair.second->patrol_with_calibration_loop(false);
    }
    return 0;
}

int CameraController::calibrate(int camera_id)
{
    // 如果指定了有效的摄像机ID，则只标定该摄像机；否则标定所有摄像机
    if (camera_id > 0 && cameras.find(camera_id) != cameras.end()) {
        Camera* camera = cameras[camera_id];
        return camera->patrol_with_calibration_loop(true);
    } else {
        // 遍历所有摄像机，设置标定模式并启动标定
        for (auto& pair : cameras) {
            pair.second->patrol_with_calibration_loop(true);
        }
    }
    return 0;
}

int CameraController::cooldown = 0;
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
            cooldown = std::stoi(config["cooldown"].get<std::string>());
        }

        // 遍历相机列表
        if (config.contains("chl_list") && config["chl_list"].is_array()) {
            for (const auto& camera_config : config["chl_list"]) {
                // 创建相机实例
                Camera* camera = new Camera();

                // 设置相机基本信息
                camera->id = std::stoi(camera_config["id"].get<std::string>());

                if (camera_config.contains("ip")) {
                    camera->ip = camera_config["ip"];
                }

                // 设置 PTZ IP
                if (camera_config.contains("ptz_ip")) {
                    camera->ptz_ip = camera_config["ptz_ip"];
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
                        pos.rotation_x = point["rotatex"];
                        pos.rotation_y = point["rotatey"];
                        pos.zoom = point["zoom"];
                        pos.focus = point["focus"];
                        pos.brightness = point["brightness"];
                        camera->add_preset_position(pos); // 设置点位信息
                    }
                }
                camera->start();

                // 将相机添加到控制器
                cameras[camera->id] = camera;
            }
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return -1;
    }
}

int CameraController::start()
{
    running = true; // 标记控制器为运行状态

    // 创建并启动一个新线程来执行receive_input函数
    input_thread = std::thread(&CameraController::receive_input_loop, this);

    return 0;
}

int CameraController::stop()
{
    running = false; // 标记控制器为停止状态

    if (input_thread.joinable()) {
        input_thread.join(); // 等待输入线程结束
    }
    // 停止所有摄像机
    for (auto camera : cameras) {
        camera.second->pause(); // 停止相机
    }
    return 0;
}

void CameraController::early_warning(int camera_id)
{
    MYALOGI("生成告警，并拍摄图片， 摄像头id%d", camera_id);
    auto &camera = this->cameras[camera_id];
    alarm_generator.generateAlarm(AlarmType::LINE_CROSSING, "", 1, camera);
}

void CameraController::setCameraPipe(int camera_id, pipeline_t *pipe)
{
    cameras[camera_id]->setPipe(pipe);
}

/* ================================== */

Camera::Camera()
{
    curl_handle = curl_easy_init();
    // 设置请求基本通用选项
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 5L);  // 5秒超时
    curl_easy_setopt(curl_handle, CURLOPT_FOLLOWLOCATION, 1L);  // 跟随重定向

}

Camera::~Camera()
{
    if (curl_handle) {
        curl_easy_cleanup(curl_handle);
        curl_handle = nullptr;
    }
}

int Camera::start()
{
    running = true;
    auto res_code = 1;

    auto rc = connect_modbus();  //云台连接

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
    // 清理旧连接
    if (modbus_ctx != nullptr) {
        modbus_close(modbus_ctx);
        modbus_free(modbus_ctx);
    }

    // 初始化Modbus 连接
    modbus_ctx = modbus_new_tcp(ptz_ip.c_str(), 8802); // 默认Modbus TCP 端口为 502改为8802
    if (!modbus_ctx) {
        std::cerr << "Failed to create Modbus context: " << modbus_strerror(errno) << std::endl;
    } else {
        // 设置 Modbus 超时时间
        uint32_t to_sec = 5; // 超时秒数
        modbus_set_response_timeout(modbus_ctx, to_sec, 0);

        // 连接到 Modbus 服务器
        if (modbus_connect(modbus_ctx) == -1) {
            std::cerr << "Modbus connection failed: " << modbus_strerror(errno) << std::endl;
            modbus_free(modbus_ctx);
            modbus_ctx = nullptr;
        } else {
            // 设置 Modbus 从站 ID（根据实际情况调整）
            //modbus_set_slave(modbus_ctx, 1);
        }
    }

    return false;
}

int Camera::get_id() const
{
    return id;
}

bool Camera::is_patroling() const
{
    return patrolling;
}

void Camera::set_patroling(bool patroling)
{
    this->patrolling = patroling;
}

int Camera::add_preset_position(Camera::PresetPosition pos)
{
    this->preset_positions.push_back(pos);
    return 0;
}

int Camera::capture_image_for_reference()
{
    // 生成文件名: 摄像机id/摄像机id_点位id_时间.jpg
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* now_time = std::localtime(&now);

    // 格式化时间字符串，格式为YYYYMMDD_HHMMSS
    char time_str[20];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", now_time);

    // 使用摄像机ID和点位ID生成文件名
    std::string camera_id_str = std::to_string(id); // 使用id作为摄像机ID
    std::string point_id_str = std::to_string(now_point_id); // 使用当前点位ID

    // 生成文件名：摄像机id_点位id_时间.jpg
    std::string filename = camera_id_str + "_" + point_id_str + "_" + std::string(time_str) + ".jpg";

    // 生成路径：摄像机id/文件名
    std::string dir_path = capture_path + "/" + camera_id_str;

    // 检查目录是否存在，不存在则创建
    #ifdef _WIN32
        _mkdir(dir_path.c_str());
    #else
        mkdir(dir_path.c_str(), 0777);
    #endif

    // 完整文件路径
    std::string filepath = dir_path + "/" + filename;

    // 构造拍照URL，使用协议中指定的URL格式
    std::string url = "http://" + ip + "/cgi-bin/snap.cgi?channel=0";

    // 设置curl请求
    curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());

    // 创建文件保存图像
    std::ofstream image_file(filepath, std::ios::binary);
    if (!image_file.is_open()) {
        return -1;
    }

    // 设置curl回调函数将数据写入文件
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, [](void* contents, size_t size, size_t nmemb, void* userp) {
        std::ofstream* file = static_cast<std::ofstream*>(userp);
        file->write(static_cast<char*>(contents), size * nmemb);
        return size * nmemb;
    });

    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &image_file);

    // 执行请求
    CURLcode res = curl_easy_perform(curl_handle);

    // 清理
    image_file.close();

    // 检查结果
    if (res != CURLE_OK) {
        std::cerr << "Failed to capture image: " << curl_easy_strerror(res) << std::endl;
        // 尝试删除可能不完整的文件
        remove(filepath.c_str());
        return -1;
    }

    // 获取HTTP响应码
    long http_code = 0;
    curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);

    if (http_code != 200) {
        std::cerr << "HTTP error code: " << http_code << std::endl;
        remove(filepath.c_str());
        return -1;
    }

    // 检查文件大小，确保图像成功保存
    std::ifstream check_file(filepath, std::ios::binary | std::ios::ate);
    if (check_file.fail()) {
        std::cerr << "Failed to check captured image file" << std::endl;
        remove(filepath.c_str());
        return -1;
    }

    std::streamsize file_size = check_file.tellg();
    check_file.close();

    if (file_size <= 0) {
        std::cerr << "Captured image file is empty or invalid" << std::endl;
        remove(filepath.c_str());
        return -1;
    }

    return 0;
}

void Camera::setPipe(pipeline_t * pipe) {
    this->pipeline = pipe;
}

int Camera::patrol_with_calibration_loop(bool is_calibrate)
{
    patrolling = true; // 标识进入巡逻模式

    /* 这里实现点位切换逻辑
     * 例如：遍历预设点位列表，定期切换到下一个点位
     */
    now_point_id = 1; // 当前所在的预置点位ID记录
    for(auto position = preset_positions.begin(); position != preset_positions.end(); ++position) {
        set_ptz(position->rotation_x, position->rotation_y, position->brightness); // 设置转向和亮度
        set_zoom_and_focus(position->zoom, position->focus); // 设置缩放级别

        // 拍摄图像 - 轮询等待姿态完成或超时10秒
        auto start_time = std::chrono::steady_clock::now();
        bool posture_completed = false;

        // 轮询直到姿态完成或超时10秒
        while (!posture_completed &&
               std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::steady_clock::now() - start_time).count() < 10) {
            if (is_posture_completed()) {
                posture_completed = true;
            } else {
                // 短暂休眠避免CPU过度占用
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        if (posture_completed) {
            if (is_calibrate) {
                capture_image_for_reference(); // 标定模式下的拍照
            } else  {
                // TODO 巡航拍照识别

            }
        } //else noop

        // 判断是否是最后一个点位
        if (std::next(position) == preset_positions.end()) {
            //最后一个点位，可以开始切换到非巡逻模式
            patrolling = false;
            set_brighten(0); //不需要光照 补光灯关闭
        }

        if (is_calibrate)
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); //进行标定的话可以快速切换点位
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(position->duration));  // 每隔duration切换下一次点位

        now_point_id++;// 更新当前点位ID
    }
    return 0;
}

bool Camera::is_posture_completed()
{
    // 调用modbus请求
    if (modbus_ctx == nullptr)
        return false;

    // 读取保持寄存器
    uint16_t regs[1];
    int rc = modbus_read_registers(modbus_ctx, 0x4461, 1, regs);
    if (rc == -1) {
        std::cerr << "Failed to read rotation registers: " << modbus_strerror(errno) << std::endl;
    }

    if (regs[0]&0x1 && regs[0]&(1<<1) && regs[0]&(1<<2)) // 位操作读取三个参数分别的状态值
        return true;
    else
        return false;
}

int Camera::set_ptz(int horizontal, int vertical, int brightness)
{
    if (modbus_ctx == nullptr)
        return -1;

    if (horizontal==-1 && vertical==-1 && brightness==-1)
        return 0; // 忽略控制请求，不进行任何操作

   if (horizontal!=-1) {
        horizontal *= 100;
   }
   if (vertical!=-1) {
        vertical *= 100;
   }
    // 准备要写入的寄存器值
    uint16_t regs[3];
    regs[0] = static_cast<uint16_t>(vertical);   // 垂直角度
    regs[1] = static_cast<uint16_t>(horizontal); // 水平角度
    regs[2] = static_cast<uint16_t>(brightness);  // 亮度

    int rc = modbus_write_registers(modbus_ctx, MODBUSPTZ, 3, regs); // 全部写入寄存器
    if (rc == -1) {
        ALOGE("Failed to write PTZ registers: %s", modbus_strerror(errno));

        // 尝试重新连接并重试一次
        if (connect_modbus()) {
            rc = modbus_write_registers(modbus_ctx, MODBUSPTZ, 3, regs);
            if (rc == -1) {
                ALOGE("Retry failed to write PTZ registers: %s", modbus_strerror(errno));
                return -1;
            }
        } else {
            return -1;
        }
    }

    // 更新内部状态
    if (horizontal != -1)
        rotation_x = horizontal;
    if (vertical != -1)
        rotation_y = vertical;
    if (brightness != -1)
        this->brightness = brightness;

    return 0;
}

int Camera::set_brighten(int brightness)
{
    // 使用Modbus设置亮度
    if (modbus_ctx == nullptr) {
        std::cerr << "Modbus context not initialized" << std::endl;
        return -1;
    }

    // 准备要写入的寄存器值（假设亮度寄存器地址为0x4452）
    uint16_t brightness_reg = static_cast<uint16_t>(brightness);

    // 写入保持寄存器
    int rc = modbus_write_registers(modbus_ctx, MODBUSPTZ+2, 1, &brightness_reg);
    if (rc == -1) {
        std::cerr << "Failed to write brightness register: " << modbus_strerror(errno) << std::endl;
        return -1;
    }

    // 更新内部状态
    this->brightness = brightness;

    return 0;
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
        url += "&CAMPOS.focuspos=" + std::to_string(focus);
        todo = true;
    }

    if (!todo)
        return 0;

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
            }

            // 解析焦距位置
            size_t focuspos_pos = response.find("root.CAMPOS.focuspos=");
            if (focuspos_pos != std::string::npos) {
                size_t focuspos_end = response.find("\n", focuspos_pos);
                std::string focuspos_str = response.substr(focuspos_pos + 20, focuspos_end - focuspos_pos - 20);
                // 焦距位置可以存储在类成员变量中，如果需要的话
                // focuspos = std::stoi(focuspos_str);
            }
        }
    }

    //调用modbus获取云台姿态
    uint16_t regs[3];
    int rc = modbus_read_registers(modbus_ctx, MODBUSPTZ, 3, regs);
    if (rc == -1) {
        return -1;
    }
    rotation_x = regs[0]/100;
    rotation_y = regs[1]/100;
    brightness = regs[2];
    return 0;
}