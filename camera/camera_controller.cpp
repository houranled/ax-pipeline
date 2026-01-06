#include "camera_controller.hpp"
#include <iostream>
#include <curl/curl.h>
#include <curl/easy.h>
#include "../examples/utilities/json.hpp"
#include <fstream>



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

        // 解析JSON字符串(处理JSON命令)
        nlohmann::json json = nlohmann::json::parse(json_str);
        nlohmann::json response;

        std::string command;
        std::string params;

        // 处理JSON命令
        int camera_id = json.value("id", -1);
        Camera* camera = NULL;
        if (camera_id > 0) {
            camera = cameras[camera_id];
        }

        std::string action = json.value("action", "");
        if (action == "get") {
            if (camera_id > 0 && camera != NULL) {
                response["id"] = camera_id;
                response["rotationx"] = camera->rotation_x;
                response["rotationy"] = camera->rotation_y;
                response["zoom"] = camera->zoom;
                response["focus"] = camera->focus;
                response["brightness"] = camera->brightness;
            } else if (camera_id > 0) {
                response["error"] = "Camera not found";
                response["status"] = "error";
            } else { // camera_id <= 0 所有摄像机信息组成json
                int index = 0;
                for (auto& pair : cameras) {
                    auto& camera = pair.second;
                    //获取每个相机的内部状态信息
                    response[index]["id"] = camera->id;
                    response[index]["rotationx"] = camera->rotation_x;
                    response[index]["rotationy"] = camera->rotation_y;
                    response[index]["zoom"] = camera->zoom;
                    response[index]["focus"] = camera->focus;
                    response[index]["brightness"] = camera->brightness;
                }
            }
        } else if (action == "set" || action == "add") {
            if (camera_id > 0 && camera != NULL) {
                int x, y, zoom, brightness;
                if (action == "set") {
                    x = json.value("rotatex", 0);
                    y = json.value("rotatey", 0);
                    zoom = json.value("zoom", 0);
                    brightness = json.value("brightness", 0);
                } else { //add
                    x = camera->rotation_x + json.value("rotatex", 0);
                    y = camera->rotation_y + json.value("rotatey", 0);
                    zoom = camera->zoom + json.value("zoom", 0);
                    brightness = camera->brightness + json.value("brightness", 0);
                }
                camera->set_ptz(x, y);
                camera->set_zoom(zoom);
                camera->set_brighten(brightness);

                response["status"] = "success";
            } else  {
                response["status"] = "fail";
            }
        } else { //unknown action
            response["status"] = "fail";
            response["message"] = "unknown action";
        }
        std::cout << response.dump() << std::endl;
    }

    return 0;
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

        // 解析 JSON
        nlohmann::json config;
        config_file >> config;

        // 遍历相机列表
        if (config.contains("chl_list") && config["chl_list"].is_array()) {
            for (const auto& camera_config : config["chl_list"]) {
                // 创建相机实例
                Camera* camera = new Camera();

                // 设置相机基本信息
                camera->id = camera_config["id"];

                if (camera_config.contains("ip")) {
                    camera->ip = camera_config["ip"];
                }

                // 设置 PTZ IP
                if (camera_config.contains("ptz_ip")) {
                    camera->ptz_ip = camera_config["ptz_ip"];
                }

                // 加载点位列表
                if (camera_config.contains("points") && camera_config["points"].is_array()) {
                    std::vector<Camera::PresetPosition> preset_positions;
                    for (const auto& point : camera_config["points"]) {
                        Camera::PresetPosition pos;
                        pos.id = point["id"];
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

//==============================

Camera::Camera()
{
    curl_handle = curl_easy_init();
    // 设置请求基本通用选项
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 5L);  // 5秒超时
    curl_easy_setopt(curl_handle, CURLOPT_FOLLOWLOCATION, 1L);  // 跟随重定向


    {   // 初始化Modbus 连接
        modbus_ctx = modbus_new_tcp(ip.c_str(), 502); // 默认Modbus TCP 端口为 502
        if (!modbus_ctx) {
            std::cerr << "Failed to create Modbus context: " << modbus_strerror(errno) << std::endl;
            // 可以选择抛出异常或设置错误标志
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
                modbus_set_slave(modbus_ctx, 1);
            }
        }
    }

}

Camera::~Camera()
{
    curl_easy_cleanup(curl_handle);
}

int Camera::start()
{
    position_thread = std::thread(&Camera::position_patrol_loop, this);
    return 0;
}

int Camera::pause()
{
    running = false;

    if (position_thread.joinable()) {
        position_thread.join();
    }
    return 0;
}

int Camera::get_id()
{
    return 0;
}

bool Camera::is_patroling() const
{
    return patroling;
}

void Camera::set_patroling(bool patroling)
{
    this->patroling = patroling;
}

std::string Camera::get_mode() const
{
    return std::string();
}

int Camera::add_preset_position(Camera::PresetPosition pos)
{
    this->preset_positions.push_back(pos);
    return 0;
}

int Camera::position_patrol_loop()  // TODO
{
    int minutes = 10;
    size_t current_position_index = 0;
    while (running) {
        patroling = true; // 标识进入巡逻模式

        /* 这里实现点位切换逻辑
         * 例如：遍历预设点位列表，定期切换到下一个点位
         */

        for(auto position = preset_positions.begin(); position != preset_positions.end(); ++position) {
            set_ptz(position->rotation_x, position->rotation_y); // 设置转向
            set_zoom(position->zoom); // 设置缩放级别
            set_brighten(position->brightness); // 设置亮度

            //查询是否已移动到指定位置
            is_on_the_place();
            //TODO 持续拍摄图像

            // 判断是否是最后一个点位
            if (std::next(position) == preset_positions.end()) {
                //最后一个点位，可以开始进入非巡航模式
                patroling = false;
            }

            // 每隔duration切换下一次点位
            std::this_thread::sleep_for(std::chrono::minutes(position->duration));
        }
    }
    return 0;
}

bool Camera::is_on_the_place()
{
    //调用modbus请求
    if (modbus_ctx == nullptr)
        return false;

    // TODO读取保持寄存器
    uint16_t regs[2];
    int rc = modbus_read_registers(modbus_ctx, 100, 2, regs);
    if (rc == -1) {
        std::cerr << "Failed to read rotation registers: " << modbus_strerror(errno) << std::endl;
    }

    return false;
}

int Camera::set_ptz(int horizontal, int vertical)
{
    if (modbus_ctx == nullptr)
        return -1;

    // 准备要写入的寄存器值
    uint16_t regs[2];
    regs[0] = static_cast<uint16_t>(horizontal);  // 水平角度
    regs[1] = static_cast<uint16_t>(vertical);    // 垂直角度

    // 写入保持寄存器（假设水平角度寄存器地址为 100，垂直角度寄存器地址为 101）
    int rc = modbus_write_registers(modbus_ctx, 100, 2, regs);

    if (rc == -1) {
        return -1;
    }

    // 更新内部状态
    rotation_x = horizontal;
    rotation_y = vertical;

    return 0;
}

int Camera::set_zoom(int zoom) // TODO
{
    //调用libcurl库
    // 构造包含缩放参数的URL
    std::string url = "http://" + ip + "/cgi-bin/param.cgi?action=update&group=CAMPOS&channel=0&CAMPOS.zoompos="
        + std::to_string(zoom);

    // 向curl设置请求URL
    curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());

    // 执行请求
    CURLcode res = curl_easy_perform(curl_handle);
    if(res != CURLE_OK) {
        return -1;
    }

    this->zoom = zoom;
    return 0;
}

int Camera::set_brighten(int brightness)
{
    // 调用libcurl库
    // 构造包含亮度参数的URL
    std::string url = "http://" + ip + "/cgi-bin/param.cgi?action=update&group=CAMPOS&channel=0&CAMPOS.brightness="
        + std::to_string(brightness);

    // 向curl设置请求URL
    curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());

    // 执行请求
    CURLcode res = curl_easy_perform(curl_handle);
    if(res != CURLE_OK) {
        std::cerr << "Failed to set brightness: " << curl_easy_strerror(res) << std::endl;
        return -1;
    }

    return 0;
}


// 回调函数，用于处理HTTP响应
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t totalSize = size * nmemb;
    ((std::string*)userp)->append((char*)contents, totalSize);
    return totalSize;
}

int Camera::fetch_remote_status()
{
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
                return -1;
            }
        }

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
    //调用modbus获取云台姿态
    uint16_t regs[3];
    int rc = modbus_read_registers(modbus_ctx, 100, 2, regs);
    if (rc == -1) {
        return -1;
    }
    rotation_x = regs[0];
    rotation_y = regs[1];
    return 0;
}