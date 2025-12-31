#include "camera_controller.hpp"
#include <iostream>
#include <curl/curl.h>
#include <curl/easy.h>
#include <modbus.h>

CameraController::CameraController()
{
    // 初始化libcurl全局环境
    CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
}

CameraController::~CameraController()
{
    // 清理libcurl全局环境
    curl_global_cleanup();
}

/* TODO 命令队列（std::queue）
* 互斥锁保护队列访问
* 条件变量用于线程同步
* 专用的工作线程处理命令执行
*/
int CameraController::receive_input_loop() {
    std::string input;

    while (running) {
        // 读取用户输入
        std::getline(std::cin, input);

        // 解析输入命令: id 命令 参数
        int camera_id = -1;  // 默认为-1，表示未指定ID
        std::string command;
        std::string params;

        // 尝试解析命令中的ID
        size_t space_pos = input.find(' ');
        if (space_pos != std::string::npos) {
            std::string id_str = input.substr(0, space_pos);
            // 检查是否是数字ID
            bool is_id = true;
            for (char c : id_str) {
                if (!isdigit(c)) {
                    is_id = false;
                    break;
                }
            }

            if (is_id) {
                camera_id = std::stoi(id_str);
                command = input.substr(space_pos + 1);
            } else {
                command = input;
            }
        } else {
            command = input;
        }

        // 根据命令执行操作
        if (command == "pause") {
            cameras[camera_id]->pause();
        } else if (command == "resume") {
            cameras[camera_id]->resume(); //恢复运行
        } else if (command.find("rotate") == 0) {
            // 解析旋转命令
            // 例如: "rotate 90 45" 表示水平旋转90度，垂直旋转45度
            int horizontal, vertical;
            if (sscanf(command.c_str(), "rotate %d %d", &horizontal, &vertical) == 2) {
                cameras[camera_id]->set_rotation(horizontal, vertical);
            } else {
                //std::cout << "Invalid rotate command. Usage: [id] rotate <horizontal> <vertical>" << std::endl;
            }
        } else if (command.find("zoom") == 0) {
            // 解析缩放命令
            // 例如: "zoom 2" 表示缩放级别为2
            int zoom;
            if (sscanf(command.c_str(), "zoom %d", &zoom) == 1) {
                cameras[camera_id]->set_zoom(zoom);
            } else {
                //std::cout << "Invalid zoom command. Usage: [id] zoom <level>" << std::endl;
            }
        } else if (command.find("status") == 0) {
            // 获取并显示摄像头状态
            int rotation, zoom, horizontal, vertical;
            cameras[camera_id]->get_status(&rotation, &zoom, &horizontal, &vertical);
            //通过标准输出向上层应用传输信息
        }
    }

    return 0;
}

int CameraController::add_camera()
{
    running = true; // 标记控制器为运行状态
    Camera* camera = new Camera(); // 创建一个新的相机对象
    cameras.push_back(camera); // 将相机对象添加到相机控制器列表中
    return 0;
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

    return 0;
}


int CameraController::get_status(int *rotation, int *zoom, int *horizontal, int *vertical) // TODO
{
    // TODO
    return 0;
}

//==============================

Camera::Camera()
{
    curl_handle = curl_easy_init();
    // 设置请求基本通用选项
    curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, 5L);  // 5秒超时
    curl_easy_setopt(curl_handle, CURLOPT_FOLLOWLOCATION, 1L);  // 跟随重定向


    { // 初始化 Modbus 连接
        modbus_ctx = modbus_new_tcp(ip.c_str(), 502); // 默认 Modbus TCP 端口为 502
        if (!modbus_ctx) {
            std::cerr << "Failed to create Modbus context: " << modbus_strerror(errno) << std::endl;
            // 可以选择抛出异常或设置错误标志
        } else {
            // 设置 Modbus 超时时间
            uint32_t to_sec = 1;    // 超时秒数
            uint32_t to_usec = 0;   // 超时微秒数
            modbus_set_response_timeout(modbus_ctx, to_sec, to_usec);

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

    position_thread = std::thread(&Camera::camera_position_switch_loop, this);
}

Camera::Camera(std::string ip)
{
    this->ip = ip;
}

Camera::~Camera()
{
    curl_easy_cleanup(curl_handle);
}

int Camera::pause()
{
    if (position_thread.joinable()) {
        position_thread.join();
    }
    return 0;
}

int Camera::resume()
{
    position_thread = std::thread(&Camera::camera_position_switch_loop, this);
    return 0;
}
int Camera::get_id()
{
    return 0;
}

int Camera::camera_position_switch_loop()  // TODO
{
    //获取点位表
    std::vector<std::pair<int, int>> preset_positions = {
        {0, 0},    // 点位1: 水平0度，垂直0度
        {45, 30},   // 点位2: 水平45度，垂直30度
        {90, 45},   // 点位3: 水平90度，垂直45度
        {135, 60},  // 点位4: 水平135度，垂直60度
        {180, 30}   // 点位5: 水平180度，垂直30度
    }; // 临时写死

    size_t current_position_index = 0;
    while (running) {

        /* 这里实现点位切换逻辑
         * 例如：遍历预设点位列表，定期切换到下一个点位
         */

        //调用set_rotation和zoom函数
        auto& position = preset_positions[current_position_index];
        set_rotation(position.first, position.second);
        set_zoom(1); // 设置缩放级别为1

        current_position_index++; //切换到下一个点位

        // 每隔3600秒切换一次点位
        std::this_thread::sleep_for(std::chrono::minutes(10));
    }
    return 0;
}

int Camera::set_rotation(int horizontal, int vertical)
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
        std::cerr << "Failed to write rotation registers: " << modbus_strerror(errno) << std::endl;
        return -1;
    }

    // 更新内部状态
    panpos = horizontal;
    tiltpos = vertical;

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

    zoompos = zoom;

    return 0;
}

// 回调函数，用于处理HTTP响应
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t totalSize = size * nmemb;
    ((std::string*)userp)->append((char*)contents, totalSize);
    return totalSize;
}

int Camera::get_status(int *rotation, int *zoom, int *horizontal, int *vertical)
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
            *zoom = std::stoi(zoompos_str);
            zoompos = *zoom;  // 更新内部状态
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
    //*rotation = rotation;
    //*horizontal = horizontal;
    return 0;
}