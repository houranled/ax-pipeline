#include "camera_controller.hpp"
#include <iostream>


CameraController::CameraController(){}

CameraController::~CameraController(){}

/* TODO 命令队列（std::queue）
* 互斥锁保护队列访问
* 条件变量用于线程同步
* 专用的工作线程处理命令执行
*/
int CameraController::receive_input_loop() { // TODO
    std::string input;

    while (running) {
        // 读取用户输入
        std::getline(std::cin, input);

        // 解析输入命令
        if (input == "pause") {
            //pause();
        } else if (input.find("rotate") == 0) {
            // 解析旋转命令
            // 例如: "rotate 90 45" 表示水平旋转90度，垂直旋转45度
            int horizontal, vertical;
            if (sscanf(input.c_str(), "rotate %d %d", &horizontal, &vertical) == 2) {
                set_rotation(horizontal, vertical);
            } else {
                //std::cout << "Invalid rotate command. Usage: rotate <horizontal> <vertical>" << std::endl;
            }
        } else if (input.find("zoom") == 0) {
            // 解析缩放命令
            // 例如: "zoom 2" 表示缩放级别为2
            int zoom;
            if (sscanf(input.c_str(), "zoom %d", &zoom) == 1) {
                set_zoom(zoom);
            } else {
                //std::cout << "Invalid zoom command. Usage: zoom <level>" << std::endl;
            }
        } else if (input == "status") {
            // 获取并显示摄像头状态
            int rotation, zoom, horizontal, vertical;
            get_status(&rotation, &zoom, &horizontal, &vertical);
        } else if (input == "help") {
            // 显示帮助
        }
    }

    return 0;
}

int CameraController::camera_position_switch_loop() { // TODO
    while (running) {
        /* 这里实现点位切换逻辑
         * 例如：遍历预设点位列表，定期切换到下一个点位
         */

        // 每隔3600秒切换一次点位
        std::this_thread::sleep_for(std::chrono::minutes(10));

        // 切换到下一个点位:调用set_rotation和zoom函数

    }
    return 0;
}

int CameraController::start()
{
    running = true; // 标记控制器为运行状态

    // 创建并启动一个新线程来执行receive_input函数
    input_thread = std::thread(&CameraController::receive_input_loop, this);

    // 创建并启动一个新线程来执行camera_position_switch_loop函数
    position_thread = std::thread(&CameraController::camera_position_switch_loop, this);

    return 0;
}

int CameraController::stop()
{
    return 0;
}

int CameraController::set_rotation(int horizontal, int vertical) // TODO
{
    return 0;
}

int CameraController::set_zoom(int zoom) // TODO
{
    return 0;
}

int CameraController::get_status(int *rotation, int *zoom, int *horizontal, int *vertical) // TODO
{
    return 0;
}

