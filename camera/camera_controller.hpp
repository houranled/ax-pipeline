#ifndef _CAMERA_CONTROLLER_H_
#define _CAMERA_CONTROLLER_H_

#include <thread>

class CameraController {
public:
    CameraController();
    ~CameraController();

    // 启动多个异步处理线程
    int start();
    // 关闭处理线程
    int stop();

    //添加转动
    int set_rotation(int horizontal, int vertical);
    //添加变焦
    int set_zoom(int zoom);
    //添加状态获取， 包括水平垂直方向，焦距
    int get_status(int *rotation, int *zoom, int *horizontal, int *vertical);

private:
    int panpos;  // 当前水平角度
    int tiltpos; // 当前垂直角度
    int zoompos; // 当前焦距

    std::thread input_thread;
    std::thread position_thread;
    int camera_id_;
    bool running;

    // a function for executing in new thread to receive the input read from std-io
    int receive_input_loop();

    // a function for executing in new thread to switch the camera state(position...)
    int camera_position_switch_loop();
};

#endif // _CAMERA_CONTROLLER_H_