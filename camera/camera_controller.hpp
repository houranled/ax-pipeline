#ifndef _CAMERA_CONTROLLER_H_
#define _CAMERA_CONTROLLER_H_

#include <thread>
#include <curl/curl.h>
#include <vector>
#include <string>

class Camera {
public:
    Camera();
    Camera(std::string ip);
    ~Camera();
    int pause();
    int resume();
    int set_rotation(int horizontal, int vertical);
    int set_zoom(int zoom);
    int get_status(int *rotation, int *zoom, int *horizontal, int *vertical);
    int get_id();

private:
    int panpos;  // 当前水平角度
    int tiltpos; // 当前垂直角度
    int zoompos; // 当前焦距
    std::string ip;   //
    int camera_id_;
    bool running;

    std::thread position_thread;
    CURL *curl_handle;  // 持久化的curl句柄
    modbus_t *modbus_ctx = nullptr;

    // a function for executing in new thread to switch the camera state(position...)
    int camera_position_switch_loop();

};

class CameraController {
public:
    CameraController();
    ~CameraController();

    //添加相机
    int add_camera();
    int start();
    // 关闭处理线程
    int stop();

    //添加状态获取， 包括水平垂直方向，焦距
    int get_status(int *rotation, int *zoom, int *horizontal, int *vertical);

private:
    std::vector<Camera*> cameras;
    std::thread input_thread;
    int camera_id_;
    bool running;
    static CURL *curl_handle;  // 持久化的curl句柄用于通过http与摄像机通信并控制

    // a function for executing in new thread to receive the input read from std-io
    int receive_input_loop();

};

#endif // _CAMERA_CONTROLLER_H_