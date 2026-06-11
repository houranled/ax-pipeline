# ax-pipeline

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](https://raw.githubusercontent.com/AXERA-TECH/ax-pipeline/main/LICENSE)
[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/AXERA-TECH/ax-pipeline/build.yml?branch=main)](https://github.com/AXERA-TECH/ax-pipeline/actions)

## 简介

**AX-Pipeline** 是基于 AXera 系列芯片的工业智能巡检多媒体应用框架。该项目以 AXera-Pi 系列板卡为载体，集成 **ISP 图像处理**、**NPU 推理**、**视频编码**、**RTSP 推流**、**HDMI 显示** 等基础多媒体能力，并在此基础上扩展了 **多路摄像机接入**、**云台 PTZ 控制**、**预置位自动巡检**、**损伤检测**、**同点位差异对比**、**告警生成与管理** 等业务功能，方便开发者快速构建和二次开发工业巡检及智能安防等应用。

### 已支持芯片

- AX630C/AX620Q
  - SDK:v1.0.0
- AX650A/AX650N
  - SDK:v1.45.0

### 已支持开发板

- [AXera-Pi Pro](https://wiki.sipeed.com/m4ndock)(AX650N)
- [AXera-Pi Zero](https://axera-pi-zero-docs-cn.readthedocs.io/zh-cn/latest/index.html)(AX620Q)

## 快速上手

### 文档

- [快速编译](docs/compile.md)  基于 cmake 实现简单的跨平台编译。
- [如何更换自己训练的 yolov5 模型](docs/how_to_deploy_custom_yolov5_model.md)
- [如何部署自己的其他模型](docs/how_to_deploy_custom_model.md)
- [如何调整图像方向](docs/how_to_adjust_image_orientation.md)
- [ModelZoo](docs/modelzoo.md) 一些支持或将支持的模型，和一些模型的说明
- [配置文件说明](docs/config_file.md)
- [简化版本 pipeline 构建 api](docs/new_pipeline.md)
- [如何加速子模块的下载](docs/how_to_speed_up_submodule_init.md)

### 示例

| 示例 | 简介|
|-|-|
| [sample_vin_ivps_npu_vo](examples/sample_vin_ivps_npu_vo) |IVPS 出两路视频，一路用作屏幕显示，一路用作 NPU 推理 |
| [sample_vin_ivps_npu_venc_rtsp](examples/sample_vin_ivps_npu_venc_rtsp) |IVPS 出三路视频，两路用作 RTSP 推流，一路用作 NPU 推理 |
| [sample_vin_ivps_npu_venc_rtsp_vo](examples/sample_vin_ivps_npu_venc_rtsp_vo) |IVPS 出三路视频，一路用作 RTSP 推流，一路用作屏幕显示，一路用作 NPU 推理|
| [sample_vin_ivps_npu_vo_h265](examples/sample_vin_ivps_npu_vo_h265) |IVPS 出三路视频，一路用作屏幕显示，一路用作 h265 文件保存，一路用作 NPU 推理|
| [sample_v4l2_ivps_npu_vo](examples/sample_v4l2_ivps_npu_vo) | USB的 jpeg 输入，IVPS 出两路视频，一路用作屏幕显示，一路用作 NPU 推理 |
| [sample_v4l2_user_ivps_npu_vo](examples/sample_v4l2_user_ivps_npu_vo) | USB的 jpeg 输入，使用libjpeg解码成NV12，输入到IVPS中，IVPS 出两路视频，一路用作屏幕显示，一路用作 NPU 推理，演示了如何将NV12的图像输入到IVPS中 |
| [sample_demux_ivps_npu_vo](examples/sample_demux_ivps_npu_vo) |读取 h264/mp4/rtsp 解码，通过IVPS 出两路视频，一路用作屏幕显示，一路用作 NPU 推理|
| [sample_demux_ivps_npu_rtsp](examples/sample_demux_ivps_npu_rtsp) | 读取 h264/mp4/rtsp 解码，IVPS 出两路视频，一路用作 RTSP 推流，一路用作 NPU 推理 |
| [sample_demux_ivps_npu_rtsp_vo](examples/sample_demux_ivps_npu_rtsp_vo) | 读取 h264/mp4/rtsp 解码，IVPS出三路视频，一路用作屏幕显示，一路用作 RTSP 推流，一路用作 NPU 推理 |
| [sample_multi_demux_ivps_npu_multi_rtsp](examples/sample_multi_demux_ivps_npu_multi_rtsp) | 读取多路 h264/mp4/rtsp 解码，推理模型进行 OSD 后，多路 rtsp 输出 |
| [sample_demux_ivps_npu_hdmi_vo](examples/sample_demux_ivps_npu_hdmi_vo) | 读取 h264/mp4/rtsp 解码，推理多个模型进行 OSD 后，分屏（分屏数量等于指定模型个数）同时输出到 HDMI 屏幕 |

### 核心模块

| 模块 | 路径 | 简介 |
|-|-|-|
| `camera` | [camera/](camera) | 摄像机控制与管理模块。支持多路 RTSP 摄像机接入、云台控制（PTZ）、雨刷/补光灯/光敏控制、预置位自动巡检、拍照与录像、损伤片段独立录制、同点位前后图像差异对比；通过 Modbus TCP 与摄像机云台通信，可由配置文件自动构建相机实例。 |
| `alarm` | [alarm/](alarm) | 告警管理模块。支持运动检测、物体检测、区域入侵、越线检测等多种告警类型，提供告警冷却、批量差异对比告警生成及按通道分队列存储。 |

## 更新日志

详情请看 [更新日志](docs/update.md)

## 联动项目

- [ax-samples](https://github.com/AXERA-TECH/ax-samples) 该项目实现了常见的 深度学习开源算法 在 爱芯元智 的 AI SoC 上的示例代码，方便社区开发者进行快速评估和适配。
- NPU 工具链在线文档
  - [Pulsar2](https://pulsar2-docs.readthedocs.io/zh_CN/latest/)(Support AX650A/AX650N/AX630C/AX620Q)

## 技术讨论

- Github issues
- QQ 群: 139953715

## **免责声明**

*本项目仅用于开发者社区技术交流使用，无任何商业交付质量承诺*
