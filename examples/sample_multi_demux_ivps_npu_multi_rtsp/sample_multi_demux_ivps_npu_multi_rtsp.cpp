/*
 * AXERA is pleased to support the open source community by making ax-samples available.
 *
 * Copyright (c) 2022, AXERA Semiconductor (Shanghai) Co., Ltd. All rights reserved.
 *
 * Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the License at
 *
 * https://opensource.org/licenses/BSD-3-Clause
 *
 * Unless required by applicable law or agreed to in writing, software distributed
 * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

/*
 * Author: ZHEQIUSHUI
 */

#include "../libaxdl/include/c_api.h"
#include "../libaxdl/include/ax_osd_helper.hpp"
#include "../common/common_func.h"
#include "common_pipeline.h"
#include "../utilities/sample_log.h"

#include "../common/video_demux.hpp"

#include "ax_ivps_api.h"

#include "fstream"
#include <getopt.h>
#include "unistd.h"
#include "stdlib.h"
#include "string.h"
#include "signal.h"
#include "vector"
#include "map"
#include <thread>
#include <string>

#include "opencv2/opencv.hpp"
#include "../../../camera/camera_controller.hpp"
#include <list>
#include <cstdint>
#include "git_version.h"

#define pipe_count 2

// 后台批量写入线程函数
typedef struct {
    pipeline_t *pipe;
    std::vector<std::vector<uint8_t>> frames;  // 移动过来的帧数据
    char filename[256];
    char ffmpeg_cmd[512];
} writer_thread_args_t;

void* batch_write_thread(void* arg);
void cleanup_frame_buffer(pipeline_t *pipe);

// ===== 损伤片段独立录像 =====
// 状态值（与 pipeline_t::damage_state 对应）
#define DC_IDLE     0
#define DC_STAYING  1
#define DC_POST     2

static bool is_hevc_keyframe(const uint8_t* data, size_t size);
static void damage_push_pre_buf(pipeline_t* pipe, const uint8_t* data, size_t size, bool is_key);
static void damage_append_clip_locked(pipeline_t* pipe, const uint8_t* data, size_t size);
static void* damage_writer_thread_fn(void* arg);
static void damage_finalize_clip(pipeline_t* pipe); // 必须在持有 damage_mutex 时调用
void cleanup_damage_buffer(pipeline_t *pipe);

// 由 Camera 类在巡检过程中调用（在 camera_controller.cpp 中通过 extern 引用）
void damage_pipeline_on_arrived(pipeline_t* pipe, const char* clip_filename);
void damage_pipeline_on_leaving(pipeline_t* pipe);
void damage_pipeline_mark_seen(pipeline_t* pipe);

#ifdef AXERA_TARGET_CHIP_AX620
#define rtsp_max_count 2
#elif defined(AXERA_TARGET_CHIP_AX650)
#define rtsp_max_count 6
#elif defined(AXERA_TARGET_CHIP_AX620E)
#define rtsp_max_count 2
#endif

AX_S32 s_sample_framerate = 25;

volatile AX_S32 gLoopExit = 0;

int SAMPLE_MAJOR_STREAM_WIDTH = 1920;
int SAMPLE_MAJOR_STREAM_HEIGHT = 1080;

int SAMPLE_IVPS_ALGO_WIDTH[rtsp_max_count] = {960};
int SAMPLE_IVPS_ALGO_HEIGHT[rtsp_max_count] = {540};

static struct _g_sample_
{
    struct _model
    {
        int bRunJoint;
        std::vector<void *> gModels;  // 改为支持多个模型
        ax_osd_helper osd_helper;
        std::vector<pipeline_t *> pipes_need_osd;
    } gModels[rtsp_max_count];

    // int bRunJoint[4];
    // void *gModels[rtsp_max_count];
    // std::map<int, ax_osd_helper> osd_helpers;
    // std::vector<pipeline_t *> pipes_need_osd[rtsp_max_count];
    std::map<int, _model *> osd_target_map;
    // std::map<int, int> osd_target;
    void Init()
    {
        for (size_t i = 0; i < rtsp_max_count; i++)
        {
            gModels[i].osd_helper.Stop();
            gModels[i].pipes_need_osd.clear();
            gModels[i].bRunJoint = 0;
            // pthread_mutex_init(&g_result_mutexs[i], NULL);
        }
        osd_target_map.clear();
        // memset(&g_result_disps[0], 0, sizeof(g_result_disps));

        ALOGN("g_sample Init\n");
    }
    void Deinit()
    {

        for (size_t i = 0; i < rtsp_max_count; i++)
        {
            gModels[i].osd_helper.Stop();
            gModels[i].pipes_need_osd.clear();
            gModels[i].bRunJoint = 0;
            // pthread_mutex_init(&g_result_mutexs[i], NULL);
        }

        ALOGN("g_sample Deinit\n");
    }
} g_sample;

void ai_inference_func(pipeline_buffer_t *buff)
{
    pipeline_t *pipe = (pipeline_t *)buff->p_pipe;
        if (g_sample.osd_target_map[pipe->pipeid]->bRunJoint) {
            static std::map<int, axdl_results_t> mResults;
            axdl_image_t tSrcFrame = {0};
            switch (buff->d_type)
            {
            case po_buff_nv12:
                tSrcFrame.eDtype = axdl_color_space_nv12;
                break;
            case po_buff_bgr:
                tSrcFrame.eDtype = axdl_color_space_bgr;
                break;
            case po_buff_rgb:
                tSrcFrame.eDtype = axdl_color_space_rgb;
                break;
            default:
                break;
            }
            tSrcFrame.nWidth = buff->n_width;
            tSrcFrame.nHeight = buff->n_height;
            tSrcFrame.pVir = (unsigned char *)buff->p_vir;
            tSrcFrame.pPhy = buff->p_phy;
            tSrcFrame.tStride_W = buff->n_stride;
            tSrcFrame.nSize = buff->n_size;

            // 初始化合并后的结果
            mResults[pipe->pipeid].nObjSize = 0;

            if (pipe->m_pcamera && (pipe->m_pcamera->ptz_ip.empty() || pipe->m_pcamera->is_patroling())) {
                // 使用所有模型对同一帧图像进行推理，并合并结果
                int max_obj_per_model = SAMPLE_MAX_BBOX_COUNT / g_sample.osd_target_map[pipe->pipeid]->gModels.size();
                for (auto each_model : g_sample.osd_target_map[pipe->pipeid]->gModels) {
                    axdl_results_t model_result;
                    memset(&model_result, 0, sizeof(axdl_results_t));

                    axdl_inference(each_model, &tSrcFrame, &model_result);

                    // 合并高层多模型结果到统一结果中（每个模型最多取[SAMPLE_MAX_BBOX_COUNT/模型总数]个目标）
                    for (int i = 0; i < model_result.nObjSize; i++) {
                        if (mResults[pipe->pipeid].nObjSize < SAMPLE_MAX_BBOX_COUNT && i < max_obj_per_model) {
                            memcpy(&mResults[pipe->pipeid].mObjects[mResults[pipe->pipeid].nObjSize],
                                &model_result.mObjects[i], sizeof(axdl_object_t));

                            mResults[pipe->pipeid].nObjSize++;
                        } else break;
                    }
                    mResults[pipe->pipeid].niFps = model_result.niFps; // 保存推理FPS
                }
            }

            // ALOGI("pipe=%d detect%d", pipe->pipeid, mResults[pipe->pipeid].nObjSize);
            g_sample.osd_target_map[pipe->pipeid]->osd_helper.Update(&mResults[pipe->pipeid]);
        }
}

void h265_save_func(pipeline_buffer_t *buff)
{
    static uint frame_num = 0;
    frame_num++;

    pipeline_t *pipe = (pipeline_t *)buff->p_pipe;

    // 检查buff指针有效性
    if (buff == NULL || pipe == NULL) {
        WTALOGI("无效的缓冲区或管道指针");
        return;
    }

    size_t one_frame_size = buff->n_size;
    void *one_frame_data = buff->p_vir;

    bool is_key = false;
    // ===== 3 秒滚动预录缓冲（常驻维护） =====
    // 不论 IsRecordVideo / damage_state 是否激活，都维护预录缓冲，
    // 以便"人员检测录像"和"损伤片段录像"都能从更早的 I 帧切入，包含触发前 3 秒画面。
    if (one_frame_data != NULL && one_frame_size > 0) {
        is_key = is_hevc_keyframe((const uint8_t*)one_frame_data, one_frame_size);

        pthread_mutex_lock(&pipe->damage_mutex);
        // 首次进入时按 fps 计算 max_frames（1 秒）
        if (pipe->damage_pre_max_frames == 0) {
            int fps = pipe->m_ivps_attr.n_ivps_fps > 0 ? pipe->m_ivps_attr.n_ivps_fps : 25;
            pipe->damage_pre_max_frames = (size_t)(fps * 1);
        }
        damage_push_pre_buf(pipe, (const uint8_t*)one_frame_data, one_frame_size, is_key);

        // 损伤片段状态机：STAYING / POST 期间累积到 damage_clip_frames
        if (pipe->damage_state == DC_STAYING || pipe->damage_state == DC_POST) {
            damage_append_clip_locked(pipe, (const uint8_t*)one_frame_data, one_frame_size);

            if (pipe->damage_state == DC_POST) {
                if (pipe->damage_post_remaining > 0) {
                    pipe->damage_post_remaining--;
                }
                if (pipe->damage_post_remaining <= 0) {
                    damage_finalize_clip(pipe);
                }
            }
        }
        pthread_mutex_unlock(&pipe->damage_mutex);
    }

    if (!pipe->IsRecordVideo && pipe->writer_running) { //结束录制，且已经在执行存储写入过程了
        return;
    }

    // 录制期间：只缓存到内存，不写入 ffmpeg
    if (pipe->IsRecordVideo) {
        bool just_started = false;
        // 首次初始化：设置内存限制，并把 3 秒前置缓冲拼到录像开头
        if (pipe->frame_list.empty() && pipe->max_memory_limit == 0) {
            pipe->max_memory_limit = 100 * 1024 * 1024;  // 默认100MB限制
            just_started = true;

            // 将常驻维护的 damage_pre_buf 作为录像开头（含本帧，已保证首帧为 I 帧）
            pthread_mutex_lock(&pipe->damage_mutex);
            pthread_mutex_lock(&pipe->buffer_mutex);
            size_t preroll_size = 0;
            for (size_t i = 0; i < pipe->damage_pre_buf.size(); ++i) {
                preroll_size += pipe->damage_pre_buf[i].size();
                pipe->frame_list.push_back(pipe->damage_pre_buf[i]); // 拷贝
            }
            pipe->total_cached_size += preroll_size;
            size_t preroll_frames = pipe->damage_pre_buf.size();
            pthread_mutex_unlock(&pipe->buffer_mutex);
            pthread_mutex_unlock(&pipe->damage_mutex);

            WTALOGI("开始录制，内存缓存限制: %zu MB，已注入前置缓冲 %zu 帧(%.2f MB)",
                    pipe->max_memory_limit / (1024*1024),
                    preroll_frames, preroll_size / (1024.0*1024.0));
        }

        // 加锁保护 frame_list
        if (pthread_mutex_lock(&pipe->buffer_mutex) != 0) {
            WTALOGI("获取互斥锁失败");
            return;
        }

        // just_started 时本帧已通过 pre_buf 注入到 frame_list 末尾，不再重复 push
        if (!just_started) {
            // 检查内存限制
            if (pipe->total_cached_size + one_frame_size > pipe->max_memory_limit) {
                WTALOGI("内存缓存将达到上限(%zu/%zu)，丢弃旧帧!!", pipe->total_cached_size, pipe->max_memory_limit);
                // 移除最早的帧直到有足够空间
                while (!pipe->frame_list.empty() && pipe->total_cached_size + one_frame_size > pipe->max_memory_limit) {
                    pipe->total_cached_size -= pipe->frame_list.front().size();
                    pipe->frame_list.erase(pipe->frame_list.begin());
                }
            }

            pipe->frame_list.emplace_back((const uint8_t*)one_frame_data, (const uint8_t*)one_frame_data + one_frame_size);
            pipe->total_cached_size += one_frame_size;
        }

        size_t cached_frames = pipe->frame_list.size();
        pthread_mutex_unlock(&pipe->buffer_mutex);

        // 每300帧打印一次日志
        if (cached_frames % 300 == 0)
            WTALOGI("已缓存 %zu 帧，总大小: %zu KB", cached_frames, pipe->total_cached_size / 1024);

    } else if (!pipe->frame_list.empty() && !pipe->writer_running) { // 录制结束：启动后台线程批量写入
        pthread_mutex_lock(&pipe->buffer_mutex);
        // 准备线程参数
        writer_thread_args_t* args = new writer_thread_args_t;
        args->pipe = pipe;
        args->frames = std::move(pipe->frame_list);  // 移动所有权
        pipe->frame_list.clear();
        strncpy(args->filename, pipe->video_filename, sizeof(args->filename) - 1);
        args->filename[sizeof(args->filename) - 1] = '\0';

        // 保存 ffmpeg 命令
        sprintf(args->ffmpeg_cmd, "ffmpeg -y -loglevel quiet -f hevc -i - -c:v copy -f mp4 %s 2>/dev/null", args->filename);

        size_t total_frames = args->frames.size();
        size_t total_size = pipe->total_cached_size;
        pipe->total_cached_size = 0;  // 重置计数

        pthread_mutex_unlock(&pipe->buffer_mutex);

        WTALOGI("录制结束，启动后台线程写入 %zu 帧(%.2f MB)到 %s", total_frames, total_size / (1024.0*1024.0), args->filename);

        // 启动后台写入线程
        pipe->writer_running = true;
        if (pthread_create(&pipe->writer_thread, NULL, batch_write_thread, args) != 0) {
            WTALOGI("创建后台写入线程失败");
            pipe->writer_running = false;
            delete args;
        }
    }

    // 处理截图请求
    if (pipe->whatPicture) {
        //record_ffmpeg_pipe_jpg(pipe, frame_data, frame_size);
    }

    // 处理人检测快照请求：等到 I 帧后用 ffmpeg 解码保存为 PNG
    if (pipe->person_snapshot_requested && one_frame_data != NULL && one_frame_size > 0) {
        if (is_key) {
            char cmd[512];
            sprintf(cmd, "ffmpeg -y -loglevel quiet -f hevc -i - -vframes 1 %s 2>/dev/null",
                    pipe->person_snapshot_filename);

            FILE* fp = popen(cmd, "w");
            if (fp) {
                size_t written = fwrite(one_frame_data, 1, one_frame_size, fp);
                fflush(fp);
                pclose(fp);
                if (written == one_frame_size) {
                    WTALOGI("[PersonSnapshot] 通道[%s] 快照已保存: %s",
                            pipe->channel_name, pipe->person_snapshot_filename);
                }
            } else {
                WTALOGI("[PersonSnapshot] 创建 ffmpeg 管道失败");
            }
            pipe->person_snapshot_requested = false;
            pipe->person_snapshot_filename[0] = '\0';
        }
    }
}

/**
 * @brief 从 H.265 帧缓冲中提取首个关键帧，用 ffmpeg 解码保存为 PNG 封面图
 * @param frames 帧数据列表
 * @param video_path 视频文件完整路径（.mp4），封面路径自动替换后缀为 .png
 */
static void generate_video_cover(const std::vector<std::vector<uint8_t>>& frames, const char* video_path)
{
    if (frames.empty() || video_path == NULL || video_path[0] == '\0') return;

    // 生成封面路径：与视频同名，后缀替换为 .png
    std::string cover_path(video_path);
    size_t dot_pos = cover_path.rfind('.');
    if (dot_pos != std::string::npos) {
        cover_path = cover_path.substr(0, dot_pos) + ".png";
    } else {
        cover_path += ".png";
    }

    // 找到首个关键帧
    const std::vector<uint8_t>* key_frame = nullptr;
    for (const auto& frame : frames) {
        if (is_hevc_keyframe(frame.data(), frame.size())) {
            key_frame = &frame;
            break;
        }
    }
    if (!key_frame) {
        WTALOGI("[封面] 未找到关键帧，跳过封面生成: %s", video_path);
        return;
    }

    // 用 ffmpeg 将关键帧解码为 PNG
    char cmd[640];
    snprintf(cmd, sizeof(cmd),
             "ffmpeg -y -loglevel quiet -f hevc -i - -vframes 1 %s 2>/dev/null",
             cover_path.c_str());

    FILE* fp = popen(cmd, "w");
    if (!fp) {
        WTALOGI("[封面] 创建 ffmpeg 管道失败: %s", cover_path.c_str());
        return;
    }
    fwrite(key_frame->data(), 1, key_frame->size(), fp);
    fflush(fp);
    pclose(fp);
    WTALOGI("[封面] 已生成: %s", cover_path.c_str());
}

/**
 * @brief 后台批量写入线程函数
 * 将缓存的所有帧数据一次性写入 ffmpeg 管道生成 mp4 文件
 */
void* batch_write_thread(void* arg)
{
    writer_thread_args_t* args = (writer_thread_args_t*)arg;
    std::vector<std::vector<uint8_t>>& frames = args->frames;
    pipeline_t* pipe = args->pipe;

    // 创建 ffmpeg 管道
    FILE* ffmpeg_pipe = popen(args->ffmpeg_cmd, "w");
    if (!ffmpeg_pipe) {
        WTALOGI("创建 ffmpeg 管道失败: %s", args->ffmpeg_cmd);
        delete args;
        pipe->writer_running = false;
        return NULL;
    }

    // 批量写入所有帧
    size_t total_written = 0;
    size_t frame_count = frames.size();

    for (size_t i = 0; i < frame_count; i++) {
        const std::vector<uint8_t>& frame = frames[i];
        size_t frame_size = frame.size();

        // 写入一帧
        size_t written = fwrite(frame.data(), 1, frame_size, ffmpeg_pipe);
        if (written != frame_size) {
            WTALOGI("写入帧 %zu/%zu 失败: 期望 %zu, 实际 %zu", i + 1, frame_count, frame_size, written);
        }

        total_written += written;
    }

    // 最终刷新确保所有数据写入
    fflush(ffmpeg_pipe);

    // 关闭管道
    int ret = pclose(ffmpeg_pipe);
    if (ret == -1) {
        WTALOGI("关闭 ffmpeg 管道失败");
    }

    WTALOGI("后台写入完成，总计 %zu 帧，%zu 字节写入文件.", frame_count, total_written);

    // 生成首帧封面图
    generate_video_cover(frames, args->filename);

    // 清理线程参数
    delete args;

    // 更新管道状态
    pthread_mutex_lock(&pipe->buffer_mutex);
    pipe->writer_running = false;
    pipe->video_filename[0] = '\0';  // 清空文件名
    pthread_mutex_unlock(&pipe->buffer_mutex);

    return NULL;
}

/**
 * @brief 清理帧缓存区
 * 如果有正在运行的后台写入线程，等待其完成
 */
void cleanup_frame_buffer(pipeline_t *pipe)
{
    if (pipe == NULL) {
        ALOGE("无效的管道指针");
        return;
    }

    // 等待后台写入线程完成
    if (pipe->writer_running) {
        ALOGI("等待后台写入线程完成...");
        pthread_join(pipe->writer_thread, NULL);
        ALOGI("后台写入线程已退出");
    }

    // 加锁保护 frame_list
    if (pthread_mutex_lock(&pipe->buffer_mutex) != 0) {
        ALOGE("获取互斥锁失败");
        return;
    }

    // 清空帧列表并释放内存
    if (!pipe->frame_list.empty()) {
        ALOGI("清理未写入的 %zu 帧缓存", pipe->frame_list.size());
        pipe->frame_list.clear();
        pipe->frame_list.shrink_to_fit();  // 释放 vector 占用的内存
    }

    pipe->total_cached_size = 0;
    pipe->max_memory_limit = 0;
    pipe->writer_running = false;

    pthread_mutex_unlock(&pipe->buffer_mutex);
}

// ============================================================
// 损伤片段独立录像实现
// ============================================================

/**
 * @brief 检测 H.265 / HEVC 一帧码流是否为关键帧（IRAP：BLA/IDR/CRA, NAL 类型 16~21）
 * 通过扫描起始码 0x000001 / 0x00000001 后的 NAL header，提取 nal_unit_type。
 */
static bool is_hevc_keyframe(const uint8_t* data, size_t size)
{
    if (data == NULL || size < 5) return false;
    for (size_t i = 0; i + 3 < size; ++i) {
        bool sc4 = (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1);
        bool sc3 = (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1);
        if (!sc3 && !sc4) continue;
        size_t nalu_pos = i + (sc4 ? 4 : 3);
        if (nalu_pos >= size) break;
        int nal_type = (data[nalu_pos] & 0x7E) >> 1;
        if (nal_type >= 16 && nal_type <= 21) return true; // IRAP
        i = nalu_pos; // 跳过本 NAL 头继续扫描
    }
    return false;
}

/**
 * @brief 将一帧追加到 3 秒滚动预录缓冲，保持首帧为关键帧。
 * 必须在持有 pipe->damage_mutex 时调用。
 */
static void damage_push_pre_buf(pipeline_t* pipe, const uint8_t* data, size_t size, bool is_key)
{
    std::vector<uint8_t> v(data, data + size);
    pipe->damage_pre_buf.emplace_back(std::move(v));
    pipe->damage_pre_keyflag.push_back(is_key ? 1 : 0);
    pipe->damage_pre_total_size += size;

    // 超过 max_frames 时丢弃旧帧，并保证首帧仍是关键帧
    while (pipe->damage_pre_buf.size() > pipe->damage_pre_max_frames) {
        pipe->damage_pre_total_size -= pipe->damage_pre_buf.front().size();
        pipe->damage_pre_buf.pop_front();
        pipe->damage_pre_keyflag.pop_front();
    }
    // 进一步把首帧前的非关键帧丢掉（直到首帧是 key 或缓冲为空）
    while (!pipe->damage_pre_keyflag.empty() && pipe->damage_pre_keyflag.front() == 0) {
        pipe->damage_pre_total_size -= pipe->damage_pre_buf.front().size();
        pipe->damage_pre_buf.pop_front();
        pipe->damage_pre_keyflag.pop_front();
    }
}

/**
 * @brief 将一帧追加到当前损伤片段累积缓冲（带 100MB 上限保护）。
 * 必须在持有 pipe->damage_mutex 时调用。
 */
static void damage_append_clip_locked(pipeline_t* pipe, const uint8_t* data, size_t size)
{
    const size_t kCap = 100 * 1024 * 1024;
    if (pipe->damage_clip_total_size + size > kCap) {
        // 内存饱和：放弃后续累积（保留已有数据，便于后续 flush）
        static int s_warn_count = 0;
        if ((s_warn_count++ % 60) == 0) {
            WTALOGI("损伤片段缓存达到上限 %zu MB，停止继续累积!", kCap / (1024*1024));
        }
        return;
    }
    std::vector<uint8_t> v(data, data + size);
    pipe->damage_clip_frames.emplace_back(std::move(v));
    pipe->damage_clip_total_size += size;
}

/**
 * @brief 收尾损伤片段：根据 damage_seen 决定写文件或丢弃。重置状态机为 IDLE。
 * 必须在持有 pipe->damage_mutex 时调用。
 */
static void damage_finalize_clip(pipeline_t* pipe)
{
    bool should_save = pipe->damage_seen && !pipe->damage_clip_frames.empty() && pipe->damage_clip_filename[0] != '\0';

    if (should_save && !pipe->damage_writer_running) {
        writer_thread_args_t* args = new writer_thread_args_t;
        args->pipe = pipe;
        args->frames = std::move(pipe->damage_clip_frames);
        strncpy(args->filename, pipe->damage_clip_filename, sizeof(args->filename) - 1);
        args->filename[sizeof(args->filename) - 1] = '\0';
        sprintf(args->ffmpeg_cmd, "ffmpeg -y -loglevel quiet -f hevc -i - -c:v copy -f mp4 %s 2>/dev/null", args->filename);

        size_t total_frames = args->frames.size();
        size_t total_size = pipe->damage_clip_total_size;
        WTALOGI("损伤片段触发落盘 %zu 帧 (%.2f MB) -> %s",
                total_frames, total_size / (1024.0 * 1024.0), args->filename);

        pipe->damage_writer_running = true;
        if (pthread_create(&pipe->damage_writer_thread, NULL, damage_writer_thread_fn, args) != 0) {
            WTALOGI("创建损伤片段写入线程失败");
            pipe->damage_writer_running = false;
            delete args;
        }
    } else {
        if (!pipe->damage_seen) {
            WTALOGI("点位停留期间未检出损伤，丢弃片段 %zu 帧", pipe->damage_clip_frames.size());
        } else if (pipe->damage_writer_running) {
            // 兜底诊断：正常情况下 on_arrived 已等待上一写入线程结束，此分支不应触发。
            // 若仍触发说明写盘超时(>10s)，记录以便定位慢磁盘/大文件问题。
            WTALOGI("上一片段写入线程仍在运行，本片段无法落盘被丢弃 %zu 帧！！！", pipe->damage_clip_frames.size());
        }
        pipe->damage_clip_frames.clear();
    }

    // 重置状态
    pipe->damage_clip_frames.clear();
    pipe->damage_clip_total_size = 0;
    pipe->damage_state = DC_IDLE;
    pipe->damage_seen = false;
    pipe->damage_post_remaining = 0;
    pipe->damage_clip_filename[0] = '\0';
}

/**
 * @brief 损伤片段写入线程：与 batch_write_thread 同构，但更新 damage_writer_running。
 */
static void* damage_writer_thread_fn(void* arg)
{
    writer_thread_args_t* args = (writer_thread_args_t*)arg;
    std::vector<std::vector<uint8_t>>& frames = args->frames;
    pipeline_t* pipe = args->pipe;

    FILE* ffmpeg_pipe = popen(args->ffmpeg_cmd, "w");
    if (!ffmpeg_pipe) {
        WTALOGI("创建 ffmpeg 管道失败(损伤): %s", args->ffmpeg_cmd);
        delete args;
        pipe->damage_writer_running = false;
        return NULL;
    }

    size_t total_written = 0;
    size_t frame_count = frames.size();
    for (size_t i = 0; i < frame_count; i++) {
        const std::vector<uint8_t>& frame = frames[i];
        size_t written = fwrite(frame.data(), 1, frame.size(), ffmpeg_pipe);
        if (written != frame.size()) {
            WTALOGI("损伤片段写入帧 %zu/%zu 失败: 期望 %zu, 实际 %zu", i + 1, frame_count, frame.size(), written);
        }
        total_written += written;
    }
    fflush(ffmpeg_pipe);
    int ret = pclose(ffmpeg_pipe);
    if (ret == -1) {
        WTALOGI("关闭 ffmpeg 管道失败(损伤)");
    }
    WTALOGI("损伤片段写入完成: %zu 帧, %zu 字节", frame_count, total_written);

    // 生成首帧封面图
    generate_video_cover(frames, args->filename);

    delete args;

    pthread_mutex_lock(&pipe->damage_mutex);
    pipe->damage_writer_running = false;
    pthread_mutex_unlock(&pipe->damage_mutex);
    return NULL;
}

/**
 * @brief 清理损伤片段相关缓存与线程（在 pipeline 销毁时调用）。
 */
void cleanup_damage_buffer(pipeline_t *pipe)
{
    if (pipe == NULL) return;

    if (pipe->damage_writer_running) {
        ALOGI("等待损伤片段写入线程完成...");
        pthread_join(pipe->damage_writer_thread, NULL);
    }

    pthread_mutex_lock(&pipe->damage_mutex);
    pipe->damage_pre_buf.clear();
    pipe->damage_pre_buf.shrink_to_fit();
    pipe->damage_pre_keyflag.clear();
    pipe->damage_pre_keyflag.shrink_to_fit();
    pipe->damage_pre_total_size = 0;

    pipe->damage_clip_frames.clear();
    pipe->damage_clip_frames.shrink_to_fit();
    pipe->damage_clip_total_size = 0;

    pipe->damage_state = DC_IDLE;
    pipe->damage_seen = false;
    pipe->damage_post_remaining = 0;
    pipe->damage_clip_filename[0] = '\0';
    pipe->damage_writer_running = false;
    pthread_mutex_unlock(&pipe->damage_mutex);
}

/**
 * @brief 进入"到位"状态：将预录缓冲整体搬到 clip_frames 作为片段开头。
 * 由 camera_controller.cpp 在到位时调用。
 */
void damage_pipeline_on_arrived(pipeline_t* pipe, const char* clip_filename)
{
    if (pipe == NULL || clip_filename == NULL) return;

    // 如果上一个点位正在 POST 状态（等待落盘），先等待其完成
    int wait_count = 0;
    const int max_wait = 100; // 最多等待 5 秒 (100 * 50ms)
    while (pipe->damage_state == DC_POST && wait_count < max_wait) {
        usleep(50 * 1000); // 50ms
        wait_count++;
    }
    if (wait_count > 0) {
        WTALOGI("on_arrived 等待上一点位落盘完成，等待了 %d ms", wait_count * 50);
    }

    // ★ 再等待上一个点位的写入线程真正结束：damage_finalize_clip 创建写线程后会立即
    //   把状态置为 DC_IDLE，上面的 DC_POST 等待并不覆盖异步写盘阶段。若此处不等待，
    //   新片段在其 finalize 时 writer 仍在运行，会命中 !damage_writer_running 分支被
    //   静默丢弃（表现为：有告警但无视频）。此等待仅发生在巡检线程的点位切换间隙，
    //   不触及视频线程(h265_save_func)与推理线程(draw_custom)的热路径。
    int writer_wait = 0;
    const int max_writer_wait = 200; // 最多等待 10 秒 (200 * 50ms)
    for (;;) {
        pthread_mutex_lock(&pipe->damage_mutex);
        bool busy = pipe->damage_writer_running;
        pthread_mutex_unlock(&pipe->damage_mutex);
        if (!busy || writer_wait >= max_writer_wait) break;
        usleep(50 * 1000); // 50ms
        writer_wait++;
    }
    if (writer_wait > 0) {
        WTALOGI("on_arrived 等待上一点位写入线程完成，等待了 %d ms", writer_wait * 50);
    }

    pthread_mutex_lock(&pipe->damage_mutex);
    // 上一次的残留若未落盘，丢弃（理论上不会发生，保险起见）
    if (pipe->damage_state != DC_IDLE) {
        WTALOGI("on_arrived 时状态非 IDLE(%d)，强制重置丢弃 %zu 帧",
                pipe->damage_state, pipe->damage_clip_frames.size());
        pipe->damage_clip_frames.clear();
        pipe->damage_clip_total_size = 0;
    }

    strncpy(pipe->damage_clip_filename, clip_filename, sizeof(pipe->damage_clip_filename) - 1);
    pipe->damage_clip_filename[sizeof(pipe->damage_clip_filename) - 1] = '\0';

    // 把预录缓冲拷入 clip_frames（首帧已保证是关键帧）
    for (size_t i = 0; i < pipe->damage_pre_buf.size(); ++i) {
        pipe->damage_clip_frames.push_back(pipe->damage_pre_buf[i]);
        pipe->damage_clip_total_size += pipe->damage_pre_buf[i].size();
    }
    pipe->damage_seen = false;
    pipe->damage_post_remaining = 0;
    pipe->damage_state = DC_STAYING;
    WTALOGI("损伤片段进入 STAYING，预录起点 %zu 帧(%.2f MB), file=%s",
            pipe->damage_clip_frames.size(),
            pipe->damage_clip_total_size / (1024.0 * 1024.0),
            pipe->damage_clip_filename);
    pthread_mutex_unlock(&pipe->damage_mutex);
}

/**
 * @brief 进入"离开"状态：从 STAYING 切换到 POST，再录 3 秒后由 h265_save_func 触发落盘。
 */
void damage_pipeline_on_leaving(pipeline_t* pipe)
{
    if (pipe == NULL) return;
    pthread_mutex_lock(&pipe->damage_mutex);
    if (pipe->damage_state == DC_STAYING) {
        int fps = pipe->m_ivps_attr.n_ivps_fps > 0 ? pipe->m_ivps_attr.n_ivps_fps : 25;
        pipe->damage_post_remaining = fps * 1; // 再录1秒
        pipe->damage_state = DC_POST;
        WTALOGI("损伤片段进入 POST，再录 %d 帧, damage_seen=%d", pipe->damage_post_remaining, (int)pipe->damage_seen);
    } else if (pipe->damage_state == DC_POST) {
        // 已经在 POST 状态，忽略重复调用
    } else {
        // IDLE: 无需处理
    }
    pthread_mutex_unlock(&pipe->damage_mutex);
}

/**
 * @brief 由损伤检测模型在检出目标时调用，标记当前停留期间存在损伤。
 */
void damage_pipeline_mark_seen(pipeline_t* pipe)
{
    if (pipe == NULL) return;
    pthread_mutex_lock(&pipe->damage_mutex);
    if (pipe->damage_state == DC_STAYING || pipe->damage_state == DC_POST) {
        pipe->damage_seen = true;
    }
    pthread_mutex_unlock(&pipe->damage_mutex);
}



void _demux_frame_callback(const void *buff, int len, void *reserve)
{
    if (len == 0)
    {
        ALOGN("demux finish,quit the loop");
        gLoopExit = 1;
        return;
    }
    pipeline_buffer_t buf_h26x = {0};
    buf_h26x.p_vir = (void *)buff;
    buf_h26x.n_size = len;
    user_input((pipeline_t *)reserve, 1, &buf_h26x);
    usleep(5 * 1000);
}

// 允许外部调用
extern "C" AX_VOID __sigExit(int iSigNo)
{
    gLoopExit = 1;
    return;
}

static AX_VOID PrintHelp(char *testApp)
{
    printf("version:%s\n", PROJECT_VERSION_FULL); // 例如 1.1.3-343972f-dirty
    printf("date:   %s\n", GIT_COMMIT_DATE);
    printf("branch: %s\n", GIT_BRANCH);
    printf("Usage:%s -h for help\n\n", testApp);
    printf("\t-p: model config file path\n");

    printf("\t-f: mp4 file/rtsp url(just only support h264 format)\n");

    printf("\t-r: Sensor&Video Framerate (framerate need supported by sensor), default is 25\n");

    exit(0);
}

int main(int argc, char *argv[])
{
    if (SAMPLE_Check_Bsp_Version() != 0)
    {
        return -1;
    }


    optind = 0;
    gLoopExit = 0;
    g_sample.Init();

    AX_S32 isExit = 0, ch;
    AX_S32 s32Ret = 0;
    COMMON_SYS_ARGS_T tCommonArgs = {0};
    char rtsp_url[512];

    std::vector<std::vector<pipeline_t>> vpipelines;

    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, __sigExit);
    signal(SIGTERM, __sigExit);

    char config_file[256] = "/wt_tech/app/ax-pipeline/config/wt_rtsp.json"; //默认json配置文件
    char config_file_test[128]={0};

    ALOGN("sample begin\n\n");

    bool first_f  = true;
    int debug_camera_id = 1;
    while ((ch = getopt(argc, argv, "p:f:r:h")) != -1)
    {
        switch (ch)
        {
        case 'f':
        {
            strcpy(rtsp_url, optarg);
            ALOGI("rtsp url : %s", rtsp_url);
            std::string tmp_url(rtsp_url);

            std::string channel_name; // 通道名称
            if (!first_f) {
                channel_name = "tc"; // 除了第一个通道，其他通道名称都是tc
            } else {
                CameraController::getInstance()->remove_all_cameras();
                channel_name = "channel-" + std::to_string(debug_camera_id);
                first_f = false;
            }

            CameraController::getInstance()->addCamera(debug_camera_id, channel_name, tmp_url); //添加相机
            debug_camera_id++;
        }
        break;
        case 'p':
        {
            strcpy(config_file_test, optarg);
            break;
        }
        case 'r':
            s_sample_framerate = (AX_S32)atoi(optarg);
            if (s_sample_framerate <= 0)
            {
                s_sample_framerate = 30;
            }
            break;
        case 'h':
            isExit = 1;
            break;
        default:
            isExit = 1;
            break;
        }
    }


    if (isExit)
    {
        PrintHelp(argv[0]);
        exit(0);
    }

    if (CameraController::getInstance()->getAllCameras().size() > rtsp_max_count)
    {
        ALOGE("support only %d rtsp inputs", rtsp_max_count);
    }


    if (CameraController::getInstance()->getAllCameras().size() == 0)
    {
        ALOGE("video urls is empty");
        PrintHelp(argv[0]);
        exit(0);
    }

#ifdef AXERA_TARGET_CHIP_AX620
    COMMON_SYS_POOL_CFG_T poolcfg[] = {
        {1920, 1088, 1920, AX_YUV420_SEMIPLANAR, 20},
    };
#elif defined(AXERA_TARGET_CHIP_AX650) || defined(AXERA_TARGET_CHIP_AX620E)
    COMMON_SYS_POOL_CFG_T poolcfg[] = {
        {1920, 1088, 1920, AX_FORMAT_YUV420_SEMIPLANAR, uint32_t(CameraController::getInstance()->getAllCameras().size() * 6)},
    };
#endif
    tCommonArgs.nPoolCfgCnt = 1;
    tCommonArgs.pPoolCfg = poolcfg;
    /*step 1:sys init*/
    s32Ret = COMMON_SYS_Init(&tCommonArgs);
    if (s32Ret)
    {
        ALOGE("COMMON_SYS_Init failed,s32Ret:0x%x\n", s32Ret);
        return -1;
    }

    /*step 3:npu init*/
#ifdef AXERA_TARGET_CHIP_AX620
    AX_NPU_SDK_EX_ATTR_T sNpuAttr;
    sNpuAttr.eHardMode = AX_NPU_VIRTUAL_1_1;
    s32Ret = AX_NPU_SDK_EX_Init_with_attr(&sNpuAttr);
    if (0 != s32Ret)
    {
        ALOGE("AX_NPU_SDK_EX_Init_with_attr failed,s32Ret:0x%x\n", s32Ret);
        return -1;
    }
#else
#ifdef AXERA_TARGET_CHIP_AX650
    AX_ENGINE_NPU_ATTR_T npu_attr;
    memset(&npu_attr, 0, sizeof(npu_attr));
    npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    s32Ret = AX_ENGINE_Init(&npu_attr);
#elif defined(AXERA_TARGET_CHIP_AX620E)
    AX_ENGINE_NPU_ATTR_T npu_attr;
    memset(&npu_attr, 0, sizeof(npu_attr));
    npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_ENABLE;
    s32Ret = AX_ENGINE_Init(&npu_attr);
#endif
    if (0 != s32Ret)
    {
        ALOGE("AX_ENGINE_Init 0x%x", s32Ret);
        return -1;
    }
#endif

    vpipelines.resize(CameraController::getInstance()->getAllCameras().size());

    int i = 0;
    for (auto const &camera : CameraController::getInstance()->getAllCameras())
    {
        if (camera->getName() == "tc") {
            strcpy(config_file, "/wt_tech/app/ax-pipeline/config/wt_rtsp_tc.json"); // 叶片尖专用模型
        } else {
            if (strlen(config_file_test))
                strcpy(config_file, config_file_test); // -f参数指定的json配置文件
            else
                strcpy(config_file, "/wt_tech/app/ax-pipeline/config/wt_rtsp.json"); // 默认json配置文件
        }

        void *model1 = nullptr;
        AX_S32 s32Ret = axdl_parse_param_init(config_file, &model1, camera->getName().c_str(), camera->get_id());
        if (s32Ret != 0) {
            WTALOGI("主模型初始化失败，退出程序");
            goto EXIT_3;
        }
        else {
            g_sample.gModels[i].gModels.push_back(model1);
            s32Ret = axdl_get_ivps_width_height(g_sample.gModels[i].gModels[0], config_file, &SAMPLE_IVPS_ALGO_WIDTH[i],
                &SAMPLE_IVPS_ALGO_HEIGHT[i]);
            WTALOGI("IVPS AI channel width=%d height=%d", SAMPLE_IVPS_ALGO_WIDTH[i], SAMPLE_IVPS_ALGO_HEIGHT[i]);
        }

        // 加载第二个模型（例如人脸识别模型）
        char config_file_people[256] = "/wt_tech/app/ax-pipeline/config/wt_rtsp_people.json"; //人体识别

        void *model2 = nullptr;
        s32Ret = axdl_parse_param_init(config_file_people, &model2, camera->getName().c_str(), camera->get_id());
        if (s32Ret == 0) {
            g_sample.gModels[i].gModels.push_back(model2);
            int width, height;
            s32Ret = axdl_get_ivps_width_height(g_sample.gModels[i].gModels[1], config_file_people, &width, &height);
            WTALOGI("IVPS AI channel width=%d height=%d", width, height);
        } else {
            WTALOGI("人员识别模型初始化失败!!");
        }

        // 如果两个模型都加载失败，则退出程序
        if (g_sample.gModels[i].gModels.empty()) {
            WTALOGI("主模型和人员识别模型都初始化失败，退出程序");
            goto EXIT_3;
        }

        // 如果至少有一个模型加载成功，则启用联合推理
        g_sample.gModels[i].bRunJoint = !g_sample.gModels[i].gModels.empty();

        auto &pipelines = vpipelines[i];
        pipelines.resize(pipe_count);

        // 创建pipeline
        {
            pipeline_t &pipe1 = pipelines[1];
            {
                pipeline_ivps_config_t &config1 = pipe1.m_ivps_attr;
                config1.n_ivps_grp = pipe_count * i + 1; // 重复的会创建失败
                config1.n_ivps_fps = 60;
                config1.n_ivps_width = SAMPLE_IVPS_ALGO_WIDTH[i];
                config1.n_ivps_height = SAMPLE_IVPS_ALGO_HEIGHT[i];
                config1.b_letterbox = axdl_get_letterbox_enable(g_sample.gModels[i].gModels[0]);
                config1.n_fifo_count = 1; // 如果想要拿到数据并输出到回调 就设为1~4
            }
            pipe1.enable = g_sample.gModels[i].bRunJoint;
            pipe1.pipeid = pipe_count * i + 1;
            pipe1.m_input_type = pi_vdec_h264;
            if (g_sample.gModels[i].gModels[0] && g_sample.gModels[i].bRunJoint)
            {
                switch (axdl_get_color_space(g_sample.gModels[i].gModels[0]))
                {
                case axdl_color_space_rgb:
                    pipe1.m_output_type = po_buff_rgb;
                    break;
                case axdl_color_space_bgr:
                    pipe1.m_output_type = po_buff_bgr;
                    break;
                case axdl_color_space_nv12:
                default:
                    pipe1.m_output_type = po_buff_nv12;
                    break;
                }
            }
            else
            {
                pipe1.enable = 0;
            }
            pipe1.n_loog_exit = 0;
            pipe1.m_vdec_attr.n_vdec_grp = i;
            pipe1.output_func = ai_inference_func; // 图像输出的回调函数

            pipeline_t &pipe0 = pipelines[0];
            // 初始化互斥锁和新字段
            if (pthread_mutex_init(&pipe0.buffer_mutex, NULL) != 0) {
                WTALOGI("初始化互斥锁失败");
                return -1;
            }
            pipe0.total_cached_size = 0;
            pipe0.max_memory_limit = 0;
            pipe0.writer_running = false;
            pipe0.ffmpeg_cmd[0] = '\0';

            // 损伤片段相关字段初始化
            if (pthread_mutex_init(&pipe0.damage_mutex, NULL) != 0) {
                WTALOGI("初始化损伤片段互斥锁失败");
                return -1;
            }
            pipe0.damage_state = DC_IDLE;
            pipe0.damage_pre_max_frames = 0; // 由 h265_save_func 首次按 fps 初始化
            pipe0.damage_pre_total_size = 0;
            pipe0.damage_seen = false;
            pipe0.damage_post_remaining = 0;
            pipe0.damage_clip_filename[0] = '\0';
            pipe0.damage_clip_total_size = 0;
            pipe0.damage_writer_running = false;
            {
                pipeline_ivps_config_t &config0 = pipe0.m_ivps_attr;
                config0.n_ivps_grp = pipe_count * i + 2; // 重复的会创建失败
                config0.n_ivps_rotate = 0;               // 旋转90度，现在rtsp流是竖着的画面了
                config0.n_ivps_fps = s_sample_framerate;
                config0.n_ivps_width = 1280;
                config0.n_ivps_height = 720;
                config0.n_osd_rgn = 2;
                config0.n_fifo_count = 1; // 如果想要拿到数据并输出到回调 就设为1~4
            }
            pipe0.enable = 1;
            pipe0.pipeid = pipe_count * i + 2; // 重复的会创建失败
            pipe0.m_input_type = pi_vdec_h264;
            pipe0.m_output_type = po_rtsp_h265;
            pipe0.n_loog_exit = 0;

            sprintf(pipe0.m_venc_attr.end_point, "%s%d", "axstream", (int)i+1); // 重复的会创建失败
            pipe0.m_venc_attr.n_venc_chn = i;                                 // 重复的会创建失败
            pipe0.m_vdec_attr.n_vdec_grp = i;
            pipe0.output_func = h265_save_func;

            /*
            pipeline_t &pipe2 = pipelines[2]; //输出到本地文件存储
            {
                pipeline_ivps_config_t &config2 = pipe2.m_ivps_attr;
                config2.n_ivps_grp = pipe_count * i + 3; // 重复的会创建失败
                config2.n_ivps_fps = s_sample_framerate; //帧率
                config2.n_ivps_width = 1920;
                config2.n_ivps_height = 1080;
                config2.n_osd_rgn = 2;
                config2.n_fifo_count = 1; // 如果想要拿到数据并输出到回调 就设为1~4
            }
            pipe2.enable = 1;
            pipe2.pipeid = pipe_count * i + 3;
            pipe2.m_input_type = pi_vdec_h264;
            pipe2.m_output_type = po_venc_h265;
            pipe2.n_loog_exit = 0;
            pipe2.n_vin_pipe = 0;
            pipe2.n_vin_chn = 0;
            pipe2.m_venc_attr.n_venc_chn = i + rtsp_urls.size();
            pipe2.m_vdec_attr.n_vdec_grp = i;
            pipe2.output_func = h265_save_func;
            */
            camera->connectPipes(&pipe0, &pipe1); //注意先后顺序
        }
        i++;
    }

    CameraController::getInstance()->start(); // 启动摄像头控制器

    for (size_t i = 0; i < vpipelines.size(); i++)
    {
        auto &pipelines = vpipelines[i];
        for (size_t j = 0; j < pipelines.size(); j++) //次数j共两次: j = 0和1
        {
            s32Ret = create_pipeline(&pipelines[j]); //包含推理输入和视频帧输出
            if (s32Ret != 0)
            {
                ALOGE("create_pipeline failed,s32Ret:0x%x\n", s32Ret);
                continue;
            }
            if (pipelines[j].m_ivps_attr.n_osd_rgn > 0)
            {
                g_sample.gModels[i].pipes_need_osd.push_back(&pipelines[j]);
            }
        }

        if (g_sample.gModels[i].pipes_need_osd.size() && g_sample.gModels[i].bRunJoint)
        {
            g_sample.gModels[i].osd_helper.Start(g_sample.gModels[i].gModels[0], g_sample.gModels[i].pipes_need_osd);  // 绘制过程线程
            g_sample.osd_target_map[pipelines[1].pipeid] = &g_sample.gModels[i];
        }
    }

    {
        std::vector<VideoDemux *> video_demuxes;
        i = 0;
        for (auto const &camera : CameraController::getInstance()->getAllCameras())
        {
            auto &pipelines = vpipelines[i];
            VideoDemux *demux = new VideoDemux();
            if (demux->Open(camera->get_camera_rtsp_url(), true, _demux_frame_callback, pipelines.data(), s_sample_framerate)) // pipelines.data接收帧输入
            {
                video_demuxes.push_back(demux);
            }
            i++;
        }

        while (!gLoopExit)
        {
            usleep(1000 * 1000);
        }
        for (size_t i = 0; i < video_demuxes.size(); i++)
        {
            VideoDemux *demux = video_demuxes[i];
            demux->Stop();
            delete demux;
        }

        gLoopExit = 1;
        //sleep(1);
        pipeline_buffer_t end_buf = {0};
        for (size_t i = 0; i < vpipelines.size(); i++)
        {
            auto &pipelines = vpipelines[i];
            user_input(pipelines.data(), 1, &end_buf);
        }
    }

    // 销毁pipeline
    {
        gLoopExit = 1;

        // 先清理所有帧缓存
        for (size_t i = 0; i < vpipelines.size(); i++)
        {
            auto &pipelines = vpipelines[i];
            for (size_t j = 0; j < pipelines.size(); j++)
            {
                cleanup_frame_buffer(&pipelines[j]);
                cleanup_damage_buffer(&pipelines[j]);
                pthread_mutex_destroy(&pipelines[j].buffer_mutex);// 销毁互斥锁
                pthread_mutex_destroy(&pipelines[j].damage_mutex);// 销毁损伤片段互斥锁
            }
        }

        for (size_t i = 0; i < vpipelines.size(); i++)
        {
            if (g_sample.gModels[i].pipes_need_osd.size() && g_sample.gModels[i].bRunJoint)
            {
                g_sample.gModels[i].osd_helper.Stop();
                //            pthread_cancel(g_sample.osd_tid);
                // s32Ret = pthread_join(g_sample.osd_tid[i], NULL);
                // if (s32Ret < 0)
                // {
                //     ALOGE(" osd_tid exit failed,s32Ret:0x%x\n", s32Ret);
                // }
            }
        }

        for (size_t i = 0; i < vpipelines.size(); i++)
        {
            auto &pipelines = vpipelines[i];
            for (size_t j = 0; j < pipelines.size(); j++)
            {
                destory_pipeline(&pipelines[j]);
            }
        }
    }

EXIT_6:

EXIT_5:

EXIT_4:

EXIT_3:
	for (size_t i = 0; i < vpipelines.size(); i++)
    {
        for (size_t j = 0; j < g_sample.gModels[i].gModels.size(); j++)
        {
            axdl_deinit(&g_sample.gModels[i].gModels[j]);
        }
    }

EXIT_2:

EXIT_1:
    COMMON_SYS_DeInit();
    g_sample.Deinit();

    WTALOGI("sample end\n");
    fprintf(stderr, "exit");
    return 0;
}
