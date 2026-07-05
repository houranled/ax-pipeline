/**********************************************************************************
 *
 * Copyright (c) 2019-2022 Beijing AXera Technology Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Beijing AXera Technology Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Beijing AXera Technology Co., Ltd.
 *
 **********************************************************************************/
#ifndef _SAMPLE_LOG_H_
#define _SAMPLE_LOG_H_

#include <stdio.h>
#include <stdlib.h>
#include <mutex>

typedef enum
{
    SAMPLE_LOG_MIN = -1,
    SAMPLE_LOG_EMERGENCY = 0,
    SAMPLE_LOG_ALERT = 1,
    SAMPLE_LOG_CRITICAL = 2,
    SAMPLE_LOG_ERROR = 3,
    SAMPLE_LOG_WARN = 4,
    SAMPLE_LOG_NOTICE = 5,
    SAMPLE_LOG_INFO = 6,
    SAMPLE_LOG_DEBUG = 7,
    SAMPLE_LOG_MAX
} SAMPLE_LOG_LEVEL_E;

static SAMPLE_LOG_LEVEL_E log_level = SAMPLE_LOG_EMERGENCY; //for weiti

#if 1
#define MACRO_BLACK "\033[1;30;30m"
#define MACRO_RED "\033[1;30;31m"
#define MACRO_GREEN "\033[1;30;32m"
#define MACRO_YELLOW "\033[1;30;33m"
#define MACRO_BLUE "\033[1;30;34m"
#define MACRO_PURPLE "\033[1;30;35m"
#define MACRO_WHITE "\033[1;30;37m"
#define MACRO_END "\033[0m"
#else
#define MACRO_BLACK
#define MACRO_RED
#define MACRO_GREEN
#define MACRO_YELLOW
#define MACRO_BLUE
#define MACRO_PURPLE
#define MACRO_WHITE
#define MACRO_END
#endif

// ============================================================================
// 日志滚动（size-based rotation）
//
// 目标：wt_ai.log / wt_ai2.log 无限增长 → 磁盘占满 的问题。
// 策略：单文件写到 WT_LOG_MAX_SIZE 就环形滚动到 .1/.2/.../.N，最老的删除。
// 磁盘上限 = WT_LOG_MAX_SIZE × (WT_LOG_KEEP_COUNT + 1) 固定封顶。
//
// 关键点：
// 1) 用 inline 变量（C++17）取代原 static FILE*，使所有 TU 共享同一句柄，
//    避免多 TU 各持一份 FILE* 时 rotate rename 相互竞争。
// 2) 用 g_log_rot_mtx 保护 fprintf + fflush + ftell + rotate 的整体原子性，
//    避免多线程写入被 rotate 中途 fclose 命中 UB。
// ============================================================================

#ifndef WT_LOG_MAX_SIZE
#define WT_LOG_MAX_SIZE   (50L * 1024 * 1024)  // 每个日志文件最大 50MB
#endif
#ifndef WT_LOG_KEEP_COUNT
#define WT_LOG_KEEP_COUNT 0                     // 不保留备份：写满 WT_LOG_MAX_SIZE 直接清空重来
#endif

inline FILE* log_file = nullptr;
inline FILE* wt_log_file = nullptr; //for weiti
inline std::mutex g_log_rot_mtx;

static inline void init_log_file() {
    if (!log_file) {
        log_file = fopen("/wt_tech/logs/node/wt_ai2.log", "a");
    }
}

static inline void init_wt_log_file() {
    if (!wt_log_file) {
        wt_log_file = fopen("/wt_tech/logs/node/wt_ai.log", "a");
    }
}

// 检查当前 FILE* 尺寸是否超阈值，超了就环形滚动并重开。
// 必须在持有 g_log_rot_mtx 的情况下调用。
static inline void _log_rotate_if_needed(FILE** fp, const char* path) {
    if (!*fp) return;
    long sz = ftell(*fp);
    if (sz < WT_LOG_MAX_SIZE) return;

    fclose(*fp); *fp = nullptr;

    if (WT_LOG_KEEP_COUNT <= 0) {
        // 不保留任何备份：直接删掉当前文件，重开一个空的
        remove(path);
    } else {
        char oldp[512], newp[512];
        // 先删掉最老一份
        snprintf(oldp, sizeof(oldp), "%s.%d", path, WT_LOG_KEEP_COUNT);
        remove(oldp);
        // 依次把 .N-1 -> .N, ..., .1 -> .2
        for (int i = WT_LOG_KEEP_COUNT - 1; i >= 1; --i) {
            snprintf(oldp, sizeof(oldp), "%s.%d", path, i);
            snprintf(newp, sizeof(newp), "%s.%d", path, i + 1);
            rename(oldp, newp);
        }
        // 当前 -> .1
        snprintf(newp, sizeof(newp), "%s.1", path);
        rename(path, newp);
    }

    // 重新以追加模式打开
    *fp = fopen(path, "a");
}

#define ALOGE(fmt, ...) do { \
    std::lock_guard<std::mutex> _lk(g_log_rot_mtx); \
    init_log_file(); \
    if (log_file) { \
        fprintf(log_file, "[E][%32s][%4d]: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        fflush(log_file); \
        _log_rotate_if_needed(&log_file, "/wt_tech/logs/node/wt_ai2.log"); \
    } \
} while(0)

#define WTALOGI(fmt, ...) do { \
    if (log_level >= SAMPLE_LOG_EMERGENCY) { \
        std::lock_guard<std::mutex> _lk(g_log_rot_mtx); \
        init_wt_log_file(); \
        if (wt_log_file) { \
            fprintf(wt_log_file, "[wt][%32s][%4d]: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            fflush(wt_log_file); \
            _log_rotate_if_needed(&wt_log_file, "/wt_tech/logs/node/wt_ai.log"); \
        } \
    } \
} while(0)

#define ALOGW(fmt, ...)               \
    if (log_level >= SAMPLE_LOG_WARN) \
    printf(MACRO_YELLOW "[W][%32s][%4d]: " fmt MACRO_END "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define ALOGI(fmt, ...)               \
    if (log_level >= SAMPLE_LOG_INFO) \
    printf(MACRO_GREEN "[I][%32s][%4d]: " fmt MACRO_END "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define ALOGD(fmt, ...)                \
    if (log_level >= SAMPLE_LOG_DEBUG) \
    printf(MACRO_WHITE "[D][%32s][%4d]: " fmt MACRO_END "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define ALOGN(fmt, ...)                 \
    if (log_level >= SAMPLE_LOG_NOTICE) \
    printf(MACRO_PURPLE "[N][%32s][%4d]: " fmt MACRO_END "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#endif /* _SAMPLE_LOG_H_ */
