#!/bin/sh
# =============================================================================
# make_new_release_to.sh
#   一键发布脚本：拉取上游 -> 生成版本头 -> 编译 -> 安装 -> 重命名可执行文件
#
# 流程：
#   1. git fetch  拉取远端最新变更；失败即退出
#   2. 检测本地是否落后于 upstream，仅提示（不阻塞）
#   3. git reset --hard @{upstream}  强制对齐远端（会丢弃本地未提交改动）
#   4. 从 git 元数据生成 git_version.h，供 C++ 源码 include
#   5. make -j7 && make install  构建并安装到 install/
#   6. 进入 install/bin，将主可执行文件重命名为 wt_ai
#
# 版本号规则（生成到 git_version.h 的宏 PROJECT_VERSION_FULL）：
#   格式：<MAJOR>.<MINOR>.<PATCH>-<hash>[-dirty]
#     MAJOR.MINOR 来自最近 git tag（如 v1.1、v1.1.x），发大版本时打新 tag
#     PATCH       = 距最近 tag 的 commit 数，自动递增（tag 上为 0）
#     hash        = git 短 hash，固定 5 位
#     -dirty      = 工作区有未提交改动时附加后缀
#   例：
#     v1.1 之后第 3 次提交且干净         -> 1.1.3-343f7
#     同上但有本地未提交改动             -> 1.1.3-343f7-dirty
#     打新 tag v1.2 后立即构建           -> 1.2.0-<hash>
#     从未打过 tag、当前是第 42 次提交   -> 0.0.42-<hash>
#
# 使用：
#   ./make_new_release_to.sh
# 发新版：
#   git tag v<MAJOR>.<MINOR>  &&  ./make_new_release_to.sh
#
# 注意：
#   - 脚本会 git reset --hard，未提交/未 stash 的改动会丢失。
#   - git_version.h 是构建时生成的临时头，不入库。
# =============================================================================

git fetch

if [ "$?" != "0" ];then
    echo "fetch失败"
    exit 1
else
   echo "fetch成功"
fi

if [ "$(git status | grep -E "Your branch is behind|Your branch and")" != "" ]; then
	echo "there is a new revision!"
	sleep 2
    git reset --hard @{upstream}
else
	echo "no new revision"
#	exit 1
fi


# ---- 生成 git_version.h（信息全部从 git 获取）----
# 版本号格式：<MAJOR>.<MINOR>.<PATCH>-<hash>[-dirty]
#   MAJOR.MINOR 来自最近的 git tag（如 v1.1 或 v1.1.x），发大版本时打新 tag
#   PATCH       = 距该 tag 的 commit 数，自动递增（tag 上为 0）
#   hash        = git 短 hash
#   -dirty      = 工作区有未提交改动时附加
LAST_TAG=$(git describe --tags --abbrev=0 2>/dev/null || echo "")
if [ -n "$LAST_TAG" ]; then
    BASE="${LAST_TAG#v}"                                      # 去掉可能的 v 前缀
    MAJOR=$(echo "$BASE" | awk -F. '{print $1+0}')
    MINOR=$(echo "$BASE" | awk -F. '{print $2+0}')
    PATCH=$(git rev-list --count "${LAST_TAG}..HEAD" 2>/dev/null || echo 0)
else
    MAJOR=0
    MINOR=0
    PATCH=$(git rev-list --count HEAD 2>/dev/null || echo 0)
fi

GIT_HASH=$(git rev-parse --short=5 HEAD 2>/dev/null || echo unknown)
# 保险：即使 git 因唯一性冲突返回更多位，也强制截断到 5 位
if [ ${#GIT_HASH} -gt 5 ]; then
    GIT_HASH="${GIT_HASH:0:5}"
fi
GIT_DATE=$(git log -1 --format=%cd --date=short 2>/dev/null || echo unknown)
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)

if git diff --quiet HEAD -- 2>/dev/null; then
    DIRTY=""
else
    DIRTY="-dirty"
fi

VERSION_BASE="${MAJOR}.${MINOR}.${PATCH}"
VERSION_FULL="${VERSION_BASE}-${GIT_HASH}${DIRTY}"

cat > git_version.h << EOF
#pragma once
#define PROJECT_VERSION      "${VERSION_BASE}"
#define PROJECT_VERSION_FULL "${VERSION_FULL}"
#define GIT_COMMIT_HASH      "${GIT_HASH}"
#define GIT_COMMIT_DATE      "${GIT_DATE}"
#define GIT_BRANCH           "${GIT_BRANCH}"
EOF

echo "Build version: ${VERSION_FULL}  branch=${GIT_BRANCH}  date=${GIT_DATE}"

make -j7 && make install

if [ "$?" != "0" ];then
    echo "编译失败"
    exit 1
fi


cd install/bin || {
    	echo "can not enter install/bin"
	exit 1
}

mv sample_multi_demux_ivps_npu_multi_rtsp wt_ai

# deploy remote server
#echo "start deploying remote server..."
#ip=IP地址
#ssh -p 35 root@${ip} "sudo rm  /wt_tech/app/ax-pipeline/wt_ai" && \
#scp -P 35 -r ./wt_ai root@${ip}:/wt_tech/app/ax-pipeline/ && \
#ssh -p 35 root@${ip} "sudo chmod -R 755 /wt_tech/app/ax-pipeline/wt_ai"