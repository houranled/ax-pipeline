#!/bin/sh
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
else
	echo "no new revision"
#	exit 1
fi


#git reset --hard @{upstream} && git stash apply stash@{0}
git reset --hard @{upstream}

# 生成 git_version.h
cat > git_version.h << EOF
#pragma once
#define GIT_COMMIT_HASH "$(git rev-parse --short HEAD)"
#define GIT_COMMIT_DATE "$(git log -1 --format=%cd --date=short)"
EOF

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