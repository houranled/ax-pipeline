cmake_minimum_required(VERSION 3.10)
project(alarm)

file(GLOB SOURCES "*.cpp" "*.c")
if(SOURCES)
	add_library(alarm STATIC ${SOURCES})
    # 添加 curl 头文件路径
    target_include_directories(alarm PRIVATE
        ${CURL_SOURCE_DIR}/include
    )

endif()
