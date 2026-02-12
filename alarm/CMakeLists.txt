cmake_minimum_required(VERSION 3.10)
project(alarm)

file(GLOB SOURCES "*.cpp" "*.c")
if(SOURCES)
	add_library(alarm STATIC ${SOURCES})
    target_include_directories(alarm PRIVATE
        ${CURL_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/examples/utilities
        ${CMAKE_SOURCE_DIR}/examples/libaxdl/include
        ${BSP_MSP_DIR}/include
        ${BSP_MSP_DIR}/arm_glibc/include
        ${BSP_MSP_DIR}/arm64_glibc/include
        ${OpenCV_INCLUDE_DIRS}  # 添加 OpenCV 包含目录
        ${CMAKE_SOURCE_DIR}/third-party/ByteTrack/include  # 添加 ByteTrack 包含目录
    )
endif()