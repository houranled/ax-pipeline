cmake_minimum_required(VERSION 3.10)
project(camera)

file(GLOB SOURCES "*.cpp" "*.c")
if(SOURCES)
	add_library(camera STATIC ${SOURCES})
	target_include_directories(camera PUBLIC
        ${CURL_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/3rdparty/curl-8.17.0/include
        ${CMAKE_SOURCE_DIR}/examples/utilities
        ${CMAKE_SOURCE_DIR}/examples/libaxdl/include

        ${BSP_MSP_DIR}/include
        ${BSP_MSP_DIR}/arm_glibc/include
        ${BSP_MSP_DIR}/arm64_glibc/include
        ${OpenCV_INCLUDE_DIRS}  # 添加 OpenCV 包含目录
        ${CMAKE_SOURCE_DIR}/third-party/ByteTrack/include  # 添加 ByteTrack 包含目录

    )
	target_link_libraries(camera PRIVATE
        alarm
        CURL::libcurl
        modbus
    )
endif()