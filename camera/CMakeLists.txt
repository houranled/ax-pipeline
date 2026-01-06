cmake_minimum_required(VERSION 3.10)
project(camera)

file(GLOB SOURCES "*.cpp" "*.c")
if(SOURCES)
	add_library(camera STATIC ${SOURCES})
	target_include_directories(camera PUBLIC ${CURL_SOURCE_DIR}/include)
	target_link_libraries(camera PRIVATE libcurl modbus)
endif()
