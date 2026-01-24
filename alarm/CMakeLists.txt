cmake_minimum_required(VERSION 3.10)
project(alarm)

file(GLOB SOURCES "*.cpp" "*.c")
if(SOURCES)
	add_library(alarm STATIC ${SOURCES})
endif()
