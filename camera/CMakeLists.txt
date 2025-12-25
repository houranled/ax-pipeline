cmake_minimum_required(VERSION 3.10)
project(camera)

file(GLOB SOURCES "*.cpp" "*.c")
if(SOURCES)
	add_library(camera ${SOURCES})
endif()
