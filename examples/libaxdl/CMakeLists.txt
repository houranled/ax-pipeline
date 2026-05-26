set(SampleName axdl)



function(targetlink name)
    target_include_directories(${name} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_include_directories(${name} PRIVATE include)
    target_include_directories(${name} PRIVATE ../utilities)
    target_include_directories(${name} PRIVATE ${BSP_MSP_DIR}/include)
    target_include_directories(${name} PRIVATE ${BSP_MSP_DIR}/arm_glibc/include)
    target_include_directories(${name} PRIVATE ${BSP_MSP_DIR}/arm64_glibc/include)
    target_include_directories(${name} PRIVATE ${BSP_MSP_DIR}/include/npu_cv_kit)
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/third-party/ByteTrack/include)
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/3rdparty/curl-8.17.0/include)

    # FreeType (用于中文水印 OSD 预渲染)
    # 优先使用 pkg-config / find_package；否则回退到系统默认头与 .so
    find_package(Freetype QUIET)
    if(Freetype_FOUND)
        target_include_directories(${name} PRIVATE ${FREETYPE_INCLUDE_DIRS})
        target_link_libraries(${name} PRIVATE ${FREETYPE_LIBRARIES})
    else()
        # 交叉编译/sysroot 下兜底：常见路径
        target_include_directories(${name} PRIVATE /usr/include/freetype2)
        target_link_libraries(${name} PRIVATE freetype)
    endif()

    # onnxruntime
    if(ONNXRUNTIME_DIR)
        target_include_directories(${name} PRIVATE ${ONNXRUNTIME_DIR}/include)
        target_link_directories(${name} PRIVATE ${ONNXRUNTIME_DIR}/lib)
        target_link_libraries(${name} PRIVATE onnxruntime)
    endif()
endfunction()

file(GLOB SRC_LIST "src/*.cpp" "src/utilities/*.cpp")

list(APPEND SRC_BASE_LIST
    ${SRC_LIST}
)

add_compile_options(-fPIC -fopenmp)

add_library(${SampleName} STATIC
    ${SRC_BASE_LIST}
)
targetlink(${SampleName})
install(TARGETS ${SampleName} DESTINATION lib)