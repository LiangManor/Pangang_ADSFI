cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CXX_STANDARD 11)
#set(ara-exec_DIR ${MDC_SDK}/sysroot/usr/lib/cmake/exec/)
set(CMAKE_FIND_ROOT_PATH ${MDC_SDK}/sysroot/ ${CMAKE_FIND_ROOT_PATH})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_CXX_STANDARD 17)
SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)
ADD_DEFINITIONS(-DMDC_PRODUCTION_CORE)
project(camera_det)
set(APP_NAME camera_det)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

#把Debug和Release状态下默认的编译选项设置为空，避免和用户的设置重复
set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_C_FLAGS_RELEASE "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_RELEASE  "")

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    set(DEFAULT_COMPILE -std=c++14 -O2 -g3 -Wall -c -fmessage-length=0 -fPIC -fPIE -fstack-protector-all)
ELSEIF (CMAKE_BUILD_TYPE MATCHES Release)
    set(DEFAULT_COMPILE -std=c++14 -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -fstack-protector-all)
ENDIF()

add_compile_options(${DEFAULT_COMPILE})

set(MDC_ADSFI_SDK ${MDC_SDK}/sysroot/usr/include/adsfi)
set(MDC_ACLLIB_SDK ${MDC_SDK}/sysroot/usr/local/Ascend/runtime)
set(GODEL_LINK_LIBS "-lara_core -lara_com -lara_exec -lddscore -lddscpp  -lJsonParser -lKeyValueStorage -llogging -lsomeip -lvcc_ddsdriver -lvcc_someipdriver -lvrtf_vcc")

#修改 /path/to/MDC_Application_visualizer-x.x.x.x -Ubuntu20为Mviz的解压安装路经
set(CMAKE_MODULE_PATH "/home/inwinic/MDC610/ShoftWare/MDC_Application_Visualizer-2.2.001-0000000-Ubuntu20/mviz_devel_kit/share/cmake/Modules;${CMAKE_MODULE_PATH}")

set(IS_MVIZDEVELKIT_EXISTS false)
foreach(path IN ITEMS ${CMAKE_MODULE_PATH})
    if(EXISTS ${path}/MvizDevelKit.cmake)
        set(IS_MVIZDEVELKIT_EXISTS true)
    endif()
endforeach()
if(NOT IS_MVIZDEVELKIT_EXISTS)
    message(FATAL_ERROR "Add the path of the directory where the \"MvizDevelKit.cmake\" file is located to the \"CMAKE_MODULE_PATH\" variable in this CMakeLists.txt file.\n Example")
endif()

include(MvizDevelKit)

# 用户需要添加自己的头文件路径
INCLUDE_DIRECTORIES(
    ./include
	/home/inwinic/MDC610/ShoftWare/MDC_Application_Visualizer-2.2.001-0000000-Ubuntu20/mviz_devel_kit/include
    ${MDC_SDK}/sysroot/usr/include/opencv4
    ${MDC_SDK}/sysroot/usr/include
    ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
    ${MDC_SDK}/sysroot/usr/include/adsfi/adsf/include
    SYSTEM ${MDC_SDK}/sysroot/usr/include/adsfi/arxml_include
    ${MDC_ACLLIB_SDK}/include
    ${MDC_ACLLIB_SDK}/include/acl
    ${MDC_ACLLIB_SDK}/include/acl/ops
    ${MDC_ACLLIB_SDK}/include/acl/dvpp
    ${MDC_ADSFI_SDK}
    ${MDC_ADSFI_SDK}/../dvpp
    ./
)

FILE(GLOB_RECURSE C_SOURCE src/*.c)
FILE(GLOB_RECURSE  CXX_SOURCE src/*.cpp)

# 用户需要添加自己的库路径
link_directories(
	${MDC_SDK}/sysroot/usr/include/opencv4/opencv2
    ${MDC_SDK}/sysroot/usr/lib/mdc_vector
    ${MDC_SDK}/sysroot/usr/lib/mdc/base-plat
    ${MDC_ACLLIB_SDK}/lib64/stub
    ${MDC_ACLLIB_SDK}/lib64
)
add_executable(${APP_NAME} ${C_SOURCE} ${CXX_SOURCE})

target_message_generation(TARGET ${APP_NAME})

# 用户需要添加自己的库名称
target_link_libraries(${APP_NAME}
	opencv_dnn
	opencv_core
	opencv_imgproc
	opencv_imgcodecs
    ${GODEL_LINK_LIBS}
    adb
    adsf
    ascendcl
    acl_dvpp_mpi
    fault_event
    iam
    easy_comm
    log
    mdc_aiv
    visual
    ${AP_LIBRARIES}
)

install(
    TARGETS ${APP_NAME} RUNTIME
    DESTINATION ${PROJECT_SOURCE_DIR}/bin
)
