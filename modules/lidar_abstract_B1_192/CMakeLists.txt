cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

#set(ara-exec_DIR ${MDC_SDK}/sysroot/usr/lib/cmake/exec/)
set(CMAKE_FIND_ROOT_PATH ${MDC_SDK}/sysroot/ ${CMAKE_FIND_ROOT_PATH})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)
ADD_DEFINITIONS(-DMDC_PRODUCTION_CORE)
project(lidar_abstract_B1_192)
set(APP_NAME lidar_abstract_B1_192)

set(DEFAULT_COMPILE -std=c++14 -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -fstack-protector-all)

add_compile_options(${DEFAULT_COMPILE})

set(MDC_ADSFI_SDK ${MDC_SDK}/sysroot/usr/include/adsfi)
set(MDC_ACLLIB_SDK ${MDC_SDK}/sysroot/usr/local/Ascend/runtime)
set(GODEL_LINK_LIBS "-lara_core -lara_com -lara_exec -lddscore -lddscpp  -lJsonParser -lKeyValueStorage -llogging -lsomeip -lvcc_ddsdriver -lvcc_someipdriver -lvrtf_vcc")

find_package(Threads)
find_package(ara-exec)
find_package(ara-com)
find_package(ara-core)
find_package(log)
find_package(rtf-crc)
find_package(rtf-e2e)
find_package(rtf-e2exf)
find_package(per)



# 用户需要添加自己的头文件路径
INCLUDE_DIRECTORIES(

    ${MDC_SDK}/sysroot/usr/include
    ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
    ${MDC_SDK}/sysroot/usr/include/adsfi/adsf/include
    SYSTEM ${MDC_SDK}/sysroot/usr/include/adsfi/arxml_include
    ${MDC_ACLLIB_SDK}/include
    ${MDC_ACLLIB_SDK}/include/acl
    ${MDC_ACLLIB_SDK}/include/acl/ops
    ${MDC_ACLLIB_SDK}/include/acl/dvpp
    ${MDC_ADSFI_SDK}
    ./
)


#ap库头文件
set(AP_INCLUDE_DIRS
    ${ara-exec_INCLUDE_DIRS}
    ${ara-com_INCLUDE_DIRS}
    ${ara-core_INCLUDE_DIRS}
	${rtf-crc_INCLUDE_DIRS}
    ${rtf-e2e_INCLUDE_DIRS}
    ${rtf-e2exf_INCLUDE_DIRS}
	${per_INCLUDE_DIRS})
set(AP_LIBRARIES
    ${ara-exec_LIBRARIES}
    ${ara-com_LIBRARIES}
    ${ara-core_LIBRARIES}
	${rtf-crc_LIBRARIES}
    ${rtf-e2e_LIBRARIES}
    ${rtf-e2exf_LIBRARIES}
	${per_LIBRARIES}
	log)
FILE(GLOB_RECURSE C_SOURCE src/*.c)
FILE(GLOB_RECURSE  CXX_SOURCE src/*.cpp)
#FILE(GLOB_RECURSE  AP_SOURCE ${GEN_CODE_DIR}/adsfi/lidar_det_base/generated/*.cpp)


# 指定链接库文件的路径
link_directories(
    ${MDC_SDK}/sysroot/usr/lib/mdc_vector
    ${MDC_SDK}/sysroot/usr/lib/mdc/base-plat
    ${MDC_ACLLIB_SDK}/lib64/stub
    ${MDC_ACLLIB_SDK}/lib64
)
add_executable(${APP_NAME} ${C_SOURCE} ${CXX_SOURCE})


# 添加头文件路径
target_include_directories(${APP_NAME}
                           PRIVATE
                           ${MDC_SDK}/sysroot/usr/include/pcl-1.11#pcl头文件
                           ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
                           ${MDC_SDK}/sysroot/usr/local/Ascend/runtime/include
                           ${MDC_SDK}/sysroot/usr/include
                           ${MDC_SDK}/sysroot/usr/include/adsfi/adsf/include
                            ${MDC_SDK}/sysroot/usr/include
                            ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
                            ${MDC_SDK}/sysroot/usr/include/adsfi/adsf/include
                            SYSTEM 
                            ${MDC_SDK}/sysroot/usr/include/adsfi/arxml_include
                            ${MDC_ACLLIB_SDK}/include
                            ${MDC_ACLLIB_SDK}/include/acl
                            ${MDC_ACLLIB_SDK}/include/acl/ops
                            ${MDC_ACLLIB_SDK}/include/acl/dvpp
                            ${MDC_ADSFI_SDK}
                            ${AP_INCLUDE_DIRS}
                           )



# 添加动态库的链接
# 体素滤波器
find_library(pcl_common
             NAMES pcl_common
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu)
find_library(pcl_filters
             NAMES pcl_filters
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu) #指定库文件的搜索路径
find_library(pcl_io
             NAMES pcl_io
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu)
find_library(pcl_segmentation
             NAMES pcl_segmentation
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu)
find_library(pcl_kdtree
             NAMES pcl_kdtree
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu)
find_library(pcl_search
             NAMES pcl_search
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu)
find_library(pcl_features
             NAMES pcl_features
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu)
find_library(pcl_sample_consensus
             NAMES pcl_sample_consensus
             PATHS ${MDC_SDK}/sysroot/usr/lib/aarch64-linux-gnu)



# 用户需要添加自己的库名称
target_link_libraries(${APP_NAME}
	# 第三方库
	pcl_common
	pcl_filters
	pcl_io
	pcl_segmentation
	pcl_kdtree
	pcl_search
	pcl_features
	pcl_sample_consensus
	# MDC库
	${AP_LIBRARIES} 
	${GODEL_LINK_LIBS}
	adb
	adsf
	fault_event
	ascendcl
	acl_dvpp_mpi
	iam
	easy_comm
	acl_dvpp
	log
	mdc_aiv
	visual
)

install(
    TARGETS ${APP_NAME} RUNTIME
    DESTINATION ${PROJECT_SOURCE_DIR}/bin
)





