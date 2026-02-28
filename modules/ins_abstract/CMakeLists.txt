#module名称
set(MODULE_NAME "ins_abstract")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_FIND_ROOT_PATH ${MDC_SDK}/sysroot/ ${CMAKE_FIND_ROOT_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(PCLOMP_A "${PROJECT_SOURCE_DIR}/include/pclomp")
set(PCLOMP_B "${PROJECT_SOURCE_DIR}/include/multigrid_pclomp")
set(PCLOMP_C "${PROJECT_SOURCE_DIR}/include/estimate_covariance")
SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)
ADD_DEFINITIONS(-DMDC_PRODUCTION_CORE)

#把Debug和Release状态下默认的编译选项设置为空，避免和用户的设置重复
set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_C_FLAGS_RELEASE "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_RELEASE  "")
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
find_package(yaml-cpp 0.6 REQUIRED)
find_package(OpenMP REQUIRED)

find_package(
  roscpp
)
find_package(
  sensor_msgs
)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")


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


#调用函数，把src，generated的子目录分别保存在变量中,filters可用于过滤包含指定字符串的文件夹
list(APPEND filters "")
#调用函数，把src子目录保存在SRC_DIRS变量中
list_sub_dir(SRC_DIRS ${CMAKE_CURRENT_SOURCE_DIR} "${filters}")
#调用函数，把generated子目录保存在GENERATED_DIRS变量中
list_sub_dir(GENERATED_DIRS ${CMAKE_SOURCE_DIR}/generated "${filters}")

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "SRC_DIRS: ${SRC_DIRS}")

#把目录下的c和c++所有源文件分别保存在变量中
file(GLOB_RECURSE
        SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../can_common/*.cpp
        ${CMAKE_SOURCE_DIR}/generated/*.cpp)





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
    ${PROJECT_SOURCE_DIR}/include
    ./
)



set(MDC_ADSFI_SDK ${MDC_SDK}/sysroot/usr/include/adsfi)
set(MDC_ACLLIB_SDK ${MDC_SDK}/sysroot/usr/local/Ascend/runtime)

# 用户需要添加自己的库路径
link_directories(
    ${MDC_SDK}/sysroot/usr/lib/mdc_vector
    ${MDC_SDK}/sysroot/usr/lib/mdc/base-plat
    ${MDC_ACLLIB_SDK}/lib64/stub
    ${MDC_ACLLIB_SDK}/lib64
)



#默认编译c和c++文件
add_executable(${MODULE_NAME} ${SOURCES})

#用户可在此添加编译选项
#eg: target_compile_options(${MODULE_NAME} PRIVATE -Wall USER_COMPILE_OPTIONS)
target_compile_options(${MODULE_NAME} PRIVATE -Wall)

find_package(ara-exec)
find_package(ara-core)
find_package(log)

#用户可在此添加头文件路径
target_include_directories(${MODULE_NAME}
        PRIVATE
        ${SRC_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../can_common
        ${GENERATED_DIRS}
        ${ara-exec_INCLUDE_DIRS}
        ${ara-core_INCLUDE_DIRS}
        ${MDC_SDK}/sysroot/usr/include
        ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
        ${MDC_SDK}/sysroot/usr/include/adsfi/adsf/include
        ${MDC_SDK}/sysroot/usr/include/mviz_devel_kit/include
        SYSTEM 
        ${MDC_SDK}/sysroot/usr/include/adsfi/arxml_include
        ${MDC_ACLLIB_SDK}/include
        ${MDC_ACLLIB_SDK}/include/acl
        ${MDC_ACLLIB_SDK}/include/acl/ops
        ${MDC_ACLLIB_SDK}/include/acl/dvpp
        ${MDC_ADSFI_SDK}

)

#用户可在此添加链接库
target_link_libraries(${MODULE_NAME}
        PRIVATE
        ${ara-exec_LIBRARIES}
        ${ara-core_LIBRARIES}
        
        log
        pthread
	pcl_common
	pcl_filters
	pcl_io
	pcl_segmentation
	pcl_kdtree
	pcl_search
	pcl_features
	pcl_sample_consensus
	${OpenMP_LIB}
	pthread
	# MDC库
    ${boost_system}
    ${boost_thread}
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

#用户可在此添加链接参数
#eg: set_target_properties(${MODULE_NAME} PROPERTIES LINK_FLAGS "-Wl,--as-needed USER_LINK_OPTIONS")
set_target_properties(${MODULE_NAME} PROPERTIES LINK_FLAGS "-Wl,--as-needed")

#用户添加module依赖
#add_dependencies(${MODULE_NAME} USER_DEPENDENCIES)
