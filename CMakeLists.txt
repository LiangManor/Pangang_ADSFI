cmake_minimum_required(VERSION 3.10.1)

project(adsfi_sample)

message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

#mdc sdk相关内容
find_package(ara-exec)
find_package(ara-com)
find_package(ara-core)
find_package(per)
find_package(log)

#ap库头文件
set(AP_INCLUDE_DIRS
    ${ara-exec_INCLUDE_DIRS}
    ${ara-com_INCLUDE_DIRS}
    ${ara-core_INCLUDE_DIRS}
    ${per_INCLUDE_DIRS}
    ${log_INCLUDE_DIRS})
#ap库 
set(AP_LIBRARIES
    ${ara-exec_LIBRARIES}
    ${ara-com_LIBRARIES}
    ${ara-core_LIBRARIES}
    ${per_LIBRARIES}
    ${log_LIBRARIES})
    
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")

#引入modules
add_subdirectory(modules)
