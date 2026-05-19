#module名称
set(MODULE_NAME "can_receive_sample")

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
        ${ara-core_INCLUDE_DIRS})

#用户可在此添加链接库
target_link_libraries(${MODULE_NAME}
        PRIVATE
        ${ara-exec_LIBRARIES}
        ${ara-core_LIBRARIES}
        log
        pthread)

#用户可在此添加链接参数
#eg: set_target_properties(${MODULE_NAME} PROPERTIES LINK_FLAGS "-Wl,--as-needed USER_LINK_OPTIONS")
set_target_properties(${MODULE_NAME} PROPERTIES LINK_FLAGS "-Wl,--as-needed")

#用户添加module依赖
#add_dependencies(${MODULE_NAME} USER_DEPENDENCIES)