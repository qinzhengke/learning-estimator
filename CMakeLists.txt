# 现代cmake版本
cmake_minimum_required(VERSION 3.12)
# 使用文件夹名称作为project名称
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" PROJECT_NAME ${PROJECT_NAME})
project(${PROJECT_NAME})

# 使用cmake定义版本号
set(MAJOR_VER 1)
set(MINOR_VER 0)
set(PATCH_VER 0)
set(FULL_VERSION "${MAJOR_VER}.${MINOR_VER}.${PATCH_VER}")

# 设置默认版本为debug
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
message("=> cmake ${PROJECT_NAME} in Debug mode")
add_compile_options(-O0 -g -ggdb)
else()
message("=> cmake ${PROJECT_NAME} in Release mode")
endif()

# 构建example子目录
add_subdirectory(3rd-party/matplotplusplus-1.0.1)
add_subdirectory(particle_filter)
