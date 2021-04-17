# 现代cmake版本
cmake_minimum_required(VERSION 3.12)
# 使用文件夹名称作为project名称
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" PROJECT_NAME ${PROJECT_NAME})
project(${PROJECT_NAME})

# 设置默认版本为debug
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
message("=> cmake ${PROJECT_NAME} in Debug mode")
add_compile_options(-O0 -g -ggdb)
else()
message("=> cmake ${PROJECT_NAME} in Release mode")
endif()

# 编译某个目录总的所有源文件
file(GLOB my_SRC
    "*.cc"
)
# 新增目标静态lib，小工程，项目名称即为目标名称
add_executable(${PROJECT_NAME}
    ${my_SRC}
)
# 目标包含路径，现代写法
target_include_directories(${PROJECT_NAME} PUBLIC
    3rd-party/trase/src
)
# 目标链接库
target_link_libraries(${PROJECT_NAME}
    trase
)
# 目标编译宏定义，现代写法
target_compile_definitions(${PROJECT_NAME} PUBLIC SOME_MACRO=1)

# 目标编译的C++标准版本
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)