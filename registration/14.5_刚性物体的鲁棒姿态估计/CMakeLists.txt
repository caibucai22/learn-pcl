cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ndt LANGUAGES CXX)

find_package(PCL 1.11.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
# 和文件名一致 方便调试
add_executable(alignment_prerejective alignment_prerejective.cpp)

target_link_libraries(align ${PCL_LIBRARIES})

# target_compile_options() # 编译选项
# target_compile_definitions() # 宏选项