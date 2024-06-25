cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ndt LANGUAGES CXX)

find_package(PCL 1.11.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(ndt normal_distributions_transform.cpp)

target_link_libraries(ndt ${PCL_LIBRARIES})

# target_compile_options() # 编译选项
# target_compile_definitions() # 宏选项