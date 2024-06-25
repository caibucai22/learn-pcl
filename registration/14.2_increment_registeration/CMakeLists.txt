cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(increment-registration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.11.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_executable(incre-registr incremental_registeration.cpp)

target_link_libraries(incre-registr ${PCL_LIBRARIES})

# target_compile_options(incre-registr "/utf-8")
# target_compile_definitions()