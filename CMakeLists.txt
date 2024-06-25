cmake_minimum_required(VERSION 3.22)

project(learn_pcl)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# structure setting

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/lib")


# pcl setting 

find_package(PCL 1.11.1 REQUIRED)

if(PCL_FOUND)
message("pcl found")

include_directories(${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})	

link_directories(${PCL_LIBRARY_DIRS})

else()
message(FATAL_ERROR "pcl not found")
endif(PCL_FOUND)


# boost setting



add_subdirectory(visualize)

