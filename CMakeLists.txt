cmake_minimum_required(VERSION 3.22)

project(learn_pcl)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 
# set(CMAKE_BUILD_TYPE Debug)

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


# opencv setting
set(OpenCV_DIR "D:/03-env/OpenCV/opencv-4.9.0-release/opencv/build")
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message("opencv found")
	include_directories(${OpenCV_INCLUDE_DIRS})

	link_directories(${OpenCV_LIBRARY_DIRS})
  link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY}) # opecv_world490d.lib
else()
  message(FATAL_ERROR "opencv not found")
endif()


# boost setting

set(Boost_ROOT "D:/03-env/boost_1_84_0")
find_package(Boost 1.70 REQUIRED)
if(Boost_FOUND)
  message("Boost found")
  message("${Boost_INCLUDE_DIRS} | ${Boost_LIBRARIES} | ${Boost_LIBRARY_DIRS}")
	include_directories(${Boost_INCLUDE_DIRS})

	link_directories(${Boost_LIBRARY_DIRS})
else()
  message(FATAL_ERROR "Boost not found")
endif()


# link function
function(link_pcl_libraries target)
    target_link_libraries(${target} PRIVATE ${PCL_LIBRARIES})
endfunction()

function(link_opencv_libraries target)
    target_link_libraries(${target} PRIVATE ${OpenCV_LIBRARIES})
endfunction()

function(link_boost_libraries target)
    target_link_libraries(${target} PRIVATE ${Boost_LIBRARIES})
endfunction()

function(link_pcl_opencv_libraries target)
    target_link_libraries(${target} PRIVATE ${PCL_LIBRARIES})
    target_link_libraries(${target} PRIVATE ${OpenCV_LIBRARIES})
endfunction()

# subdir
add_subdirectory(basic)

add_subdirectory(visualize)

