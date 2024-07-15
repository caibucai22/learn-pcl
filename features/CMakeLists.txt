add_executable(detect_harris 01-Harris.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(detect_harris)

add_executable(detect_narf 01-NARF.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(detect_narf)

add_executable(detect_pfh 01-pfh.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(detect_pfh)

add_executable(detect_vfh 01-vfh.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(detect_vfh)

add_executable(detect_fpfh 10-FPFH.cpp)
link_pcl_libraries(detect_fpfh)