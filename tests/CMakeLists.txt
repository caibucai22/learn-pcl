# set_source_files_properties(Test_OMP.cpp PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
# add_executable(Test_OMP Test_OMP.cpp)

# add_executable(Test_detect_edge test_detect_edge.cpp)
# link_pcl_libraries(Test_detect_edge)

add_executable(Test_PCA Test_PCA.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(Test_PCA)