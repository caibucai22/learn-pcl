add_executable(passthrough PassThroughFilter.cpp)
link_pcl_libraries(passthrough)

add_executable(voxelgrid VoxelGridFilter.cpp)
link_pcl_libraries(voxelgrid)

add_executable(approx_voxelgrid ApproximateVoxelGridFilter.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(approx_voxelgrid)