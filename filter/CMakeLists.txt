add_executable(passthrough PassThroughFilter.cpp)
link_pcl_libraries(passthrough)

add_executable(voxelgrid VoxelGridFilter.cpp)
link_pcl_libraries(voxelgrid)