
add_executable(cloud_viewer cloud_viewer.cpp)

link_pcl_libraries(cloud_viewer)

add_executable(pcl_visualizer_ pcl_visualizer_.cpp)

# link_pcl_libraries(pcl_visualizer_)
# link_boost_libraries(pcl_visualizer_)