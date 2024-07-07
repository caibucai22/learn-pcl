
add_executable(cloud_viewer cloud_viewer.cpp)

link_pcl_libraries(cloud_viewer)

add_executable(pcl_visualizer_ pcl_visualizer_.cpp ../my_functions.cpp ../my_functions.h)

link_pcl_libraries(pcl_visualizer_)


add_executable(vtk_visualizer vtk_visualizer.cpp ../my_functions.cpp ../my_functions.h)

link_pcl_libraries(vtk_visualizer)