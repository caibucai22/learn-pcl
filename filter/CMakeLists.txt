add_executable(passthrough PassThroughFilter.cpp)
link_pcl_libraries(passthrough)

add_executable(voxelgrid VoxelGridFilter.cpp)
link_pcl_libraries(voxelgrid)

add_executable(approx_voxelgrid ApproximateVoxelGridFilter.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(approx_voxelgrid)


add_executable(statistical_filter StatisticalFilter.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(statistical_filter)


add_executable(radius_filter RadiusFilter.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(radius_filter)


add_executable(condition_filter_by_z ConditionFilter.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(condition_filter_by_z)


add_executable(condition_filter_by_curvature ConditionFilter2.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(condition_filter_by_curvature)

add_executable(project_filter_to_plane ProjectFilter_to_Plane.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(project_filter_to_plane)


add_executable(project_filter_to_sphere ProjectFilter_to_Sphere.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(project_filter_to_sphere)

add_executable(project_filter_to_cylinder ProjectFilter_to_Cylinder.cpp ../my_functions.cpp ../my_functions.h)
link_pcl_libraries(project_filter_to_cylinder)
