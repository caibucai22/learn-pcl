add_executable(lidar_pointxyz2img lidar_pointxyz_2image.cpp)

add_executable(lidar_pointxyzrgb2img lidar_pointxyzrgb_2image.cpp)

link_pcl_opencv_libraries(lidar_pointxyz2img)

link_pcl_opencv_libraries(lidar_pointxyzrgb2img)

add_executable(pcd2txt pcd_2_text.cpp)
link_pcl_libraries(pcd2txt)


# add_executable(supervoxel_clustering supervoxel_clustering.cpp)
# link_pcl_libraries(supervoxel_clustering)


add_executable(kmeans kmeans/main.cpp kmeans/kmeans.cpp kmeans/kmeans.h)
link_pcl_libraries(kmeans)