detect_egde

利用 PCL 提供的 BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> 类 接口 来完成边界提取
使用这个类，需要向其提供原始点云，法线，以及一个输出点云 也就是 Boundary

detect_edge 暴露了两个参数出来，分别是

- 用于法线估计的 radius search 的 r
- 用于边界计算的 radius search 的 r

后续可以暴露更多参数出来，如

- 法线估计 中 kserach 中的 k

存在的问题
现有方案，简单调用 处理时间很长

- [ ] CSDN 上有一个优化方案 https://blog.csdn.net/lddx_123456/article/details/127942376
