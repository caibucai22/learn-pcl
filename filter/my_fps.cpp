/**
 * @file my_fps.cpp 伪代码
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-09-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <vector>
/**
 * 最远点采样
 * 1. 点云A随机选取一个点作为起始点，放入 最远点集B中
 * 2. 计算剩余点 和 点集B中的 距离 选择一个最大值对于的点 B_new, 此时在这个过程中存储 剩余点 到 点集B中的 距离(最远距离)
 * 3. 再次计算  A中剩余点 和 B_new 的距离，判断 是否距离 > 之前到B的距离 大于则更新
 * 同时维护一个当前一次循环中的 最大值 和 最大值索引，判断每一个点的距离 和 最大距离，更新索引
 *
 * 即新算的 剩余点 到 上次选择的点 的距离 要做两件事 a 判断是否 > 之前选择点的距离，大于则更新，没有大于 则判断 剩余点 到 点集B的距离
 * 是否大于 最大距离
 * 
 * 应该是小于 所谓的最远点采样 其实是 最小距离的最大化 
 *
 */

void fps()
{
  const int n = 1000000;
  float remain2choose[n];
  int sample_size = 10;
  std::vector<int> chooseIdx;
  // 随机初始化一个

  for (int i = 1; i < sample_size; i++)
  {
    float cur_loop_maxdistance = -1;
    int cur_loop_choose_idx = -1;

    for (int j = 0; j < n; j++)
    {
      float dis = chooseIdx.back()*j;

      if(dis > remain2choose[j])
      {
        remain2choose[j] = dis;
      }
      if(remain2choose[j] > cur_loop_maxdistance)
      {
        cur_loop_maxdistance = remain2choose[j];
        cur_loop_choose_idx = j;
      }
    }
    chooseIdx.push_back(cur_loop_choose_idx);
  }
}