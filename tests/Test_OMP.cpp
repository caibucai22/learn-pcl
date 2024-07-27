#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>

void test_for()
{
  std::vector<int> v;
  for (int i = 0; i < 100000000; i++)
  {
    v.emplace_back(i);
  }
}

void test_omp_for()
{
  std::vector<int> v;
#pragma omp parallel for
  for (int i = 0; i < 100000000; i++)
  {
    v.emplace_back(rand() + rand());
  }
  std::cout << "omp for done!" << std::endl;
}

int main()
{
  //   int nthreads, tid;

  // /* Fork a team of threads giving them their own copies of variables */
  // #pragma omp parallel private(nthreads, tid)
  //   {

  //     /* Obtain thread number */
  //     tid = omp_get_thread_num();
  //     printf("Hello World from thread = %d\n", tid);

  //     /* Only master thread does this */
  //     if (tid == 0)
  //     {
  //       nthreads = omp_get_num_threads();
  //       printf("Number of threads = %d\n", nthreads);
  //     }

  //   } /* All threads join master thread and disband */

  auto t1 = clock();
  double omp_t1 = omp_get_wtime();
  test_for();
  std::cout << "normal for loop, elapsed time " << double(clock() - t1) / CLOCKS_PER_SEC << std::endl;
  std::cout << "normal for loop, elapsed time " << double(omp_get_wtime() - omp_t1) << std::endl;

  auto t2 = clock();
  double omp_t2 = omp_get_wtime();
  test_omp_for();
  std::cout << "omp loop, elapsed time " << double(clock() - t2) / CLOCKS_PER_SEC << std::endl;
  std::cout << "omp loop, elapsed time " << double(omp_get_wtime() - omp_t2) << std::endl;

  return 0;
}