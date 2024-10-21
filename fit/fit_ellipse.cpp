#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// ref https://github.com/bdhammel/least-squares-ellipse-fitting/blob/master/ellipse.py

Eigen::MatrixXd coef_;

// std::vector<double> xs;
// std::vector<double> ys;
Eigen::MatrixXd X;

bool check_data()
{
  return X.rows() >= 5;
}
// bool check_data()
// {
//   return xs.size() == ys.size() && xs.size() >= 5;
// }

void fit(const Eigen::MatrixXd &X)
{
  if (!check_data())
  {
    std::cout << "points num < 5" << std::endl;
    return;
  }
  Eigen::VectorXd x = X.col(0);
  Eigen::VectorXd y = X.col(1);
  Eigen::MatrixXd D1(X.rows(), 3);

  D1.col(0) = x.array().square();    // x^2
  D1.col(1) = x.array() * y.array(); // xy
  D1.col(2) = y.array().square();    // y^2

  Eigen::MatrixXd D2(X.rows(), 3);
  D2.col(0) = x;
  D2.col(1) = y;
  D2.col(2) = Eigen::VectorXd::Ones(X.rows());

  Eigen::Matrix3d S1 = D1.transpose() * D1;
  Eigen::MatrixXd S2 = D1.transpose() * D2;
  Eigen::Matrix3d S3 = D2.transpose() * D2;

  // constraint matrix
  Eigen::Matrix3d C1;
  C1 << 0, 0, 2,
      0, -1, 0,
      2, 0, 0;

  // reduced scatter matrix
  Eigen::Matrix3d M = C1.inverse() * (S1 - S2 * S3.inverse() * S2.transpose());

  // svd
  Eigen::EigenSolver<Eigen::Matrix3d> eigensolver(M); // selfadjointEigenSolver error
  if (eigensolver.info() != Eigen::Success)
  {
    std::cerr << "Eigen decoposition failed" << std::endl;
    return;
  }

  // extract
  Eigen::Vector3d eigval = eigensolver.eigenvalues().real();
  Eigen::Matrix3d eigvec = eigensolver.eigenvectors().real();

  // eigenvector must meet the constratint 4ac-b^2 > 0
  Eigen::VectorXd cond = 4 * eigvec.row(0).array() * eigvec.row(2).array() - eigvec.row(1).array().square();
  int index = -1;
  for (size_t i = 0; i < cond.size(); i++)
  {
    if (cond(i) > 0)
    {
      index = i;
      break;
    }
  }

  if (index == -1)
  {
    std::cerr << "No valid solution found" << std::endl;
    return;
  }
  Eigen::Vector3d a1 = eigvec.col(index);
  if (a1(0) > 0) // 与numpy 保持一致 影响 后续偏转角计算 不影响其他指标
  {
      a1 = -1 * a1;
  }

  // |d f g> = -S3^(-1) * S2^(T)*|a b c>
  Eigen::Vector3d a2 = -S3.inverse() * S2.transpose() * a1;

  coef_.resize(6, 1);
  coef_ << a1, a2;
}

void parseCoeff2GeoFeatures()
{
  // a*x^2 +   b*x*y + c*y^2 +   d*x +   e*y + f = 0  (*)  Eqn 1
  // a*x^2 + 2*b*x*y + c*y^2 + 2*d*x + 2*f*y + g = 0  (**) Eqn 15
  double a = coef_(0, 0);
  double b = coef_(1, 0) * 0.5;
  double c = coef_(2, 0);
  double d = coef_(3, 0) * 0.5;
  double f = coef_(4, 0) * 0.5;
  double g = coef_(5, 0);

  double x0 = (c * d - b * f) / (b * b - a * c);
  double y0 = (a * f - b * d) / (b * b - a * c);

  double numerator = 2 * (a * f * f + c * d * d + g * b * b - 2 * b * d * f - a * c * g);
  double denominator1 = (b * b - a * c) * (std::sqrt((a - c) * (a - c) + 4 * b * b) - (c + a));
  double denominator2 = (b * b - a * c) * (-std::sqrt((a - c) * (a - c) + 4 * b * b) - (c + a));

  if (denominator1 <= 0 || denominator2 <= 0) // 负值正常 后续 sqrt 内 为正
  {
      //throw std::runtime_error("Negative denominator in semi-axes length calculation");
      std::cout<< "Negative denominator in semi-axes length calculation, Maybe lead to following error" <<std::endl;
  }

  double height = std::sqrt(numerator / denominator1);
  double width = std::sqrt(numerator / denominator2);

  double phi; // 偏转角 长轴与x轴夹角
  if (b == 0 && a > c)
  {
    phi = 0.0;
  }
  else if (b == 0 && a < c)
  {
    phi = M_PI * 0.5;
  }
  else if (b != 0 && a > c)
  {
    phi = 0.5 * std::atan(2 * b / (a - c));
  }
  else if (b != 0 && a < c)
  {
    phi = 0.5 * (M_PI + std::atan(2 * b / (a - c)));
  }
  else if (a == c)
  {
    std::cerr << "Ellipse is a perfect circle, the answer is degenerate." << std::endl;
    phi = 0.0;
  }
  else
  {
    throw std::runtime_error("Unreachable state in angle calculation");
  }

  std::cout << "Center: (" << x0 << ", " << y0 << ")" << std::endl;
  std::cout << "Width: " << width << std::endl;
  std::cout << "Height: " << height << std::endl;
  std::cout << "Rotation angle (phi): " << phi/M_PI << " PI" << std::endl;
}

void xy2ellipse(double cx, double cy, double width, double height, double phi, int n_points)
{
  std::vector<double> t_values(n_points);
  double step = 2.0 * M_PI / (n_points - 1);
  for (int i = 0; i < n_points; ++i)
  {
    t_values[i] = i * step;
  }

  // Prepare x and y arrays
  std::vector<double> x(n_points);
  std::vector<double> y(n_points);

  // Generate the points on the ellipse
  for (int i = 0; i < n_points; ++i)
  {
    double t = t_values[i];
    x[i] = cx + width * std::cos(t) * std::cos(phi) - height * std::sin(t) * std::sin(phi);
    y[i] = cy + width * std::cos(t) * std::sin(phi) + height * std::sin(t) * std::cos(phi);
  }
}

void readPCDToMatrix(std::string pcd_path, Eigen::MatrixXd &pointsxy)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read pcd file \n");
    return;
  }
  std::cout << "the num of points: " << cloud->points.size() << std::endl;
  // 处理点云应该是经过PCA 变换到标准坐标系下 主方向为Z轴, 投影到xy面上 为需要拟合的椭圆数据
  // Resize the matrix to hold all points
  pointsxy.resize(cloud->points.size(), 2); // Rows for points, 2 columns (x, y)
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    // pointsxy << cloud->points.at(i).x, cloud->points.at(i).y; // 固定大小可以这样初始化
    pointsxy(i, 0) = cloud->points[i].x; // x-coordinate
    pointsxy(i, 1) = cloud->points[i].y; // y-coordinate
  }
}

void readTxtToMatrix(const char *txt_path, const std::string &delimiter, Eigen::MatrixXd &pointsxy)
{
  std::ifstream infile(txt_path);
  std::string line;
  std::vector<std::string> lines;

  if (infile.is_open())
  {
    while (std::getline(infile, line))
    {
      lines.emplace_back(line);
    }
    infile.close();
  }
  else
  {
    std::cerr << "Unable to open file" << std::endl;
  }

  // parse
  std::cout << "the num of points: " << lines.size() << std::endl;
  pointsxy.resize(lines.size(), 2);
  int cnt = 0;
  size_t delim_len = delimiter.length();
  for (size_t i = 0; i < lines.size(); i++)
  {
    cnt = 0;
    size_t pos_start = 0, pos_end = 0;
    std::string line_ = lines[i];
    while ((pos_end = line_.find(delimiter, pos_start)) != std::string::npos)
    {
      pointsxy(i, cnt++) = atof(line_.substr(pos_start, pos_end - pos_start).c_str());
      pos_start = pos_end + delim_len;
      if (cnt == 2)
      {
        break;
      }
    }
    if (cnt == 2)
    {
      continue;
    }
    else
    {
      pointsxy(i, cnt++) = atof(line_.substr(pos_start).c_str());
    }
  }
}

// ref https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
std::vector<std::string> split(std::string s, std::string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

int main(int argc, char **argv)
{
  std::string pcd_path = "E:/01-LabProjects/149/211/in0003 - Cloud.pcd";
  std::string txt_path = "E:/01-LabProjects/149/211/in_0003.txt";
  txt_path = "E:/my-github-repos/01-my_repos/learn_pcl/python_algorithms/ellipse_test.csv";
  // readPCDToMatrix(pcd_path, X);
  readTxtToMatrix(txt_path.c_str(), ",", X);
  fit(X);
  parseCoeff2GeoFeatures();

  return 0;
}