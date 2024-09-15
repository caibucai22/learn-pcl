#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/filters/filter.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3D.h> // 拟合3D圆

#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include "../my_functions.h"
#include <thread>
#include <limits>

constexpr float ZERO_TOLERANCE_F = std::numeric_limits<float>::epsilon();
constexpr double ZERO_TOLERANCE_D = static_cast<double>(ZERO_TOLERANCE_F);
constexpr double ZERO_SQUARED_TOLERANCE_D = ZERO_TOLERANCE_D * ZERO_TOLERANCE_D;
constexpr float PC_NAN = std::numeric_limits<float>::quiet_NaN();

class Neighbourhood
{

public:
  enum GeomElement
  {
    FLAG_DEPRECATED = 0,
    FLAG_GRAVITY_CENTER = 1,
    FLAG_LS_PLANE = 2,
    FLAG_QUADRIC = 4
  };

  enum CurvatureType
  {
    GAUSSIAN_CURV = 1,
    MEAN_CURV,
    NORMAL_CHANGE_RATE
  };

  explicit Neighbourhood(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  virtual ~Neighbourhood() = default;

  virtual void reset();

  pcl::PointCloud<pcl::PointXYZ>::Ptr getInputCloud() { return cloud; }

  enum InputVectorsUsage
  {
    UseOXYasBase,
    UseYAsUpDir,
    None
  };

  enum GeomFeature
  {
    EigenValueSum = 1,
    Omnivariance,
    EigenEntropy,
    Anisotropy,
    Planarity,
    Linearity,
    PCA1,
    PCA2,
    SurfaceVariation,
    Sphericity,
    Verticality,
    EigenValue1,
    EigenValue2,
    EigenValue3
  };

  static void computeBaseVectors(const Eigen::Vector3f &N, Eigen::Vector3f &X, Eigen::Vector3f &Y)
  {
    Eigen::Vector3f Nunit = N;
    Nunit.normalize();

    X = Nunit.unitOrthogonal();
    Y = N.cross(X);
  }

  bool projectPointsOn2DPlane(std::vector<Eigen::Vector2f> &points2D,
                              const float *planeEquation = nullptr,
                              Eigen::Vector3f *o = nullptr,
                              Eigen::Vector3f *x = nullptr,
                              Eigen::Vector3f *y = nullptr,
                              InputVectorsUsage vectorsUsage = None)
  {
    unsigned count = (cloud ? cloud->size() : 0);
    if (!count)
    {
      return false;
    }

    if (!planeEquation)
    {
      planeEquation = getLSPlane();
      if (!planeEquation)
      {
        return false;
      }
    }
    try
    {
      points2D.resize(count);
    }
    catch (const std::bad_alloc &e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }

    Eigen::Vector3f G(0, 0, 0), u(1, 0, 0), v(0, 1, 0);
    if ((vectorsUsage == UseOXYasBase) && o && x && y)
    {
      G = *o;
      u = *x;
      v = *y;
    }
    else
    {
      Eigen::Vector3f N(planeEquation[0], planeEquation[1], planeEquation[2]);
      if ((vectorsUsage == UseYAsUpDir) && y)
      {
        v = (*y - y->dot(N) * N);
        v.normalize();
        u = v.cross(N);
      }
      else
      {
        computeBaseVectors(N, u, v);
      }
      const Eigen::Vector3f *_G = getGravityCenter();
      assert(_G);
      G = *_G;
    }

    // project the points
    for (unsigned i = 0; i < count; i++)
    {
      Eigen::Vector3f oldP(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
      const Eigen::Vector3f P = oldP - G;
      points2D[i] = Eigen::Vector2f(P.dot(u), P.dot(v));
    }

    // output local base
    if (vectorsUsage != UseOXYasBase)
    {
      if (o)
      {
        *o = G;
      }
      if (x)
      {
        *x = u;
      }
      if (y)
      {
        *y = v;
      }
    }
    return true;
  }

  void setGravityCenter(const Eigen::Vector3f &G);
  const Eigen::Vector3f *getGravityCenter();

  const float *getLSPlane();
  void setLSPlane(const float eq[4],
                  Eigen::Vector3f &X,
                  Eigen::Vector3f &Y,
                  Eigen::Vector3f &N);
  const Eigen::Vector3f *getLSPlaneX();
  const Eigen::Vector3f *getLSPlaneY();
  const Eigen::Vector3f *getLSPlaneNormal();

  //const float *getQuadric();
  //bool compute3DQuadric(double quadricEquation[10]);

  void computeEigenValuesAndVectors(Eigen::Vector3f &eigValues, Eigen::Matrix3f &eigVectors);
  float computeLargestRadius();

  inline bool LessThanSquareEpsilon(double x)
  {
    return x < ZERO_SQUARED_TOLERANCE_D;
  }

protected:
  // 2.5D Quadric equation
  // Array [a,b,c,d,e,f] such that Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2.
  float m_quardricEquation[6];

  //
  char m_quadricEquationDirections[3]; // x y z

  float m_lsPlaneEquation[4];

  Eigen::Vector3f m_lsPlaneVectors[3];

  Eigen::Vector3f m_gravityCenter;

  unsigned char m_structuresValidity;

  void computeGravityCenter();
  bool computeLeastSquareFittingPlane();
  //bool computeQuadirc();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

Neighbourhood::Neighbourhood(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    //: m_quadricEquationDirections(0, 1, 2)
    : m_structuresValidity(FLAG_DEPRECATED)
    , cloud(cloud)
{
    //memset(m_quadricEquation, 0, sizeof(PointCoordinateType) * 6);
    memset(m_lsPlaneEquation, 0, sizeof(float) * 4);

    assert(cloud);
}

void Neighbourhood::reset()
{
    m_structuresValidity = FLAG_DEPRECATED;
}

void Neighbourhood::computeGravityCenter()
{
  pcl::PointXYZ center;
  pcl::computeCentroid(*cloud, center);
  setGravityCenter(Eigen::Vector3f(center.x, center.y, center.z));
}


const Eigen::Vector3f* Neighbourhood::getGravityCenter()
{
    if (!(m_structuresValidity & FLAG_GRAVITY_CENTER))
        computeGravityCenter();
    return ((m_structuresValidity & FLAG_GRAVITY_CENTER) ? &m_gravityCenter : nullptr);
}
void Neighbourhood::setGravityCenter(const Eigen::Vector3f &G)
{
  m_gravityCenter = G;
  m_structuresValidity |= FLAG_GRAVITY_CENTER;
}

void Neighbourhood::computeEigenValuesAndVectors(Eigen::Vector3f &eigValues, Eigen::Matrix3f &eigVectors)
{
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);

  // pca.project(*cloud, *pca_projet_cloud);
  // std::cout << "eigenvectors: \n"
  //           << pca.getEigenVectors() << std::endl;
  // std::cout << "eigenvalues: \n"
  //           << pca.getEigenValues() << std::endl;

  eigValues = pca.getEigenValues();
  eigVectors = pca.getEigenVectors();
}

static void getMinEigenValueAndVector(const Eigen::Vector3f & eigenValues,
                                      const Eigen::Matrix3f & eigenVectors,
                                      double &minEigenValue,
                                      Eigen::Vector3f &minEigenVector)
{
  unsigned n = eigenValues.size();

  unsigned minIndex = 0;
  for (unsigned i = 1; i < n; ++i)
  {
    if (eigenValues[i] < eigenValues[minIndex])
    {
      minIndex = i;
    }
  }

  minEigenValue = eigenValues[minIndex];
  minEigenVector = eigenVectors.col(minIndex);
}

static void getMaxEigenValueAndVector(const Eigen::Vector3f & eigenValues,
                                      const Eigen::Matrix3f & eigenVectors,
                                      double &maxEigenValue,
                                      Eigen::Vector3f &maxEigenVector)
{
  unsigned n = eigenValues.size();

  unsigned maxIndex = 0;
  for (unsigned i = 1; i < n; ++i)
  {
    if (eigenValues[i] > eigenValues[maxIndex])
    {
      maxIndex = i;
    }
  }

  maxEigenValue = eigenValues[maxIndex];
  maxEigenVector = eigenVectors.col(maxIndex);
}

bool Neighbourhood::computeLeastSquareFittingPlane()
{
  // invalidate previous LS plane (if any)
  m_structuresValidity &= (~FLAG_LS_PLANE);

  assert(cloud);
  unsigned pointcount = (cloud ? cloud->size() : 0);

  if (pointcount < 3)
  {
    // not enough points!
    return false;
  }
  Eigen::Vector3f G(0, 0, 0);
  if (pointcount > 3)
  {
    Eigen::Vector3f eigValues;
    Eigen::Matrix3f eigVectos;
    computeEigenValuesAndVectors(eigValues, eigVectos);

    // get normal
    {
      Eigen::Vector3f vec(0, 0, 1);
      double minEigValue = 0;
      getMinEigenValueAndVector(eigValues, eigVectos, minEigValue, vec);
      m_lsPlaneVectors[2] = vec;
    }

    // get also X
    {
      Eigen::Vector3f vec;
      double maxEigValue = 0;
      getMaxEigenValueAndVector(eigValues, eigVectos, maxEigValue, vec);
      m_lsPlaneVectors[0] = vec;
    }

    G = *getGravityCenter();
  }
  else
  {
    // we simply compute the normal of the 3 points by cross product!
    auto p2EigenV = [](pcl::PointXYZ p)
    {
      return new Eigen::Vector3f(p.x, p.y, p.z);
    };
    const Eigen::Vector3f *A = p2EigenV(cloud->points[0]);
    const Eigen::Vector3f *B = p2EigenV(cloud->points[1]);
    const Eigen::Vector3f *C = p2EigenV(cloud->points[2]);

    // get X (AB by default) and Y (AC - will be updated later) and deduce N = X ^ Y
    m_lsPlaneVectors[0] = (*B - *A);
    m_lsPlaneVectors[1] = (*C - *A);
    m_lsPlaneVectors[2] = m_lsPlaneVectors[0].cross(m_lsPlaneVectors[1]);

    // the plane passes through any of the 3 points
    G = *A;
  }
  if (LessThanSquareEpsilon(m_lsPlaneVectors[2].squaredNorm()))
  {
    return false;
  }
  else
  {
    m_lsPlaneVectors[2].normalize();
  }
  // normalize x as well
  m_lsPlaneVectors[0].normalize();
  // update y
  m_lsPlaneVectors[1] = m_lsPlaneVectors[2].cross(m_lsPlaneVectors[0]);

  // deduce the proper equation
  m_lsPlaneEquation[0] = m_lsPlaneVectors[2][0];
  m_lsPlaneEquation[1] = m_lsPlaneVectors[2][1];
  m_lsPlaneEquation[2] = m_lsPlaneVectors[2][2];

  m_lsPlaneEquation[3] = G.dot(m_lsPlaneVectors[2]);

  m_structuresValidity |= FLAG_LS_PLANE;
  return true;
}


const float* Neighbourhood::getLSPlane()
{
    if (!(m_structuresValidity & FLAG_LS_PLANE))
        computeLeastSquareFittingPlane();
    return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneEquation : nullptr);
}

void Neighbourhood::setLSPlane(const float eq[4],
                               Eigen::Vector3f &X,
                               Eigen::Vector3f &Y,
                               Eigen::Vector3f &N)
{
  memcpy(m_lsPlaneEquation, eq, sizeof(float) * 4);
  m_lsPlaneVectors[0] = X;
  m_lsPlaneVectors[1] = Y;
  m_lsPlaneVectors[2] = N;

  m_structuresValidity |= FLAG_LS_PLANE;
}

const Eigen::Vector3f *Neighbourhood::getLSPlaneX()
{
  if (!(m_structuresValidity & FLAG_LS_PLANE))
  {
    computeLeastSquareFittingPlane();
  }
  return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors : nullptr);
}

const Eigen::Vector3f *Neighbourhood::getLSPlaneY()
{
  if (!(m_structuresValidity & FLAG_LS_PLANE))
  {
    computeLeastSquareFittingPlane();
  }
  return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors + 1 : nullptr);
}

const Eigen::Vector3f *Neighbourhood::getLSPlaneNormal()
{
  if (!(m_structuresValidity & FLAG_LS_PLANE))
  {
    computeLeastSquareFittingPlane();
  }
  return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors + 2 : nullptr);
}

float Neighbourhood::computeLargestRadius()
{
  assert(cloud);
  unsigned pointcount = (cloud ? cloud->size() : 0);

  const Eigen::Vector3f *G = getGravityCenter();
  if (!G)
  {
    assert(false);
    return PC_NAN;
  }

  double maxSquareDist = 0;
  for (unsigned i = 0; i < pointcount; i++)
  {
    pcl::PointXYZ p = cloud->points[i];
    const Eigen::Vector3f *P = new Eigen::Vector3f(p.x, p.y, p.z);
    const double d2 = (*P - *G).squaredNorm();
    if (d2 > maxSquareDist)
    {
      maxSquareDist = d2;
    }
  }
  return static_cast<float>(sqrt(maxSquareDist));
}