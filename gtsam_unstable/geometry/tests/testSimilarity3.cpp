/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSimilarity3.cpp
 * @brief  Unit tests for Similarity3 class
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

/**
 * 3D similarity transform
 */
class Similarity3: public LieGroup<Similarity3, 7> {

  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Translation;

private:
  Rot3 R_;
  Point3 t_;
  double s_;

public:
  /// Construct from Eigen types
  Similarity3(const Matrix3& R, const Vector3& t, double s) {
    R_ = R;
    t_ = t;
    s_ = s;
  }

  /// Return the translation
  const Vector3 t() const {
    return t_.vector();
  }

  /// Return the rotation matrix
  const Matrix3 R() const {
    return R_.matrix();
  }

public:

  Similarity3() :
    R_(), t_(), s_(1){
  }

  /// Construct pure scaling
  Similarity3(double s) {
    s_ = s;
  }

  /// Construct from GTSAM types
  Similarity3(const Rot3& R, const Point3& t, double s) {
    R_ = R;
    t_ = t;
    s_ = s;
  }

  static Similarity3 identity() {
    std::cout << "Identity!" << std::endl;
    return Similarity3(); }

  static Vector7 Logmap(const Similarity3& s, OptionalJacobian<7, 7> Hm = boost::none) {
    std::cout << "Logmap!" << std::endl;
    return Vector7();
  }

  static Similarity3 Expmap(const Vector7& v, OptionalJacobian<7, 7> Hm = boost::none) {
    std::cout << "Expmap!" << std::endl;
    return Similarity3();
  }

  bool operator==(const Similarity3& other) const {
    return (R_.equals(other.R_)) && (t_ == other.t_) && (s_ == other.s_);
  }

  /// Compare with tolerance
  bool equals(const Similarity3& sim, double tol) const {
    return rotation().equals(sim.rotation(), tol) && translation().equals(sim.translation(), tol)
        && scale() < (sim.scale()+tol) && scale() > (sim.scale()-tol);
  }

  Point3 transform_from(const Point3& p) const {
    /*if (Dpose) {
      const Matrix3 R = R_.matrix();
      Matrix3 DR = R * skewSymmetric(-p.x(), -p.y(), -p.z());
      (*Dpose) << DR, R;
    }
    if (Dpoint)
      *Dpoint = R_.matrix();*/
    return R_ * (s_ * p) + t_;
  }

  Matrix7 AdjointMap() const{
    const Matrix3 R = R_.matrix();
    const Vector3 t = t_.vector();
    Matrix3 A = s_ * skewSymmetric(t) * R;
    Matrix7 adj;
    adj << s_*R, A, -s_*t, Z_3x3, R, Eigen::Matrix<double, 3, 1>::Zero(), Eigen::Matrix<double, 1, 6>::Zero(), 1;
    return adj;
  }

  /** syntactic sugar for transform_from */
  inline Point3 operator*(const Point3& p) const {
    return transform_from(p);
  }

  Similarity3 inverse() const {
    Rot3 Rt = R_.inverse();
    Point3 sRt = R_.inverse() * (-s_ * t_);
    return Similarity3(Rt, sRt, 1.0/s_);
  }

  Similarity3 operator*(const Similarity3& T) const {
      return Similarity3(R_ * T.R_, ((1.0/T.s_)*t_) + R_ * T.t_, s_*T.s_);
    }

  void print(const std::string& s) const {
    std::cout << std::endl;
    std::cout << s;
    rotation().print("R:\n");
    translation().print("t: ");
    std::cout << "s: " << scale() << std::endl;
  }

  /// @name Manifold
  /// @{

  /// Dimensionality of tangent space = 7 DOF - used to autodetect sizes
  inline static size_t Dim() {
    return 7;
  }

  /// Dimensionality of tangent space = 7 DOF
  inline size_t dim() const {
    return 7;
  }

  /// Return the rotation matrix
  Rot3 rotation() const {
    return R_;
  }

  /// Return the translation
  Point3 translation() const {
    return t_;
  }

  /// Return the scale
  double scale() const {
    return s_;
  }

  /// Update Similarity transform via 7-dim vector in tangent space
  struct ChartAtOrigin {
    static Similarity3 Retract(const Vector7& v,  ChartJacobian H = boost::none) {

    // Will retracting or localCoordinating R work if R is not a unit rotation?
    // Also, how do we actually get s out?  Seems like we need to store it somewhere.
    Rot3 r;  //Create a zero rotation to do our retraction.
    return Similarity3( //
        r.retract(v.head<3>()), // retract rotation using v[0,1,2]
        Point3(v.segment<3>(3)), // Retract the translation
        1.0 + v[6]); //finally, update scale using v[6]
  }

  /// 7-dimensional vector v in tangent space that makes other = this->retract(v)
  static Vector7 Local(const Similarity3& other,  ChartJacobian H = boost::none) {
    Rot3 r;  //Create a zero rotation to do the retraction
    Vector7 v;
    v.head<3>() = r.localCoordinates(other.R_);
    v.segment<3>(3) = other.t_.vector();
    //v.segment<3>(3) = translation().localCoordinates(other.translation());
    v[6] = other.s_ - 1.0;
    return v;
  }
  };

  using LieGroup<Similarity3, 7>::inverse; // version with derivative

  /// @}

  /// @name Lie Group
  /// @{

  // compose T1*T2
  // between T2*inverse(T1)
  // identity I4
  // inverse inverse(T)

  /// @}

};

template<>
struct traits<Similarity3> : public internal::LieGroupTraits<Similarity3> {};
}

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;
using symbol_shorthand::X;

GTSAM_CONCEPT_TESTABLE_INST(Similarity3)

static Point3 P(0.2,0.7,-2);
static Rot3 R = Rot3::rodriguez(0.3,0,0);
static Similarity3 T(R,Point3(3.5,-8.2,4.2),1);
static Similarity3 T2(Rot3::rodriguez(0.3,0.2,0.1),Point3(3.5,-8.2,4.2),1);
static Similarity3 T3(Rot3::rodriguez(-90, 0, 0), Point3(1, 2, 3), 1);

//******************************************************************************
TEST(Similarity3, Constructors) {
  Similarity3 test;
}

//******************************************************************************
TEST(Similarity3, Getters) {
  Similarity3 test;
  EXPECT(assert_equal(Rot3(), test.rotation()));
  EXPECT(assert_equal(Point3(), test.translation()));
  EXPECT_DOUBLES_EQUAL(1.0, test.scale(), 1e-9);
}

//******************************************************************************
TEST(Similarity3, Getters2) {
  Similarity3 test(Rot3::ypr(1, 2, 3), Point3(4, 5, 6), 7);
  EXPECT(assert_equal(Rot3::ypr(1, 2, 3), test.rotation()));
  EXPECT(assert_equal(Point3(4, 5, 6), test.translation()));
  EXPECT_DOUBLES_EQUAL(7.0, test.scale(), 1e-9);
}

TEST(Similarity3, AdjointMap) {
  Similarity3 test(Rot3::ypr(1,2,3).inverse(), Point3(4,5,6), 7);
  Matrix7 result;
  result <<    -1.5739,   -2.4512,   -6.3651,  -50.7671,  -11.2503,   16.8859,  -28.0000,
      6.3167,   -2.9884,   -0.4111,    0.8502,    8.6373,  -49.7260,  -35.0000,
     -2.5734,   -5.8362,    2.8839,   33.1363,    0.3024,   30.1811,  -42.0000,
           0,         0,         0,   -0.2248,   -0.3502,   -0.9093,         0,
           0,         0,         0,    0.9024,   -0.4269,   -0.0587,         0,
           0,         0,         0,   -0.3676,   -0.8337,    0.4120,         0,
           0,         0,         0,         0,         0,         0,    1.0000;
  EXPECT(assert_equal(result, test.AdjointMap(), 1e-3));
}

TEST(Similarity3, inverse) {
  Similarity3 test(Rot3::ypr(1,2,3).inverse(), Point3(4,5,6), 7);
  Matrix3 Re;
  Re <<    -0.2248,    0.9024,   -0.3676,
   -0.3502,   -0.4269,   -0.8337,
   -0.9093,   -0.0587,    0.4120;
  Vector3 te(-9.8472, 59.7640, 10.2125);
  Similarity3 expected(Re, te, 1.0/7.0);
  EXPECT(assert_equal(expected, test.inverse(), 1e-3));
}

TEST(Similarity3, multiplication) {
  Similarity3 test1(Rot3::ypr(1,2,3).inverse(), Point3(4,5,6), 7);
  Similarity3 test2(Rot3::ypr(1,2,3).inverse(), Point3(8,9,10), 11);
  Matrix3 re;
  re << 0.0688,    0.9863,   -0.1496,
     -0.5665,   -0.0848,   -0.8197,
     -0.8211,    0.1412,    0.5530;
  Vector3 te(-13.6797, 3.2441, -5.7794);
  Similarity3 expected(re, te, 77);
  EXPECT(assert_equal(expected, test1*test2, 1e-2));
}

//******************************************************************************
TEST(Similarity3, Manifold) {
  EXPECT_LONGS_EQUAL(7, Similarity3::Dim());
  Vector z = Vector7::Zero();
  Similarity3 sim;
  EXPECT(sim.retract(z) == sim);

  Vector7 v = Vector7::Zero();
  v(6) = 2;
  Similarity3 sim2;
  EXPECT(sim2.retract(z) == sim2);

  EXPECT(assert_equal(z, sim2.localCoordinates(sim)));

  Similarity3 sim3 = Similarity3(Rot3(), Point3(1, 2, 3), 1);
  Vector v3(7);
  v3 << 0, 0, 0, 1, 2, 3, 0;
  EXPECT(assert_equal(v3, sim2.localCoordinates(sim3)));

//  Similarity3 other = Similarity3(Rot3::ypr(0.01, 0.02, 0.03), Point3(0.4, 0.5, 0.6), 1);
  Similarity3 other = Similarity3(Rot3::ypr(0.1, 0.2, 0.3),Point3(4,5,6),1);

  Vector vlocal = sim.localCoordinates(other);

  EXPECT(assert_equal(sim.retract(vlocal), other, 1e-2));

  Similarity3 other2 = Similarity3(Rot3::ypr(0.3, 0, 0),Point3(4,5,6),1);
  Rot3 R = Rot3::rodriguez(0.3,0,0);

  Vector vlocal2 = sim.localCoordinates(other2);

  EXPECT(assert_equal(sim.retract(vlocal2), other2, 1e-2));

  // TODO add unit tests for retract and localCoordinates
}

/* ************************************************************************* */
TEST( Similarity3, retract_first_order)
{
  Similarity3 id;
  Vector v = zero(7);
  v(0) = 0.3;
  EXPECT(assert_equal(Similarity3(R, Point3(), 1), id.retract(v),1e-2));
  v(3)=0.2;v(4)=0.7;v(5)=-2;
  EXPECT(assert_equal(Similarity3(R, P, 1),id.retract(v),1e-2));
}

/* ************************************************************************* */
TEST(Similarity3, localCoordinates_first_order)
{
  Vector d12 = repeat(7,0.1);
  d12(6) = 1.0;
  Similarity3 t1 = T, t2 = t1.retract(d12);
  EXPECT(assert_equal(d12, t1.localCoordinates(t2)));
}

/* ************************************************************************* */
TEST(Similarity3, manifold_first_order)
{
  Similarity3 t1 = T;
  Similarity3 t2 = T3;
  Similarity3 origin;
  Vector d12 = t1.localCoordinates(t2);
  EXPECT(assert_equal(t2, t1.retract(d12)));
  Vector d21 = t2.localCoordinates(t1);
  EXPECT(assert_equal(t1, t2.retract(d21)));
}

TEST(Similarity3, Optimization) {
  Similarity3 prior = Similarity3(Rot3::ypr(0.1, 0.2, 0.3), Point3(1, 2, 3), 4);
  //prior.print("Goal Transform");
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(7, 1);
  Symbol key('x',1);
  PriorFactor<Similarity3> factor(key, prior, model);

  NonlinearFactorGraph graph;
  graph.push_back(factor);
  //graph.print("Full Graph");

  Values initial;
  initial.insert<Similarity3>(key, Similarity3());
  //initial.print("Initial Estimate");

  Values result;
  LevenbergMarquardtParams params;
  params.setVerbosityLM("TRYCONFIG");
  result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  //result.print("Optimized Estimate");
  EXPECT(assert_equal(prior, result.at<Similarity3>(key), 1e-4));
}

TEST(Similarity3, Optimization2) {
  Similarity3 prior = Similarity3();//Rot3::ypr(0, 0, 0), Point3(1, 2, 3), 4);
  Similarity3 m1 = Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(1.0, 0, 0), 1.0);
  Similarity3 m2 = Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(0.9, 0, 0), 1.0);
  Similarity3 m3 = Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(0.8, 0, 0), 1.0);
  Similarity3 m4 = Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(0.7, 0, 0), 1.42);

  //prior.print("Goal Transform");
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(7, 1);
  SharedDiagonal betweenNoise = noiseModel::Diagonal::Sigmas(
        (Vector(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.010).finished());
  PriorFactor<Similarity3> factor(X(1), prior, model);
  BetweenFactor<Similarity3> b1(X(1), X(2), m1, betweenNoise);
  BetweenFactor<Similarity3> b2(X(2), X(3), m2, betweenNoise);
  BetweenFactor<Similarity3> b3(X(3), X(4), m3, betweenNoise);
  BetweenFactor<Similarity3> b4(X(4), X(1), m4, betweenNoise);


  NonlinearFactorGraph graph;
  graph.push_back(factor);
  graph.push_back(b1);
  graph.push_back(b2);
  graph.push_back(b3);
  graph.push_back(b4);

  graph.print("Full Graph");

  Values initial;
  initial.insert<Similarity3>(X(1), Similarity3());
  initial.insert<Similarity3>(X(2), Similarity3(Rot3::ypr(M_PI/2.0, 0, 0), Point3(1, 0, 0), 1.0));
  initial.insert<Similarity3>(X(3), Similarity3(Rot3::ypr(2.0*M_PI/2.0, 0, 0), Point3(1, 1.5, 0), 1.0));
  initial.insert<Similarity3>(X(4), Similarity3(Rot3::ypr(3.0*M_PI/2.0, 0, 0), Point3(0, 1, 0), 1.0));

  initial.print("Initial Estimate");

  Values result;
  result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Optimized Estimate");
  //EXPECT(assert_equal(prior, result.at<Similarity3>(key), 1e-4));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

