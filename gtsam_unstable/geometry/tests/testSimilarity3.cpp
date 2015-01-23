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

  Matrix7 adjointMap() const{
    const Matrix3 R = R_.matrix();
    const Vector3 t = t_.vector();
    Matrix3 A = s_ * skewSymmetric(t) * R;
    Matrix7 adj;
    adj << s_*R, A, -s_*t, Z_3x3, R, Eigen::Matrix<double, 3, 1>::Zero(), Z_3x3, Z_3x3, 1;
  }

  /** syntactic sugar for transform_from */
  inline Point3 operator*(const Point3& p) const {
    return transform_from(p);
  }

  /*Similarity3 inverse() const {
    Rot3 Rt = R_.inverse();
    return Pose3(Rt, Rt * (-t_));
  }*/

  Similarity3 operator*(const Similarity3& T) const {
      return Similarity3(R_ * T.R_, ((1.0/s_)*t_) + R_ * T.t_, s_*T.s_);
    }

  void print(const std::string& s) const {
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
/*  Similarity3 retract(const Vector7& v) const {

    // Will retracting or localCoordinating R work if R is not a unit rotation?
    // Also, how do we actually get s out?  Seems like we need to store it somewhere.
    return Similarity3( //
        R_.retract(v.head<3>()), // retract rotation using v[0,1,2]
        t_.retract(R() * v.segment<3>(3)), // Retract the translation
        scale() + v[6]); //finally, update scale using v[6]
  }

  /// 7-dimensional vector v in tangent space that makes other = this->retract(v)
  Vector7 localCoordinates(const Similarity3& other) const {

    Vector7 v;
    v.head<3>() = R_.localCoordinates(other.R_);
    v.segment<3>(3) = R_.unrotate(other.t_ - t_).vector();
    //v.segment<3>(3) = translation().localCoordinates(other.translation());
    v[6] = other.s_ - s_;
    return v;
  }*/

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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

//GTSAM_CONCEPT_POSE_INST(Similarity3);

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
  prior.print("goal angle");
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(7, 1);
  Symbol key('x',1);
  Symbol key2('x',2);
  PriorFactor<Similarity3> factor(key, prior, model);

  NonlinearFactorGraph graph;
  graph.push_back(factor);
  graph.print("full graph");

  Values initial;
  initial.insert<Similarity3>(key, Similarity3());
  initial.print("initial estimate");

  Values result;
  result.insert(key2, LevenbergMarquardtOptimizer(graph, initial).optimize());
  result.print("final result");
  EXPECT(assert_equal(prior, result.at<Similarity3>(key2), 1e-2));
}


//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

