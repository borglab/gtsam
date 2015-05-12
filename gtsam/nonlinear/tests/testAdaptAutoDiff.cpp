/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExpression.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam/3rdparty/ceres/example.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

namespace gtsam {

// Special version of Cal3Bundler so that default constructor = 0,0,0
struct Cal: public Cal3Bundler {
  Cal(double f = 0, double k1 = 0, double k2 = 0, double u0 = 0, double v0 = 0) :
      Cal3Bundler(f, k1, k2, u0, v0) {
  }
  Cal retract(const Vector& d) const {
    return Cal(fx() + d(0), k1() + d(1), k2() + d(2), u0(), v0());
  }
  Vector3 localCoordinates(const Cal& T2) const {
    return T2.vector() - vector();
  }
};

template<>
struct traits<Cal> : public internal::Manifold<Cal> {};

// With that, camera below behaves like Snavely's 9-dim vector
typedef PinholeCamera<Cal> Camera;

}

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// charts
TEST(AdaptAutoDiff, Canonical) {

  Canonical<Point2> chart1;
  EXPECT(chart1.Local(Point2(1, 0))==Vector2(1, 0));
  EXPECT(chart1.Retract(Vector2(1, 0))==Point2(1, 0));

  Vector v2(2);
  v2 << 1, 0;
  Canonical<Vector2> chart2;
  EXPECT(assert_equal(v2, chart2.Local(Vector2(1, 0))));
  EXPECT(chart2.Retract(v2)==Vector2(1, 0));

  Canonical<double> chart3;
  Eigen::Matrix<double, 1, 1> v1;
  v1 << 1;
  EXPECT(chart3.Local(1)==v1);
  EXPECT_DOUBLES_EQUAL(chart3.Retract(v1), 1, 1e-9);

  Canonical<Point3> chart4;
  Point3 point(1, 2, 3);
  Vector v3(3);
  v3 << 1, 2, 3;
  EXPECT(assert_equal(v3, chart4.Local(point)));
  EXPECT(assert_equal(chart4.Retract(v3), point));

  Canonical<Pose3> chart5;
  Pose3 pose(Rot3::identity(), point);
  Vector v6(6);
  v6 << 0, 0, 0, 1, 2, 3;
  EXPECT(assert_equal(v6, chart5.Local(pose)));
  EXPECT(assert_equal(chart5.Retract(v6), pose));

  Canonical<Cal> chart6;
  Cal cal0;
  Vector z3 = Vector3::Zero();
  EXPECT(assert_equal(z3, chart6.Local(cal0)));
  EXPECT(assert_equal(chart6.Retract(z3), cal0));

  Canonical<Camera> chart7;
  Camera camera(Pose3(), cal0);
  Vector z9 = Vector9::Zero();
  EXPECT(assert_equal(z9, chart7.Local(camera)));
  EXPECT(assert_equal(chart7.Retract(z9), camera));
}

/* ************************************************************************* */
// Some Ceres Snippets copied for testing
// Copyright 2010, 2011, 2012 Google Inc. All rights reserved.
template<typename T> inline T &RowMajorAccess(T *base, int rows, int cols,
    int i, int j) {
  return base[cols * i + j];
}

inline double RandDouble() {
  double r = static_cast<double>(rand());
  return r / RAND_MAX;
}

// A structure for projecting a 3x4 camera matrix and a
// homogeneous 3D point, to a 2D inhomogeneous point.
struct Projective {
  // Function that takes P and X as separate vectors:
  //   P, X -> x
  template<typename A>
  bool operator()(A const P[12], A const X[4], A x[2]) const {
    A PX[3];
    for (int i = 0; i < 3; ++i) {
      PX[i] = RowMajorAccess(P, 3, 4, i, 0) * X[0]
          + RowMajorAccess(P, 3, 4, i, 1) * X[1]
          + RowMajorAccess(P, 3, 4, i, 2) * X[2]
          + RowMajorAccess(P, 3, 4, i, 3) * X[3];
    }
    if (PX[2] != 0.0) {
      x[0] = PX[0] / PX[2];
      x[1] = PX[1] / PX[2];
      return true;
    }
    return false;
  }

  // Adapt to eigen types
  Vector2 operator()(const MatrixRowMajor& P, const Vector4& X) const {
    Vector2 x;
    if (operator()(P.data(), X.data(), x.data()))
      return x;
    else
      throw std::runtime_error("Projective fail");
  }
};

/* ************************************************************************* */
// Test Ceres AutoDiff
TEST(AdaptAutoDiff, AutoDiff) {
  using ceres::internal::AutoDiff;

  // Instantiate function
  Projective projective;

  // Make arguments
  typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> M;
  M P;
  P << 1, 0, 0, 0, 0, 1, 0, 5, 0, 0, 1, 0;
  Vector4 X(10, 0, 5, 1);

  // Apply the mapping, to get image point b_x.
  Vector expected = Vector2(2, 1);
  Vector2 actual = projective(P, X);
  EXPECT(assert_equal(expected,actual,1e-9));

  // Get expected derivatives
  Matrix E1 = numericalDerivative21<Vector2, M, Vector4>(Projective(), P, X);
  Matrix E2 = numericalDerivative22<Vector2, M, Vector4>(Projective(), P, X);

  // Get derivatives with AutoDiff
  Vector2 actual2;
  MatrixRowMajor H1(2, 12), H2(2, 4);
  double *parameters[] = { P.data(), X.data() };
  double *jacobians[] = { H1.data(), H2.data() };
  CHECK(
      (AutoDiff<Projective, double, 12, 4>::Differentiate( projective, parameters, 2, actual2.data(), jacobians)));
  EXPECT(assert_equal(E1,H1,1e-8));
  EXPECT(assert_equal(E2,H2,1e-8));
}

/* ************************************************************************* */
// Test Ceres AutoDiff on Snavely, defined in ceres_example.h
// Adapt to GTSAM types
Vector2 adapted(const Vector9& P, const Vector3& X) {
  SnavelyProjection snavely;
  Vector2 x;
  if (snavely(P.data(), X.data(), x.data()))
    return x;
  else
    throw std::runtime_error("Snavely fail");
}

TEST(AdaptAutoDiff, AutoDiff2) {
  using ceres::internal::AutoDiff;

  // Instantiate function
  SnavelyProjection snavely;

  // Make arguments
  Vector9 P; // zero rotation, (0,5,0) translation, focal length 1
  P << 0, 0, 0, 0, 5, 0, 1, 0, 0;
  Vector3 X(10, 0, -5); // negative Z-axis convention of Snavely!

  // Apply the mapping, to get image point b_x.
  Vector expected = Vector2(2, 1);
  Vector2 actual = adapted(P, X);
  EXPECT(assert_equal(expected,actual,1e-9));

  // Get expected derivatives
  Matrix E1 = numericalDerivative21<Vector2, Vector9, Vector3>(adapted, P, X);
  Matrix E2 = numericalDerivative22<Vector2, Vector9, Vector3>(adapted, P, X);

  // Get derivatives with AutoDiff
  Vector2 actual2;
  MatrixRowMajor H1(2, 9), H2(2, 3);
  double *parameters[] = { P.data(), X.data() };
  double *jacobians[] = { H1.data(), H2.data() };
  CHECK(
      (AutoDiff<SnavelyProjection, double, 9, 3>::Differentiate( snavely, parameters, 2, actual2.data(), jacobians)));
  EXPECT(assert_equal(E1,H1,1e-8));
  EXPECT(assert_equal(E2,H2,1e-8));
}

/* ************************************************************************* */
// Test AutoDiff wrapper Snavely
TEST(AdaptAutoDiff, AutoDiff3) {

  // Make arguments
  Camera P(Pose3(Rot3(), Point3(0, 5, 0)), Cal(1, 0, 0));
  Point3 X(10, 0, -5); // negative Z-axis convention of Snavely!

  typedef AdaptAutoDiff<SnavelyProjection, Point2, Camera, Point3> Adaptor;
  Adaptor snavely;

  // Apply the mapping, to get image point b_x.
  Point2 expected(2, 1);
  Point2 actual = snavely(P, X);
  EXPECT(assert_equal(expected,actual,1e-9));

//  // Get expected derivatives
  Matrix E1 = numericalDerivative21<Point2, Camera, Point3>(Adaptor(), P, X);
  Matrix E2 = numericalDerivative22<Point2, Camera, Point3>(Adaptor(), P, X);

  // Get derivatives with AutoDiff, not gives RowMajor results!
  Matrix29 H1;
  Matrix23 H2;
  Point2 actual2 = snavely(P, X, H1, H2);
  EXPECT(assert_equal(expected,actual2,1e-9));
  EXPECT(assert_equal(E1,H1,1e-8));
  EXPECT(assert_equal(E2,H2,1e-8));
}

/* ************************************************************************* */
// Test AutoDiff wrapper in an expression
TEST(AdaptAutoDiff, Snavely) {
  Expression<Camera> P(1);
  Expression<Point3> X(2);
  typedef AdaptAutoDiff<SnavelyProjection, Point2, Camera, Point3> Adaptor;
  Expression<Point2> expression(Adaptor(), P, X);
#ifdef GTSAM_USE_QUATERNIONS
  EXPECT_LONGS_EQUAL(384,expression.traceSize()); // Todo, should be zero
#else
  EXPECT_LONGS_EQUAL(416,expression.traceSize()); // Todo, should be zero
#endif
  set<Key> expected = list_of(1)(2);
  EXPECT(expected == expression.keys());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

