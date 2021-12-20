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
struct Cal3Bundler0 : public Cal3Bundler {
  Cal3Bundler0(double f = 0, double k1 = 0, double k2 = 0, double u0 = 0,
               double v0 = 0)
      : Cal3Bundler(f, k1, k2, u0, v0) {}
  Cal3Bundler0 retract(const Vector& d) const {
    return Cal3Bundler0(fx() + d(0), k1() + d(1), k2() + d(2), px(), py());
  }
  Vector3 localCoordinates(const Cal3Bundler0& T2) const {
    return T2.vector() - vector();
  }
};

template <>
struct traits<Cal3Bundler0> : public internal::Manifold<Cal3Bundler0> {};

// With that, camera below behaves like Snavely's 9-dim vector
typedef PinholeCamera<Cal3Bundler0> Camera;
}

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Check that ceres rotation convention is the same
TEST(AdaptAutoDiff, Rotation) {
  Vector3 axisAngle(0.1, 0.2, 0.3);
  Matrix3 expected = Rot3::Rodrigues(axisAngle).matrix();
  Matrix3 actual;
  ceres::AngleAxisToRotationMatrix(axisAngle.data(), actual.data());
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Some Ceres Snippets copied for testing
// Copyright 2010, 2011, 2012 Google Inc. All rights reserved.
template <typename T>
inline T& RowMajorAccess(T* base, int rows, int cols, int i, int j) {
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
  template <typename A>
  bool operator()(A const P[12], A const X[4], A x[2]) const {
    A PX[3];
    for (int i = 0; i < 3; ++i) {
      PX[i] = RowMajorAccess(P, 3, 4, i, 0) * X[0] +
              RowMajorAccess(P, 3, 4, i, 1) * X[1] +
              RowMajorAccess(P, 3, 4, i, 2) * X[2] +
              RowMajorAccess(P, 3, 4, i, 3) * X[3];
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
  typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> RowMajorMatrix34;
  RowMajorMatrix34 P;
  P << 1, 0, 0, 0, 0, 1, 0, 5, 0, 0, 1, 0;
  Vector4 X(10, 0, 5, 1);

  // Apply the mapping, to get image point b_x.
  Vector expected = Vector2(2, 1);
  Vector2 actual = projective(P, X);
  EXPECT(assert_equal(expected, actual, 1e-9));

  // Get expected derivatives
  Matrix E1 = numericalDerivative21<Vector2, RowMajorMatrix34, Vector4>(
      Projective(), P, X);
  Matrix E2 = numericalDerivative22<Vector2, RowMajorMatrix34, Vector4>(
      Projective(), P, X);

  // Get derivatives with AutoDiff
  Vector2 actual2;
  MatrixRowMajor H1(2, 12), H2(2, 4);
  double* parameters[] = {P.data(), X.data()};
  double* jacobians[] = {H1.data(), H2.data()};
  CHECK((AutoDiff<Projective, double, 12, 4>::Differentiate(
      projective, parameters, 2, actual2.data(), jacobians)));
  EXPECT(assert_equal(E1, H1, 1e-8));
  EXPECT(assert_equal(E2, H2, 1e-8));
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

/* ************************************************************************* */
namespace example {
Camera camera(Pose3(Rot3().retract(Vector3(0.1, 0.2, 0.3)), Point3(0, 5, 0)),
              Cal3Bundler0(1, 0, 0));
Point3 point(10, 0, -5);  // negative Z-axis convention of Snavely!
Vector9 P = Camera().localCoordinates(camera);
Vector3 X = point;
#ifdef GTSAM_POSE3_EXPMAP
Vector2 expectedMeasurement(1.3124675, 1.2057287);
#else
Vector2 expectedMeasurement(1.2431567, 1.2525694);
#endif
Matrix E1 = numericalDerivative21<Vector2, Vector9, Vector3>(adapted, P, X);
Matrix E2 = numericalDerivative22<Vector2, Vector9, Vector3>(adapted, P, X);
}

/* ************************************************************************* */
// Check that Local worked as expected
TEST(AdaptAutoDiff, Local) {
  using namespace example;
#ifdef GTSAM_POSE3_EXPMAP
  Vector9 expectedP = (Vector9() << 0.1, 0.2, 0.3, 0.7583528428, 4.9582357859, -0.224941471539, 1, 0, 0).finished();
#else
  Vector9 expectedP = (Vector9() << 0.1, 0.2, 0.3, 0, 5, 0, 1, 0, 0).finished();
#endif
  EXPECT(equal_with_abs_tol(expectedP, P));
  Vector3 expectedX(10, 0, -5);  // negative Z-axis convention of Snavely!
  EXPECT(equal_with_abs_tol(expectedX, X));
}

/* ************************************************************************* */
// Test Ceres AutoDiff
TEST(AdaptAutoDiff, AutoDiff2) {
  using namespace example;
  using ceres::internal::AutoDiff;

  // Apply the mapping, to get image point b_x.
  Vector2 actual = adapted(P, X);
  EXPECT(assert_equal(expectedMeasurement, actual, 1e-6));

  // Instantiate function
  SnavelyProjection snavely;

  // Get derivatives with AutoDiff
  Vector2 actual2;
  MatrixRowMajor H1(2, 9), H2(2, 3);
  double* parameters[] = {P.data(), X.data()};
  double* jacobians[] = {H1.data(), H2.data()};
  CHECK((AutoDiff<SnavelyProjection, double, 9, 3>::Differentiate(
      snavely, parameters, 2, actual2.data(), jacobians)));
  EXPECT(assert_equal(E1, H1, 1e-8));
  EXPECT(assert_equal(E2, H2, 1e-8));
}

/* ************************************************************************* */
// Test AutoDiff wrapper Snavely
TEST(AdaptAutoDiff, AdaptAutoDiff) {
  using namespace example;

  typedef AdaptAutoDiff<SnavelyProjection, 2, 9, 3> Adaptor;
  Adaptor snavely;

  // Apply the mapping, to get image point b_x.
  Vector2 actual = snavely(P, X);
  EXPECT(assert_equal(expectedMeasurement, actual, 1e-6));

  // Get derivatives with AutoDiff, not gives RowMajor results!
  Matrix29 H1;
  Matrix23 H2;
  Vector2 actual2 = snavely(P, X, H1, H2);
  EXPECT(assert_equal(expectedMeasurement, actual2, 1e-6));
  EXPECT(assert_equal(E1, H1, 1e-8));
  EXPECT(assert_equal(E2, H2, 1e-8));
}

/* ************************************************************************* */
// Test AutoDiff wrapper in an expression
TEST(AdaptAutoDiff, SnavelyExpression) {
  typedef AdaptAutoDiff<SnavelyProjection, 2, 9, 3> Adaptor;

  Expression<Vector9> P(1);
  Expression<Vector3> X(2);

  Expression<Vector2> expression(Adaptor(), P, X);

  std::size_t RecordSize =
    sizeof(internal::BinaryExpression<Vector2, Vector9, Vector3>::Record);

  EXPECT_LONGS_EQUAL(
    internal::upAligned(RecordSize) + P.traceSize() + X.traceSize(),
    expression.traceSize());

  set<Key> expected = list_of(1)(2);

  EXPECT(expected == expression.keys());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
