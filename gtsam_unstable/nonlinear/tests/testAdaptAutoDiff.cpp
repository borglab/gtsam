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

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam_unstable/nonlinear/Expression.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/LieScalar.h>

#include <gtsam_unstable/nonlinear/ceres_autodiff.h>
#include <gtsam_unstable/nonlinear/ceres_example.h>

#undef CHECK
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

using namespace std;
using namespace gtsam;

// The DefaultChart of Camera below is laid out like Snavely's 9-dim vector
typedef PinholeCamera<Cal3Bundler> Camera;

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
TEST(Expression, AutoDiff) {
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

TEST(Expression, AutoDiff2) {
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
// Adapt ceres-style autodiff
template<typename F, typename T, typename A1, typename A2>
class AdaptAutoDiff {

  static const int N = traits::dimension<T>::value;
  static const int M1 = traits::dimension<A1>::value;
  static const int M2 = traits::dimension<A2>::value;

  typedef Eigen::Matrix<double, N, M1, Eigen::RowMajor> RowMajor1;
  typedef Eigen::Matrix<double, N, M2, Eigen::RowMajor> RowMajor2;

  typedef Canonical<T> CanonicalT;
  typedef Canonical<A1> Canonical1;
  typedef Canonical<A2> Canonical2;

  typedef typename CanonicalT::vector VectorT;
  typedef typename Canonical1::vector Vector1;
  typedef typename Canonical2::vector Vector2;

  // Instantiate function and charts
  CanonicalT chartT;
  Canonical1 chart1;
  Canonical2 chart2;
  F f;

public:

  typedef Eigen::Matrix<double, N, M1> JacobianTA1;
  typedef Eigen::Matrix<double, N, M2> JacobianTA2;

  T operator()(const A1& a1, const A2& a2, boost::optional<JacobianTA1&> H1 =
      boost::none, boost::optional<JacobianTA2&> H2 = boost::none) {

    using ceres::internal::AutoDiff;

    // Make arguments
    Vector1 v1 = chart1.apply(a1);
    Vector2 v2 = chart2.apply(a2);

    bool success;
    VectorT result;

    if (H1 || H2) {

      // Get derivatives with AutoDiff
      double *parameters[] = { v1.data(), v2.data() };
      double rowMajor1[N * M1], rowMajor2[N * M2]; // om the stack
      double *jacobians[] = { rowMajor1, rowMajor2 };
      success = AutoDiff<F, double, 9, 3>::Differentiate(f, parameters, 2,
          result.data(), jacobians);

      // Convert from row-major to columnn-major
      // TODO: if this is a bottleneck (probably not!) fix Autodiff to be Column-Major
      *H1 = Eigen::Map<RowMajor1>(rowMajor1);
      *H2 = Eigen::Map<RowMajor2>(rowMajor2);

    } else {
      // Apply the mapping, to get result
      success = f(v1.data(), v2.data(), result.data());
    }
    if (!success)
      throw std::runtime_error(
          "AdaptAutoDiff: function call resulted in failure");
    return chartT.retract(result);
  }

};

/* ************************************************************************* */
// Test AutoDiff wrapper Snavely
TEST(Expression, AutoDiff3) {

  // Make arguments
  Camera P(Pose3(Rot3(), Point3(0, 5, 0)), Cal3Bundler(1, 0, 0));
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
  EXPECT(assert_equal(expected,actual,1e-9));
  EXPECT(assert_equal(E1,H1,1e-8));
  EXPECT(assert_equal(E2,H2,1e-8));
}

/* ************************************************************************* */
// Test AutoDiff wrapper in an expression
TEST(Expression, Snavely) {
  Expression<Camera> P(1);
  Expression<Point3> X(2);
  typedef AdaptAutoDiff<SnavelyProjection, Point2, Camera, Point3> Adaptor;
  Expression<Point2> expression(Adaptor(), P, X);
  set<Key> expected = list_of(1)(2);
  EXPECT(expected == expression.keys());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

