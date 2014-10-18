/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

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
#include <gtsam_unstable/nonlinear/Expression.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/LieScalar.h>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#undef CHECK
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
template<class CAL>
Point2 uncalibrate(const CAL& K, const Point2& p,
    boost::optional<Matrix25&> Dcal, boost::optional<Matrix2&> Dp) {
  return K.uncalibrate(p, Dcal, Dp);
}

static const Rot3 someR = Rot3::RzRyRx(1, 2, 3);

/* ************************************************************************* */
// Constant
TEST(Expression, constant) {
  Expression<Rot3> R(someR);
  Values values;
  JacobianMap actualMap;
  Rot3 actual = R.value(values, actualMap);
  EXPECT(assert_equal(someR, actual));
  JacobianMap expected;
  EXPECT(actualMap == expected);
  EXPECT_LONGS_EQUAL(0, R.traceSize())
}

/* ************************************************************************* */
// Leaf
TEST(Expression, Leaf) {
  Expression<Rot3> R(100);
  Values values;
  values.insert(100, someR);

  JacobianMap expected;
  Matrix H = eye(3);
  expected.insert(make_pair(100, H.block(0, 0, 3, 3)));

  JacobianMap actualMap2;
  actualMap2.insert(make_pair(100, H.block(0, 0, 3, 3)));
  Rot3 actual2 = R.reverse(values, actualMap2);
  EXPECT(assert_equal(someR, actual2));
  EXPECT(actualMap2 == expected);
}

/* ************************************************************************* */
// Many Leaves
TEST(Expression, Leaves) {
  Values values;
  Point3 somePoint(1, 2, 3);
  values.insert(Symbol('p', 10), somePoint);
  std::vector<Expression<Point3> > points = createUnknowns<Point3>(10, 'p', 1);
  EXPECT(assert_equal(somePoint,points.back().value(values)));
}

/* ************************************************************************* */

//TEST(Expression, NullaryMethod) {
//  Expression<Point3> p(67);
//  Expression<LieScalar> norm(p, &Point3::norm);
//  Values values;
//  values.insert(67,Point3(3,4,5));
//  Augmented<LieScalar> a = norm.augmented(values);
//  EXPECT(a.value() == sqrt(50));
//  JacobianMap expected;
//  expected[67] = (Matrix(1,3) << 3/sqrt(50),4/sqrt(50),5/sqrt(50));
//  EXPECT(assert_equal(expected.at(67),a.jacobians().at(67)));
//}
/* ************************************************************************* */
// Binary(Leaf,Leaf)
namespace binary {
// Create leaves
Expression<Pose3> x(1);
Expression<Point3> p(2);
Expression<Point3> p_cam(x, &Pose3::transform_to, p);
}
/* ************************************************************************* */
// keys
TEST(Expression, BinaryKeys) {
  set<Key> expected = list_of(1)(2);
  EXPECT(expected == binary::p_cam.keys());
}
/* ************************************************************************* */
// dimensions
TEST(Expression, BinaryDimensions) {
  map<Key, size_t> actual, expected = map_list_of<Key, size_t>(1, 6)(2, 3);
  binary::p_cam.dims(actual);
  EXPECT(actual==expected);
}
/* ************************************************************************* */
// dimensions
TEST(Expression, BinaryTraceSize) {
  typedef BinaryExpression<Point3, Pose3, Point3> Binary;
  size_t expectedTraceSize = sizeof(Binary::Record);
  EXPECT_LONGS_EQUAL(expectedTraceSize, binary::p_cam.traceSize());
}
/* ************************************************************************* */
// Binary(Leaf,Unary(Binary(Leaf,Leaf)))
namespace tree {
using namespace binary;
// Create leaves
Expression<Cal3_S2> K(3);

// Create expression tree
Expression<Point2> projection(PinholeCamera<Cal3_S2>::project_to_camera, p_cam);
Expression<Point2> uv_hat(uncalibrate<Cal3_S2>, K, projection);
}
/* ************************************************************************* */
// keys
TEST(Expression, TreeKeys) {
  set<Key> expected = list_of(1)(2)(3);
  EXPECT(expected == tree::uv_hat.keys());
}
/* ************************************************************************* */
// dimensions
TEST(Expression, TreeDimensions) {
  map<Key, size_t> actual, expected = map_list_of<Key, size_t>(1, 6)(2, 3)(3,
      5);
  tree::uv_hat.dims(actual);
  EXPECT(actual==expected);
}
/* ************************************************************************* */
// TraceSize
TEST(Expression, TreeTraceSize) {
  typedef UnaryExpression<Point2, Point3> Unary;
  typedef BinaryExpression<Point3, Pose3, Point3> Binary1;
  typedef BinaryExpression<Point2, Point2, Cal3_S2> Binary2;
  size_t expectedTraceSize = sizeof(Unary::Record) + sizeof(Binary1::Record)
      + sizeof(Binary2::Record);
  EXPECT_LONGS_EQUAL(expectedTraceSize, tree::uv_hat.traceSize());
}
/* ************************************************************************* */

TEST(Expression, compose1) {

  // Create expression
  Expression<Rot3> R1(1), R2(2);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  set<Key> expected = list_of(1)(2);
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test compose with arguments referring to the same rotation
TEST(Expression, compose2) {

  // Create expression
  Expression<Rot3> R1(1), R2(1);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  set<Key> expected = list_of(1);
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test compose with one arguments referring to constant rotation
TEST(Expression, compose3) {

  // Create expression
  Expression<Rot3> R1(Rot3::identity()), R2(3);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  set<Key> expected = list_of(3);
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test with ternary function
Rot3 composeThree(const Rot3& R1, const Rot3& R2, const Rot3& R3,
    boost::optional<Matrix3&> H1, boost::optional<Matrix3&> H2,
    boost::optional<Matrix3&> H3) {
  // return dummy derivatives (not correct, but that's ok for testing here)
  if (H1)
    *H1 = eye(3);
  if (H2)
    *H2 = eye(3);
  if (H3)
    *H3 = eye(3);
  return R1 * (R2 * R3);
}

TEST(Expression, ternary) {

  // Create expression
  Expression<Rot3> A(1), B(2), C(3);
  Expression<Rot3> ABC(composeThree, A, B, C);

  // Check keys
  set<Key> expected = list_of(1)(2)(3);
  EXPECT(expected == ABC.keys());
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

// Templated pinhole camera model for used with Ceres. The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {

  template<typename T>
  bool operator()(const T* const camera, const T* const point,
      T* predicted) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp * xp + yp * yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2);

    // Compute final projected point position.
    const T& focal = camera[6];
    predicted[0] = focal * distortion * xp;
    predicted[1] = focal * distortion * yp;

    return true;
  }

  // Adapt to GTSAM types
  Vector2 operator()(const Vector9& P, const Vector3& X) const {
    Vector2 x;
    if (operator()(P.data(), X.data(), x.data()))
      return x;
    else
      throw std::runtime_error("Snavely fail");
  }

};

/* ************************************************************************* */
// manifold_traits prototype
// Same style as Boost.TypeTraits
// All meta-functions below ever only declare a single type
// or a type/value/value_type
#include <boost/static_assert.hpp>
#include <type_traits>

// is manifold
template<typename T>
struct is_manifold: public false_type {
};

// dimension
template<typename T>
struct dimension: public integral_constant<size_t, T::dimension> {
};

// Fixed size Eigen::Matrix type
template<int M, int N, int Options>
struct is_manifold<Eigen::Matrix<double, M, N, Options> > : public true_type {
};

template<int M, int N, int Options>
struct dimension<Eigen::Matrix<double, M, N, Options> > : public integral_constant<
    size_t, M * N> {
  BOOST_STATIC_ASSERT(M!=Eigen::Dynamic && N!=Eigen::Dynamic);
};

template<typename T>
struct manifold_traits {
  typedef T type;
  static const size_t dim = dimension<T>::value;
  typedef Eigen::Matrix<double, dim, 1> tangent;
  static tangent localCoordinates(const T& t1, const T& t2) {
    return t1.localCoordinates(t2);
  }
  static type retract(const type& t, const tangent& d) {
    return t.retract(d);
  }
};

// Adapt constant size Eigen::Matrix types as manifold types
template<int M, int N, int Options>
struct manifold_traits<Eigen::Matrix<double, M, N, Options> > {
  BOOST_STATIC_ASSERT(M!=Eigen::Dynamic && N!=Eigen::Dynamic);
  typedef Eigen::Matrix<double, M, N, Options> type;
  static const size_t dim = dimension<type>::value;
  typedef Eigen::Matrix<double, dim, 1> tangent;
  static tangent localCoordinates(const type& t1, const type& t2) {
    type diff = t2 - t1;
    return tangent(Eigen::Map<tangent>(diff.data()));
  }
  static type retract(const type& t, const tangent& d) {
    type sum = t + Eigen::Map<const type>(d.data());
    return sum;
  }
};

// Test dimension traits
TEST(Expression, Traits) {
  EXPECT_LONGS_EQUAL(2, dimension<Point2>::value);
  EXPECT_LONGS_EQUAL(8, dimension<Matrix24>::value);
}

/* ************************************************************************* */
// New-style numerical derivatives using manifold_traits
template<typename Y, typename X>
Matrix numericalDerivative(boost::function<Y(const X&)> h, const X& x,
    double delta = 1e-5) {
  BOOST_STATIC_ASSERT(is_manifold<Y>::value);
  BOOST_STATIC_ASSERT(is_manifold<X>::value);
  Y hx = h(x);
  double factor = 1.0 / (2.0 * delta);
  static const size_t M = dimension<Y>::value;
  static const size_t N = dimension<X>::value;
  Eigen::Matrix<double, N, 1> d;
  Matrix H = zeros(M, N);
  for (size_t j = 0; j < N; j++) {
    d.setZero();
    d(j) = delta;
    Vector hxplus = manifold_traits<Y>::localCoordinates(hx,
        h(manifold_traits<X>::retract(x, d)));
    d(j) = -delta;
    Vector hxmin = manifold_traits<Y>::localCoordinates(hx,
        h(manifold_traits<X>::retract(x, d)));
    H.block<M, 1>(0, j) << (hxplus - hxmin) * factor;
  }
  return H;
}

template<typename Y, typename X1, typename X2>
Matrix numericalDerivative21(boost::function<Y(const X1&, const X2&)> h,
    const X1& x1, const X2& x2, double delta = 1e-5) {
  return numericalDerivative<Y, X1>(boost::bind(h, _1, x2), x1, delta);
}

template<typename Y, typename X1, typename X2>
Matrix numericalDerivative22(boost::function<Y(const X1&, const X2&)> h,
    const X1& x1, const X2& x2, double delta = 1e-5) {
  return numericalDerivative<Y, X2>(boost::bind(h, x1, _1), x2, delta);
}

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
// Test Ceres AutoDiff on Snavely
TEST(Expression, AutoDiff2) {
  using ceres::internal::AutoDiff;

  // Instantiate function
  SnavelyReprojectionError snavely;

  // Make arguments
  Vector9 P; // zero rotation, (0,5,0) translation, focal length 1
  P << 0, 0, 0, 0, 5, 0, 1, 0, 0;
  Vector3 X(10, 0, -5); // negative Z-axis convention of Snavely!

  // Apply the mapping, to get image point b_x.
  Vector expected = Vector2(2, 1);
  Vector2 actual = snavely(P, X);
  EXPECT(assert_equal(expected,actual,1e-9));

  // Get expected derivatives
  Matrix E1 = numericalDerivative21<Vector2, Vector9, Vector3>(
      SnavelyReprojectionError(), P, X);
  Matrix E2 = numericalDerivative22<Vector2, Vector9, Vector3>(
      SnavelyReprojectionError(), P, X);

  // Get derivatives with AutoDiff
  Vector2 actual2;
  MatrixRowMajor H1(2, 9), H2(2, 3);
  double *parameters[] = { P.data(), X.data() };
  double *jacobians[] = { H1.data(), H2.data() };
  CHECK(
      (AutoDiff<SnavelyReprojectionError, double, 9, 3>::Differentiate( snavely, parameters, 2, actual2.data(), jacobians)));
  EXPECT(assert_equal(E1,H1,1e-8));
  EXPECT(assert_equal(E2,H2,1e-8));
}

/* ************************************************************************* */
// keys
TEST(Expression, SnavelyKeys) {
//  Expression<Vector2> expression(1);
//  set<Key> expected = list_of(1)(2);
//  EXPECT(expected == expression.keys());
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

