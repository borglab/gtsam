/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPoint1.cpp
 * @brief  Unit tests for Point1 class
 * @author Sven Lilge
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point1.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Point1)
GTSAM_CONCEPT_LIE_INST(Point1)

//******************************************************************************
TEST(Double, Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<double>);
  GTSAM_CONCEPT_ASSERT(IsManifold<double>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<double>);
}

//******************************************************************************
TEST(Double, Invariants) {
  double p1(2), p2(5);
  EXPECT(check_group_invariants(p1, p2));
  EXPECT(check_manifold_invariants(p1, p2));
}

//******************************************************************************
TEST(Point1, Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Point1>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Point1>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<Point1>);
}

//******************************************************************************
TEST(Point1, Invariants) {
  Point1 p1(1), p2(3);
  EXPECT(check_group_invariants(p1, p2));
  EXPECT(check_manifold_invariants(p1, p2));
}

/* ************************************************************************* */
TEST(Point1, constructor) {
  Point1 p1(1), p2 = p1;
  EXPECT(assert_equal(p1, p2));
}

/* ************************************************************************* */
TEST(Point1, equality) {
  Point1 p1(2), p2(3);
  EXPECT(!(p1 == p2));
}

/* ************************************************************************* */
TEST(Point1, Lie) {
  Point1 p1(1), p2(3);
  Matrix H1, H2;

  EXPECT(assert_equal(Point1(4), traits<Point1>::Compose(p1, p2, H1, H2)));
  EXPECT(assert_equal(I_1x1, H1));
  EXPECT(assert_equal(I_1x1, H2));

  EXPECT(assert_equal(Point1(2), traits<Point1>::Between(p1, p2, H1, H2)));
  EXPECT(assert_equal(-I_1x1, H1));
  EXPECT(assert_equal(I_1x1, H2));

  EXPECT(assert_equal(Point1(5), traits<Point1>::Retract(p1, Vector1(4))));
  EXPECT(assert_equal(Vector1(2), traits<Point1>::Local(p1, p2)));
}

/* ************************************************************************* */
TEST(Point1, expmap) {
  Vector d(1);
  d(0) = 1;
  Point1 a(4), b = traits<Point1>::Retract(a, d), c(5);
  EXPECT(assert_equal(b, c));
}

/* ************************************************************************* */
TEST(Point1, arithmetic) {
  EXPECT(assert_equal<Point1>(Point1(-5), -Point1(5)));
  EXPECT(assert_equal<Point1>(Point1(5), Point1(4) + Point1(1)));
  EXPECT(assert_equal<Point1>(Point1(3), Point1(4) - Point1(1)));
  EXPECT(assert_equal<Point1>(Point1(8), Point1(4) * 2));
  EXPECT(assert_equal<Point1>(Point1(4), 2.0 * Point1(2)));
  EXPECT(assert_equal<Point1>(Point1(2), Point1(4) / 2));
}

/* ************************************************************************* */
TEST(Point1, unit) {
  Point1 p0(10), p1(0);
  EXPECT(assert_equal(Point1(1), Point1(p0.normalized()), 1e-6));
  EXPECT(assert_equal(Point1(0), Point1(p1.normalized()), 1e-6));
}

namespace {
/* ************************************************************************* */
// some shared test values
Point1 x1(0), x2(2);
Point1 l1(0), l2(1);

/* ************************************************************************* */
double norm_proxy(const Point1& point) { return point.norm(); }
}  // namespace
TEST(Point1, norm) {
  Point1 p0(10), p1(-10);
  DOUBLES_EQUAL(10, p0.norm(), 1e-6);
  DOUBLES_EQUAL(10, p1.norm(), 1e-6);

  Matrix expectedH, actualH;
  double actual;

  // exception, for (0,0) derivative is [Inf,Inf] but we return [1,1]
  actual = norm1(x1, actualH);
  EXPECT_DOUBLES_EQUAL(0, actual, 1e-9);
  expectedH = (Matrix(1, 1) << 1.0).finished();
  EXPECT(assert_equal(expectedH, actualH));

  actual = norm1(x2, actualH);
  EXPECT_DOUBLES_EQUAL(2, actual, 1e-9);
  expectedH = numericalDerivative11(norm_proxy, x2);
  EXPECT(assert_equal(expectedH, actualH));

  // analytical
  expectedH = (Matrix(1, 1) << (x2.normalized())).finished();
  EXPECT(assert_equal(expectedH, actualH));
}

/* ************************************************************************* */
namespace {
double distance_proxy(const Point1& location, const Point1& point) {
  return distance1(location, point);
}
}  // namespace
TEST(Point1, distance) {
  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish distance is indeed zero
  EXPECT_DOUBLES_EQUAL(1, distance1(x1, l2), 1e-9);

  // Another pair
  double actual21 = distance1(x2, l1, actualH1, actualH2);
  EXPECT_DOUBLES_EQUAL(2, actual21, 1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(distance_proxy, x2, l1);
  expectedH2 = numericalDerivative22(distance_proxy, x2, l1);
  EXPECT(assert_equal(expectedH1, actualH1));
  EXPECT(assert_equal(expectedH2, actualH2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
