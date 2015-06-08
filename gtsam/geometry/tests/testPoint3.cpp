/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPoint3.cpp
 * @brief  Unit tests for Point3 class
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Point3)
GTSAM_CONCEPT_LIE_INST(Point3)

static Point3 P(0.2, 0.7, -2);

//******************************************************************************
TEST(Point3 , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<Point3>));
  BOOST_CONCEPT_ASSERT((IsManifold<Point3>));
  BOOST_CONCEPT_ASSERT((IsVectorSpace<Point3>));
}

//******************************************************************************
TEST(Point3 , Invariants) {
  Point3 p1(1, 2, 3), p2(4, 5, 6);
  EXPECT(check_group_invariants(p1, p2));
  EXPECT(check_manifold_invariants(p1, p2));
}

/* ************************************************************************* */
TEST(Point3, Lie) {
  Point3 p1(1, 2, 3);
  Point3 p2(4, 5, 6);
  Matrix H1, H2;

  EXPECT(assert_equal(Point3(5, 7, 9), traits<Point3>::Compose(p1, p2, H1, H2)));
  EXPECT(assert_equal(eye(3), H1));
  EXPECT(assert_equal(eye(3), H2));

  EXPECT(assert_equal(Point3(3, 3, 3), traits<Point3>::Between(p1, p2, H1, H2)));
  EXPECT(assert_equal(-eye(3), H1));
  EXPECT(assert_equal(eye(3), H2));

  EXPECT(assert_equal(Point3(5, 7, 9), traits<Point3>::Retract(p1, Vector3(4,5,6))));
  EXPECT(assert_equal(Vector3(3, 3, 3), traits<Point3>::Local(p1,p2)));
}

/* ************************************************************************* */
TEST( Point3, arithmetic) {
  CHECK(P * 3 == 3 * P);
  CHECK(assert_equal(Point3(-1, -5, -6), -Point3(1, 5, 6)));
  CHECK(assert_equal(Point3(2, 5, 6), Point3(1, 4, 5) + Point3(1, 1, 1)));
  CHECK(assert_equal(Point3(0, 3, 4), Point3(1, 4, 5) - Point3(1, 1, 1)));
  CHECK(assert_equal(Point3(2, 8, 6), Point3(1, 4, 3) * 2));
  CHECK(assert_equal(Point3(2, 2, 6), 2 * Point3(1, 1, 3)));
  CHECK(assert_equal(Point3(1, 2, 3), Point3(2, 4, 6) / 2));
}

/* ************************************************************************* */
TEST( Point3, equals) {
  CHECK(P.equals(P));
  Point3 Q;
  CHECK(!P.equals(Q));
}

/* ************************************************************************* */
TEST( Point3, dot) {
  Point3 origin, ones(1, 1, 1);
  CHECK(origin.dot(Point3(1, 1, 0)) == 0);
  CHECK(ones.dot(Point3(1, 1, 0)) == 2);
}

/* ************************************************************************* */
TEST( Point3, stream) {
  Point3 p(1, 2, -3);
  std::ostringstream os;
  os << p;
  EXPECT(os.str() == "[1, 2, -3]';");
}

//*************************************************************************
TEST (Point3, normalize) {
  Matrix actualH;
  Point3 point(1, -2, 3); // arbitrary point
  Point3 expected(point / sqrt(14.0));
  EXPECT(assert_equal(expected, point.normalize(actualH), 1e-8));
  Matrix expectedH = numericalDerivative11<Point3, Point3>(
      boost::bind(&Point3::normalize, _1, boost::none), point);
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

//*************************************************************************
double norm_proxy(const Point3& point) {
  return double(point.norm());
}

TEST (Point3, norm) {
  Matrix actualH;
  Point3 point(3,4,5); // arbitrary point
  double expected = sqrt(50);
  EXPECT_DOUBLES_EQUAL(expected, point.norm(actualH), 1e-8);
  Matrix expectedH = numericalDerivative11<double, Point3>(norm_proxy, point);
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

/* ************************************************************************* */
double testFunc(const Point3& P, const Point3& Q) {
  return P.distance(Q);
}

TEST (Point3, distance) {
  Point3 P(1., 12.8, -32.), Q(52.7, 4.9, -13.3);
  Matrix H1, H2;
  double d = P.distance(Q, H1, H2);
  double expectedDistance = 55.542686;
  Matrix numH1 = numericalDerivative21(testFunc, P, Q);
  Matrix numH2 = numericalDerivative22(testFunc, P, Q);
  DOUBLES_EQUAL(expectedDistance, d, 1e-5);
  EXPECT(assert_equal(numH1, H1, 1e-8));
  EXPECT(assert_equal(numH2, H2, 1e-8));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

