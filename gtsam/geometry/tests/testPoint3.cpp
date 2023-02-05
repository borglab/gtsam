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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>

using namespace std::placeholders;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Point3)
GTSAM_CONCEPT_LIE_INST(Point3)

static Point3 P(0.2, 0.7, -2);

//******************************************************************************
TEST(Point3 , Constructor) {
  Point3 p;
}

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
  EXPECT(assert_equal(I_3x3, H1));
  EXPECT(assert_equal(I_3x3, H2));

  EXPECT(assert_equal(Point3(3, 3, 3), traits<Point3>::Between(p1, p2, H1, H2)));
  EXPECT(assert_equal(-I_3x3, H1));
  EXPECT(assert_equal(I_3x3, H2));

  EXPECT(assert_equal(Point3(5, 7, 9), traits<Point3>::Retract(p1, Vector3(4,5,6))));
  EXPECT(assert_equal(Vector3(3, 3, 3), traits<Point3>::Local(p1,p2)));
}

/* ************************************************************************* */
TEST( Point3, arithmetic) {
  CHECK(P * 3 == 3 * P);
  CHECK(assert_equal<Point3>(Point3(-1, -5, -6), -Point3(1, 5, 6)));
  CHECK(assert_equal<Point3>(Point3(2, 5, 6), Point3(1, 4, 5) + Point3(1, 1, 1)));
  CHECK(assert_equal<Point3>(Point3(0, 3, 4), Point3(1, 4, 5) - Point3(1, 1, 1)));
  CHECK(assert_equal<Point3>(Point3(2, 8, 6), Point3(1, 4, 3) * 2));
  CHECK(assert_equal<Point3>(Point3(2, 2, 6), 2 * Point3(1, 1, 3)));
  CHECK(assert_equal<Point3>(Point3(1, 2, 3), Point3(2, 4, 6) / 2));
}

/* ************************************************************************* */
TEST( Point3, equals) {
  CHECK(traits<Point3>::Equals(P,P));
  Point3 Q(0,0,0);
  CHECK(!traits<Point3>::Equals(P,Q));
}

/* ************************************************************************* */
TEST( Point3, dot) {
  Point3 origin(0,0,0), ones(1, 1, 1);
  CHECK(origin.dot(Point3(1, 1, 0)) == 0);
  CHECK(ones.dot(Point3(1, 1, 0)) == 2);

  Point3 p(1, 0.2, 0.3);
  Point3 q = p + Point3(0.5, 0.2, -3.0);
  Point3 r = p + Point3(0.8, 0, 0);
  Point3 t = p + Point3(0, 0.3, -0.4);
  EXPECT(assert_equal(1.130000, p.dot(p), 1e-8));
  EXPECT(assert_equal(0.770000, p.dot(q), 1e-5));
  EXPECT(assert_equal(1.930000, p.dot(r), 1e-5));
  EXPECT(assert_equal(1.070000, p.dot(t), 1e-5));

  // Use numerical derivatives to calculate the expected Jacobians
  Matrix H1, H2;
  std::function<double(const Point3&, const Point3&)> f =
      [](const Point3& p, const Point3& q) { return gtsam::dot(p, q); };
  {
    gtsam::dot(p, q, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<double,Point3>(f, p, q), H1, 1e-9));
    EXPECT(assert_equal(numericalDerivative22<double,Point3>(f, p, q), H2, 1e-9));
  }
  {
    gtsam::dot(p, r, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<double,Point3>(f, p, r), H1, 1e-9));
    EXPECT(assert_equal(numericalDerivative22<double,Point3>(f, p, r), H2, 1e-9));
  }
  {
    gtsam::dot(p, t, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<double,Point3>(f, p, t), H1, 1e-9));
    EXPECT(assert_equal(numericalDerivative22<double,Point3>(f, p, t), H2, 1e-9));
  }
}

/* ************************************************************************* */
TEST(Point3, cross) {
  Matrix aH1, aH2;
  std::function<Point3(const Point3&, const Point3&)> f = 
    [](const Point3& p, const Point3& q) { return gtsam::cross(p, q); };
  const Point3 omega(0, 1, 0), theta(4, 6, 8);
  cross(omega, theta, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(f, omega, theta), aH1));
  EXPECT(assert_equal(numericalDerivative22(f, omega, theta), aH2));
}

/* ************************************************************************* */
TEST( Point3, cross2) {
  Point3 p(1, 0.2, 0.3);
  Point3 q = p + Point3(0.5, 0.2, -3.0);
  Point3 r = p + Point3(0.8, 0, 0);
  EXPECT(assert_equal(Point3(0, 0, 0), p.cross(p), 1e-8));
  EXPECT(assert_equal(Point3(-0.66, 3.15, 0.1), p.cross(q), 1e-5));
  EXPECT(assert_equal(Point3(0, 0.24, -0.16), p.cross(r), 1e-5));

  // Use numerical derivatives to calculate the expected Jacobians
  Matrix H1, H2;
  std::function<Point3(const Point3&, const Point3&)> f =
    [](const Point3& p, const Point3& q) { return gtsam::cross(p, q); };
  {
    gtsam::cross(p, q, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<Point3,Point3>(f, p, q), H1, 1e-9));
    EXPECT(assert_equal(numericalDerivative22<Point3,Point3>(f, p, q), H2, 1e-9));
  }
  {
    gtsam::cross(p, r, H1, H2);
    EXPECT(assert_equal(numericalDerivative21<Point3,Point3>(f, p, r), H1, 1e-9));
    EXPECT(assert_equal(numericalDerivative22<Point3,Point3>(f, p, r), H2, 1e-9));
  }
}

//*************************************************************************
TEST (Point3, normalize) {
  Matrix actualH;
  Point3 point(1, -2, 3); // arbitrary point
  Point3 expected(point / sqrt(14.0));
  EXPECT(assert_equal(expected, normalize(point, actualH), 1e-8));
  std::function<Point3(const Point3&)> fn = [](const Point3& p) { return normalize(p); };
  Matrix expectedH = numericalDerivative11<Point3, Point3>(fn, point);
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

//*************************************************************************
TEST(Point3, mean) {
  Point3 expected(2, 2, 2);
  Point3 a1(0, 0, 0), a2(1, 2, 3), a3(5, 4, 3);
  std::vector<Point3> a_points{a1, a2, a3};
  Point3 actual = mean(a_points);
  EXPECT(assert_equal(expected, actual));
}

TEST(Point3, mean_pair) {
  Point3 a_mean(2, 2, 2), b_mean(-1, 1, 0);
  Point3Pair expected = {a_mean, b_mean};
  Point3 a1(0, 0, 0), a2(1, 2, 3), a3(5, 4, 3);
  Point3 b1(-1, 0, 0), b2(-2, 4, 0), b3(0, -1, 0);
  std::vector<Point3Pair> point_pairs{{a1, b1}, {a2, b2}, {a3, b3}};
  Point3Pair actual = means(point_pairs);
  EXPECT(assert_equal(expected.first, actual.first));
  EXPECT(assert_equal(expected.second, actual.second));
}

//*************************************************************************
double norm_proxy(const Point3& point) {
  return double(point.norm());
}

TEST (Point3, norm) {
  Matrix actualH;
  Point3 point(3,4,5); // arbitrary point
  double expected = sqrt(50);
  EXPECT_DOUBLES_EQUAL(expected, norm3(point, actualH), 1e-8);
  Matrix expectedH = numericalDerivative11<double, Point3>(norm_proxy, point);
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

/* ************************************************************************* */
double testFunc(const Point3& P, const Point3& Q) {
  return distance3(P, Q);
}

TEST (Point3, distance) {
  Point3 P(1., 12.8, -32.), Q(52.7, 4.9, -13.3);
  Matrix H1, H2;
  double d = distance3(P, Q, H1, H2);
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

