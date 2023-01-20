/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testLie.cpp
 * @date May, 2015
 * @author Frank Dellaert
 * @brief unit tests for Lie group type machinery
 */

#include <gtsam/base/ProductLieGroup.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/testLie.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static const double tol = 1e-9;

//******************************************************************************
typedef ProductLieGroup<Point2, Pose2> Product;

// Define any direct product group to be a model of the multiplicative Group concept
namespace gtsam {
template<> struct traits<Product> : internal::LieGroupTraits<Product> {
  static void Print(const Product& m, const string& s = "") {
    cout << s << "(" << m.first << "," << m.second.translation() << "/"
        << m.second.theta() << ")" << endl;
  }
  static bool Equals(const Product& m1, const Product& m2, double tol = 1e-8) {
    return traits<Point2>::Equals(m1.first, m2.first, tol) && m1.second.equals(m2.second, tol);
  }
};
}

//******************************************************************************
TEST(Lie, ProductLieGroup) {
  BOOST_CONCEPT_ASSERT((IsGroup<Product>));
  BOOST_CONCEPT_ASSERT((IsManifold<Product>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<Product>));
  Product pair1;
  Vector5 d;
  d << 1, 2, 0.1, 0.2, 0.3;
  Product expected(Point2(1, 2), Pose2::Expmap(Vector3(0.1, 0.2, 0.3)));
  Product pair2 = pair1.expmap(d);
  EXPECT(assert_equal(expected, pair2, 1e-9));
  EXPECT(assert_equal(d, pair1.logmap(pair2), 1e-9));
}

/* ************************************************************************* */
Product compose_proxy(const Product& A, const Product& B) {
  return A.compose(B);
}
TEST( testProduct, compose ) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5)), state2 = state1;

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(compose_proxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(compose_proxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, tol));
  EXPECT(assert_equal(numericH2, actH2, tol));
}

/* ************************************************************************* */
Product between_proxy(const Product& A, const Product& B) {
  return A.between(B);
}
TEST( testProduct, between ) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5)), state2 = state1;

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(between_proxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(between_proxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, tol));
  EXPECT(assert_equal(numericH2, actH2, tol));
}

/* ************************************************************************* */
Product inverse_proxy(const Product& A) {
  return A.inverse();
}
TEST( testProduct, inverse ) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5));

  Matrix actH1;
  state1.inverse(actH1);
  Matrix numericH1 = numericalDerivative11(inverse_proxy, state1);
  EXPECT(assert_equal(numericH1, actH1, tol));
}

/* ************************************************************************* */
Product expmap_proxy(const Vector5& vec) {
  return Product::Expmap(vec);
}
TEST( testProduct, Expmap ) {
  Vector5 vec;
  vec << 1, 2, 0.1, 0.2, 0.3;

  Matrix actH;
  Product::Expmap(vec, actH);
  Matrix numericH = numericalDerivative11(expmap_proxy, vec);
  EXPECT(assert_equal(numericH, actH, tol));
}

/* ************************************************************************* */
Vector5 logmap_proxy(const Product& p) {
  return Product::Logmap(p);
}
TEST( testProduct, Logmap ) {
  Product state(Point2(1, 2), Pose2(3, 4, 5));

  Matrix actH;
  Product::Logmap(state, actH);
  Matrix numericH = numericalDerivative11(logmap_proxy, state);
  EXPECT(assert_equal(numericH, actH, tol));
}

/* ************************************************************************* */
Product interpolate_proxy(const Product& x, const Product& y, double t) {
  return interpolate<Product>(x, y, t);
}

TEST(Lie, Interpolate) {
  Product x(Point2(1, 2), Pose2(3, 4, 5));
  Product y(Point2(6, 7), Pose2(8, 9, 0));

  double t;
  Matrix actH1, numericH1, actH2, numericH2;

  t = 0.0;
  interpolate<Product>(x, y, t, actH1, actH2);
  numericH1 = numericalDerivative31<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH1, actH1, tol));
  numericH2 = numericalDerivative32<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH2, actH2, tol));

  t = 0.5;
  interpolate<Product>(x, y, t, actH1, actH2);
  numericH1 = numericalDerivative31<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH1, actH1, tol));
  numericH2 = numericalDerivative32<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH2, actH2, tol));

  t = 1.0;
  interpolate<Product>(x, y, t, actH1, actH2);
  numericH1 = numericalDerivative31<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH1, actH1, tol));
  numericH2 = numericalDerivative32<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH2, actH2, tol));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

