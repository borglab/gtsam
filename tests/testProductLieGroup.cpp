/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1-------------------------------------------
 */

/**
 * @file testLie.cpp
 * @date May, 2015
 * @author Frank Dellaert
 * @brief unit tests for Lie group type machinery
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include <iostream>

using namespace gtsam;

constexpr double kTol = 1e-9;

using Product = ProductLieGroup<Point2, Pose2>;
constexpr size_t kPowerComponents = 2;
using Power = PowerLieGroup<Pose2, kPowerComponents>;
using PowerTangent = Power::TangentVector;

namespace gtsam {
template <>
struct traits<Product> : internal::LieGroupTraits<Product> {
  static void Print(const Product& m, const std::string& s = "") {
    std::cout << s << "(" << m.first << "," << m.second.translation() << "/"
              << m.second.theta() << ")" << std::endl;
  }
  static bool Equals(const Product& m1, const Product& m2, double tol = 1e-8) {
    return traits<Point2>::Equals(m1.first, m2.first, tol) &&
           m1.second.equals(m2.second, tol);
  }
};

template <>
struct traits<Power> : internal::LieGroupTraits<Power> {
  static void Print(const Power& m, const std::string& s = "") {
    std::cout << s << "[";
    for (size_t i = 0; i < kPowerComponents; ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << "Pose2(" << m[i].x() << "," << m[i].y() << ","
                << m[i].theta() << ")";
    }
    std::cout << "]" << std::endl;
  }
  static bool Equals(const Power& m1, const Power& m2, double tol = 1e-8) {
    for (size_t i = 0; i < kPowerComponents; ++i) {
      if (!m1[i].equals(m2[i], tol)) return false;
    }
    return true;
  }
};
}  // namespace gtsam

namespace {
Product composeProductProxy(const Product& A, const Product& B) {
  return A.compose(B);
}

Product betweenProductProxy(const Product& A, const Product& B) {
  return A.between(B);
}

Product inverseProductProxy(const Product& A) { return A.inverse(); }

Product expmapProductProxy(const Vector5& vec) { return Product::Expmap(vec); }

Vector5 logmapProductProxy(const Product& p) { return Product::Logmap(p); }

Power composePowerProxy(const Power& A, const Power& B) { return A.compose(B); }

Power betweenPowerProxy(const Power& A, const Power& B) { return A.between(B); }

Power inversePowerProxy(const Power& A) { return A.inverse(); }

Power powerExpmapProxy(const PowerTangent& vec) { return Power::Expmap(vec); }

PowerTangent powerLogmapProxy(const Power& p) { return Power::Logmap(p); }
}  // namespace

/* ************************************************************************* */
TEST(Lie, ProductLieGroup) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Product>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Product>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Product>);
  Product pair1;
  Vector5 d;
  d << 1, 2, 0.1, 0.2, 0.3;
  Product expected(Point2(1, 2), Pose2::Expmap(Vector3(0.1, 0.2, 0.3)));
  Product pair2 = pair1.expmap(d);
  EXPECT(assert_equal(expected, pair2, kTol));
  EXPECT(assert_equal(d, pair1.logmap(pair2), kTol));
  const auto adj = pair1.AdjointMap();
  EXPECT_LONGS_EQUAL(5, adj.rows());
  EXPECT_LONGS_EQUAL(5, adj.cols());
}

/* ************************************************************************* */
TEST(testProduct, compose) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5)), state2 = state1;

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(composeProductProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(composeProductProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testProduct, between) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5)), state2 = state1;

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(betweenProductProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(betweenProductProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testProduct, inverse) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5));

  Matrix actH1;
  state1.inverse(actH1);
  Matrix numericH1 = numericalDerivative11(inverseProductProxy, state1);
  EXPECT(assert_equal(numericH1, actH1, kTol));
}

/* ************************************************************************* */
TEST(testProduct, Expmap) {
  Vector5 vec;
  vec << 1, 2, 0.1, 0.2, 0.3;

  Matrix actH;
  Product::Expmap(vec, actH);
  Matrix numericH = numericalDerivative11(expmapProductProxy, vec);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testProduct, Logmap) {
  Product state(Point2(1, 2), Pose2(3, 4, 5));

  Matrix actH;
  Product::Logmap(state, actH);
  Matrix numericH = numericalDerivative11(logmapProductProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testProduct, AdjointMap) {
  Product state(Point2(1, 2), Pose2(3, 4, 5));
  const Matrix actual = state.AdjointMap();

  Matrix expected = Matrix::Zero(5, 5);
  expected.topLeftCorner<2, 2>() = Matrix2::Identity();
  expected.bottomRightCorner<3, 3>() = state.second.AdjointMap();

  EXPECT(assert_equal(expected, actual, kTol));
}

/* ************************************************************************* */
Product interpolate_proxy(const Product& x, const Product& y, double t) {
  return interpolate<Product>(x, y, t);
}

TEST(Lie, Interpolate) {
  Product x(Point2(1, 2), Pose2(3, 4, 5));
  Product y(Point2(6, 7), Pose2(8, 9, 0));

  double t;
  Matrix actH1, numericH1, actH2, numericH2, actH3, numericH3;

  t = 0.0;
  interpolate<Product>(x, y, t, actH1, actH2, actH3);
  numericH1 = numericalDerivative31<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  numericH2 = numericalDerivative32<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH2, actH2, kTol));
  numericH3 = numericalDerivative33<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH3, actH3, kTol));

  t = 0.5;
  interpolate<Product>(x, y, t, actH1, actH2);
  numericH1 = numericalDerivative31<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  numericH2 = numericalDerivative32<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH2, actH2, kTol));
  numericH3 = numericalDerivative33<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH3, actH3, kTol));

  t = 1.0;
  interpolate<Product>(x, y, t, actH1, actH2);
  numericH1 = numericalDerivative31<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  numericH2 = numericalDerivative32<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH2, actH2, kTol));
  numericH3 = numericalDerivative33<Product, Product, Product, double>(
      interpolate_proxy, x, y, t);
  EXPECT(assert_equal(numericH3, actH3, kTol));
}

/* ************************************************************************* */
TEST(Lie, PowerLieGroup) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Power>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Power>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Power>);

  Power identity;
  PowerTangent xi;
  xi << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  Power expected({Pose2::Expmap(xi.head<3>()), Pose2::Expmap(xi.tail<3>())});

  Power actual = identity.expmap(xi);
  EXPECT(assert_equal(expected, actual, kTol));
  EXPECT(assert_equal(xi, identity.logmap(actual), kTol));
  const auto adj = identity.AdjointMap();
  EXPECT_LONGS_EQUAL(6, adj.rows());
  EXPECT_LONGS_EQUAL(6, adj.cols());
}

/* ************************************************************************* */
TEST(testPower, compose) {
  Power state1({Pose2(1, 2, 3), Pose2(4, 5, 6)}), state2 = state1;

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(composePowerProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(composePowerProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testPower, between) {
  Power state1({Pose2(1, 2, 3), Pose2(4, 5, 6)}), state2 = state1;

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(betweenPowerProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(betweenPowerProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testPower, inverse) {
  Power state1({Pose2(1, 2, 3), Pose2(4, 5, 6)});

  Matrix actH1;
  state1.inverse(actH1);
  Matrix numericH1 = numericalDerivative11(inversePowerProxy, state1);
  EXPECT(assert_equal(numericH1, actH1, kTol));
}

/* ************************************************************************* */
TEST(testPower, Expmap) {
  PowerTangent vec;
  vec << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  Matrix actH;
  Power::Expmap(vec, actH);
  Matrix numericH = numericalDerivative11(powerExpmapProxy, vec);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testPower, Logmap) {
  Power state({Pose2(1, 2, 3), Pose2(4, 5, 6)});

  Matrix actH;
  Power::Logmap(state, actH);
  Matrix numericH = numericalDerivative11(powerLogmapProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testPower, AdjointMap) {
  Power state({Pose2(1, 2, 3), Pose2(4, 5, 6)});
  const Matrix actual = state.AdjointMap();

  Matrix expected = Matrix::Zero(6, 6);
  expected.block<3, 3>(0, 0) = state[0].AdjointMap();
  expected.block<3, 3>(3, 3) = state[1].AdjointMap();

  EXPECT(assert_equal(expected, actual, kTol));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
