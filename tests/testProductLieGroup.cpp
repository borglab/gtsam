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
#include <gtsam/geometry/Rot3.h>

#include <iostream>

using namespace gtsam;

constexpr double kTol = 1e-9;

using Product = ProductLieGroup<Point2, Pose2>;
using ProductVR = ProductLieGroup<Vector, Rot3>;
using ProductVV = ProductLieGroup<Vector, Vector>;
constexpr int kPowerComponents = 2;
using Power = PowerLieGroup<Pose2, kPowerComponents>;
using PowerTangent = Power::TangentVector;
using DynamicPower = PowerLieGroup<Pose2, Eigen::Dynamic>;
using DynamicPowerTangent = DynamicPower::TangentVector;

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

template <>
struct traits<DynamicPower> : internal::LieGroupTraits<DynamicPower> {
  static void Print(const DynamicPower& m, const std::string& s = "") {
    std::cout << s << "[";
    for (size_t i = 0; i < m.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << "Pose2(" << m[i].x() << "," << m[i].y() << ","
                << m[i].theta() << ")";
    }
    std::cout << "]" << std::endl;
  }
  static bool Equals(const DynamicPower& m1, const DynamicPower& m2,
                     double tol = 1e-8) {
    if (m1.size() != m2.size()) return false;
    for (size_t i = 0; i < m1.size(); ++i) {
      if (!m1[i].equals(m2[i], tol)) return false;
    }
    return true;
  }
};
}  // namespace gtsam

namespace {
Vector makeVector(std::initializer_list<double> values) {
  Vector vector(static_cast<Eigen::Index>(values.size()));
  Eigen::Index index = 0;
  for (double value : values) {
    vector(index++) = value;
  }
  return vector;
}

Product composeProductProxy(const Product& A, const Product& B) {
  return A.compose(B);
}

Product betweenProductProxy(const Product& A, const Product& B) {
  return A.between(B);
}

Product inverseProductProxy(const Product& A) { return A.inverse(); }

Product expmapProductProxy(const Vector5& vec) { return Product::Expmap(vec); }

Vector5 logmapProductProxy(const Product& p) { return Product::Logmap(p); }

ProductVR composeProductVRProxy(const ProductVR& A, const ProductVR& B) {
  return A.compose(B);
}

ProductVR betweenProductVRProxy(const ProductVR& A, const ProductVR& B) {
  return A.between(B);
}

ProductVR inverseProductVRProxy(const ProductVR& A) { return A.inverse(); }

ProductVR expmapProductVRProxy(const Vector& vec) {
  return ProductVR::Expmap(vec);
}

Vector logmapProductVRProxy(const ProductVR& p) { return ProductVR::Logmap(p); }

ProductVV composeProductVVProxy(const ProductVV& A, const ProductVV& B) {
  return A.compose(B);
}

ProductVV betweenProductVVProxy(const ProductVV& A, const ProductVV& B) {
  return A.between(B);
}

ProductVV inverseProductVVProxy(const ProductVV& A) { return A.inverse(); }

Vector logmapProductVVProxy(const ProductVV& p) { return ProductVV::Logmap(p); }

Power composePowerProxy(const Power& A, const Power& B) { return A.compose(B); }

Power betweenPowerProxy(const Power& A, const Power& B) { return A.between(B); }

Power inversePowerProxy(const Power& A) { return A.inverse(); }

Power powerExpmapProxy(const PowerTangent& vec) { return Power::Expmap(vec); }

PowerTangent powerLogmapProxy(const Power& p) { return Power::Logmap(p); }

DynamicPower composeDynamicPowerProxy(const DynamicPower& A,
                                      const DynamicPower& B) {
  return A.compose(B);
}

DynamicPower betweenDynamicPowerProxy(const DynamicPower& A,
                                      const DynamicPower& B) {
  return A.between(B);
}

DynamicPower inverseDynamicPowerProxy(const DynamicPower& A) {
  return A.inverse();
}

DynamicPower dynamicPowerExpmapProxy(const DynamicPowerTangent& vec) {
  return DynamicPower::Expmap(vec);
}

DynamicPowerTangent dynamicPowerLogmapProxy(const DynamicPower& p) {
  return DynamicPower::Logmap(p);
}
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
TEST(Lie, ProductLieGroupDynamicVectorRot3) {
  GTSAM_CONCEPT_ASSERT(IsGroup<ProductVR>);
  GTSAM_CONCEPT_ASSERT(IsManifold<ProductVR>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<ProductVR>);

  ProductVR pair1(Vector::Zero(2), Rot3::Identity());
  Vector d = makeVector({1.0, 2.0, 0.1, 0.2, 0.3});
  ProductVR expected(makeVector({1.0, 2.0}),
                     Rot3::Expmap(Vector3(0.1, 0.2, 0.3)));
  ProductVR pair2 = pair1.expmap(d);

  EXPECT_LONGS_EQUAL(Eigen::Dynamic, ProductVR::dimension);
  EXPECT_LONGS_EQUAL(5, pair1.dim());
  EXPECT(assert_equal(expected, pair2, kTol));
  EXPECT(assert_equal(d, pair1.logmap(pair2), kTol));

  const Matrix adj = pair1.AdjointMap();
  EXPECT_LONGS_EQUAL(5, adj.rows());
  EXPECT_LONGS_EQUAL(5, adj.cols());
}

/* ************************************************************************* */
TEST(testProductDynamicVR, compose) {
  ProductVR state1(makeVector({1.0, 2.0}), Rot3::RzRyRx(0.1, 0.2, 0.3));
  ProductVR state2(makeVector({-0.5, 4.0}), Rot3::RzRyRx(-0.3, 0.1, -0.2));

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21<ProductVR, ProductVR, ProductVR, 5>(
      composeProductVRProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22<ProductVR, ProductVR, ProductVR, 5>(
      composeProductVRProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVR, between) {
  ProductVR state1(makeVector({1.0, 2.0}), Rot3::RzRyRx(0.1, 0.2, 0.3));
  ProductVR state2(makeVector({-0.5, 4.0}), Rot3::RzRyRx(-0.3, 0.1, -0.2));

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21<ProductVR, ProductVR, ProductVR, 5>(
      betweenProductVRProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22<ProductVR, ProductVR, ProductVR, 5>(
      betweenProductVRProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVR, inverse) {
  ProductVR state(makeVector({1.0, 2.0}), Rot3::RzRyRx(0.1, 0.2, 0.3));

  Matrix actH;
  state.inverse(actH);
  Matrix numericH = numericalDerivative11<ProductVR, ProductVR, 5>(
      inverseProductVRProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVR, Expmap) {
  Vector vec = makeVector({1.0, 2.0, 0.1, 0.2, 0.3});

  Matrix actH;
  ProductVR::Expmap(vec, actH);
  Matrix numericH =
      numericalDerivative11<ProductVR, Vector, 5>(expmapProductVRProxy, vec);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVR, Logmap) {
  ProductVR state(makeVector({1.0, 2.0}), Rot3::RzRyRx(0.1, 0.2, 0.3));

  Matrix actH;
  ProductVR::Logmap(state, actH);
  Matrix numericH =
      numericalDerivative11<Vector, ProductVR, 5>(logmapProductVRProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVR, AdjointMap) {
  ProductVR state(makeVector({1.0, 2.0}), Rot3::RzRyRx(0.1, 0.2, 0.3));
  const Matrix actual = state.AdjointMap();

  Matrix expected = Matrix::Zero(5, 5);
  expected.topLeftCorner(2, 2) = Matrix::Identity(2, 2);
  expected.bottomRightCorner(3, 3) = state.second.AdjointMap();

  EXPECT(assert_equal(expected, actual, kTol));
}

/* ************************************************************************* */
TEST(Lie, ProductLieGroupDynamicVectorVector) {
  GTSAM_CONCEPT_ASSERT(IsGroup<ProductVV>);
  GTSAM_CONCEPT_ASSERT(IsManifold<ProductVV>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<ProductVV>);

  ProductVV pair1(makeVector({0.0, 0.0}), makeVector({0.0, 0.0, 0.0}));
  Vector d = makeVector({1.0, 2.0, 3.0, 4.0, 5.0});
  ProductVV expected(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));
  ProductVV pair2 = ProductVV::Expmap(Vector(d.head(2)), Vector(d.tail(3)));

  EXPECT_LONGS_EQUAL(Eigen::Dynamic, ProductVV::dimension);
  EXPECT_LONGS_EQUAL(5, pair1.dim());
  EXPECT(assert_equal(expected, pair2, kTol));
  EXPECT(assert_equal(d, pair1.logmap(pair2), kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, compose) {
  ProductVV state1(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));
  ProductVV state2(makeVector({-0.5, 4.0}), makeVector({-1.0, 2.0, 1.5}));

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21<ProductVV, ProductVV, ProductVV, 5>(
      composeProductVVProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22<ProductVV, ProductVV, ProductVV, 5>(
      composeProductVVProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, between) {
  ProductVV state1(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));
  ProductVV state2(makeVector({-0.5, 4.0}), makeVector({-1.0, 2.0, 1.5}));

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21<ProductVV, ProductVV, ProductVV, 5>(
      betweenProductVVProxy, state1, state2);
  Matrix numericH2 = numericalDerivative22<ProductVV, ProductVV, ProductVV, 5>(
      betweenProductVVProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, inverse) {
  ProductVV state(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));

  Matrix actH;
  state.inverse(actH);
  Matrix numericH = numericalDerivative11<ProductVV, ProductVV, 5>(
      inverseProductVVProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, retractAndLocalCoordinates) {
  ProductVV state(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));
  Vector delta = makeVector({0.1, -0.2, 0.3, -0.4, 0.5});

  ProductVV updated = state.retract(delta);
  EXPECT(assert_equal(delta, state.localCoordinates(updated), kTol));
  EXPECT(assert_equal(updated,
                      state.compose(ProductVV::Expmap(Vector(delta.head(2)),
                                                      Vector(delta.tail(3)))),
                      kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, Expmap) {
  Vector vec = makeVector({1.0, 2.0, 3.0, 4.0, 5.0});
  ProductVV expected(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));

  Matrix actH1, actH2;
  ProductVV actual =
      ProductVV::Expmap(Vector(vec.head(2)), Vector(vec.tail(3)), actH1, actH2);
  Matrix expectedH1 = Matrix::Zero(5, 2);
  Matrix expectedH2 = Matrix::Zero(5, 3);
  expectedH1.block(0, 0, 2, 2).setIdentity();
  expectedH2.block(2, 0, 3, 3).setIdentity();
  EXPECT(assert_equal(expected, actual, kTol));
  EXPECT(assert_equal(vec, ProductVV::Logmap(actual), kTol));
  EXPECT(assert_equal(expectedH1, actH1, kTol));
  EXPECT(assert_equal(expectedH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, Logmap) {
  ProductVV state(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));

  Matrix actH;
  ProductVV::Logmap(state, actH);
  Matrix numericH =
      numericalDerivative11<Vector, ProductVV, 5>(logmapProductVVProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, AdjointMap) {
  ProductVV state(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));
  const Matrix actual = state.AdjointMap();
  const Matrix expected = Matrix::Identity(5, 5);

  EXPECT(assert_equal(expected, actual, kTol));
}

/* ************************************************************************* */
TEST(testProductDynamicVV, Exceptions) {
  ProductVV state1(makeVector({1.0, 2.0}), makeVector({3.0, 4.0, 5.0}));
  ProductVV state2(makeVector({1.0, 2.0, 3.0}), makeVector({4.0, 5.0}));
  Vector vec = makeVector({1.0, 2.0, 3.0, 4.0, 5.0});

  CHECK_EXCEPTION(ProductVV::Expmap(vec), std::invalid_argument);
  CHECK_EXCEPTION(state1.compose(state2), std::invalid_argument);
  CHECK_EXCEPTION(state1.between(state2), std::invalid_argument);
  CHECK_EXCEPTION(state1.localCoordinates(state2), std::invalid_argument);
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

/* ************************************************************************* */
TEST(Lie, PowerLieGroupDynamicCount) {
  GTSAM_CONCEPT_ASSERT(IsGroup<DynamicPower>);
  GTSAM_CONCEPT_ASSERT(IsManifold<DynamicPower>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<DynamicPower>);

  DynamicPower identity;
  EXPECT_LONGS_EQUAL(Eigen::Dynamic, DynamicPower::dimension);
  EXPECT_LONGS_EQUAL(0, identity.size());
  EXPECT_LONGS_EQUAL(0, identity.dim());
  EXPECT_LONGS_EQUAL(0, DynamicPower::Identity().size());
  EXPECT(assert_equal(Vector::Zero(0), DynamicPower::Logmap(identity), kTol));
  const Matrix emptyAdj = identity.AdjointMap();
  EXPECT_LONGS_EQUAL(0, emptyAdj.rows());
  EXPECT_LONGS_EQUAL(0, emptyAdj.cols());

  DynamicPower sizedIdentity(2);
  EXPECT_LONGS_EQUAL(2, sizedIdentity.size());
  EXPECT(assert_equal(Pose2(), sizedIdentity[0], kTol));
  EXPECT(assert_equal(Pose2(), sizedIdentity[1], kTol));

  DynamicPowerTangent xi = makeVector({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
  DynamicPower expected(
      {Pose2::Expmap(xi.head<3>()), Pose2::Expmap(xi.tail<3>())});
  DynamicPower actual = DynamicPower::Expmap(xi);
  EXPECT(assert_equal(expected, actual, kTol));
  EXPECT(assert_equal(xi, DynamicPower::Logmap(actual), kTol));
  EXPECT_LONGS_EQUAL(6, actual.dim());
}

/* ************************************************************************* */
TEST(testPowerDynamic, compose) {
  DynamicPower state1({Pose2(1, 2, 3), Pose2(4, 5, 6)});
  DynamicPower state2({Pose2(-0.5, 4, -1.0), Pose2(2.0, -3.0, 0.25)});

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 =
      numericalDerivative21<DynamicPower, DynamicPower, DynamicPower, 6>(
          composeDynamicPowerProxy, state1, state2);
  Matrix numericH2 =
      numericalDerivative22<DynamicPower, DynamicPower, DynamicPower, 6>(
          composeDynamicPowerProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testPowerDynamic, between) {
  DynamicPower state1({Pose2(1, 2, 3), Pose2(4, 5, 6)});
  DynamicPower state2({Pose2(-0.5, 4, -1.0), Pose2(2.0, -3.0, 0.25)});

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 =
      numericalDerivative21<DynamicPower, DynamicPower, DynamicPower, 6>(
          betweenDynamicPowerProxy, state1, state2);
  Matrix numericH2 =
      numericalDerivative22<DynamicPower, DynamicPower, DynamicPower, 6>(
          betweenDynamicPowerProxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1, kTol));
  EXPECT(assert_equal(numericH2, actH2, kTol));
}

/* ************************************************************************* */
TEST(testPowerDynamic, inverse) {
  DynamicPower state({Pose2(1, 2, 3), Pose2(4, 5, 6)});

  Matrix actH;
  state.inverse(actH);
  Matrix numericH = numericalDerivative11<DynamicPower, DynamicPower, 6>(
      inverseDynamicPowerProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testPowerDynamic, retractAndLocalCoordinates) {
  DynamicPower state({Pose2(1, 2, 3), Pose2(4, 5, 6)});
  DynamicPowerTangent delta = makeVector({0.1, 0.2, 0.3, -0.2, 0.1, -0.1});

  DynamicPower updated = state.retract(delta);
  DynamicPower expected(
      {state[0].retract(delta.head<3>()), state[1].retract(delta.tail<3>())});
  EXPECT(assert_equal(expected, updated, kTol));
  EXPECT(assert_equal(delta, state.localCoordinates(updated), kTol));
}

/* ************************************************************************* */
TEST(testPowerDynamic, Expmap) {
  DynamicPowerTangent vec = makeVector({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
  DynamicPower expected(
      {Pose2::Expmap(vec.head<3>()), Pose2::Expmap(vec.tail<3>())});

  Matrix actH;
  DynamicPower actual = DynamicPower::Expmap(vec, actH);
  Matrix numericH = numericalDerivative11<DynamicPower, DynamicPowerTangent, 6>(
      dynamicPowerExpmapProxy, vec);
  EXPECT(assert_equal(expected, actual, kTol));
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testPowerDynamic, Logmap) {
  DynamicPower state({Pose2(1, 2, 3), Pose2(4, 5, 6)});

  Matrix actH;
  DynamicPower::Logmap(state, actH);
  Matrix numericH = numericalDerivative11<DynamicPowerTangent, DynamicPower, 6>(
      dynamicPowerLogmapProxy, state);
  EXPECT(assert_equal(numericH, actH, kTol));
}

/* ************************************************************************* */
TEST(testPowerDynamic, AdjointMap) {
  DynamicPower state({Pose2(1, 2, 3), Pose2(4, 5, 6)});
  const Matrix actual = state.AdjointMap();

  Matrix expected = Matrix::Zero(6, 6);
  expected.block<3, 3>(0, 0) = state[0].AdjointMap();
  expected.block<3, 3>(3, 3) = state[1].AdjointMap();

  EXPECT(assert_equal(expected, actual, kTol));
}

/* ************************************************************************* */
TEST(testPowerDynamic, Exceptions) {
  DynamicPower state1({Pose2(1, 2, 3), Pose2(4, 5, 6)});
  DynamicPower state2({Pose2(1, 2, 3)});
  CHECK_EXCEPTION(state1.compose(state2), std::invalid_argument);
  CHECK_EXCEPTION(state1.between(state2), std::invalid_argument);
  CHECK_EXCEPTION(state1.localCoordinates(state2), std::invalid_argument);

  DynamicPowerTangent vec = makeVector({0.1, 0.2, 0.3, 0.4, 0.5});
  CHECK_EXCEPTION(DynamicPower::Expmap(vec), std::invalid_argument);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
