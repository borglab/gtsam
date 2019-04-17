/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSOnBase.cpp
 * @brief  Unit tests for Base class of SO(n) classes.
 * @author Frank Dellaert
 **/

// #include <gtsam/base/Manifold.h>
// #include <gtsam/base/Testable.h>
// #include <gtsam/base/lieProxies.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(SOn, SO5) {
  const auto R = SOn(5);
  EXPECT_LONGS_EQUAL(5, R.rows());
  Values values;
  const Key key(0);
  values.insert(key, R);
  const auto B = values.at<SOn>(key);
  EXPECT_LONGS_EQUAL(5, B.rows());
}

/* ************************************************************************* */
TEST(SOn, Random) {
  static boost::mt19937 rng(42);
  EXPECT_LONGS_EQUAL(3, SOn::Random(rng, 3).rows());
  EXPECT_LONGS_EQUAL(4, SOn::Random(rng, 4).rows());
  EXPECT_LONGS_EQUAL(5, SOn::Random(rng, 5).rows());
}

/* ************************************************************************* */
TEST(SOn, HatVee) {
  Vector6 v;
  v << 1, 2, 3, 4, 5, 6;

  Matrix expected2(2, 2);
  expected2 << 0, -1, 1, 0;
  const auto actual2 = SOn::Hat(2, v.head<1>());
  CHECK(assert_equal(expected2, actual2));
  CHECK(assert_equal((Vector)v.head<1>(), SOn::Vee(actual2)));

  Matrix expected3(3, 3);
  expected3 << 0, -3, 2,  //
      3, 0, -1,           //
      -2, 1, 0;
  const auto actual3 = SOn::Hat(3, v.head<3>());
  CHECK(assert_equal(expected3, actual3));
  CHECK(assert_equal(skewSymmetric(1, 2, 3), actual3));
  CHECK(assert_equal((Vector)v.head<3>(), SOn::Vee(actual3)));

  Matrix expected4(4, 4);
  expected4 << 0, -6, 5, -3,  //
      6, 0, -4, 2,            //
      -5, 4, 0, -1,           //
      3, -2, 1, 0;
  const auto actual4 = SOn::Hat(4, v);
  CHECK(assert_equal(expected4, actual4));
  CHECK(assert_equal((Vector)v, SOn::Vee(actual4)));
}

/* ************************************************************************* */
TEST(SOn, RetractLocal) {
  // If we do expmap in SO(3) subgroup, topleft should be equal to R1.
  Vector6 v1 = (Vector(6) << 0, 0, 0, 0.01, 0, 0).finished();
  Matrix R1 = SO3::Retract(v1.tail<3>());
  SOn Q1 = SOn::Retract(4, v1);
  CHECK(assert_equal(R1, Q1.block(0, 0, 3, 3), 1e-7));
  CHECK(assert_equal(v1, SOn::Local(Q1), 1e-7));
}

/* ************************************************************************* */
TEST(SOn, vec) {
  auto x = SOn(5);
  Vector6 v;
  v << 1, 2, 3, 4, 5, 6;
  x.block(0, 0, 4, 4) = SO4::Expmap(v).matrix();
  Matrix actualH;
  const Vector actual = x.vec(actualH);
  boost::function<Vector(const SOn&)> h = [](const SOn& x) { return x.vec(); };
  const Matrix H = numericalDerivative11<Vector, SOn, 10>(h, x, 1e-5);
  CHECK(assert_equal(H, actualH));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
