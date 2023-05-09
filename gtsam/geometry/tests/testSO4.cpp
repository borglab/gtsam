/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSO4.cpp
 * @brief  Unit tests for SO4, as a GTSAM-adapted Lie Group
 * @author Frank Dellaert
 **/

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SO3.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <random>

using namespace std;
using namespace gtsam;

//******************************************************************************
TEST(SO4, Identity) {
  const SO4 R;
  EXPECT_LONGS_EQUAL(4, R.rows());
  EXPECT_LONGS_EQUAL(6, SO4::dimension);
  EXPECT_LONGS_EQUAL(6, SO4::Dim());
  EXPECT_LONGS_EQUAL(6, R.dim());
  EXPECT_LONGS_EQUAL(6, traits<SO4>::GetDimension(R));
}

//******************************************************************************
TEST(SO4, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<SO4>));
  BOOST_CONCEPT_ASSERT((IsManifold<SO4>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<SO4>));
}

//******************************************************************************
SO4 id;
Vector6 v1 = (Vector(6) << 0, 0, 0, 0.1, 0, 0).finished();
SO4 Q1 = SO4::Expmap(v1);
Vector6 v2 = (Vector(6) << 0.00, 0.00, 0.00, 0.01, 0.02, 0.03).finished();
SO4 Q2 = SO4::Expmap(v2);
Vector6 v3 = (Vector(6) << 1, 2, 3, 4, 5, 6).finished();
SO4 Q3 = SO4::Expmap(v3);

//******************************************************************************
TEST(SO4, Random) {
  std::mt19937 rng(42);
  auto Q = SO4::Random(rng);
  EXPECT_LONGS_EQUAL(4, Q.matrix().rows());
}
//******************************************************************************
TEST(SO4, Expmap) {
  // If we do exponential map in SO(3) subgroup, topleft should be equal to R1.
  auto R1 = SO3::Expmap(v1.tail<3>()).matrix();
  EXPECT((Q1.matrix().topLeftCorner<3, 3>().isApprox(R1)));

  // Same here
  auto R2 = SO3::Expmap(v2.tail<3>()).matrix();
  EXPECT((Q2.matrix().topLeftCorner<3, 3>().isApprox(R2)));

  // Check commutative subgroups
  for (size_t i = 0; i < 6; i++) {
    Vector6 xi = Vector6::Zero();
    xi[i] = 2;
    SO4 Q1 = SO4::Expmap(xi);
    xi[i] = 3;
    SO4 Q2 = SO4::Expmap(xi);
    EXPECT(assert_equal(Q1 * Q2, Q2 * Q1));
  }
}

//******************************************************************************
TEST(SO4, Hat) {
  // Check that Hat specialization is equal to dynamic version
  EXPECT(assert_equal(SO4::Hat(v1), SOn::Hat(v1)));
  EXPECT(assert_equal(SO4::Hat(v2), SOn::Hat(v2)));
  EXPECT(assert_equal(SO4::Hat(v3), SOn::Hat(v3)));
}

//******************************************************************************
TEST(SO4, Vee) {
  // Check that Hat specialization is equal to dynamic version
  auto X1 = SOn::Hat(v1), X2 = SOn::Hat(v2), X3 = SOn::Hat(v3);
  EXPECT(assert_equal(SO4::Vee(X1), SOn::Vee(X1)));
  EXPECT(assert_equal(SO4::Vee(X2), SOn::Vee(X2)));
  EXPECT(assert_equal(SO4::Vee(X3), SOn::Vee(X3)));
}

//******************************************************************************
TEST(SO4, Retract) {
  // Check that Cayley yields the same as Expmap for small values
  EXPECT(assert_equal(id.retract(v1 / 100), SO4::Expmap(v1 / 100)));
  EXPECT(assert_equal(id.retract(v2 / 100), SO4::Expmap(v2 / 100)));

  // Check that Cayley is identical to dynamic version
  EXPECT(assert_equal(SOn(id.retract(v1 / 100)), SOn(4).retract(v1 / 100)));
  EXPECT(assert_equal(SOn(id.retract(v2 / 100)), SOn(4).retract(v2 / 100)));

  auto v = Vector6::Zero();
  SO4 actual = traits<SO4>::Retract(id, v);
  EXPECT(assert_equal(id, actual));
}

//******************************************************************************
TEST(SO4, Local) {
  // Check that Cayley is identical to dynamic version
  EXPECT(
      assert_equal(id.localCoordinates(Q1), SOn(4).localCoordinates(SOn(Q1))));
  EXPECT(
      assert_equal(id.localCoordinates(Q2), SOn(4).localCoordinates(SOn(Q2))));

  auto v0 = Vector6::Zero();
  Vector6 actual = traits<SO4>::Local(id, id);
  EXPECT(assert_equal((Vector)v0, actual));
}

//******************************************************************************
TEST(SO4, Invariants) {
  EXPECT(check_group_invariants(id, id));
  EXPECT(check_group_invariants(id, Q1));
  EXPECT(check_group_invariants(Q2, id));
  EXPECT(check_group_invariants(Q2, Q1));
  EXPECT(check_group_invariants(Q1, Q2));

  EXPECT(check_manifold_invariants(id, id));
  EXPECT(check_manifold_invariants(id, Q1));
  EXPECT(check_manifold_invariants(Q2, id));
  EXPECT(check_manifold_invariants(Q2, Q1));
  EXPECT(check_manifold_invariants(Q1, Q2));
}

//******************************************************************************
TEST(SO4, compose) {
  SO4 expected = Q1 * Q2;
  Matrix actualH1, actualH2;
  SO4 actual = Q1.compose(Q2, actualH1, actualH2);
  EXPECT(assert_equal(expected, actual));

  Matrix numericalH1 =
      numericalDerivative21(testing::compose<SO4>, Q1, Q2, 1e-2);
  EXPECT(assert_equal(numericalH1, actualH1));

  Matrix numericalH2 =
      numericalDerivative22(testing::compose<SO4>, Q1, Q2, 1e-2);
  EXPECT(assert_equal(numericalH2, actualH2));
}

//******************************************************************************
TEST(SO4, vec) {
  using Vector16 = SO4::VectorN2;
  const Vector16 expected = Eigen::Map<const Vector16>(Q2.matrix().data());
  Matrix actualH;
  const Vector16 actual = Q2.vec(actualH);
  EXPECT(assert_equal(expected, actual));
  std::function<Vector16(const SO4&)> f = [](const SO4& Q) {
    return Q.vec();
  };
  const Matrix numericalH = numericalDerivative11(f, Q2, 1e-5);
  EXPECT(assert_equal(numericalH, actualH));
}

//******************************************************************************
TEST(SO4, topLeft) {
  const Matrix3 expected = Q3.matrix().topLeftCorner<3, 3>();
  Matrix actualH;
  const Matrix3 actual = topLeft(Q3, actualH);
  EXPECT(assert_equal(expected, actual));
  std::function<Matrix3(const SO4&)> f = [](const SO4& Q3) {
    return topLeft(Q3);
  };
  const Matrix numericalH = numericalDerivative11(f, Q3, 1e-5);
  EXPECT(assert_equal(numericalH, actualH));
}

//******************************************************************************
TEST(SO4, stiefel) {
  const Matrix43 expected = Q3.matrix().leftCols<3>();
  Matrix actualH;
  const Matrix43 actual = stiefel(Q3, actualH);
  EXPECT(assert_equal(expected, actual));
  std::function<Matrix43(const SO4&)> f = [](const SO4& Q3) {
    return stiefel(Q3);
  };
  const Matrix numericalH = numericalDerivative11(f, Q3, 1e-5);
  EXPECT(assert_equal(numericalH, actualH));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
