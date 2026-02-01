/* ************************************************************************* */
/**
 * @file    testSL4.cpp
 * @brief   Unit tests for SL4 manifold
 * @author  Hyungtae Lim
 */
/* ************************************************************************* */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/SL4.h>

#include <cmath>
#include <vector>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(SL4)
GTSAM_CONCEPT_MATRIX_LIE_GROUP_INST(SL4)

// Common static variables for tests
static const Vector15 xi0 =
    (Vector15() << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10,
     0.11, 0.12, 0.13, 0.14, 0.15)
        .finished();

static const Vector15 xi1 =
    (Vector15() << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10,
     0.11, 0.12, 0.13, 0.14, 0.15)
        .finished();

static const Vector15 xi2 =
    (Vector15() << 0.05, 0.04, 0.09, 0.02, 0.03, 0.08, 0.07, 0.06, 0.01, 0.10,
     0.12, 0.11, 0.15, 0.13, 0.14)
        .finished();

// define xi_large - moderately large values to test numerical stability
// while remaining within the feasible range for matrix exponential
static const Vector15 xi_large =
    (Vector15() << 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4,
     1.5, 1.6, 1.7, 1.8, 1.9)
        .finished();
// Create a SL4
const Matrix4 T_matrix =
    (Matrix4() << 1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1).finished();
const SL4 T1(T_matrix);

const Matrix4 T_matrix2 =
    (Matrix4() << 1, 0, 0, 4, 0, 1, 0, 5, 0, 0, 1, 6, 0, 0, 0, 1).finished();
const SL4 T2(T_matrix2);

/* ************************************************************************* */
TEST(SL4, Constructor) {
  SL4 sl4_default;
  EXPECT(assert_equal(SL4(), sl4_default));
}

/* ************************************************************************* */
TEST(SL4, Identity) {
  SL4 identity;
  EXPECT(assert_equal<Matrix4>(Matrix4::Identity(), identity.matrix(), 1e-8));
}

/* ************************************************************************* */
TEST(SL4, Expmap) {
  SL4 expected(SL4::Expmap(xi0));
  EXPECT(assert_equal(expected, SL4::Expmap(xi0), 1e-8));
  
  // Verify that the result has determinant 1
  Matrix4 T = expected.matrix();
  double det = T.determinant();
  EXPECT_DOUBLES_EQUAL(1.0, det, 1e-8);
}

/* ************************************************************************* */
TEST(SL4, ExpmapLargeTangent) {
  // Test with large tangent vector - the numerical stability fix should handle this
  SL4 result = SL4::Expmap(xi_large);
  // Verify determinant is 1
  Matrix4 T = result.matrix();
  double det = T.determinant();
  EXPECT_DOUBLES_EQUAL(1.0, det, 1e-6);  // Looser tolerance for large inputs
}

/* ************************************************************************* */
TEST(SL4, Logmap) {
  SL4 sl4_exp(SL4::Expmap(xi0));
  EXPECT(assert_equal(xi0, SL4::Logmap(sl4_exp), 1e-8));
}

/* ************************************************************************* */
TEST(SL4, Retract) {
  Vector15 xi = xi0 / 100;
  SL4 actual = SL4::Retract(xi);
  SL4 expected(I_4x4 + SL4::Hat(xi));
  EXPECT(assert_equal(expected, actual, 1e-8));
  
  // Verify that the result has determinant 1
  Matrix4 T = actual.matrix();
  double det = T.determinant();
  EXPECT_DOUBLES_EQUAL(1.0, det, 1e-8);
}

/* ************************************************************************* */
TEST(SL4, RetractLargeTangent) {
  // Test with large tangent vector - the retraction will fall back to Expmap
  // for large inputs that produce invalid first-order approximations.
  SL4 result = SL4::Retract(xi_large);
  // Verify determinant is 1
  Matrix4 T = result.matrix();
  double det = T.determinant();
  EXPECT_DOUBLES_EQUAL(1.0, det, 1e-6);  // Looser tolerance for large inputs
}

/* ************************************************************************* */
TEST(SL4, LocalCoordinates) {
  Vector15 xi = xi0 / 100;
  SL4 sl4_retracted = T1.retract(xi);
  Vector xi_retrieved = T1.localCoordinates(sl4_retracted);
  EXPECT(assert_equal(sl4_retracted, T1.retract(xi_retrieved), 1e-5));
}

/* ************************************************************************* */
TEST(SL4, AdjointMapMatchesMatrixLieGroup) {
  SL4 sl4 = SL4::Expmap(xi0);
  Matrix adj_SL4 = sl4.AdjointMap();
  Matrix adj_generic = sl4.MatrixLieGroup<SL4, 15, 4>::AdjointMap();

  EXPECT(assert_equal(adj_generic, adj_SL4, 1e-8));
}

/* ************************************************************************* */
TEST(SL4, ComposeAndInverse) {
  SL4 sl4_1 = SL4::Expmap(xi1);
  SL4 sl4_2 = SL4::Expmap(xi2);

  SL4 composed = sl4_1.compose(sl4_2);
  SL4 expected(composed.matrix());
  EXPECT(assert_equal(expected, composed, 1e-8));

  SL4 inv = sl4_1.inverse();
  SL4 identity = sl4_1.compose(inv);
  EXPECT(assert_equal(SL4(), identity, 1e-8));
}

/* ************************************************************************* */
TEST(SL4, Between) {
  SL4 sl4_1 = SL4::Expmap(xi1);
  SL4 sl4_2 = SL4::Expmap(xi2);

  Matrix H1, H2;
  SL4 expected(sl4_1.matrix().inverse() * sl4_2.matrix());
  SL4 actual = sl4_1.between(sl4_2, H1, H2);
  EXPECT(assert_equal(expected, actual, 1e-8));

  Matrix numericalH1 =
      numericalDerivative21(testing::between<SL4>, sl4_1, sl4_2);
  EXPECT(assert_equal(numericalH1, H1, 5e-3));

  Matrix numericalH2 =
      numericalDerivative22(testing::between<SL4>, sl4_1, sl4_2);
  EXPECT(assert_equal(numericalH2, H2, 1e-5));
}

/* ************************************************************************* */
// Test that Hat and Vee are inverses for random tangent vectors
TEST(SL4, HatVeeAreInverses) {
  Matrix4 hat = SL4::Hat(xi0);
  Vector xi_recovered = SL4::Vee(hat);
  EXPECT(assert_equal(xi0, xi_recovered, 1e-8));
}

/* ************************************************************************* */
TEST(SL4, HatTraceIsZero) {
  Vector15 eta =
      (Vector15() << 0.31, -0.22, 0.14, -0.09, 0.27, -0.18, 0.05, -0.12, 0.08,
       0.11, -0.06, 0.02, 0.07, -0.04, 0.13)
          .finished();
  Matrix4 A = SL4::Hat(eta);
  EXPECT_DOUBLES_EQUAL(0.0, A.trace(), 1e-12);
}

/* ************************************************************************* */
TEST(SL4, HatVeeRoundTrip) {
  Vector15 eta =
      (Vector15() << -0.25, 0.19, -0.11, 0.07, -0.03, 0.29, -0.17, 0.09, 0.21,
       -0.13, 0.04, -0.06, 0.15, -0.08, 0.02)
          .finished();
  Vector eta_recovered = SL4::Vee(SL4::Hat(eta));
  EXPECT(assert_equal(eta, eta_recovered, 1e-12));
}

/* ************************************************************************* */
TEST(SL4, HatBasisIsOrthonormal) {
  std::vector<Matrix4> generators;
  generators.reserve(15);

  for (int k = 0; k < 15; ++k) {
    Vector15 e = Vector15::Zero();
    e(k) = 1.0;
    generators.push_back(SL4::Hat(e));
  }

  for (int i = 0; i < 15; ++i) {
    for (int j = 0; j < 15; ++j) {
      const double inner =
          (generators[i].array() * generators[j].array()).sum();
      const double expected = (i == j) ? 1.0 : 0.0;
      EXPECT_DOUBLES_EQUAL(expected, inner, 1e-12);
    }
  }
}

/* ************************************************************************* */
TEST(SL4, HatMatchesOrthonormalDefinition) {
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
  const double inv_sqrt6 = 1.0 / std::sqrt(6.0);
  const double inv_sqrt12 = 1.0 / std::sqrt(12.0);

  const Vector15 eta = xi0;
  Matrix4 expected = Matrix4::Zero();

  expected(0, 1) += inv_sqrt2 * eta(0);
  expected(1, 0) -= inv_sqrt2 * eta(0);
  expected(0, 2) += inv_sqrt2 * eta(1);
  expected(2, 0) -= inv_sqrt2 * eta(1);
  expected(0, 3) += inv_sqrt2 * eta(2);
  expected(3, 0) -= inv_sqrt2 * eta(2);
  expected(1, 2) += inv_sqrt2 * eta(3);
  expected(2, 1) -= inv_sqrt2 * eta(3);
  expected(1, 3) += inv_sqrt2 * eta(4);
  expected(3, 1) -= inv_sqrt2 * eta(4);
  expected(2, 3) += inv_sqrt2 * eta(5);
  expected(3, 2) -= inv_sqrt2 * eta(5);

  expected(0, 1) += inv_sqrt2 * eta(6);
  expected(1, 0) += inv_sqrt2 * eta(6);
  expected(0, 2) += inv_sqrt2 * eta(7);
  expected(2, 0) += inv_sqrt2 * eta(7);
  expected(0, 3) += inv_sqrt2 * eta(8);
  expected(3, 0) += inv_sqrt2 * eta(8);
  expected(1, 2) += inv_sqrt2 * eta(9);
  expected(2, 1) += inv_sqrt2 * eta(9);
  expected(1, 3) += inv_sqrt2 * eta(10);
  expected(3, 1) += inv_sqrt2 * eta(10);
  expected(2, 3) += inv_sqrt2 * eta(11);
  expected(3, 2) += inv_sqrt2 * eta(11);

  const double a = inv_sqrt2 * eta(12);
  const double b = inv_sqrt6 * eta(13);
  const double c = inv_sqrt12 * eta(14);
  expected(0, 0) = a + b + c;
  expected(1, 1) = -a + b + c;
  expected(2, 2) = -2.0 * b + c;
  expected(3, 3) = -3.0 * c;

  EXPECT(assert_equal(expected, SL4::Hat(eta), 1e-12));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
