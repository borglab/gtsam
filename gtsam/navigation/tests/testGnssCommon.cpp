/**
 * @file    testGnssCommon.cpp
 * @brief   Unit tests for shared GNSS utilities (gnss::geodist).
 * @date   April 16, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/GnssCommon.h>

#include <cmath>

using namespace gtsam;

/// Representative ECEF positions for an Earth-surface receiver and a satellite.
static const Point3 kReceiverEcef(-3961900.0, 3349000.0, 3698215.0);
static const Point3 kSatelliteEcef(15525000.0, 12000000.0, 19050000.0);

// *************************************************************************
TEST(TestGnssGeodist, RangeMatchesEuclideanPlusSagnac) {
  Point3 e;
  const double r = gnss::geodist(kSatelliteEcef, kReceiverEcef, e);

  const double euclidean = (kSatelliteEcef - kReceiverEcef).norm();
  const double sagnac =
      gnss::OMGE *
      (kSatelliteEcef.x() * kReceiverEcef.y() -
       kSatelliteEcef.y() * kReceiverEcef.x()) /
      gnss::C_LIGHT;
  EXPECT_DOUBLES_EQUAL(euclidean + sagnac, r, 1e-6);
}

// *************************************************************************
TEST(TestGnssGeodist, UnitVectorIsNormalized) {
  Point3 e;
  gnss::geodist(kSatelliteEcef, kReceiverEcef, e);
  EXPECT_DOUBLES_EQUAL(1.0, e.norm(), 1e-9);

  const Point3 expected =
      (kSatelliteEcef - kReceiverEcef) / (kSatelliteEcef - kReceiverEcef).norm();
  EXPECT(assert_equal(expected, e, 1e-9));
}

// *************************************************************************
TEST(TestGnssGeodist, CoincidentPointsDoNotProduceNaN) {
  // When the satellite and receiver positions coincide, geodist must return
  // without dividing by zero.
  Point3 e(1.0, 1.0, 1.0);  // Sentinel to ensure it gets overwritten.
  Matrix13 H;
  const double r = gnss::geodist(kReceiverEcef, kReceiverEcef, e, H);

  EXPECT(!std::isnan(r));
  EXPECT(!(e.array() != e.array()).any());  // no NaNs
  EXPECT(!(H.array() != H.array()).any());  // no NaNs
  EXPECT(assert_equal(Point3(Point3::Zero()), e, 1e-9));
  EXPECT_DOUBLES_EQUAL(H.norm(), 0.0, 1e-9);
}

// *************************************************************************
TEST(TestGnssGeodist, AnalyticJacobianMatchesNumerical) {
  // Wrap geodist as a function of the receiver position only.
  std::function<Vector1(const Point3&)> range_of = [](const Point3& rcv) {
    Point3 unused;
    return Vector1(gnss::geodist(kSatelliteEcef, rcv, unused));
  };

  const Matrix expectedH =
      numericalDerivative11<Vector1, Point3>(range_of, kReceiverEcef);

  Point3 e;
  Matrix13 actualH;
  gnss::geodist(kSatelliteEcef, kReceiverEcef, e, actualH);

  EXPECT(assert_equal(expectedH, Matrix(actualH), 1e-3));
}

// *************************************************************************
TEST(TestGnssGeodist, JacobianIncludesSagnacContribution) {
  // The Jacobian has both the geometric part (-e^T) and the Sagnac part.
  // Verify the Sagnac term is actually contributing to the Jacobian by
  // comparing against the pure Euclidean Jacobian.
  Point3 e;
  Matrix13 H_with_sagnac;
  gnss::geodist(kSatelliteEcef, kReceiverEcef, e, H_with_sagnac);

  // Pure Euclidean-only Jacobian (no Sagnac).
  const Matrix13 H_euclidean = -e.transpose();

  // Expected Sagnac derivative w.r.t. receiver position.
  Matrix13 H_sagnac_expected;
  H_sagnac_expected(0, 0) = -gnss::OMGE * kSatelliteEcef.y() / gnss::C_LIGHT;
  H_sagnac_expected(0, 1) = gnss::OMGE * kSatelliteEcef.x() / gnss::C_LIGHT;
  H_sagnac_expected(0, 2) = 0.0;

  EXPECT(assert_equal(Matrix(H_euclidean + H_sagnac_expected),
                      Matrix(H_with_sagnac), 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
