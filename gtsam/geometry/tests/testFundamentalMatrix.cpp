/*
 * @file testFundamentalMatrix.cpp
 * @brief Test FundamentalMatrix class
 * @author Frank Dellaert
 * @date October 23, 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(FundamentalMatrix)
GTSAM_CONCEPT_MANIFOLD_INST(FundamentalMatrix)

//*************************************************************************
// Create two rotations and corresponding fundamental matrix F
Rot3 trueU = Rot3::Yaw(M_PI_2);
Rot3 trueV = Rot3::Yaw(M_PI_4);
double trueS = 0.5;
FundamentalMatrix trueF(trueU, trueS, trueV);

//*************************************************************************
TEST(FundamentalMatrix, localCoordinates) {
  Vector expected = Z_7x1;  // Assuming 7 dimensions for U, V, and s
  Vector actual = trueF.localCoordinates(trueF);
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*************************************************************************
TEST(FundamentalMatrix, retract) {
  FundamentalMatrix actual = trueF.retract(Z_7x1);
  EXPECT(assert_equal(trueF, actual));
}

//*************************************************************************
TEST(FundamentalMatrix, RoundTrip) {
  Vector7 d;
  d << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  FundamentalMatrix hx = trueF.retract(d);
  Vector actual = trueF.localCoordinates(hx);
  EXPECT(assert_equal(d, actual, 1e-8));
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
