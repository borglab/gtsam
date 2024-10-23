/*
 * @file testFundamentalMatrix.cpp
 * @brief Test FundamentalMatrix classes
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
// Create essential matrix and focal lengths for
// SimpleFundamentalMatrix
EssentialMatrix trueE;  // Assuming a default constructor is available
double trueFa = 1.0;
double trueFb = 1.0;
Point2 trueCa(0.0, 0.0);
Point2 trueCb(0.0, 0.0);
SimpleFundamentalMatrix trueSimpleF(trueE, trueFa, trueFb, trueCa, trueCb);

//*************************************************************************
TEST(SimpleFundamentalMatrix, localCoordinates) {
  Vector expected = Z_7x1;
  Vector actual = trueSimpleF.localCoordinates(trueSimpleF);
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*************************************************************************
TEST(SimpleFundamentalMatrix, retract) {
  SimpleFundamentalMatrix actual = trueSimpleF.retract(Z_9x1);
  EXPECT(assert_equal(trueSimpleF, actual));
}

//*************************************************************************
TEST(SimpleFundamentalMatrix, RoundTrip) {
  Vector7 d;
  d << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  SimpleFundamentalMatrix hx = trueSimpleF.retract(d);
  Vector actual = trueSimpleF.localCoordinates(hx);
  EXPECT(assert_equal(d, actual, 1e-8));
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
