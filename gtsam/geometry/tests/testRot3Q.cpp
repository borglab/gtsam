/* ------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRot3.cpp
 * @brief   Unit tests for Rot3 class, Quaternion specific
 * @author  Alireza Fathi
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Quaternion.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

#ifdef GTSAM_USE_QUATERNIONS

//******************************************************************************
TEST(Rot3Q , Compare) {

  // We set up expected values with quaternions
  typedef Quaternion Q;
  typedef traits<Q> TQ;
  typedef TQ::ChartJacobian OJ;

  // We check Rot3 expressions
  typedef Rot3 R;
  typedef traits<R> TR;

  // Define test values
  Q q1(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
  Q q2(Eigen::AngleAxisd(2, Vector3(0, 1, 0)));
  R R1(q1), R2(q2);

  // Check Compose
  Q q3 = TQ::Compose(q1, q2, {}, {});
  R R3 = TR::Compose(R1, R2, {}, {});
  EXPECT(assert_equal(R(q3), R3));

  // Check Retract
  Vector3 v(1e-5, 0, 0);
  Q q4 = TQ::Retract(q3, v);
  R R4 = TR::Retract(R3, v);
  EXPECT(assert_equal(R(q4), R4));

  // Check Between
  Q q5 = TQ::Between(q3, q4);
  R R5 = R3.between(R4);
  EXPECT(assert_equal(R(q5), R5));

  // Check toQuaternion
  EXPECT(assert_equal(q5, R5.toQuaternion()));

  // Check Logmap
  Vector3 vQ = TQ::Logmap(q5);
  Vector3 vR = R::Logmap(R5);
  EXPECT(assert_equal(vQ, vR));

  // Check Local
  vQ = TQ::Local(q3, q4);
  vR = TR::Local(R3, R4);
  EXPECT(assert_equal(vQ, vR));

  // Check Retract/Local of Compose
  Vector3 vQ1 = TQ::Local(q3, TQ::Compose(q1, TQ::Retract(q2, v)));
  Vector3 vR1 = TR::Local(R3, TR::Compose(R1, TR::Retract(R2, v)));
  EXPECT(assert_equal(vQ1, vR1));
  Vector3 vQ2 = TQ::Local(q3, TQ::Compose(q1, TQ::Retract(q2, -v)));
  Vector3 vR2 = TR::Local(R3, TR::Compose(R1, TR::Retract(R2, -v)));
  EXPECT(assert_equal(vQ2, vR2));
  EXPECT(assert_equal<Vector3>((vQ1 - vQ2) / 0.2, (vR1 - vR2) / 0.2));

  // Check Compose Derivatives
  Matrix HQ, HR;
  HQ = numericalDerivative42<Q, Q, Q, OJ, OJ>(TQ::Compose, q1, q2, {}, {});
  HR = numericalDerivative42<R, R, R, OJ, OJ>(TR::Compose, R1, R2, {}, {});
  EXPECT(assert_equal(HQ, HR));

}

#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

