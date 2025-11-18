/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRot3M.cpp
 * @brief   Unit tests for Rot3 class, matrix version
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

#ifndef GTSAM_USE_QUATERNIONS

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Rot3)
GTSAM_CONCEPT_MATRIX_LIE_GROUP_INST(Rot3)

static Rot3 R = Rot3::Rodrigues(0.1, 0.4, 0.2);
static Point3 P(0.2, 0.7, -2.0);

/* ************************************************************************* */
TEST(Rot3, manifold_cayley)
{
  Rot3 gR1 = Rot3::Rodrigues(0.1, 0.4, 0.2);
  Rot3 gR2 = Rot3::Rodrigues(0.3, 0.1, 0.7);
  Rot3 origin;

  // log behaves correctly
  Vector d12 = gR1.localCayley(gR2);
  CHECK(assert_equal(gR2, gR1.retractCayley(d12)));
  Vector d21 = gR2.localCayley(gR1);
  CHECK(assert_equal(gR1, gR2.retractCayley(d21)));

  // Check that log(t1,t2)=-log(t2,t1)
  CHECK(assert_equal(d12,-d21));

  // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
  Vector d = Vector3(0.1, 0.2, 0.3);
  // exp(-d)=inverse(exp(d))
  CHECK(assert_equal(Rot3::Expmap(-d),Rot3::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
  Rot3 R2 = Rot3::Expmap (2 * d);
  Rot3 R3 = Rot3::Expmap (3 * d);
  Rot3 R5 = Rot3::Expmap (5 * d);
  CHECK(assert_equal(R5,R2*R3));
  CHECK(assert_equal(R5,R3*R2));
}

#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

