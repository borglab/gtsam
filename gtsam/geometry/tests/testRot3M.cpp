/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRot3.cpp
 * @brief   Unit tests for Rot3 class
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>

#include <boost/math/constants/constants.hpp>

#include <CppUnitLite/TestHarness.h>

#ifndef GTSAM_USE_QUATERNIONS

using namespace std;
using namespace gtsam;

static Rot3 R = Rot3::rodriguez(0.1, 0.4, 0.2);
static Point3 P(0.2, 0.7, -2.0);
static const Matrix I3 = eye(3);

/* ************************************************************************* */
TEST(Rot3, manifold_cayley)
{
  Rot3 gR1 = Rot3::rodriguez(0.1, 0.4, 0.2);
  Rot3 gR2 = Rot3::rodriguez(0.3, 0.1, 0.7);
  Rot3 origin;

  // log behaves correctly
  Vector d12 = gR1.localCoordinates(gR2, Rot3::CAYLEY);
  CHECK(assert_equal(gR2, gR1.retract(d12, Rot3::CAYLEY)));
  Vector d21 = gR2.localCoordinates(gR1, Rot3::CAYLEY);
  CHECK(assert_equal(gR1, gR2.retract(d21, Rot3::CAYLEY)));

  // Check that log(t1,t2)=-log(t2,t1)
  CHECK(assert_equal(d12,-d21));

  // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
  Vector d = (Vector(3) << 0.1, 0.2, 0.3);
  // exp(-d)=inverse(exp(d))
  CHECK(assert_equal(Rot3::Expmap(-d),Rot3::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
  Rot3 R2 = Rot3::Expmap (2 * d);
  Rot3 R3 = Rot3::Expmap (3 * d);
  Rot3 R5 = Rot3::Expmap (5 * d);
  CHECK(assert_equal(R5,R2*R3));
  CHECK(assert_equal(R5,R3*R2));
}

/* ************************************************************************* */
TEST(Rot3, manifold_slow_cayley)
{
  Rot3 gR1 = Rot3::rodriguez(0.1, 0.4, 0.2);
  Rot3 gR2 = Rot3::rodriguez(0.3, 0.1, 0.7);
  Rot3 origin;

  // log behaves correctly
  Vector d12 = gR1.localCoordinates(gR2, Rot3::SLOW_CAYLEY);
  CHECK(assert_equal(gR2, gR1.retract(d12, Rot3::SLOW_CAYLEY)));
  Vector d21 = gR2.localCoordinates(gR1, Rot3::SLOW_CAYLEY);
  CHECK(assert_equal(gR1, gR2.retract(d21, Rot3::SLOW_CAYLEY)));

  // Check that log(t1,t2)=-log(t2,t1)
  CHECK(assert_equal(d12,-d21));

  // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
  Vector d = (Vector(3) << 0.1, 0.2, 0.3);
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

