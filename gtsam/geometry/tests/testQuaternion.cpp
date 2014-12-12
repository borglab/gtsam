/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testQuaternion.cpp
 * @brief  Unit tests for Quaternion, as a GTSAM-adapted Lie Group
 * @author Frank Dellaert
 **/

#include <gtsam/geometry/Quaternion.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Quaternion Q; // Typedef
typedef traits_x<Q>::ChartJacobian QuaternionJacobian;

//******************************************************************************
TEST(Quaternion , Concept) {
  //BOOST_CONCEPT_ASSERT((IsGroup<Quaternion >));
  //BOOST_CONCEPT_ASSERT((IsManifold<Quaternion >));
  BOOST_CONCEPT_ASSERT((IsLieGroup<Quaternion >));
}

//******************************************************************************
TEST(Quaternion , Constructor) {
  Q q(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
}

//******************************************************************************
TEST(Quaternion , Invariants) {
  Q q1(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
  Q q2(Eigen::AngleAxisd(2, Vector3(0, 1, 0)));
  check_group_invariants(q1,q2);
}

//******************************************************************************
TEST(Quaternion , Local) {
  Vector3 z_axis(0, 0, 1);
  Q q1(Eigen::AngleAxisd(0, z_axis));
  Q q2(Eigen::AngleAxisd(0.1, z_axis));
  QuaternionJacobian H1,H2;
  Vector3 expected(0, 0, 0.1);
  Vector3 actual = traits<Q>::Local(q1, q2, H1, H2);
  EXPECT(assert_equal((Vector)expected,actual));
}

//******************************************************************************
TEST(Quaternion , Retract) {
  Vector3 z_axis(0, 0, 1);
  Q q(Eigen::AngleAxisd(0, z_axis));
  Q expected(Eigen::AngleAxisd(0.1, z_axis));
  Vector3 v(0, 0, 0.1);
  QuaternionJacobian Hq,Hv;
  Q actual = traits<Q>::Retract(q, v, Hq, Hv);
  EXPECT(actual.isApprox(expected));
}

//******************************************************************************
TEST(Quaternion , Compose) {
  EXPECT(false);
}

//******************************************************************************
TEST(Quaternion , Between) {
  EXPECT(false);
}

//******************************************************************************
TEST(Quaternion , Inverse) {
  EXPECT(false);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

