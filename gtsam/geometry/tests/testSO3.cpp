/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testQuaternion.cpp
 * @brief  Unit tests for SO3, as a GTSAM-adapted Lie Group
 * @author Frank Dellaert
 **/

#include <gtsam/geometry/SO3.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef OptionalJacobian<3,3> SO3Jacobian;

#if 0
//******************************************************************************
TEST(SO3 , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<SO3 >));
  BOOST_CONCEPT_ASSERT((IsManifold<SO3 >));
//  BOOST_CONCEPT_ASSERT((IsLieGroup<SO3 >));
}

//******************************************************************************
TEST(SO3 , Constructor) {
  SO3 q(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
}

//******************************************************************************
TEST(SO3 , Invariants) {
  SO3 q1(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
  SO3 q2(Eigen::AngleAxisd(2, Vector3(0, 1, 0)));
  // group::check_invariants(q1,q2); Does not satisfy Testable concept (yet!)
}

//******************************************************************************
TEST(SO3 , Local) {
  Vector3 z_axis(0, 0, 1);
  SO3 q1(Eigen::AngleAxisd(0, z_axis));
  SO3 q2(Eigen::AngleAxisd(0.1, z_axis));
  SO3Jacobian H1,H2;
  Vector3 expected(0, 0, 0.1);
  Vector3 actual = traits_x<SO3>::Local(q1, q2, H1, H2);
  EXPECT(assert_equal((Vector)expected,actual));
}

//******************************************************************************
TEST(SO3 , Retract) {
  Vector3 z_axis(0, 0, 1);
  SO3 q(Eigen::AngleAxisd(0, z_axis));
  SO3 expected(Eigen::AngleAxisd(0.1, z_axis));
  Vector3 v(0, 0, 0.1);
  SO3 actual = traits_x<SO3>::Retract(q, v);
  EXPECT(actual.isApprox(expected));
}

#endif

//******************************************************************************
TEST(SO3 , Compose) {
  EXPECT(false);
}

//******************************************************************************
TEST(SO3 , Between) {
  EXPECT(false);
}

//******************************************************************************
TEST(SO3 , Inverse) {
  EXPECT(false);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

