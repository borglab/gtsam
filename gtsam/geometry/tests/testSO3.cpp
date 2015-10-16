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
#include <gtsam/base/testLie.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//******************************************************************************
TEST(SO3 , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<SO3 >));
  BOOST_CONCEPT_ASSERT((IsManifold<SO3 >));
  BOOST_CONCEPT_ASSERT((IsLieGroup<SO3 >));
}

//******************************************************************************
TEST(SO3 , Constructor) {
  SO3 q(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
}

//******************************************************************************
SO3 id;
Vector3 z_axis(0, 0, 1);
SO3 R1(Eigen::AngleAxisd(0.1, z_axis));
SO3 R2(Eigen::AngleAxisd(0.2, z_axis));

//******************************************************************************
TEST(SO3 , Local) {
  Vector3 expected(0, 0, 0.1);
  Vector3 actual = traits<SO3>::Local(R1, R2);
  EXPECT(assert_equal((Vector)expected,actual));
}

//******************************************************************************
TEST(SO3 , Retract) {
  Vector3 v(0, 0, 0.1);
  SO3 actual = traits<SO3>::Retract(R1, v);
  EXPECT(actual.isApprox(R2));
}

//******************************************************************************
TEST(SO3 , Invariants) {
  EXPECT(check_group_invariants(id,id));
  EXPECT(check_group_invariants(id,R1));
  EXPECT(check_group_invariants(R2,id));
  EXPECT(check_group_invariants(R2,R1));

  EXPECT(check_manifold_invariants(id,id));
  EXPECT(check_manifold_invariants(id,R1));
  EXPECT(check_manifold_invariants(R2,id));
  EXPECT(check_manifold_invariants(R2,R1));
}

//******************************************************************************
TEST(SO3 , LieGroupDerivatives) {
  CHECK_LIE_GROUP_DERIVATIVES(id,id);
  CHECK_LIE_GROUP_DERIVATIVES(id,R2);
  CHECK_LIE_GROUP_DERIVATIVES(R2,id);
  CHECK_LIE_GROUP_DERIVATIVES(R2,R1);
}

//******************************************************************************
TEST(SO3 , ChartDerivatives) {
  CHECK_CHART_DERIVATIVES(id,id);
  CHECK_CHART_DERIVATIVES(id,R2);
  CHECK_CHART_DERIVATIVES(R2,id);
  CHECK_CHART_DERIVATIVES(R2,R1);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

