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
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Quaternion Q; // Typedef
typedef traits<Q>::ChartJacobian QuaternionJacobian;

//******************************************************************************
TEST(Quaternion , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<Quaternion >));
  BOOST_CONCEPT_ASSERT((IsManifold<Quaternion >));
  BOOST_CONCEPT_ASSERT((IsLieGroup<Quaternion >));
}

//******************************************************************************
TEST(Quaternion , Constructor) {
  Q q(Eigen::AngleAxisd(1, Vector3(0, 0, 1)));
}

//******************************************************************************
TEST(Quaternion , Logmap) {
  Q q1(5e-06, 0, 0, 1), q2(-5e-06, 0, 0, -1);
  Vector3 v1 = traits<Q>::Logmap(q1);
  Vector3 v2 = traits<Q>::Logmap(q2);
  EXPECT(assert_equal(v1, v2));
}

//******************************************************************************
TEST(Quaternion , Local) {
  Vector3 z_axis(0, 0, 1);
  Q q1(Eigen::AngleAxisd(0, z_axis));
  Q q2(Eigen::AngleAxisd(0.1, z_axis));
  QuaternionJacobian H1, H2;
  Vector3 expected(0, 0, 0.1);
  Vector3 actual = traits<Q>::Local(q1, q2, H1, H2);
  EXPECT(assert_equal((Vector )expected, actual));
}

//******************************************************************************
TEST(Quaternion , Retract) {
  Vector3 z_axis(0, 0, 1);
  Q q(Eigen::AngleAxisd(0, z_axis));
  Q expected(Eigen::AngleAxisd(0.1, z_axis));
  Vector3 v(0, 0, 0.1);
  QuaternionJacobian Hq, Hv;
  Q actual = traits<Q>::Retract(q, v, Hq, Hv);
  EXPECT(actual.isApprox(expected));
}

//******************************************************************************
TEST(Quaternion , Compose) {
  Vector3 z_axis(0, 0, 1);
  Q q1(Eigen::AngleAxisd(0.2, z_axis));
  Q q2(Eigen::AngleAxisd(0.1, z_axis));

  Q expected = q1 * q2;
  Q actual = traits<Q>::Compose(q1, q2);
  EXPECT(traits<Q>::Equals(expected, actual));
}

//******************************************************************************
Vector3 Q_z_axis(0, 0, 1);
Q id(Eigen::AngleAxisd(0, Q_z_axis));
Q R1(Eigen::AngleAxisd(1, Q_z_axis));
Q R2(Eigen::AngleAxisd(2, Vector3(0, 1, 0)));

//******************************************************************************
TEST(Quaternion , Between) {
  Vector3 z_axis(0, 0, 1);
  Q q1(Eigen::AngleAxisd(0.2, z_axis));
  Q q2(Eigen::AngleAxisd(0.1, z_axis));

  Q expected = q1.inverse() * q2;
  Q actual = traits<Q>::Between(q1, q2);
  EXPECT(traits<Q>::Equals(expected, actual));
}

//******************************************************************************
TEST(Quaternion , Inverse) {
  Vector3 z_axis(0, 0, 1);
  Q q1(Eigen::AngleAxisd(0.1, z_axis));
  Q expected(Eigen::AngleAxisd(-0.1, z_axis));

  Q actual = traits<Q>::Inverse(q1);
  EXPECT(traits<Q>::Equals(expected, actual));
}

//******************************************************************************
TEST(Quaternion , Invariants) {
  EXPECT(check_group_invariants(id, id));
  EXPECT(check_group_invariants(id, R1));
  EXPECT(check_group_invariants(R2, id));
  EXPECT(check_group_invariants(R2, R1));

  EXPECT(check_manifold_invariants(id, id));
  EXPECT(check_manifold_invariants(id, R1));
  EXPECT(check_manifold_invariants(R2, id));
  EXPECT(check_manifold_invariants(R2, R1));
}

//******************************************************************************
TEST(Quaternion , LieGroupDerivatives) {
  CHECK_LIE_GROUP_DERIVATIVES(id, id);
  CHECK_LIE_GROUP_DERIVATIVES(id, R2);
  CHECK_LIE_GROUP_DERIVATIVES(R2, id);
  CHECK_LIE_GROUP_DERIVATIVES(R2, R1);
}

//******************************************************************************
TEST(Quaternion , ChartDerivatives) {
  CHECK_CHART_DERIVATIVES(id, id);
  CHECK_CHART_DERIVATIVES(id, R2);
  CHECK_CHART_DERIVATIVES(R2, id);
  CHECK_CHART_DERIVATIVES(R2, R1);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

