/**
 * @file testLinearContainerFactor.cpp
 *
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/TestableAssertions.h>
#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

const gtsam::noiseModel::Diagonal::shared_ptr diag_model2 = noiseModel::Diagonal::Sigmas(Vector_(2, 1.0, 1.0));
const double tol = 1e-5;

gtsam::Key  l1 = 101, l2 = 102, x1 = 1, x2 = 2;

Point2 landmark1(5.0, 1.5), landmark2(7.0, 1.5);
Pose2 poseA1(0.0, 0.0, 0.0), poseA2(2.0, 0.0, 0.0);

/* ************************************************************************* */
TEST( testLinearContainerFactor, generic_jacobian_factor ) {

  Ordering initOrdering; initOrdering += x1, x2, l1, l2;

  Matrix A1 = Matrix_(2,2,
      2.74222, -0.0067457,
      0.0,  2.63624);
  Matrix A2 = Matrix_(2,2,
      -0.0455167, -0.0443573,
      -0.0222154, -0.102489);
  Vector b = Vector_(2, 0.0277052,
      -0.0533393);

  JacobianFactor expLinFactor(initOrdering[l1], A1, initOrdering[l2], A2, b, diag_model2);

  LinearContainerFactor actFactor(expLinFactor, initOrdering);
  EXPECT_LONGS_EQUAL(2, actFactor.size());
  EXPECT(actFactor.isJacobian());
  EXPECT(!actFactor.isHessian());

  // check keys
  std::vector<gtsam::Key> expKeys; expKeys += l1, l2;
  EXPECT(assert_container_equality(expKeys, actFactor.keys()));

  Values values;
  values.insert(l1, landmark1);
  values.insert(l2, landmark2);
  values.insert(x1, poseA1);
  values.insert(x2, poseA2);

  // Check reconstruction from same ordering
  GaussianFactor::shared_ptr actLinearizationA = actFactor.linearize(values, initOrdering);
  EXPECT(assert_equal(*expLinFactor.clone(), *actLinearizationA, tol));

  // Check reconstruction from new ordering
  Ordering newOrdering; newOrdering += x1, l1, x2, l2;
  GaussianFactor::shared_ptr actLinearizationB = actFactor.linearize(values, newOrdering);
  JacobianFactor expLinFactor2(newOrdering[l1], A1, newOrdering[l2], A2, b, diag_model2);
  EXPECT(assert_equal(*expLinFactor2.clone(), *actLinearizationB, tol));
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, jacobian_factor_withlinpoints ) {

  Ordering ordering; ordering += x1, x2, l1, l2;

  Matrix A1 = Matrix_(2,2,
      2.74222, -0.0067457,
      0.0,  2.63624);
  Matrix A2 = Matrix_(2,2,
      -0.0455167, -0.0443573,
      -0.0222154, -0.102489);
  Vector b = Vector_(2, 0.0277052,
      -0.0533393);

  JacobianFactor expLinFactor(ordering[l1], A1, ordering[l2], A2, b, diag_model2);

  Values values;
  values.insert(l1, landmark1);
  values.insert(l2, landmark2);
  values.insert(x1, poseA1);
  values.insert(x2, poseA2);

  LinearContainerFactor actFactor(expLinFactor, ordering, values);
  LinearContainerFactor actFactorNolin(expLinFactor, ordering);

  EXPECT(assert_equal(actFactor, actFactor, tol));
  EXPECT(assert_inequal(actFactor, actFactorNolin, tol));
  EXPECT(assert_inequal(actFactorNolin, actFactor, tol));

  // Check contents
  Values expLinPoint;
  expLinPoint.insert(l1, landmark1);
  expLinPoint.insert(l2, landmark2);
  CHECK(actFactor.linearizationPoint());
  EXPECT(actFactor.hasLinearizationPoint());
  EXPECT(assert_equal(expLinPoint, *actFactor.linearizationPoint()));

  // Check error evaluation
  Vector delta_l1 = Vector_(2, 1.0, 2.0);
  Vector delta_l2 = Vector_(2, 3.0, 4.0);

  VectorValues delta = values.zeroVectors(ordering);
  delta.at(ordering[l1]) = delta_l1;
  delta.at(ordering[l2]) = delta_l2;
  Values noisyValues = values.retract(delta, ordering);
  double expError = expLinFactor.error(delta);
  EXPECT_DOUBLES_EQUAL(expError, actFactor.error(noisyValues), tol);
  EXPECT_DOUBLES_EQUAL(expLinFactor.error(values.zeroVectors(ordering)), actFactor.error(values), tol);

  // Check linearization with corrections for updated linearization point
  Ordering newOrdering; newOrdering += x1, l1, x2, l2;
  GaussianFactor::shared_ptr actLinearizationB = actFactor.linearize(noisyValues, newOrdering);
  Vector bprime = b - A1 * delta_l1 - A2 * delta_l2;
  JacobianFactor expLinFactor2(newOrdering[l1], A1, newOrdering[l2], A2, bprime, diag_model2);
  EXPECT(assert_equal(*expLinFactor2.clone(), *actLinearizationB, tol));
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, generic_hessian_factor ) {
  Matrix G11 = Matrix_(1,1, 1.0);
  Matrix G12 = Matrix_(1,2, 2.0, 4.0);
  Matrix G13 = Matrix_(1,3, 3.0, 6.0, 9.0);

  Matrix G22 = Matrix_(2,2, 3.0, 5.0,
                            0.0, 6.0);
  Matrix G23 = Matrix_(2,3, 4.0, 6.0, 8.0,
                            1.0, 2.0, 4.0);

  Matrix G33 = Matrix_(3,3, 1.0, 2.0, 3.0,
                            0.0, 5.0, 6.0,
                            0.0, 0.0, 9.0);

  Vector g1 = Vector_(1, -7.0);
  Vector g2 = Vector_(2, -8.0, -9.0);
  Vector g3 = Vector_(3,  1.0,  2.0,  3.0);

  double f = 10.0;

  Ordering initOrdering; initOrdering += x1, x2, l1, l2;
  HessianFactor initFactor(initOrdering[x1], initOrdering[x2], initOrdering[l1],
      G11, G12, G13, g1, G22, G23, g2, G33, g3, f);

  Values values;
  values.insert(l1, landmark1);
  values.insert(l2, landmark2);
  values.insert(x1, poseA1);
  values.insert(x2, poseA2);

  LinearContainerFactor actFactor(initFactor, initOrdering);
  EXPECT(!actFactor.isJacobian());
  EXPECT(actFactor.isHessian());

  GaussianFactor::shared_ptr actLinearization1 = actFactor.linearize(values, initOrdering);
  EXPECT(assert_equal(*initFactor.clone(), *actLinearization1, tol));

  Ordering newOrdering; newOrdering += l1, x1, x2, l2;
  HessianFactor expLinFactor(newOrdering[x1], newOrdering[x2], newOrdering[l1],
      G11, G12, G13, g1, G22, G23, g2, G33, g3, f);
  GaussianFactor::shared_ptr actLinearization2 = actFactor.linearize(values, newOrdering);
   EXPECT(assert_equal(*expLinFactor.clone(), *actLinearization2, tol));
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, hessian_factor_withlinpoints ) {
  // 2 variable example, one pose, one landmark (planar)
  // Initial ordering: x1, l1

  Matrix G11 = Matrix_(3,3,
      1.0, 2.0, 3.0,
      0.0, 5.0, 6.0,
      0.0, 0.0, 9.0);
  Matrix G12 = Matrix_(3,2,
      1.0, 2.0,
      3.0, 5.0,
      4.0, 6.0);
  Vector g1 = Vector_(3,  1.0,  2.0,  3.0);

  Matrix G22 = Matrix_(2,2,
        0.5, 0.2,
        0.0, 0.6);

  Vector g2 = Vector_(2, -8.0, -9.0);

  double f = 10.0;

  // Construct full matrices
  Matrix G(5,5);
  G << G11, G12, Matrix::Zero(2,3), G22;

  Ordering ordering; ordering += x1, x2, l1;
  HessianFactor initFactor(ordering[x1], ordering[l1], G11, G12, g1, G22, g2, f);

  Values linearizationPoint, expLinPoints;
  linearizationPoint.insert(l1, landmark1);
  linearizationPoint.insert(x1, poseA1);
  expLinPoints = linearizationPoint;
  linearizationPoint.insert(x2, poseA2);

  LinearContainerFactor actFactor(initFactor, ordering, linearizationPoint);
  EXPECT(!actFactor.isJacobian());
  EXPECT(actFactor.isHessian());

  EXPECT(actFactor.hasLinearizationPoint());
  Values actLinPoint = *actFactor.linearizationPoint();
  EXPECT(assert_equal(expLinPoints, actLinPoint));

  // Create delta
  Vector delta_l1 = Vector_(2, 1.0, 2.0);
  Vector delta_x1 = Vector_(3, 3.0, 4.0, 0.5);
  Vector delta_x2 = Vector_(3, 6.0, 7.0, 0.3);

  // Check error calculation
  VectorValues delta = linearizationPoint.zeroVectors(ordering);
  delta.at(ordering[l1]) = delta_l1;
  delta.at(ordering[x1]) = delta_x1;
  delta.at(ordering[x2]) = delta_x2;
  EXPECT(assert_equal(Vector_(5, 3.0, 4.0, 0.5, 1.0, 2.0), delta.vector(initFactor.keys())));
  Values noisyValues = linearizationPoint.retract(delta, ordering);

  double expError = initFactor.error(delta);
  EXPECT_DOUBLES_EQUAL(expError, actFactor.error(noisyValues), tol);
  EXPECT_DOUBLES_EQUAL(initFactor.error(linearizationPoint.zeroVectors(ordering)), actFactor.error(linearizationPoint), tol);

  // Compute updated versions
  Vector dv = Vector_(5, 3.0, 4.0, 0.5, 1.0, 2.0);
  Vector g(5); g << g1, g2;
  Vector g_prime = g - G.selfadjointView<Eigen::Upper>() * dv;

  // Check linearization with corrections for updated linearization point
  Vector g1_prime = g_prime.head(3);
  Vector g2_prime = g_prime.tail(2);
  double f_prime = f + dv.transpose() * G.selfadjointView<Eigen::Upper>() * dv - 2.0 * dv.transpose() * g;
  HessianFactor expNewFactor(ordering[x1], ordering[l1], G11, G12, g1_prime, G22, g2_prime, f_prime);
  EXPECT(assert_equal(*expNewFactor.clone(), *actFactor.linearize(noisyValues, ordering), tol));
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, creation ) {
  // Create a set of local keys (No robot label)
  Key  l1 = 11, l2 = 12,
      l3 = 13, l4 = 14,
      l5 = 15, l6 = 16,
      l7 = 17, l8 = 18;

  // creating an ordering to decode the linearized factor
  Ordering ordering;
  ordering += l1,l2,l3,l4,l5,l6,l7,l8;

  // create a linear factor
  SharedDiagonal model = noiseModel::Unit::Create(2);
  JacobianFactor::shared_ptr linear_factor(new JacobianFactor(
      ordering[l3], eye(2,2), ordering[l5], 2.0 * eye(2,2), zero(2), model));

  // create a set of values - build with full set of values
  gtsam::Values full_values, exp_values;
  full_values.insert(l3, Point2(1.0, 2.0));
  full_values.insert(l5, Point2(4.0, 3.0));
  exp_values = full_values;
  full_values.insert(l1, Point2(3.0, 7.0));

  LinearContainerFactor actual(linear_factor, ordering, full_values);

  // Verify the keys
  std::vector<gtsam::Key> expKeys;
  expKeys += l3, l5;
  EXPECT(assert_container_equality(expKeys, actual.keys()));

  // Verify subset of linearization points
  EXPECT(assert_equal(exp_values, actual.linearizationPoint(), tol));
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, jacobian_relinearize )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  gtsam::Key key1(1);
  gtsam::Key key2(2);
  gtsam::Ordering ordering;
  ordering.push_back(key1);
  ordering.push_back(key2);
  gtsam::Values linpoint1;
  linpoint1.insert(key1, gtsam::Point3(-22.4,  +8.5,  +2.4));
  linpoint1.insert(key2, gtsam::Point3(-21.0,  +5.0, +21.0));

  gtsam::Point3 measured(1.0, -2.5, 17.8);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::BetweenFactor<gtsam::Point3> betweenFactor(key1, key2, measured, model);

  // Create a jacobian container factor at linpoint 1
  gtsam::JacobianFactor::shared_ptr jacobian(new gtsam::JacobianFactor(*betweenFactor.linearize(linpoint1, ordering)));
  gtsam::LinearContainerFactor jacobianContainer(jacobian, ordering, linpoint1);

  // Create a second linearization point
  gtsam::Values linpoint2;
  linpoint2.insert(key1, gtsam::Point3(+18.0, -0.25, +1.11));
  linpoint2.insert(key2, gtsam::Point3(-10.0, +11.2, +0.05));

  // Check the error at linpoint2 versus the original factor
  double expected_error = betweenFactor.error(linpoint2);
  double actual_error = jacobianContainer.error(linpoint2);
  EXPECT_DOUBLES_EQUAL(expected_error, actual_error, 1e-9 );

  // Re-linearize around the new point and check the factors
  gtsam::GaussianFactor::shared_ptr expected_factor = betweenFactor.linearize(linpoint2, ordering);
  gtsam::GaussianFactor::shared_ptr actual_factor   = jacobianContainer.linearize(linpoint2, ordering);
  CHECK(gtsam::assert_equal(*expected_factor, *actual_factor));
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, hessian_relinearize )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  gtsam::Key key1(1);
  gtsam::Key key2(2);
  gtsam::Ordering ordering;
  ordering.push_back(key1);
  ordering.push_back(key2);
  gtsam::Values linpoint1;
  linpoint1.insert(key1, gtsam::Point3(-22.4,  +8.5,  +2.4));
  linpoint1.insert(key2, gtsam::Point3(-21.0,  +5.0, +21.0));

  gtsam::Point3 measured(1.0, -2.5, 17.8);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::BetweenFactor<gtsam::Point3> betweenFactor(key1, key2, measured, model);

  // Create a hessian container factor at linpoint 1
  gtsam::HessianFactor::shared_ptr hessian(new gtsam::HessianFactor(*betweenFactor.linearize(linpoint1, ordering)));
  gtsam::LinearContainerFactor hessianContainer(hessian, ordering, linpoint1);

  // Create a second linearization point
  gtsam::Values linpoint2;
  linpoint2.insert(key1, gtsam::Point3(+18.0, -0.25, +1.11));
  linpoint2.insert(key2, gtsam::Point3(-10.0, +11.2, +0.05));

  // Check the error at linpoint2 versus the original factor
  double expected_error = betweenFactor.error(linpoint2);
  double actual_error = hessianContainer.error(linpoint2);
  EXPECT_DOUBLES_EQUAL(expected_error, actual_error, 1e-9 );

  // Re-linearize around the new point and check the factors
  gtsam::GaussianFactor::shared_ptr expected_factor = gtsam::HessianFactor::shared_ptr(new gtsam::HessianFactor(*betweenFactor.linearize(linpoint2, ordering)));
  gtsam::GaussianFactor::shared_ptr actual_factor   = hessianContainer.linearize(linpoint2, ordering);
  CHECK(gtsam::assert_equal(*expected_factor, *actual_factor));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

