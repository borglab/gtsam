/**
 * @file testLinearContainerFactor.cpp
 *
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

const gtsam::noiseModel::Diagonal::shared_ptr diag_model2 = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));
const double tol = 1e-5;

gtsam::Key  l1 = 101, l2 = 102, x1 = 1, x2 = 2;

Point2 landmark1(5.0, 1.5), landmark2(7.0, 1.5);
Pose2 poseA1(0.0, 0.0, 0.0), poseA2(2.0, 0.0, 0.0);

/* ************************************************************************* */
TEST(TestLinearContainerFactor, generic_jacobian_factor) {

  Matrix A1 = (Matrix(2, 2) <<
      2.74222, -0.0067457,
      0.0,  2.63624).finished();
  Matrix A2 = (Matrix(2, 2) <<
      -0.0455167, -0.0443573,
      -0.0222154, -0.102489).finished();
  Vector b = Vector2(0.0277052,
      -0.0533393);

  JacobianFactor expLinFactor(l1, A1, l2, A2, b, diag_model2);

  LinearContainerFactor actFactor(expLinFactor);
  EXPECT_LONGS_EQUAL(2, actFactor.size());
  EXPECT(actFactor.isJacobian());
  EXPECT(!actFactor.isHessian());

  // check keys
  EXPECT(assert_container_equality({l1, l2}, actFactor.keys()));

  Values values;
  values.insert(l1, landmark1);
  values.insert(l2, landmark2);
  values.insert(x1, poseA1);
  values.insert(x2, poseA2);

  // Check reconstruction
  GaussianFactor::shared_ptr actLinearizationA = actFactor.linearize(values);
  EXPECT(assert_equal(*expLinFactor.clone(), *actLinearizationA, tol));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, jacobian_factor_withlinpoints) {

  Matrix A1 = (Matrix(2, 2) <<
      2.74222, -0.0067457,
      0.0,  2.63624).finished();
  Matrix A2 = (Matrix(2, 2) <<
      -0.0455167, -0.0443573,
      -0.0222154, -0.102489).finished();
  Vector b = Vector2(0.0277052,
      -0.0533393);

  JacobianFactor expLinFactor(l1, A1, l2, A2, b, diag_model2);

  Values values;
  values.insert(l1, landmark1);
  values.insert(l2, landmark2);
  values.insert(x1, poseA1);
  values.insert(x2, poseA2);

  LinearContainerFactor actFactor(expLinFactor, values);
  LinearContainerFactor actFactorNolin(expLinFactor);

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
  Vector delta_l1 = Vector2(1.0, 2.0);
  Vector delta_l2 = Vector2(3.0, 4.0);

  VectorValues delta = values.zeroVectors();
  delta.at(l1) = delta_l1;
  delta.at(l2) = delta_l2;
  Values noisyValues = values.retract(delta);
  double expError = expLinFactor.error(delta);
  EXPECT_DOUBLES_EQUAL(expError, actFactor.error(noisyValues), tol);
  EXPECT_DOUBLES_EQUAL(expLinFactor.error(values.zeroVectors()), actFactor.error(values), tol);

  // Check linearization with corrections for updated linearization point
  GaussianFactor::shared_ptr actLinearizationB = actFactor.linearize(noisyValues);
  Vector bprime = b - A1 * delta_l1 - A2 * delta_l2;
  JacobianFactor expLinFactor2(l1, A1, l2, A2, bprime, diag_model2);
  EXPECT(assert_equal(*expLinFactor2.clone(), *actLinearizationB, tol));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, generic_hessian_factor) {
  Matrix G11 = (Matrix(1, 1) << 1.0).finished();
  Matrix G12 = (Matrix(1, 2) << 2.0, 4.0).finished();
  Matrix G13 = (Matrix(1, 3) << 3.0, 6.0, 9.0).finished();

  Matrix G22 = (Matrix(2, 2) << 3.0, 5.0,
                            0.0, 6.0).finished();
  Matrix G23 = (Matrix(2, 3) << 4.0, 6.0, 8.0,
                            1.0, 2.0, 4.0).finished();

  Matrix G33 = (Matrix(3, 3) << 1.0, 2.0, 3.0,
                            0.0, 5.0, 6.0,
                            0.0, 0.0, 9.0).finished();

  Vector g1 = (Vector(1) << -7.0).finished();
  Vector g2 = Vector2(-8.0, -9.0);
  Vector g3 = Vector3(1.0,  2.0,  3.0);

  double f = 10.0;

  HessianFactor initFactor(x1, x2, l1,
      G11, G12, G13, g1, G22, G23, g2, G33, g3, f);

  Values values;
  values.insert(l1, landmark1);
  values.insert(l2, landmark2);
  values.insert(x1, poseA1);
  values.insert(x2, poseA2);

  LinearContainerFactor actFactor(initFactor);
  EXPECT(!actFactor.isJacobian());
  EXPECT(actFactor.isHessian());

  GaussianFactor::shared_ptr actLinearization1 = actFactor.linearize(values);
  EXPECT(assert_equal(*initFactor.clone(), *actLinearization1, tol));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, hessian_factor_withlinpoints) {
  // 2 variable example, one pose, one landmark (planar)
  // Initial ordering: x1, l1

  Matrix G11 = (Matrix(3, 3) <<
      1.0, 2.0, 3.0,
      0.0, 5.0, 6.0,
      0.0, 0.0, 9.0).finished();
  Matrix G12 = (Matrix(3, 2) <<
      1.0, 2.0,
      3.0, 5.0,
      4.0, 6.0).finished();
  Vector g1 = Vector3(1.0,  2.0,  3.0);

  Matrix G22 = (Matrix(2, 2) <<
        0.5, 0.2,
        0.0, 0.6).finished();

  Vector g2 = Vector2(-8.0, -9.0);

  double f = 10.0;

  // Construct full matrices
  Matrix G(5,5);
  G << G11, G12, Matrix::Zero(2,3), G22;

  HessianFactor initFactor(x1, l1, G11, G12, g1, G22, g2, f);

  Values linearizationPoint, expLinPoints;
  linearizationPoint.insert(l1, landmark1);
  linearizationPoint.insert(x1, poseA1);
  expLinPoints = linearizationPoint;
  linearizationPoint.insert(x2, poseA2);

  LinearContainerFactor actFactor(initFactor, linearizationPoint);
  EXPECT(!actFactor.isJacobian());
  EXPECT(actFactor.isHessian());

  EXPECT(actFactor.hasLinearizationPoint());
  Values actLinPoint = *actFactor.linearizationPoint();
  EXPECT(assert_equal(expLinPoints, actLinPoint));

  // Create delta
  Vector delta_l1 = Vector2(1.0, 2.0);
  Vector delta_x1 = Vector3(3.0, 4.0, 0.5);
  Vector delta_x2 = Vector3(6.0, 7.0, 0.3);

  // Check error calculation
  VectorValues delta = linearizationPoint.zeroVectors();
  delta.at(l1) = delta_l1;
  delta.at(x1) = delta_x1;
  delta.at(x2) = delta_x2;
  EXPECT(assert_equal((Vector(5) << 3.0, 4.0, 0.5, 1.0, 2.0).finished(), delta.vector(initFactor.keys())));
  Values noisyValues = linearizationPoint.retract(delta);

  double expError = initFactor.error(delta);
  EXPECT_DOUBLES_EQUAL(expError, actFactor.error(noisyValues), tol);
  EXPECT_DOUBLES_EQUAL(initFactor.error(linearizationPoint.zeroVectors()), actFactor.error(linearizationPoint), tol);

  // Compute updated versions
  Vector dv = (Vector(5) << 3.0, 4.0, 0.5, 1.0, 2.0).finished();
  Vector g(5); g << g1, g2;
  Vector g_prime = g - G.selfadjointView<Eigen::Upper>() * dv;

  // Check linearization with corrections for updated linearization point
  Vector g1_prime = g_prime.head(3);
  Vector g2_prime = g_prime.tail(2);
  double f_prime = f + dv.transpose() * G.selfadjointView<Eigen::Upper>() * dv - 2.0 * dv.transpose() * g;
  HessianFactor expNewFactor(x1, l1, G11, G12, g1_prime, G22, g2_prime, f_prime);
  EXPECT(assert_equal(*expNewFactor.clone(), *actFactor.linearize(noisyValues), tol));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, Creation) {
  // Create a set of local keys (No robot label)
  Key  l1 = 11, l3 = 13, l5 = 15;

  // create a linear factor
  SharedDiagonal model = noiseModel::Unit::Create(2);
  JacobianFactor::shared_ptr linear_factor(new JacobianFactor(
      l3, I_2x2, l5, 2.0 * I_2x2, Z_2x1, model));

  // create a set of values - build with full set of values
  gtsam::Values full_values, exp_values;
  full_values.insert(l3, Point2(1.0, 2.0));
  full_values.insert(l5, Point2(4.0, 3.0));
  exp_values = full_values;
  full_values.insert(l1, Point2(3.0, 7.0));

  LinearContainerFactor actual(linear_factor, full_values);

  // Verify the keys
  EXPECT(assert_container_equality({l3, l5}, actual.keys()));

  // Verify subset of linearization points
  EXPECT(assert_equal(exp_values, actual.linearizationPoint(), tol));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, jacobian_relinearize)
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  gtsam::Key key1(1);
  gtsam::Key key2(2);
  gtsam::Values linpoint1;
  linpoint1.insert(key1, gtsam::Point3(-22.4,  +8.5,  +2.4));
  linpoint1.insert(key2, gtsam::Point3(-21.0,  +5.0, +21.0));

  gtsam::Point3 measured(1.0, -2.5, 17.8);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::BetweenFactor<gtsam::Point3> betweenFactor(key1, key2, measured, model);

  // Create a jacobian container factor at linpoint 1
  gtsam::JacobianFactor::shared_ptr jacobian(new gtsam::JacobianFactor(*betweenFactor.linearize(linpoint1)));
  gtsam::LinearContainerFactor jacobianContainer(jacobian, linpoint1);

  // Create a second linearization point
  gtsam::Values linpoint2;
  linpoint2.insert(key1, gtsam::Point3(+18.0, -0.25, +1.11));
  linpoint2.insert(key2, gtsam::Point3(-10.0, +11.2, +0.05));

  // Check the error at linpoint2 versus the original factor
  double expected_error = betweenFactor.error(linpoint2);
  double actual_error = jacobianContainer.error(linpoint2);
  EXPECT_DOUBLES_EQUAL(expected_error, actual_error, 1e-9 );

  // Re-linearize around the new point and check the factors
  gtsam::GaussianFactor::shared_ptr expected_factor = betweenFactor.linearize(linpoint2);
  gtsam::GaussianFactor::shared_ptr actual_factor   = jacobianContainer.linearize(linpoint2);
  CHECK(gtsam::assert_equal(*expected_factor, *actual_factor));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, hessian_relinearize)
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  gtsam::Key key1(1);
  gtsam::Key key2(2);
  gtsam::Values linpoint1;
  linpoint1.insert(key1, gtsam::Point3(-22.4,  +8.5,  +2.4));
  linpoint1.insert(key2, gtsam::Point3(-21.0,  +5.0, +21.0));

  gtsam::Point3 measured(1.0, -2.5, 17.8);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::BetweenFactor<gtsam::Point3> betweenFactor(key1, key2, measured, model);

  // Create a hessian container factor at linpoint 1
  gtsam::HessianFactor::shared_ptr hessian(new gtsam::HessianFactor(*betweenFactor.linearize(linpoint1)));
  gtsam::LinearContainerFactor hessianContainer(hessian, linpoint1);

  // Create a second linearization point
  gtsam::Values linpoint2;
  linpoint2.insert(key1, gtsam::Point3(+18.0, -0.25, +1.11));
  linpoint2.insert(key2, gtsam::Point3(-10.0, +11.2, +0.05));

  // Check the error at linpoint2 versus the original factor
  double expected_error = betweenFactor.error(linpoint2);
  double actual_error = hessianContainer.error(linpoint2);
  EXPECT_DOUBLES_EQUAL(expected_error, actual_error, 1e-9 );

  // Re-linearize around the new point and check the factors
  gtsam::GaussianFactor::shared_ptr expected_factor = gtsam::HessianFactor::shared_ptr(new gtsam::HessianFactor(*betweenFactor.linearize(linpoint2)));
  gtsam::GaussianFactor::shared_ptr actual_factor   = hessianContainer.linearize(linpoint2);
  CHECK(gtsam::assert_equal(*expected_factor, *actual_factor));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, Rekey) {
  // Make an example factor
  auto nonlinear_factor =
      std::make_shared<gtsam::BetweenFactor<gtsam::Point3>>(
          gtsam::Symbol('x', 0), gtsam::Symbol('l', 0), gtsam::Point3(0, 0, 0),
          gtsam::noiseModel::Isotropic::Sigma(3, 1));

  // Linearize and create an LCF
  gtsam::Values linearization_pt;
  linearization_pt.insert(gtsam::Symbol('x', 0), gtsam::Point3(0, 0, 0));
  linearization_pt.insert(gtsam::Symbol('l', 0), gtsam::Point3(0, 0, 0));

  LinearContainerFactor lcf_factor(
      nonlinear_factor->linearize(linearization_pt), linearization_pt);

  // Define a key mapping
  std::map<gtsam::Key, gtsam::Key> key_map;
  key_map[gtsam::Symbol('x', 0)] = gtsam::Symbol('x', 4);
  key_map[gtsam::Symbol('l', 0)] = gtsam::Symbol('l', 4);

  // Rekey (Calls NonlinearFactor::rekey() which should probably be overriden)
  // This of type boost_ptr<NonlinearFactor>
  auto lcf_factor_rekeyed = lcf_factor.rekey(key_map);

  // Cast back to LCF ptr
  LinearContainerFactor::shared_ptr lcf_factor_rekey_ptr =
      boost::static_pointer_cast<LinearContainerFactor>(lcf_factor_rekeyed);
  CHECK(lcf_factor_rekey_ptr);

  // For extra fun lets try linearizing this LCF
  gtsam::Values linearization_pt_rekeyed;
  for (auto key_val : linearization_pt) {
    linearization_pt_rekeyed.insert(key_map.at(key_val.key), key_val.value);
  }

  // Check independent values since we don't want to unnecessarily sort
  // The keys are just in the reverse order wrt the other container
  CHECK(assert_equal(linearization_pt_rekeyed.keys()[1], lcf_factor_rekey_ptr->keys()[0]));
  CHECK(assert_equal(linearization_pt_rekeyed.keys()[0], lcf_factor_rekey_ptr->keys()[1]));
}

/* ************************************************************************* */
TEST(TestLinearContainerFactor, Rekey2) {
  // Make an example factor
  auto nonlinear_factor =
      std::make_shared<gtsam::BetweenFactor<gtsam::Point3>>(
          gtsam::Symbol('x', 0), gtsam::Symbol('l', 0), gtsam::Point3(0, 0, 0),
          gtsam::noiseModel::Isotropic::Sigma(3, 1));

  // Linearize and create an LCF
  gtsam::Values linearization_pt;
  linearization_pt.insert(gtsam::Symbol('x', 0), gtsam::Point3(0, 0, 0));
  linearization_pt.insert(gtsam::Symbol('l', 0), gtsam::Point3(0, 0, 0));

  LinearContainerFactor lcf_factor(
      nonlinear_factor->linearize(linearization_pt), linearization_pt);

  // Define a key mapping with only a single key remapped.
  // This should throw an exception if there is a bug.
  std::map<gtsam::Key, gtsam::Key> key_map;
  key_map[gtsam::Symbol('x', 0)] = gtsam::Symbol('x', 4);

  // Cast back to LCF ptr
  LinearContainerFactor::shared_ptr lcf_factor_rekey_ptr =
      boost::static_pointer_cast<LinearContainerFactor>(
          lcf_factor.rekey(key_map));
  CHECK(lcf_factor_rekey_ptr);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

