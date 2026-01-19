/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testRISAM.cpp
 *  @brief Unit tests for RISAM and supporting classes
 *  @author Dan McGann
 *  @date January 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/RISAM.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, error) {
  SIGKernel kernel(1.0);
  // At zero error is not dependent on mu
  CHECK(assert_equal(0.0, kernel.error(0.0, 0.0), 1e-9));
  CHECK(assert_equal(0.0, kernel.error(0.0, 0.5), 1e-9));
  CHECK(assert_equal(0.0, kernel.error(0.0, 1.0), 1e-9));

  // For Mu = 0.0 error is quadratic
  CHECK(assert_equal(0.0025, kernel.error(0.1, 0.0), 1e-9));
  CHECK(assert_equal(0.4225, kernel.error(1.3, 0.0), 1e-9));
  CHECK(assert_equal(38.750625, kernel.error(12.45, 0.0), 1e-9));

  // For 0.0 < Mu Error depends on shape
  CHECK(assert_equal(0.00454545454, kernel.error(0.1, 0.5), 1e-9));
  CHECK(assert_equal(0.36739130434, kernel.error(1.3, 0.5), 1e-9));
  CHECK(assert_equal(5.76217472119, kernel.error(12.45, 0.5), 1e-9));

  // For Mu == 1.0 error depends is Geman-Mcclure
  CHECK(assert_equal(0.00495049504, kernel.error(0.1, 1.0), 1e-9));
  CHECK(assert_equal(0.31412639405, kernel.error(1.3, 1.0), 1e-9));
  CHECK(assert_equal(0.49679492315, kernel.error(12.45, 1.0), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, weight) {
  SIGKernel kernel(1.0);
  // At Mu = 0 weights are identical at 0.5
  CHECK(assert_equal(0.5, kernel.weight(0.1, 0.0), 1e-9));
  CHECK(assert_equal(0.5, kernel.weight(1.3, 0.0), 1e-9));
  CHECK(assert_equal(0.5, kernel.weight(12.45, 0.0), 1e-9));

  // At Mu = 1 weights are higher for low error
  CHECK(assert_equal(0.9802960494, kernel.weight(0.1, 1.0), 1e-9));
  CHECK(assert_equal(0.13819598955, kernel.weight(1.3, 1.0), 1e-9));
  CHECK(assert_equal(0.00004109007, kernel.weight(12.45, 1.0), 1e-9));

  // At Mu = 1 large residuals have ~0 weight
  CHECK(assert_equal(0.0, kernel.weight(2000.0, 1.0), 1e-9));

  // Across Mu residual of 0 will have weight = 1
  CHECK(assert_equal(1.0, kernel.weight(0.0, 0.1), 1e-9));
  CHECK(assert_equal(1.0, kernel.weight(0.0, 0.5), 1e-9));
  CHECK(assert_equal(1.0, kernel.weight(0.0, 0.8), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, weightSystem) {
  SIGKernel kernel(1.0);
  vector<Matrix> A;
  A.push_back(Matrix::Identity(3, 3));
  Vector b = Vector::Ones(3);

  kernel.weightSystem(A, b, 0.1, 0.0);
  CHECK(assert_equal(Matrix::Identity(3, 3) * sqrt(0.5), A[0], 1e-9));
  CHECK(assert_equal(Vector::Ones(3) * sqrt(0.5), b, 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, muInitInc) {
  SIGKernel kernel(1.0, SIGKernel::muUpdateStable, 0.1);
  CHECK(assert_equal(0.1, kernel.incrementMuInit(0.0), 1e-9));
  CHECK(assert_equal(1.0, kernel.incrementMuInit(1.0), 1e-9));

  CHECK(assert_equal(0.0, kernel.incrementMuInitInv(0.0), 1e-9));
  CHECK(assert_equal(0.9, kernel.incrementMuInitInv(1.0), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, isMuConverged) {
  SIGKernel kernel(1.0, SIGKernel::muUpdateStable, 0.1);
  CHECK(!kernel.isMuConverged(0.5));
  CHECK(kernel.isMuConverged(1.0));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, muUpdateMcGann2023) {
  // Standard Sequence
  CHECK(assert_equal(0.12, SIGKernel::muUpdateMcGann2023(0.0, 0.0, 0), 1e-9));
  CHECK(assert_equal(0.384, SIGKernel::muUpdateMcGann2023(0.12, 0.0, 1), 1e-9));
  CHECK(
      assert_equal(0.9648, SIGKernel::muUpdateMcGann2023(0.384, 0.0, 2), 1e-9));
  CHECK(assert_equal(1.0, SIGKernel::muUpdateMcGann2023(0.9648, 0.0, 3), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, muUpdateStable) {
  // Standard Sequence
  CHECK(assert_equal(0.5, SIGKernel::muUpdateStable(0.0, 0.0, 0), 1e-9));
  CHECK(assert_equal(0.9, SIGKernel::muUpdateStable(0.5, 0.0, 1), 1e-9));
  CHECK(assert_equal(0.95, SIGKernel::muUpdateStable(0.9, 0.0, 2), 1e-9));
  CHECK(assert_equal(1.0, SIGKernel::muUpdateStable(0.95, 0.0, 3), 1e-9));

  // Sequence with Non-Zero mu_init
  CHECK(assert_equal(0.7, SIGKernel::muUpdateStable(0.2, 0.0, 0), 1e-9));
  CHECK(assert_equal(1.0, SIGKernel::muUpdateStable(0.7, 0.0, 1), 1e-9));

  // Sequence with Large Non-Zero mu_init
  CHECK(assert_equal(1.0, SIGKernel::muUpdateStable(0.7, 0.0, 0), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedFactor, linearize) {
  SIGKernel::shared_ptr kernel = std::make_shared<SIGKernel>(1.0);
  NonlinearFactor::shared_ptr factor =
      RISAM::make_graduated<PriorFactor<double>>(kernel, 0, 0.0,
                                                 noiseModel::Unit::Create(1));
  gtsam::Values values;

  // Linearize at Weight = 0.5
  values.insert(0, 0.0);
  GaussianFactor::shared_ptr lin_factor = factor->linearize(values);
  auto [A, b] = lin_factor->jacobian();
  CHECK(assert_equal(Matrix::Identity(1, 1) * sqrt(0.5), A, 1e-9));
  CHECK(assert_equal(Vector::Zero(1, 1), b, 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedFactor, error) {
  SIGKernel::shared_ptr kernel = std::make_shared<SIGKernel>(1.0);
  NonlinearFactor::shared_ptr factor =
      RISAM::make_graduated<PriorFactor<double>>(kernel, 0, 0.0,
                                                 noiseModel::Unit::Create(1));
  gtsam::Values values;
  values.insert(0, 1.0);
  CHECK(assert_equal(factor->error(values), 0.03125, 1e-9));

  GraduatedFactor::shared_ptr grad_factor =
      std::dynamic_pointer_cast<GraduatedFactor>(factor);
  CHECK(assert_equal(grad_factor->residual(values), 1.0, 1e-9));
  CHECK(assert_equal(grad_factor->robustResidual(values), 0.25, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
