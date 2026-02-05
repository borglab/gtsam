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
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RISAM.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, Error) {
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
TEST(RISAMGraduatedKernel, Weight) {
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
TEST(RISAMGraduatedKernel, WeightSystem) {
  SIGKernel kernel(1.0);
  vector<Matrix> A;
  A.push_back(Matrix::Identity(3, 3));
  Vector b = Vector::Ones(3);

  kernel.weightSystem(A, b, 0.1, 0.0);
  CHECK(assert_equal(Matrix::Identity(3, 3) * sqrt(0.5), A[0], 1e-9));
  CHECK(assert_equal(Vector::Ones(3) * sqrt(0.5), b, 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, MuInitInc) {
  SIGKernel kernel(1.0, SIGKernel::muUpdateStable, 0.1);
  CHECK(assert_equal(0.1, kernel.incrementMuInit(0.0), 1e-9));
  CHECK(assert_equal(1.0, kernel.incrementMuInit(1.0), 1e-9));

  CHECK(assert_equal(0.0, kernel.incrementMuInitInv(0.0), 1e-9));
  CHECK(assert_equal(0.9, kernel.incrementMuInitInv(1.0), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, IsMuConverged) {
  SIGKernel kernel(1.0, SIGKernel::muUpdateStable, 0.1);
  CHECK(!kernel.isMuConverged(0.5));
  CHECK(kernel.isMuConverged(1.0));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, MuUpdateMcGann2023) {
  // Standard Sequence
  CHECK(assert_equal(0.12, SIGKernel::muUpdateMcGann2023(0.0, 0, 0), 1e-9));
  CHECK(assert_equal(0.384, SIGKernel::muUpdateMcGann2023(0.12, 0, 1), 1e-9));
  CHECK(assert_equal(0.9648, SIGKernel::muUpdateMcGann2023(0.384, 0, 2), 1e-9));
  CHECK(assert_equal(1.0, SIGKernel::muUpdateMcGann2023(0.9648, 0, 3), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedKernel, MuUpdateStable) {
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
TEST(RISAMGraduatedFactor, Linearize) {
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
TEST(RISAMGraduatedFactor, Error) {
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
TEST(RISAM, RISAMIntegrationTest) {
  SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << 0.1, 0.1, M_PI / 100.0).finished());
  SharedDiagonal brNoise =
      noiseModel::Diagonal::Sigmas((Vector(2) << M_PI / 100.0, 0.1).finished());

  // Setup the Solver
  SIGKernel::shared_ptr kernel = std::make_shared<SIGKernel>(
      SIGKernel::shapeParamFromInfThresh(0.1, 3, 0.95));
  RISAM::Parameters params;
  params.isam2_params = ISAM2Params(
      ISAM2DoglegLineSearchParams(0.02, 1.0, 1.5, 1e-3, 1e-4, false));
  RISAM risam(params);

  // Setup Container for the full problem
  Values fullinit;
  NonlinearFactorGraph fullgraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update risam
  {
    NonlinearFactorGraph newfactors;
    newfactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((0), Pose2(0.01, 0.01, 0.01));
    fullinit.insert((0), Pose2(0.01, 0.01, 0.01));

    risam.update(newfactors, init);
  }

  // Add odometry from time 0 to time 5
  for (; i < 5; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));
    fullinit.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));

    risam.update(newfactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.push_back(
        RISAM::make_graduated<BearingRangeFactor<Pose2, Point2>>(
            kernel, i, 100, Rot2::fromAngle(M_PI / 2.0), 5.0, brNoise));
    newfactors.push_back(
        RISAM::make_graduated<BearingRangeFactor<Pose2, Point2>>(
            kernel, i, 101, Rot2::fromAngle(-M_PI / 2.0), 5.0, brNoise));
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i + 1), Pose2(1.01, 0.01, 0.01));
    init.insert(100, Point2(5.0 / sqrt(2.0), 5.0 / sqrt(2.0)));
    init.insert(101, Point2(5.0 / sqrt(2.0), -5.0 / sqrt(2.0)));
    fullinit.insert((i + 1), Pose2(1.01, 0.01, 0.01));
    fullinit.insert(100, Point2(5.0 / sqrt(2.0), 5.0 / sqrt(2.0)));
    fullinit.insert(101, Point2(5.0 / sqrt(2.0), -5.0 / sqrt(2.0)));

    risam.update(newfactors, init);
    ++i;
  }

  // Add odometry from time 6 to time 10
  for (; i < 10; ++i) {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullgraph.push_back(newfactors);

    Values init;
    init.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));
    fullinit.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));

    risam.update(newfactors, init);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    NonlinearFactorGraph newfactors;
    newfactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newfactors.push_back(
        RISAM::make_graduated<BearingRangeFactor<Pose2, Point2>>(
            kernel, i, 100, Rot2::fromAngle((3 * M_PI) / 4.0), 7.07106,
            brNoise));
    newfactors.push_back(
        RISAM::make_graduated<BearingRangeFactor<Pose2, Point2>>(
            kernel, i, 101, Rot2::fromAngle(-(3 * M_PI) / 4.0), 7.07106,
            brNoise));
    fullgraph.push_back(newfactors);

    // Add an Outlier Measurement [is not added to fullGraph]
    newfactors.push_back(
        RISAM::make_graduated<BearingRangeFactor<Pose2, Point2>>(
            kernel, i, 100, Rot2::fromAngle(0.4), 300, brNoise));

    Values init;
    init.insert((i + 1), Pose2(double(i + 1) + 0.1, 0.1, 0.01));
    fullinit.insert((i + 1), Pose2(double(i + 1) + 0.1, 0.1, 0.01));

    risam.update(newfactors, init);
    ++i;
  }

  /** Compare Results **/
  // Compute Actual
  risam.update();
  Values actual = risam.calculateEstimate();
  std::set<size_t> actual_outliers = risam.getOutliers(0.95);
  // Compute Expected
  LevenbergMarquardtParams parameters;
  Values expected =
      LevenbergMarquardtOptimizer(fullgraph, fullinit, parameters).optimize();

  // Test Solution Quality
  CHECK(assert_equal(expected, actual, 0.01));
  // Test Outlier Identification
  CHECK(1 == actual_outliers.size());
  CHECK(fullgraph.size() == *(actual_outliers.begin()));
}

/* *************************************************************************
 */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* *************************************************************************
 */
