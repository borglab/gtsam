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
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RISAM.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;
typedef noiseModel::mEstimator::GemanMcClure GMLoss;
typedef GraduationScheduler GradSch;

/* ************************************************************************* */
TEST(GraduationScheduler, UpdateMuInit) {
  GradSch scheduler(GradSch::MuUpdateStable, 0.1);
  CHECK(assert_equal(0.1, scheduler.updateMuInit(0.0, false), 1e-9));
  CHECK(assert_equal(1.0, scheduler.updateMuInit(1.0, false), 1e-9));

  CHECK(assert_equal(0.0, scheduler.updateMuInit(0.0, true), 1e-9));
  CHECK(assert_equal(0.9, scheduler.updateMuInit(1.0, true), 1e-9));
}

/* ************************************************************************* */
TEST(GraduationScheduler, IsMuConverged) {
  GradSch scheduler(GradSch::MuUpdateStable, 0.1);
  CHECK(!scheduler.isMuConverged(0.5));
  CHECK(scheduler.isMuConverged(1.0));
}

/* ************************************************************************* */
TEST(GraduationScheduler, MuUpdateMcGann2023) {
  // Standard Sequence
  CHECK(assert_equal(0.12, GradSch::MuUpdateMcGann2023(0.0, 0, 0), 1e-9));
  CHECK(assert_equal(0.384, GradSch::MuUpdateMcGann2023(0.12, 0, 1), 1e-9));
  CHECK(assert_equal(0.9648, GradSch::MuUpdateMcGann2023(0.384, 0, 2), 1e-9));
  CHECK(assert_equal(1.0, GradSch::MuUpdateMcGann2023(0.9648, 0, 3), 1e-9));
}

/* ************************************************************************* */
TEST(GraduationScheduler, MuUpdateStable) {
  // Standard Sequence
  CHECK(assert_equal(0.5, GradSch::MuUpdateStable(0.0, 0.0, 0), 1e-9));
  CHECK(assert_equal(0.9, GradSch::MuUpdateStable(0.5, 0.0, 1), 1e-9));
  CHECK(assert_equal(0.95, GradSch::MuUpdateStable(0.9, 0.0, 2), 1e-9));
  CHECK(assert_equal(1.0, GradSch::MuUpdateStable(0.95, 0.0, 3), 1e-9));

  // Sequence with Non-Zero muInit
  CHECK(assert_equal(0.7, GradSch::MuUpdateStable(0.2, 0.0, 0), 1e-9));
  CHECK(assert_equal(1.0, GradSch::MuUpdateStable(0.7, 0.0, 1), 1e-9));

  // Sequence with Large Non-Zero muInit
  CHECK(assert_equal(1.0, GradSch::MuUpdateStable(0.7, 0.0, 0), 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedFactor, Linearize) {
  GMLoss::shared_ptr robustLoss =
      GMLoss::Create(1.0, GMLoss::GradScheme::SCALE_INVARIANT);
  GradSch::shared_ptr scheduler = std::make_shared<GradSch>();

  NonlinearFactor::shared_ptr factor =
      RISAM::MakeGraduated<PriorFactor<double>>(robustLoss, scheduler)(
          0, 0.0, noiseModel::Unit::Create(1));
  gtsam::Values values;

  // Linearize at Weight = 0.5
  values.insert(0, 0.0);
  GaussianFactor::shared_ptr linearFactor = factor->linearize(values);
  auto [A, b] = linearFactor->jacobian();
  CHECK(assert_equal(Matrix::Identity(1, 1) * sqrt(0.5), A, 1e-9));
  CHECK(assert_equal(Vector::Zero(1, 1), b, 1e-9));
}

/* ************************************************************************* */
TEST(RISAMGraduatedFactor, Error) {
  GMLoss::shared_ptr robustLoss =
      GMLoss::Create(1.0, GMLoss::GradScheme::SCALE_INVARIANT);
  GradSch::shared_ptr scheduler = std::make_shared<GradSch>();

  NonlinearFactor::shared_ptr factor =
      RISAM::MakeGraduated<PriorFactor<double>>(robustLoss, scheduler)(
          0, 0.0, noiseModel::Unit::Create(1));
  gtsam::Values values;
  values.insert(0, 1.0);
  CHECK(assert_equal(factor->error(values), 0.25, 1e-9));

  GraduatedFactor::shared_ptr graduatedFactor =
      std::dynamic_pointer_cast<GraduatedFactor>(factor);
  CHECK(assert_equal(graduatedFactor->residual(values), 1.0, 1e-9));
  CHECK(assert_equal(graduatedFactor->robustLoss(values), 0.25, 1e-9));
}

/* ************************************************************************* */
/// @brief Test helper that exposes the protected mu control parameter
class TestableGraduatedPrior
    : public GenericGraduatedFactor<PriorFactor<double>> {
 public:
  typedef GenericGraduatedFactor<PriorFactor<double>> Base;
  using Base::Base;
  void setMu(double mu) { *mu_ = mu; }
  double getMu() const { return *mu_; }
};

/* ************************************************************************* */
TEST(RISAMGraduatedFactor, ErrorMatchesLinearization) {
  GMLoss::shared_ptr robustLoss =
      GMLoss::Create(1.0, GMLoss::GradScheme::SCALE_INVARIANT);
  GradSch::shared_ptr scheduler = std::make_shared<GradSch>();

  TestableGraduatedPrior factor(robustLoss, scheduler, 0, 0.0,
                                noiseModel::Unit::Create(1));
  gtsam::Values values;
  values.insert(0, 1.0);

  VectorValues zeroDelta;
  zeroDelta.insert(0, Vector1::Zero());

  // At mu = 0 the scale-invariant GM loss is exactly quadratic
  factor.setMu(0.0);
  CHECK(assert_equal(factor.error(values),
                     factor.linearize(values)->error(zeroDelta), 1e-9));

  // For mu != 0 the loss is no longer quadratic, so the IRLS surrogate is only
  // tangent to the error rather than equal to it.
  auto errorAt = [&factor](const double& x) {
    gtsam::Values v;
    v.insert(0, x);
    return Vector1(factor.error(v));
  };
  for (const double mu : {0.0, 0.5, 1.0}) {
    factor.setMu(mu);
    const Matrix expectedGradient =
        numericalDerivative11<Vector1, double>(errorAt, 1.0);
    CHECK(assert_equal(expectedGradient(0, 0),
                       factor.linearize(values)->gradientAtZero().at(0)(0),
                       1e-6));
  }

  // For mu != 0 the loss is no longer no longer directly matches the linearized
  factor.setMu(0.5);
  CHECK(std::abs(factor.error(values) -
                 factor.linearize(values)->error(zeroDelta)) > 1e-9);
}

/* ************************************************************************* */
TEST(RISAMGraduatedFactor, ClonePreservesMu) {
  GMLoss::shared_ptr robustLoss =
      GMLoss::Create(1.0, GMLoss::GradScheme::SCALE_INVARIANT);
  GradSch::shared_ptr scheduler = std::make_shared<GradSch>();

  TestableGraduatedPrior factor(robustLoss, scheduler, 0, 0.0,
                                noiseModel::Unit::Create(1));
  gtsam::Values values;
  values.insert(0, 1.0);  // residual == 1.0

  // Advance the factor to an intermediate mu (i.e. not muInit and not
  // converged)
  const double kIntermediateMu = 0.5;
  factor.setMu(kIntermediateMu);
  CHECK(std::abs(kIntermediateMu - scheduler->muInit()) > 1e-9);

  // The clone must behave as if it were graduated to the intermediate mu
  NonlinearFactor::shared_ptr cloned = factor.clone();
  GraduatedFactor::shared_ptr gradClone =
      std::dynamic_pointer_cast<GraduatedFactor>(cloned);
  CHECK(gradClone);

  const double expectedLoss = robustLoss->graduatedLoss(1.0, kIntermediateMu);
  const double expectedWeight =
      robustLoss->graduatedWeight(1.0, kIntermediateMu);

  CHECK(assert_equal(expectedLoss, gradClone->robustLoss(values), 1e-9));
  CHECK(assert_equal(factor.robustLoss(values), gradClone->robustLoss(values),
                     1e-9));

  // The weight used to linearize must match as well
  auto jacobianClone = cloned->linearize(values)->jacobian();
  CHECK(assert_equal(Matrix::Identity(1, 1) * sqrt(expectedWeight),
                     jacobianClone.first, 1e-9));

  // The two mu states must not alias: advancing the original leaves the clone
  // graduated at the mu it was cloned with
  factor.setMu(1.0);
  CHECK(assert_equal(expectedLoss, gradClone->robustLoss(values), 1e-9));
  auto jacobianCloneAfter = cloned->linearize(values)->jacobian();
  CHECK(assert_equal(jacobianClone.first, jacobianCloneAfter.first, 1e-9));
  CHECK(assert_equal(jacobianClone.second, jacobianCloneAfter.second, 1e-9));
}

/* ************************************************************************* */
TEST(RISAM, RISAMIntegrationTest) {
  SharedDiagonal odoNoise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << 0.1, 0.1, M_PI / 100.0).finished());
  SharedDiagonal brNoise =
      noiseModel::Diagonal::Sigmas((Vector(2) << M_PI / 100.0, 0.1).finished());

  // Setup the Solver
  GMLoss::shared_ptr robustLoss =
      GMLoss::Create(GMLoss::ShapeParameterFromInfluenceThreshold(0.1, 3, 0.95),
                     GMLoss::GradScheme::SCALE_INVARIANT);
  GradSch::shared_ptr scheduler = std::make_shared<GradSch>();

  // All landmark measurements share the same graduation policy
  auto makeGraduatedBearingRange =
      RISAM::MakeGraduated<BearingRangeFactor<Pose2, Point2>>(robustLoss,
                                                              scheduler);

  RISAM::Parameters params;
  params.isam2Params = ISAM2Params(
      ISAM2DoglegLineSearchParams(0.02, 1.0, 1.5, 1e-3, false, 1e-4));
  RISAM risam(params);

  // Setup Container for the full problem
  Values fullInit;
  NonlinearFactorGraph fullGraph;

  // i keeps track of the time step
  size_t i = 0;

  // Add a prior at time 0 and update risam
  {
    NonlinearFactorGraph newFactors;
    newFactors.addPrior(0, Pose2(0.0, 0.0, 0.0), odoNoise);
    fullGraph.push_back(newFactors);

    Values init;
    init.insert((0), Pose2(0.01, 0.01, 0.01));
    fullInit.insert((0), Pose2(0.01, 0.01, 0.01));

    risam.update(newFactors, init);
  }

  // Add odometry from time 0 to time 5
  for (; i < 5; ++i) {
    NonlinearFactorGraph newFactors;
    newFactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullGraph.push_back(newFactors);

    Values init;
    init.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));
    fullInit.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));

    risam.update(newFactors, init);
  }

  // Add odometry from time 5 to 6 and landmark measurement at time 5
  {
    NonlinearFactorGraph newFactors;
    newFactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newFactors.push_back(makeGraduatedBearingRange(
        i, 100, Rot2::fromAngle(M_PI / 2.0), 5.0, brNoise));
    newFactors.push_back(makeGraduatedBearingRange(
        i, 101, Rot2::fromAngle(-M_PI / 2.0), 5.0, brNoise));
    fullGraph.push_back(newFactors);

    Values init;
    init.insert((i + 1), Pose2(1.01, 0.01, 0.01));
    init.insert(100, Point2(5.0 / sqrt(2.0), 5.0 / sqrt(2.0)));
    init.insert(101, Point2(5.0 / sqrt(2.0), -5.0 / sqrt(2.0)));
    fullInit.insert((i + 1), Pose2(1.01, 0.01, 0.01));
    fullInit.insert(100, Point2(5.0 / sqrt(2.0), 5.0 / sqrt(2.0)));
    fullInit.insert(101, Point2(5.0 / sqrt(2.0), -5.0 / sqrt(2.0)));

    risam.update(newFactors, init);
    ++i;
  }

  // Add odometry from time 6 to time 10
  for (; i < 10; ++i) {
    NonlinearFactorGraph newFactors;
    newFactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    fullGraph.push_back(newFactors);

    Values init;
    init.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));
    fullInit.insert((i + 1), Pose2(double(i + 1) + 0.1, -0.1, 0.01));

    risam.update(newFactors, init);
  }

  // Add odometry from time 10 to 11 and landmark measurement at time 10
  {
    NonlinearFactorGraph newFactors;
    newFactors.emplace_shared<BetweenFactor<Pose2>>(
        i, i + 1, Pose2(1.0, 0.0, 0.0), odoNoise);
    newFactors.push_back(makeGraduatedBearingRange(
        i, 100, Rot2::fromAngle((3 * M_PI) / 4.0), 7.07106, brNoise));
    newFactors.push_back(makeGraduatedBearingRange(
        i, 101, Rot2::fromAngle(-(3 * M_PI) / 4.0), 7.07106, brNoise));
    fullGraph.push_back(newFactors);

    // Add an Outlier Measurement [is not added to fullGraph]
    newFactors.push_back(
        makeGraduatedBearingRange(i, 100, Rot2::fromAngle(0.4), 300, brNoise));

    Values init;
    init.insert((i + 1), Pose2(double(i + 1) + 0.1, 0.1, 0.01));
    fullInit.insert((i + 1), Pose2(double(i + 1) + 0.1, 0.1, 0.01));

    risam.update(newFactors, init);
    ++i;
  }

  /** Compare Results **/
  // Compute Actual
  risam.update();
  Values actual = risam.calculateEstimate();
  std::set<size_t> actualOutliers = risam.getOutliers(0.95);
  // Compute Expected
  LevenbergMarquardtParams parameters;
  Values expected =
      LevenbergMarquardtOptimizer(fullGraph, fullInit, parameters).optimize();

  // Test Solution Quality
  CHECK(assert_equal(expected, actual, 0.01));
  // Test Outlier Identification
  CHECK(1 == actualOutliers.size());
  CHECK(fullGraph.size() == *(actualOutliers.begin()));
}

/* ************************************************************************ */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************ */
