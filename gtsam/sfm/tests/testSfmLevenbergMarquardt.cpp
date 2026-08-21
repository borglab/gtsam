/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSfmLevenbergMarquardt.cpp
 * @brief Tests for the public CPU SFM Levenberg-Marquardt optimizer.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/sfm/SfmLevenbergMarquardt.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using symbol_shorthand::P;

namespace {

struct SmallProblem {
  NonlinearFactorGraph graph;
  Values initial;
};

class ExposedSfmOptimizer : public SfmLevenbergMarquardtOptimizer {
 public:
  using LevenbergMarquardtOptimizer::buildDampedSystem;
  using SfmLevenbergMarquardtOptimizer::SfmLevenbergMarquardtOptimizer;
  using SfmLevenbergMarquardtOptimizer::solve;
};

SmallProblem smallProblem(bool hardConstraints = true) {
  const SfmData data =
      SfmData::FromBalFile(findExampleDataFile("dubrovnik-3-7-pre"));
  SmallProblem result{
      hardConstraints ? data.sfmFactorGraph() : data.generalSfmFactors(),
      initialCamerasAndPointsEstimate(data)};
  if (!hardConstraints) {
    result.graph.emplace_shared<PriorFactor<SfmCamera>>(
        0, result.initial.at<SfmCamera>(0),
        noiseModel::Isotropic::Sigma(9, 1e-3));
    result.graph.emplace_shared<PriorFactor<Point3>>(
        P(0), result.initial.at<Point3>(P(0)),
        noiseModel::Isotropic::Sigma(3, 1e-3));
  }

  Vector cameraPerturbation = Vector::Zero(9);
  cameraPerturbation(0) = 1e-2;
  cameraPerturbation(3) = 2e-2;
  result.initial.update(
      1, result.initial.at<SfmCamera>(1).retract(cameraPerturbation));
  result.initial.update(
      P(2), result.initial.at<Point3>(P(2)) + Point3(1e-2, -2e-2, 1e-2));
  return result;
}

SfmLevenbergMarquardtParams parameters(SfmEliminationMode mode) {
  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.setEliminationMode(mode);
  params.setMaxIterations(20);
  params.setRelativeErrorTol(1e-10);
  params.setAbsoluteErrorTol(1e-10);
  return params;
}

double optimizeError(const SmallProblem& problem,
                     SfmLevenbergMarquardtParams params) {
  SfmLevenbergMarquardtOptimizer optimizer(problem.graph, problem.initial,
                                           params);
  optimizer.optimize();
  return optimizer.error();
}

std::string iterativeFailure(
    const SmallProblem& problem, SfmEliminationMode mode,
    const IterativeOptimizationParameters::shared_ptr& iterativeParams) {
  auto params = parameters(mode);
  params.setLinearSolver(NonlinearOptimizerParams::Iterative);
  params.iterativeParams = iterativeParams;
  ExposedSfmOptimizer optimizer(problem.graph, problem.initial, params);
  const auto linear = problem.graph.linearize(problem.initial);
  const GaussianFactorGraph damped =
      optimizer.buildDampedSystem(*linear, VectorValues{});
  try {
    optimizer.solve(damped, optimizer.sfmParams());
  } catch (const std::runtime_error& error) {
    return error.what();
  }
  return {};
}

}  // namespace

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, TypedLinearSolverCompatibility) {
  NonlinearOptimizerParams params;
  params.setLinearSolver(NonlinearOptimizerParams::SEQUENTIAL_QR);
  EXPECT_LONGS_EQUAL(NonlinearOptimizerParams::SEQUENTIAL_QR,
                     params.getLinearSolver());
  EXPECT(params.getLinearSolverType() == "SEQUENTIAL_QR");
  params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
  EXPECT_LONGS_EQUAL(NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                     params.getLinearSolver());
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, FullAndSchurAgree) {
  const SmallProblem problem = smallProblem();
  auto full = parameters(SfmEliminationMode::Full);
  auto schur = parameters(SfmEliminationMode::Schur);
  full.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);
  schur.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);

  const double fullError = optimizeError(problem, full);
  const double schurError = optimizeError(problem, schur);
  EXPECT_DOUBLES_EQUAL(fullError, schurError, 0.2);
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, FullAndSchurDampedStepsAgree) {
  const SmallProblem problem = smallProblem();
  auto fullParams = parameters(SfmEliminationMode::Full);
  auto schurParams = parameters(SfmEliminationMode::Schur);
  fullParams.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);
  schurParams.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);
  fullParams.dampingParams.diagonalDamping = false;
  schurParams.dampingParams.diagonalDamping = false;

  ExposedSfmOptimizer full(problem.graph, problem.initial, fullParams);
  ExposedSfmOptimizer schur(problem.graph, problem.initial, schurParams);
  const auto linear = problem.graph.linearize(problem.initial);
  const GaussianFactorGraph fullDamped =
      full.buildDampedSystem(*linear, VectorValues{});
  const GaussianFactorGraph schurDamped =
      schur.buildDampedSystem(*linear, VectorValues{});
  const VectorValues fullDelta = full.solve(fullDamped, full.sfmParams());
  const VectorValues schurDelta = schur.solve(schurDamped, schur.sfmParams());
  EXPECT(assert_equal(fullDelta, schurDelta, 1e-5));
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, DirectSolverDispatch) {
  const SmallProblem problem = smallProblem(false);
  const std::vector<NonlinearOptimizerParams::LinearSolverType> solvers{
      NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
      NonlinearOptimizerParams::MULTIFRONTAL_QR,
      NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY,
      NonlinearOptimizerParams::SEQUENTIAL_QR,
      NonlinearOptimizerParams::CHOLMOD};

  for (const auto mode :
       {SfmEliminationMode::Full, SfmEliminationMode::Schur}) {
    for (const auto solver : solvers) {
      auto params = parameters(mode);
      params.setMaxIterations(1);
      params.setLinearSolver(solver);
      try {
        const double finalError = optimizeError(problem, params);
        EXPECT(finalError < problem.graph.error(problem.initial));
      } catch (const std::runtime_error& error) {
        if (solver != NonlinearOptimizerParams::CHOLMOD ||
            std::string(error.what()).find("no CHOLMOD support") ==
                std::string::npos) {
          throw;
        }
      }
    }
  }
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, IterativeDispatch) {
  const SmallProblem problem = smallProblem();
  auto params = parameters(SfmEliminationMode::Schur);
  params.setMaxIterations(1);
  params.setLinearSolver(NonlinearOptimizerParams::Iterative);
  auto pcg = std::make_shared<PCGSolverParameters>();
  pcg->preconditioner = std::make_shared<BlockJacobiPreconditionerParameters>();
  pcg->epsilon_rel = 1e-8;
  pcg->epsilon_abs = 1e-8;
  params.iterativeParams = pcg;
  EXPECT(optimizeError(problem, params) < problem.graph.error(problem.initial));
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, IterativeFailuresMatchOrdinaryDispatch) {
  const SmallProblem problem = smallProblem();

  const std::string fullMissing =
      iterativeFailure(problem, SfmEliminationMode::Full, nullptr);
  const std::string schurMissing =
      iterativeFailure(problem, SfmEliminationMode::Schur, nullptr);
  EXPECT(!fullMissing.empty());
  EXPECT(fullMissing == schurMissing);

  const auto unrecognized = std::make_shared<IterativeOptimizationParameters>();
  const std::string fullUnknown =
      iterativeFailure(problem, SfmEliminationMode::Full, unrecognized);
  const std::string schurUnknown =
      iterativeFailure(problem, SfmEliminationMode::Schur, unrecognized);
  EXPECT(!fullUnknown.empty());
  EXPECT(fullUnknown == schurUnknown);
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, CameraOrderingValidation) {
  const SmallProblem problem = smallProblem();
  auto params = parameters(SfmEliminationMode::Schur);

  params.ordering = Ordering{0, 1, P(0)};
  CHECK_EXCEPTION(
      SfmLevenbergMarquardtOptimizer(problem.graph, problem.initial, params),
      std::invalid_argument);

  params.ordering = Ordering{0, 1, 1};
  CHECK_EXCEPTION(
      SfmLevenbergMarquardtOptimizer(problem.graph, problem.initial, params),
      std::invalid_argument);

  params.ordering = Ordering{0, 1, 9999};
  CHECK_EXCEPTION(
      SfmLevenbergMarquardtOptimizer(problem.graph, problem.initial, params),
      std::invalid_argument);

  params.ordering.reset();
  SfmLevenbergMarquardtOptimizer automatic(problem.graph, problem.initial,
                                           params);
  EXPECT_LONGS_EQUAL(3, automatic.sfmParams().ordering->size());
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, CreatesReusablePointFirstOrdering) {
  const SmallProblem problem = smallProblem();
  const Ordering cameras = SfmLevenbergMarquardtOptimizer::CreateCameraOrdering(
      problem.graph, problem.initial);
  const Ordering full =
      SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(problem.graph,
                                                               cameras);

  EXPECT_LONGS_EQUAL(3, cameras.size());
  EXPECT_LONGS_EQUAL(problem.graph.keys().size(), full.size());
  const size_t pointCount = full.size() - cameras.size();
  EXPECT(std::is_sorted(full.begin(), full.begin() + pointCount));
  EXPECT(std::equal(cameras.begin(), cameras.end(), full.begin() + pointCount));

  auto params = parameters(SfmEliminationMode::Schur);
  params.setOrdering(cameras);
  SfmLevenbergMarquardtOptimizer optimizer(problem.graph, problem.initial,
                                           params);
  EXPECT(optimizer.sfmParams().ordering->equals(cameras));
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, FusedMultifrontalCompletesCameraOrdering) {
  const SmallProblem problem = smallProblem();
  auto params = parameters(SfmEliminationMode::Schur);
  params.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER);
  const Ordering cameras{2, 1, 0};
  params.setOrdering(cameras);

  SfmLevenbergMarquardtOptimizer optimizer(problem.graph, problem.initial,
                                           params);
  EXPECT(optimizer.sfmParams().ordering->equals(cameras));

  const Ordering& internal = *optimizer.params().ordering;
  EXPECT_LONGS_EQUAL(problem.graph.keys().size(), internal.size());
  const KeySet keys = problem.graph.keys();
  Ordering expectedPoints;
  for (Key key : keys) {
    if (std::find(cameras.begin(), cameras.end(), key) == cameras.end()) {
      expectedPoints.push_back(key);
    }
  }
  EXPECT(std::equal(expectedPoints.begin(), expectedPoints.end(),
                    internal.begin()));
  EXPECT(std::equal(cameras.begin(), cameras.end(),
                    internal.end() - cameras.size()));
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, DefaultsCloneAndEquality) {
  const SfmLevenbergMarquardtParams defaults;
  EXPECT(defaults.getEliminationMode() == SfmEliminationMode::Full);
  EXPECT_LONGS_EQUAL(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                     defaults.getLinearSolver());
  const SfmLevenbergMarquardtParams legacy =
      SfmLevenbergMarquardtParams::legacyDefaults();
  const SfmLevenbergMarquardtParams ceres =
      SfmLevenbergMarquardtParams::ceresDefaults();
  EXPECT(legacy.getEliminationMode() == SfmEliminationMode::Full);
  EXPECT(ceres.getEliminationMode() == SfmEliminationMode::Full);
  EXPECT_LONGS_EQUAL(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                     legacy.getLinearSolver());
  EXPECT_LONGS_EQUAL(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                     ceres.getLinearSolver());
  auto changed = defaults;
  changed.setEliminationMode(SfmEliminationMode::Schur);
  EXPECT(!defaults.equals(changed));
  const auto clone = changed.clone();
  const auto typed =
      std::dynamic_pointer_cast<SfmLevenbergMarquardtParams>(clone);
  EXPECT(typed != nullptr);
  EXPECT(typed->getEliminationMode() == SfmEliminationMode::Schur);
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, DefaultCreatesFastPointFirstOrdering) {
  const SmallProblem problem = smallProblem();
  const Ordering cameras = SfmLevenbergMarquardtOptimizer::CreateCameraOrdering(
      problem.graph, problem.initial);
  const Ordering expected =
      SfmLevenbergMarquardtOptimizer::CreatePointFirstOrdering(problem.graph,
                                                               cameras);

  SfmLevenbergMarquardtOptimizer optimizer(problem.graph, problem.initial);
  EXPECT(optimizer.sfmParams().getEliminationMode() ==
         SfmEliminationMode::Full);
  EXPECT_LONGS_EQUAL(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                     optimizer.sfmParams().getLinearSolver());
  EXPECT(optimizer.sfmParams().ordering->equals(expected));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
