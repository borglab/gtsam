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
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/constrained/NonlinearEquality.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/sfm/SfmLevenbergMarquardt.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::P;
using symbol_shorthand::X;

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

SmallProblem globalCalibrationProblem() {
  const std::vector<Pose3> poses{
      Pose3(), Pose3(Rot3::Ypr(0.03, -0.02, 0.01), Point3(1.0, 0.0, 0.0))};
  const std::vector<Point3> points{
      Point3(-1.0, -0.5, 5.0), Point3(0.8, -0.6, 4.5), Point3(-0.7, 0.9, 5.5),
      Point3(1.1, 0.8, 6.0),   Point3(0.2, 0.1, 4.0),  Point3(-0.2, 0.4, 6.5)};
  const Cal3_S2 calibration(500.0, 505.0, 0.0, 320.0, 240.0);
  const Key calibrationKey = Symbol('k', 0);
  const auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

  SmallProblem result;
  for (size_t i = 0; i < poses.size(); ++i) {
    result.initial.insert(X(i), poses[i]);
    const PinholeCamera<Cal3_S2> camera(poses[i], calibration);
    for (size_t j = 0; j < points.size(); ++j) {
      result.graph.emplace_shared<GeneralSFMFactor2<Cal3_S2>>(
          camera.project(points[j]), measurementNoise, X(i), L(j),
          calibrationKey);
    }
  }
  for (size_t j = 0; j < points.size(); ++j) {
    result.initial.insert(L(j), points[j] + Point3(0.01, -0.02, 0.01));
  }
  result.initial.insert(calibrationKey,
                        Cal3_S2(510.0, 495.0, 0.0, 318.0, 242.0));

  result.graph.emplace_shared<PriorFactor<Pose3>>(
      X(0), poses[0], noiseModel::Isotropic::Sigma(6, 1e-3));
  result.graph.emplace_shared<PriorFactor<Point3>>(
      L(0), points[0], noiseModel::Isotropic::Sigma(3, 1e-3));
  result.graph.emplace_shared<PriorFactor<Cal3_S2>>(
      calibrationKey, calibration,
      noiseModel::Diagonal::Sigmas(Vector5{50.0, 50.0, 0.1, 10.0, 10.0}));
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
TEST(SfmLevenbergMarquardt, SchurOrderingValidation) {
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
TEST(SfmLevenbergMarquardt, ReducedSystemIncludesGlobalCalibration) {
  const SmallProblem problem = globalCalibrationProblem();
  const Key calibrationKey = Symbol('k', 0);
  const Ordering reduced =
      SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(problem.graph,
                                                            problem.initial);

  EXPECT_LONGS_EQUAL(3, reduced.size());
  EXPECT(std::find(reduced.begin(), reduced.end(), X(0)) != reduced.end());
  EXPECT(std::find(reduced.begin(), reduced.end(), X(1)) != reduced.end());
  EXPECT(std::find(reduced.begin(), reduced.end(), calibrationKey) !=
         reduced.end());
  for (size_t i = 0; i < 6; ++i) {
    EXPECT(std::find(reduced.begin(), reduced.end(), L(i)) == reduced.end());
  }

  for (const auto solver : {NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                            NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY}) {
    auto full = parameters(SfmEliminationMode::Full);
    auto schur = parameters(SfmEliminationMode::Schur);
    full.setLinearSolver(solver);
    schur.setLinearSolver(solver);
    const double fullError = optimizeError(problem, full);
    const double schurError = optimizeError(problem, schur);
    EXPECT_DOUBLES_EQUAL(fullError, schurError, 1e-5);
  }
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, SchurEliminatesPoint3AndUnit3Only) {
  const Key pointKey = Symbol('l', 0);
  const Key directionKey = Symbol('u', 0);
  const Key poseKey = Symbol('x', 0);
  const Key calibrationKey = Symbol('k', 0);
  NonlinearFactorGraph graph;
  Values values;

  const Point3 point(0.0, 0.0, 5.0);
  const Unit3 direction(0.0, 0.0, 1.0);
  const Pose3 pose;
  const Cal3_S2 calibration(500.0, 500.0, 0.0, 320.0, 240.0);
  values.insert(pointKey, point + Point3(0.1, -0.1, 0.1));
  values.insert(directionKey, Unit3(0.1, 0.0, 1.0));
  values.insert(poseKey, Pose3(Rot3(), Point3(0.1, 0.0, 0.0)));
  values.insert(calibrationKey, Cal3_S2(510.0, 490.0, 0.0, 318.0, 242.0));
  graph.emplace_shared<PriorFactor<Point3>>(pointKey, point,
                                            noiseModel::Unit::Create(3));
  graph.emplace_shared<PriorFactor<Unit3>>(directionKey, direction,
                                           noiseModel::Unit::Create(2));
  graph.emplace_shared<PriorFactor<Pose3>>(poseKey, pose,
                                           noiseModel::Unit::Create(6));
  graph.emplace_shared<PriorFactor<Cal3_S2>>(calibrationKey, calibration,
                                             noiseModel::Unit::Create(5));

  const Ordering reduced =
      SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(graph, values);
  EXPECT_LONGS_EQUAL(2, reduced.size());
  EXPECT(std::find(reduced.begin(), reduced.end(), poseKey) != reduced.end());
  EXPECT(std::find(reduced.begin(), reduced.end(), calibrationKey) !=
         reduced.end());
  EXPECT(std::find(reduced.begin(), reduced.end(), pointKey) == reduced.end());
  EXPECT(std::find(reduced.begin(), reduced.end(), directionKey) ==
         reduced.end());

  const Ordering schur =
      SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(graph, reduced);
  EXPECT_LONGS_EQUAL(pointKey, schur[0]);
  EXPECT_LONGS_EQUAL(directionKey, schur[1]);

  auto params = parameters(SfmEliminationMode::Schur);
  params.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);
  SfmLevenbergMarquardtOptimizer optimizer(graph, values, params);
  const double initialError = graph.error(values);
  optimizer.optimize();
  EXPECT(optimizer.error() < initialError);
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, CreatesReusableSchurOrdering) {
  const SmallProblem problem = smallProblem();
  const Ordering reduced =
      SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(problem.graph,
                                                            problem.initial);
  const Ordering schur = SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(
      problem.graph, reduced);

  EXPECT_LONGS_EQUAL(3, reduced.size());
  EXPECT_LONGS_EQUAL(problem.graph.keys().size(), schur.size());
  const size_t pointCount = schur.size() - reduced.size();
  EXPECT(std::is_sorted(schur.begin(), schur.begin() + pointCount));
  EXPECT(
      std::equal(reduced.begin(), reduced.end(), schur.begin() + pointCount));

  auto params = parameters(SfmEliminationMode::Schur);
  params.setOrdering(reduced);
  SfmLevenbergMarquardtOptimizer optimizer(problem.graph, problem.initial,
                                           params);
  EXPECT(optimizer.sfmParams().ordering->equals(reduced));
}

/* ************************************************************************* */
TEST(SfmLevenbergMarquardt, FusedMultifrontalCompletesSchurOrdering) {
  const SmallProblem problem = smallProblem();
  auto params = parameters(SfmEliminationMode::Schur);
  params.setLinearSolver(NonlinearOptimizerParams::MULTIFRONTAL_SOLVER);
  const Ordering reduced{2, 1, 0};
  params.setOrdering(reduced);

  SfmLevenbergMarquardtOptimizer optimizer(problem.graph, problem.initial,
                                           params);
  EXPECT(optimizer.sfmParams().ordering->equals(reduced));

  const Ordering& internal = *optimizer.params().ordering;
  EXPECT_LONGS_EQUAL(problem.graph.keys().size(), internal.size());
  const KeySet keys = problem.graph.keys();
  Ordering expectedEliminated;
  for (Key key : keys) {
    if (std::find(reduced.begin(), reduced.end(), key) == reduced.end()) {
      expectedEliminated.push_back(key);
    }
  }
  EXPECT(std::equal(expectedEliminated.begin(), expectedEliminated.end(),
                    internal.begin()));
  EXPECT(std::equal(reduced.begin(), reduced.end(),
                    internal.end() - reduced.size()));
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
TEST(SfmLevenbergMarquardt, DefaultCreatesFastSchurOrdering) {
  const SmallProblem problem = smallProblem();
  const Ordering reduced =
      SfmLevenbergMarquardtOptimizer::CreateReducedOrdering(problem.graph,
                                                            problem.initial);
  const Ordering expected = SfmLevenbergMarquardtOptimizer::CreateSchurOrdering(
      problem.graph, reduced);

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
