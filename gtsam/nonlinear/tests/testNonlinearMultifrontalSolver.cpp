/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testNonlinearMultifrontalSolver.cpp
 * @brief Test for NonlinearMultifrontalSolver
 * @author Frank Dellaert
 * @date   January 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearMultifrontalSolver.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <tests/smallExample.h>

#include <cmath>
#include <stdexcept>

using namespace std;
using namespace gtsam;

using symbol_shorthand::P;
using symbol_shorthand::X;

namespace {
struct SolverTestProblem {
  NonlinearFactorGraph graph;
  Values values;
  Ordering ordering;
};

SolverTestProblem makeTwoPoseProblem(const Point2& between = Point2(2, 0)) {
  SolverTestProblem problem;
  auto priorModel = noiseModel::Isotropic::Sigma(2, 1.0);
  auto model = noiseModel::Isotropic::Sigma(2, 1.0);

  problem.graph.emplace_shared<PriorFactor<Point2>>(X(1), Point2(0, 0),
                                                    priorModel);
  problem.graph.emplace_shared<BetweenFactor<Point2>>(X(1), X(2), between,
                                                      model);

  problem.values.insert(X(1), Point2(0, 0));
  problem.values.insert(X(2), Point2(0, 0));

  problem.ordering.push_back(X(1));
  problem.ordering.push_back(X(2));
  return problem;
}

// Helper to run solver iterations
void runIterations(const NonlinearFactorGraph& graph,
                   NonlinearMultifrontalSolver& solver, Values& values,
                   size_t iterations, double lambda = 0.0,
                   GaussianFactorGraph::shared_ptr initialLinear = {}) {
  for (size_t i = 0; i < iterations; ++i) {
    GaussianFactorGraph::shared_ptr linear;
    if (i == 0 && initialLinear) {
      linear = initialLinear;
    } else {
      linear = graph.linearize(values);
    }
    solver.eliminateInPlace(*linear, lambda);
    VectorValues delta = solver.updateSolution();
    values = values.retract(delta);
  }
}

}  // namespace

/* ************************************************************************* */
// One linearization + one elimination + one back-solve on a tiny 2-pose
// problem. Tests the basic happy-path flow: construct -> load ->
// eliminateInPlace -> updateSolution, and verifies the expected single-step
// update.
TEST(NonlinearMultifrontalSolver, OneStep) {
  auto problem = makeTwoPoseProblem(Point2(1, 0));
  auto linear = problem.graph.linearize(problem.values);

  NonlinearMultifrontalSolver solver(problem.graph, problem.values,
                                     problem.ordering);

  // 4. Load and eliminate
  solver.eliminateInPlace(*linear, 0.0);

  // 5. Solve and Update
  VectorValues delta = solver.updateSolution();
  Values result = problem.values.retract(delta);

  // 6. Check results
  EXPECT(assert_equal(Point2(0, 0), result.at<Point2>(X(1)), 1e-9));
  EXPECT(assert_equal(Point2(1, 0), result.at<Point2>(X(2)), 1e-9));
}

/* ************************************************************************* */
// Precompute the symbolic structure (junction tree, dimensions, fixed keys)
// from a nonlinear graph and initial values.
// Tests that Precompute succeeds and produces consistent sizing information.
TEST(NonlinearMultifrontalSolver, OneStepPrecomputed) {
  auto problem = makeTwoPoseProblem(Point2(1, 0));

  auto data = NonlinearMultifrontalSolver::Precompute(
      problem.graph, problem.values, problem.ordering);
  EXPECT(data.dims.count(X(1)));
  EXPECT(data.dims.count(X(2)));
  EXPECT(data.fixedKeys.empty());

  auto linear = problem.graph.linearize(problem.values);
  NonlinearMultifrontalSolver solver(problem.graph, problem.values,
                                     problem.ordering);
  solver.eliminateInPlace(*linear, {});

  VectorValues delta = solver.updateSolution();
  Values result = problem.values.retract(delta);

  EXPECT(assert_equal(Point2(0, 0), result.at<Point2>(X(1)), 1e-9));
  EXPECT(assert_equal(Point2(1, 0), result.at<Point2>(X(2)), 1e-9));
}

/* ************************************************************************* */
TEST(NonlinearMultifrontalSolver, PrecomputeHardConstraint) {
  NonlinearFactorGraph graph;
  Values values;
  Ordering ordering;

  graph.emplace_shared<NonlinearEquality<Point2>>(X(1), Point2(1.0, 2.0));
  values.insert(X(1), Point2(0.0, 0.0));
  ordering.push_back(X(1));

  auto data = NonlinearMultifrontalSolver::Precompute(graph, values, ordering);
  CHECK_EQUAL(1, data.fixedKeys.size());
}

/* ************************************************************************* */
TEST(NonlinearMultifrontalSolver, PrecomputeSoftConstraintThrows) {
  NonlinearFactorGraph graph;
  Values values;
  Ordering ordering;

  graph.emplace_shared<NonlinearEquality<Point2>>(X(1), Point2(1.0, 2.0), 10.0);
  values.insert(X(1), Point2(0.0, 0.0));
  ordering.push_back(X(1));

  CHECK_EXCEPTION(
      NonlinearMultifrontalSolver::Precompute(graph, values, ordering),
      std::runtime_error);
}

/* ************************************************************************* */
// Run a first elimination to get an updated estimate, then deliberately
// perturb the estimate, re-linearize, reload, and eliminate again.
// Tests that load() correctly overwrites numeric data for subsequent solves.
TEST(NonlinearMultifrontalSolver, ReLinearize) {
  auto problem = makeTwoPoseProblem();
  auto linear = problem.graph.linearize(problem.values);
  NonlinearMultifrontalSolver solver(problem.graph, problem.values,
                                     problem.ordering);

  // 4. First Step
  solver.eliminateInPlace(*linear, {});
  VectorValues delta1 = solver.updateSolution();
  Values currentValues = problem.values.retract(delta1);

  // Expect X(2) to be (2,0) because it's linear P2 problem
  EXPECT(assert_equal(Point2(2, 0), currentValues.at<Point2>(X(2)), 1e-9));

  // 5. Disturb values to check re-linearization logic
  currentValues.update(X(2), Point2(1.0, 0.0));

  // 6. Re-linearize
  auto linear2 = problem.graph.linearize(currentValues);
  solver.eliminateInPlace(*linear2, {});  // Re-linearize and eliminate.
  VectorValues delta2 = solver.updateSolution();
  Values actual = currentValues.retract(delta2);

  // Expect X(2) to differ by delta = 1.0, so back to 2.0
  EXPECT(assert_equal(Point2(2, 0), actual.at<Point2>(X(2)), 1e-9));
}

/* ************************************************************************* */
// Repeatedly linearize at the current estimate and run eliminate/solve to
// convergence on a small problem (Gauss-Newton-style iterations).
// Tests that repeated load/eliminate/update cycles behave consistently.
TEST(NonlinearMultifrontalSolver, MultiIteration) {
  auto problem = makeTwoPoseProblem();
  auto linear = problem.graph.linearize(problem.values);
  NonlinearMultifrontalSolver solver(problem.graph, problem.values,
                                     problem.ordering);
  constexpr size_t kIterations = 5;
  runIterations(problem.graph, solver, problem.values, kIterations, 0, linear);

  EXPECT(assert_equal(Point2(0, 0), problem.values.at<Point2>(X(1)), 1e-9));
  EXPECT(assert_equal(Point2(2, 0), problem.values.at<Point2>(X(2)), 1e-9));
}

/* ************************************************************************* */
// Run multiple iterations with LM-style damping where damping is applied to
// the diagonal of the frontal Hessian blocks.
// Tests damped elimination and the diagonal-damping code path.
TEST(NonlinearMultifrontalSolver, DampedIterationsDiagonal) {
  auto problem = makeTwoPoseProblem();
  NonlinearMultifrontalSolver::DampingParams params;
  params.diagonalDamping = true;
  auto linear = problem.graph.linearize(problem.values);

  MultifrontalSolver::Parameters mfParams;
  mfParams.mergeDimCap = 0;
  mfParams.qrMode = MultifrontalParameters::QRMode::Off;
  NonlinearMultifrontalSolver solver(problem.graph, problem.values,
                                     problem.ordering, mfParams, params);
  constexpr size_t kIterations = 10;
  runIterations(problem.graph, solver, problem.values, kIterations, 0.1,
                linear);

  EXPECT(assert_equal(Point2(0, 0), problem.values.at<Point2>(X(1)), 1e-6));
  EXPECT(assert_equal(Point2(2, 0), problem.values.at<Point2>(X(2)), 1e-3));
}

/* ************************************************************************* */
// Run multiple iterations with LM-style damping where damping is applied as
// an identity term on the frontal blocks (full damping).
// Tests damped elimination and the identity-damping code path.
TEST(NonlinearMultifrontalSolver, DampedIterationsFull) {
  auto problem = makeTwoPoseProblem();
  NonlinearMultifrontalSolver::DampingParams params;
  params.diagonalDamping = false;
  auto linear = problem.graph.linearize(problem.values);

  MultifrontalSolver::Parameters mfParams;
  mfParams.mergeDimCap = 0;
  mfParams.qrMode = MultifrontalParameters::QRMode::Off;
  NonlinearMultifrontalSolver solver(problem.graph, problem.values,
                                     problem.ordering, mfParams, params);
  constexpr size_t kIterations = 10;
  runIterations(problem.graph, solver, problem.values, kIterations, 0.1,
                linear);

  EXPECT(assert_equal(Point2(0, 0), problem.values.at<Point2>(X(1)), 1e-6));
  EXPECT(assert_equal(Point2(2, 0), problem.values.at<Point2>(X(2)), 1e-3));
}

/* ************************************************************************* */
// Compare QR vs Cholesky on a single-clique elimination with damping.
// This is expected to fail until QR supports LM-style damping.
TEST(NonlinearMultifrontalSolver, DampedSingleCliqueQRVsCholesky) {
  auto problem = makeTwoPoseProblem();
  auto linear = problem.graph.linearize(problem.values);

  MultifrontalSolver::Parameters qrParams;
  qrParams.mergeDimCap = 0;
  qrParams.qrMode = MultifrontalParameters::QRMode::Force;
  NonlinearMultifrontalSolver qrSolver(problem.graph, problem.values,
                                       problem.ordering, qrParams);

  MultifrontalSolver::Parameters choleskyParams;
  choleskyParams.mergeDimCap = 0;
  choleskyParams.qrMode = MultifrontalParameters::QRMode::Off;
  NonlinearMultifrontalSolver choleskySolver(problem.graph, problem.values,
                                             problem.ordering, choleskyParams);

  const double lambda = 1e-3;
  choleskySolver.eliminateInPlace(*linear, lambda);
  const VectorValues choleskyDelta = choleskySolver.updateSolution();

  qrSolver.eliminateInPlace(*linear, lambda);
  const VectorValues qrDelta = qrSolver.updateSolution();
  EXPECT(assert_equal(choleskyDelta, qrDelta, 1e-9));
}

/* ************************************************************************* */
// Forced QR and Cholesky match for a damped smoother linearization across
// identity, diagonal, and exact diagonal damping variants.
TEST(NonlinearMultifrontalSolver, DampedSmootherQRMatchesCholesky) {
  const double lambda = 1e-2;
  auto [nlfg, poses] = example::createNonlinearSmoother(7);
  poses.update(X(1), Point2(1.1, 0.2));
  auto linear = nlfg.linearize(poses);
  const Ordering ordering{X(1), X(3), X(5), X(7), X(2), X(6), X(4)};

  const auto solve =
      [](const NonlinearFactorGraph& graph, const Values& values,
         const Ordering& ordering, const GaussianFactorGraph& linear,
         MultifrontalParameters::QRMode qrMode,
         const NonlinearMultifrontalSolver::DampingParams& dampingParams,
         double lambda) {
        MultifrontalSolver::Parameters params;
        params.mergeDimCap = 0;
        params.qrMode = qrMode;
        NonlinearMultifrontalSolver solver(graph, values, ordering, params,
                                           dampingParams);
        solver.eliminateInPlace(linear, lambda);
        return solver.updateSolution();
      };

  NonlinearMultifrontalSolver::DampingParams identity;
  identity.diagonalDamping = false;
  VectorValues identityQr =
      solve(nlfg, poses, ordering, *linear,
            MultifrontalParameters::QRMode::Force, identity, lambda);
  VectorValues identityCholesky =
      solve(nlfg, poses, ordering, *linear, MultifrontalParameters::QRMode::Off,
            identity, lambda);
  EXPECT(assert_equal(identityCholesky, identityQr, 1e-9));

  NonlinearMultifrontalSolver::DampingParams diagonal;
  diagonal.diagonalDamping = true;
  diagonal.exactHessianDiagonal = false;
  VectorValues diagonalQr =
      solve(nlfg, poses, ordering, *linear,
            MultifrontalParameters::QRMode::Force, diagonal, lambda);
  VectorValues diagonalCholesky =
      solve(nlfg, poses, ordering, *linear, MultifrontalParameters::QRMode::Off,
            diagonal, lambda);
  EXPECT(assert_equal(diagonalCholesky, diagonalQr, 1e-9));

  NonlinearMultifrontalSolver::DampingParams exact;
  exact.diagonalDamping = true;
  exact.exactHessianDiagonal = true;
  VectorValues exactQr =
      solve(nlfg, poses, ordering, *linear,
            MultifrontalParameters::QRMode::Force, exact, lambda);
  VectorValues exactCholesky =
      solve(nlfg, poses, ordering, *linear, MultifrontalParameters::QRMode::Off,
            exact, lambda);
  EXPECT(assert_equal(exactCholesky, exactQr, 1e-9));
}

/* ************************************************************************* */
// Load a linearized graph once and then eliminate multiple times with
// different damping values (lambda), without reloading. This models common LM
// behavior where a fixed linearization is probed with several lambdas.
TEST(NonlinearMultifrontalSolver, ProbeMultipleLambdasSameLinearization) {
  auto problem = makeTwoPoseProblem(Point2(2, 0));
  auto linear = problem.graph.linearize(problem.values);

  NonlinearMultifrontalSolver::DampingParams dampingParams;
  dampingParams.diagonalDamping = false;

  MultifrontalSolver::Parameters mfParams;
  mfParams.mergeDimCap = 0;
  mfParams.qrMode = MultifrontalParameters::QRMode::Off;
  NonlinearMultifrontalSolver solver(problem.graph, problem.values,
                                     problem.ordering, mfParams, dampingParams);

  solver.load(*linear);

  const double lambdaSmall = 1e-3;
  solver.eliminateInPlace(lambdaSmall);
  const VectorValues deltaSmall = solver.updateSolution();

  const double lambdaLarge = 1e3;
  solver.eliminateInPlace(lambdaLarge);
  const VectorValues deltaLarge = solver.updateSolution();

  EXPECT(deltaLarge.norm() < deltaSmall.norm());
}

/* ************************************************************************* */
// Check the BAL 16-camera dataset error before and after one LM iteration for
// legacy multifrontal Cholesky and the new multifrontal solver (QR off/forced).
# ifdef TEST_BAL16_DATASET
double runBal16OneIteration(
    const NonlinearFactorGraph& graph, const Values& initial,
    const Ordering& ordering,
    NonlinearOptimizerParams::LinearSolverType solverType) {
  const double lambda = 1e-5;
  GaussianFactorGraph::shared_ptr linear = graph.linearize(initial);
  VectorValues delta;
  if (solverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    MultifrontalSolver::Parameters params;
    params.qrMode = MultifrontalParameters::QRMode::Force;
    NonlinearMultifrontalSolver solver(graph, initial, ordering, params);
    solver.eliminateInPlace(*linear, lambda);
    delta = solver.updateSolution();
  } else if (solverType == NonlinearOptimizerParams::MULTIFRONTAL_QR) {
    GaussianFactorGraph damped = *linear;
    const auto dims = initial.dims();
    const double sigma = 1.0 / std::sqrt(lambda);
    for (const auto& [key, dim] : dims) {
      damped.emplace_shared<JacobianFactor>(
          key, Matrix::Identity(dim, dim), Vector::Zero(dim),
          noiseModel::Isotropic::Sigma(dim, sigma));
    }
    delta = damped.optimize(ordering, EliminateQR);
  } else {
    throw std::runtime_error("Unsupported solver type for BAL test.");
  }
  const Values updated = initial.retract(delta);
  return graph.error(updated);
}

TEST(NonlinearMultifrontalSolver, Bal16ErrorOneIteration) {
  const string filename = findExampleDataFile("dubrovnik-16-22106-pre");
  SfmData db = SfmData::FromBalFile(filename);
  db.tracks.resize(1000);
  NonlinearFactorGraph graph = db.generalSfmFactors();
  const Values initial = initialCamerasAndPointsEstimate(db);

  // Create ordering: first points, then cameras
  Ordering ordering;
  ordering.reserve(db.numberTracks() + db.numberCameras());
  for (size_t j = 0; j < db.numberTracks(); ++j) {
    ordering.push_back(P(j));
  }
  for (size_t i = 0; i < db.numberCameras(); ++i) {
    ordering.push_back(i);
  }

  // Run legacy multifrontal Cholesky and new solver (QR forced)
  const double legacyAfterError = runBal16OneIteration(
      graph, initial, ordering, NonlinearOptimizerParams::MULTIFRONTAL_QR);
  const double solverForceAfterError = runBal16OneIteration(
      graph, initial, ordering, NonlinearOptimizerParams::MULTIFRONTAL_SOLVER);
  EXPECT_DOUBLES_EQUAL(legacyAfterError, solverForceAfterError, 20.0);
}
#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
