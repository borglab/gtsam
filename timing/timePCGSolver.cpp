/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timePCGSolver.cpp
 * @brief Compare legacy, contiguous, and Eigen PCG performance.
 * @author Fan Jiang
 */

#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/SparseEigen.h>
#include <gtsam/slam/dataset.h>

#include <Eigen/IterativeLinearSolvers>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "internal/TimingUtils.h"

using namespace gtsam;

namespace {

struct Options {
  std::string dataset = "w10000.graph";
  size_t warmups = 5;
  size_t repeats = 30;
  size_t iterations = 20;
  size_t eigenThreads = 1;
  size_t gtsamThreads = 0;
  size_t profileIterations = 0;
  bool check = false;
};

Options parseOptions(int argc, char** argv) {
  gtsam::timing::Arguments arguments(argc, argv);
  Options options;
  options.dataset = arguments.stringValue("--dataset", options.dataset);
  options.warmups = arguments.sizeValue("--warmup", options.warmups);
  options.repeats = arguments.sizeValue("--repeats", options.repeats);
  options.iterations = arguments.sizeValue("--iterations", options.iterations);
  options.eigenThreads =
      arguments.sizeValue("--eigen-threads", options.eigenThreads);
  options.gtsamThreads =
      arguments.sizeValue("--gtsam-threads", options.gtsamThreads);
  options.profileIterations =
      arguments.sizeValue("--profile", options.profileIterations);
  options.check = arguments.flag("--check");
  arguments.validateAllConsumed();
  if (options.repeats == 0) {
    throw std::invalid_argument("--repeats must be positive");
  }
  if (options.eigenThreads == 0 ||
      options.eigenThreads >
          static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("--eigen-threads must be a positive int");
  }
  return options;
}

struct TimingStats {
  double median = 0.0;
  double p10 = 0.0;
  double p90 = 0.0;
};

struct EigenLinearSystem {
  SparseEigen hessian;
  Vector rhs;
};

template <class FUNCTION>
TimingStats timeMilliseconds(FUNCTION&& function, size_t warmups,
                             size_t repeats) {
  const auto samples = gtsam::timing::measureMilliseconds(
      std::forward<FUNCTION>(function), warmups, repeats);
  const auto summary = gtsam::timing::summarizeSamples(
      samples, gtsam::timing::MedianPolicy::kUpperMiddle);
  return {summary.median, summary.p10, summary.p90};
}

class LegacyGaussianFactorGraphSystem {
  const GaussianFactorGraph& graph_;
  const Preconditioner& preconditioner_;
  KeyInfo keyInfo_;

 public:
  LegacyGaussianFactorGraphSystem(const GaussianFactorGraph& graph,
                                  const Preconditioner& preconditioner,
                                  const KeyInfo& keyInfo)
      : graph_(graph), preconditioner_(preconditioner), keyInfo_(keyInfo) {}

  void residual(const Vector& x, Vector& residual) const {
    getb(residual);
    Vector product = Vector::Zero(residual.size());
    multiply(x, product);
    residual -= product;
  }

  void multiply(const Vector& x, Vector& product) const {
    const VectorValues vectorValuesX = buildVectorValues(x, keyInfo_);
    VectorValues vectorValuesProduct = keyInfo_.x0();
    graph_.multiplyHessianAdd(1.0, vectorValuesX, vectorValuesProduct);
    product = vectorValuesProduct.vector(keyInfo_.ordering());
  }

  void getb(Vector& rhs) const {
    rhs = -graph_.gradientAtZero().vector(keyInfo_.ordering());
  }

  void leftPrecondition(const Vector& x, Vector& y) const {
    preconditioner_.solve(x, y);
  }
  void rightPrecondition(const Vector& x, Vector& y) const {
    preconditioner_.transposeSolve(x, y);
  }
  void scal(double alpha, Vector& x) const { x *= alpha; }
  double dot(const Vector& x, const Vector& y) const { return x.dot(y); }
  void axpy(double alpha, const Vector& x, Vector& y) const { y += alpha * x; }
};

class EigenSparseSystem {
  const SparseEigen& hessian_;
  Vector rhs_;
  Vector splitPreconditionerScale_;

 public:
  EigenSparseSystem(const SparseEigen& hessian, Vector rhs,
                    Vector splitPreconditionerScale = Vector())
      : hessian_(hessian),
        rhs_(std::move(rhs)),
        splitPreconditionerScale_(std::move(splitPreconditionerScale)) {}

  void residual(const Vector& x, Vector& residual) const {
    residual.noalias() = rhs_ - hessian_ * x;
  }
  void multiply(const Vector& x, Vector& product) const {
    product.noalias() = hessian_ * x;
  }
  void leftPrecondition(const Vector& x, Vector& y) const {
    if (splitPreconditionerScale_.size() == 0) {
      y = x;
    } else {
      y = (splitPreconditionerScale_.array() * x.array()).matrix();
    }
  }
  void rightPrecondition(const Vector& x, Vector& y) const {
    leftPrecondition(x, y);
  }
  void scal(double alpha, Vector& x) const { x *= alpha; }
  double dot(const Vector& x, const Vector& y) const { return x.dot(y); }
  void axpy(double alpha, const Vector& x, Vector& y) const { y += alpha * x; }
};

using EigenIdentityCg =
    Eigen::ConjugateGradient<SparseEigen, Eigen::Lower | Eigen::Upper,
                             Eigen::IdentityPreconditioner>;
using EigenDiagonalPcg =
    Eigen::ConjugateGradient<SparseEigen, Eigen::Lower | Eigen::Upper,
                             Eigen::DiagonalPreconditioner<double>>;

EigenLinearSystem buildEigenLinearSystem(const GaussianFactorGraph& graph,
                                         const KeyInfo& keyInfo) {
  const SparseEigen augmented = sparseJacobianEigen(graph, keyInfo.ordering());
  const DenseIndex columns = static_cast<DenseIndex>(keyInfo.numCols());
  const SparseEigen jacobian = augmented.leftCols(columns);

  Vector rowRhs = Vector::Zero(augmented.rows());
  for (SparseEigen::InnerIterator entry(augmented, columns); entry; ++entry) {
    rowRhs(entry.row()) = entry.value();
  }
  return {jacobian.transpose() * jacobian, jacobian.transpose() * rowRhs};
}

Vector diagonalSplitPreconditioner(const SparseEigen& hessian) {
  Vector scale(hessian.rows());
  for (DenseIndex index = 0; index < hessian.rows(); ++index) {
    const double diagonal = hessian.coeff(index, index);
    if (diagonal < 0.0) {
      throw std::runtime_error(
          "Eigen PCG comparison requires a non-negative diagonal");
    }
    scale(index) = diagonal == 0.0 ? 1.0 : 1.0 / std::sqrt(diagonal);
  }
  return scale;
}

double relativeError(const Vector& expected, const Vector& actual) {
  return (expected - actual).norm() / std::max(1.0, expected.norm());
}

double relativeResidual(const SparseEigen& hessian, const Vector& rhs,
                        const Vector& solution) {
  return (rhs - hessian * solution).norm() / std::max(1.0, rhs.norm());
}

const char* terminationReason(ConjugateGradientTerminationReason reason) {
  switch (reason) {
    case ConjugateGradientTerminationReason::kConverged:
      return "converged";
    case ConjugateGradientTerminationReason::kMaxIterations:
      return "max_iterations";
    case ConjugateGradientTerminationReason::kNumericalBreakdown:
      return "numerical_breakdown";
  }
  return "unknown";
}

const char* eigenTerminationReason(Eigen::ComputationInfo info) {
  switch (info) {
    case Eigen::Success:
      return "converged";
    case Eigen::NoConvergence:
      return "max_iterations";
    case Eigen::NumericalIssue:
      return "numerical_issue";
    case Eigen::InvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

void printRow(const std::string& metric, const std::string& implementation,
              const TimingStats& timing, double speedup, double error) {
  std::cout << metric << ',' << implementation << ',' << std::setprecision(9)
            << timing.median << ',' << timing.p10 << ',' << timing.p90 << ','
            << speedup << ',' << error << '\n';
}

GaussianFactorGraph createBatchGraph() {
  constexpr size_t keyCount = 64;
  constexpr size_t rowCount = 2048;
  using Batch = BatchJacobianFactor<2, 3, 3>;

  KeyVector keys;
  std::vector<size_t> dimensions;
  keys.reserve(keyCount);
  dimensions.reserve(keyCount);
  for (size_t key = 0; key < keyCount; ++key) {
    keys.push_back(key);
    dimensions.push_back(3);
  }

  auto batch = std::make_shared<Batch>(keys, dimensions);
  batch->reserve(rowCount);
  for (size_t row = 0; row < rowCount; ++row) {
    const size_t first = row % keyCount;
    size_t second = (17 * row + 1) % keyCount;
    if (second == first) second = (second + 1) % keyCount;
    const double scale = 1.0 + 0.001 * static_cast<double>(row % 31);
    const Matrix firstBlock =
        scale * (Matrix(2, 3) << 1.0, 0.2, -0.1, -0.3, 0.7, 0.4).finished();
    const Matrix secondBlock =
        scale * (Matrix(2, 3) << -0.2, 0.5, 0.3, 0.8, -0.4, 0.1).finished();
    batch->addRow(
        {static_cast<DenseIndex>(first), static_cast<DenseIndex>(second)},
        {firstBlock, secondBlock}, Vector2(0.1, -0.2));
  }

  GaussianFactorGraph graph;
  graph.push_back(batch);
  return graph;
}

struct ConvergenceMeasurement {
  double elapsedMilliseconds = 0.0;
  size_t iterations = 0;
  double reportedResidual = 0.0;
  double trueRelativeResidual = 0.0;
  std::string reason;
};

struct BenchmarkReport {
  TimingStats eigenSystemSetup;
  TimingStats legacyMultiply;
  TimingStats contiguousMultiply;
  TimingStats parallelMultiply;
  TimingStats sparseMultiply;
  TimingStats eigenCgSparseMultiply;
  TimingStats legacyPcg;
  TimingStats contiguousPcg;
  TimingStats parallelPcg;
  TimingStats sparsePcg;
  TimingStats sparseDiagonalPcg;
  TimingStats eigenIdentitySetup;
  TimingStats eigenIdentityPcg;
  TimingStats eigenDiagonalSetup;
  TimingStats eigenDiagonalPcg;
  TimingStats legacyBlockPcg;
  TimingStats contiguousBlockPcg;
  TimingStats parallelBlockPcg;
  TimingStats legacyBatchMultiply;
  TimingStats contiguousBatchMultiply;
  PCGSolverResult detailed;

  double multiplyError = 0.0;
  double parallelMultiplyError = 0.0;
  double sparseMultiplyError = 0.0;
  double eigenCgMultiplyError = 0.0;
  double solutionError = 0.0;
  double parallelSolutionError = 0.0;
  double sparseSolutionError = 0.0;
  double eigenIdentitySolutionError = 0.0;
  double eigenDiagonalSolutionError = 0.0;
  double blockSolutionError = 0.0;
  double parallelBlockSolutionError = 0.0;
  double batchMultiplyError = 0.0;

  double contiguousResidual = 0.0;
  double parallelResidual = 0.0;
  double contiguousBlockResidual = 0.0;
  double parallelBlockResidual = 0.0;
  double sparseDiagonalResidual = 0.0;
  double eigenIdentityResidual = 0.0;
  double eigenDiagonalResidual = 0.0;
  Eigen::Index eigenIdentityIterations = 0;
  Eigen::Index eigenDiagonalIterations = 0;
  bool eigenFixedIterationCountsMatch = false;

  ConvergenceMeasurement legacyConvergence;
  ConvergenceMeasurement contiguousConvergence;
  ConvergenceMeasurement parallelConvergence;
  ConvergenceMeasurement eigenConvergence;
  size_t gtsamThreads = 1;
};

struct FixedPcgSolutions {
  Vector legacy;
  Vector contiguous;
  Vector parallel;
  Vector sparse;
  Vector sparseDiagonal;
  Vector eigenIdentity;
  Vector eigenDiagonal;
};

ConjugateGradientParameters fixedIterationParameters(const Options& options) {
  ConjugateGradientParameters parameters;
  parameters.minIterations = options.iterations;
  parameters.maxIterations = options.iterations;
  parameters.reset = options.iterations + 1;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = 0.0;
  return parameters;
}

bool runProfile(const Options& options,
                const GaussianFactorGraphSystem& parallel, const Vector& x) {
  if (options.profileIterations == 0) return false;

  Vector product;
  for (size_t iteration = 0; iteration < options.profileIterations;
       ++iteration) {
    parallel.multiply(x, product);
  }
  std::cout << "profile_checksum," << std::setprecision(17)
            << product.squaredNorm() << '\n';
  return true;
}

void measureProducts(const Options& options,
                     const LegacyGaussianFactorGraphSystem& legacy,
                     const GaussianFactorGraphSystem& contiguous,
                     const GaussianFactorGraphSystem& parallel, const Vector& x,
                     const SparseEigen& hessian, BenchmarkReport* report) {
  Vector legacyProduct, contiguousProduct, parallelProduct;
  legacy.multiply(x, legacyProduct);
  contiguous.multiply(x, contiguousProduct);
  parallel.multiply(x, parallelProduct);
  const Vector sparseProduct = hessian * x;

  report->legacyMultiply =
      timeMilliseconds([&] { legacy.multiply(x, legacyProduct); },
                       options.warmups, options.repeats);
  report->contiguousMultiply =
      timeMilliseconds([&] { contiguous.multiply(x, contiguousProduct); },
                       options.warmups, options.repeats);
  report->parallelMultiply =
      timeMilliseconds([&] { parallel.multiply(x, parallelProduct); },
                       options.warmups, options.repeats);

  Vector timedSparseProduct;
  report->sparseMultiply =
      timeMilliseconds([&] { timedSparseProduct = hessian * x; },
                       options.warmups, options.repeats);
  Vector timedEigenCgProduct;
  report->eigenCgSparseMultiply =
      timeMilliseconds([&] { timedEigenCgProduct = hessian.transpose() * x; },
                       options.warmups, options.repeats);

  report->multiplyError =
      std::max(relativeError(legacyProduct, contiguousProduct),
               relativeError(sparseProduct, contiguousProduct));
  report->parallelMultiplyError =
      relativeError(contiguousProduct, parallelProduct);
  report->sparseMultiplyError =
      relativeError(sparseProduct, timedSparseProduct);
  report->eigenCgMultiplyError =
      relativeError(sparseProduct, timedEigenCgProduct);
}

void measureGtsamFixedPcg(const Options& options,
                          const LegacyGaussianFactorGraphSystem& legacy,
                          const GaussianFactorGraphSystem& contiguous,
                          const GaussianFactorGraphSystem& parallel,
                          const SparseEigen& hessian, const Vector& rhs,
                          FixedPcgSolutions* solutions,
                          BenchmarkReport* report) {
  const auto parameters = fixedIterationParameters(options);
  const Vector zero = Vector::Zero(hessian.rows());

  // Time the graph-backed operators with identical iteration parameters.
  report->legacyPcg = timeMilliseconds(
      [&] {
        solutions->legacy =
            preconditionedConjugateGradient(legacy, zero, parameters);
      },
      options.warmups, options.repeats);
  report->contiguousPcg = timeMilliseconds(
      [&] {
        solutions->contiguous =
            preconditionedConjugateGradient(contiguous, zero, parameters);
      },
      options.warmups, options.repeats);
  report->parallelPcg = timeMilliseconds(
      [&] {
        solutions->parallel =
            preconditionedConjugateGradient(parallel, zero, parameters);
      },
      options.warmups, options.repeats);

  // Isolate GTSAM's CG recurrence over explicit sparse Eigen operators.
  const EigenSparseSystem sparseSystem(hessian, rhs);
  report->sparsePcg = timeMilliseconds(
      [&] {
        solutions->sparse =
            preconditionedConjugateGradient(sparseSystem, zero, parameters);
      },
      options.warmups, options.repeats);

  const EigenSparseSystem sparseDiagonalSystem(
      hessian, rhs, diagonalSplitPreconditioner(hessian));
  report->sparseDiagonalPcg = timeMilliseconds(
      [&] {
        solutions->sparseDiagonal = preconditionedConjugateGradient(
            sparseDiagonalSystem, zero, parameters);
      },
      options.warmups, options.repeats);
}

void measureEigenFixedPcg(const Options& options, const SparseEigen& hessian,
                          const Vector& rhs, FixedPcgSolutions* solutions,
                          BenchmarkReport* report) {
  const Vector zero = Vector::Zero(hessian.rows());

  // Measure Eigen's identity-preconditioned setup and solve separately.
  EigenIdentityCg eigenIdentityCg;
  eigenIdentityCg.setMaxIterations(
      static_cast<Eigen::Index>(options.iterations));
  eigenIdentityCg.setTolerance(0.0);
  report->eigenIdentitySetup =
      timeMilliseconds([&] { eigenIdentityCg.compute(hessian); },
                       options.warmups, options.repeats);
  if (eigenIdentityCg.info() != Eigen::Success) {
    throw std::runtime_error("Eigen identity CG setup failed");
  }
  report->eigenIdentityPcg = timeMilliseconds(
      [&] {
        solutions->eigenIdentity = eigenIdentityCg.solveWithGuess(rhs, zero);
      },
      options.warmups, options.repeats);

  // Repeat with Eigen's diagonal preconditioner for the practical comparison.
  EigenDiagonalPcg eigenDiagonalPcg;
  eigenDiagonalPcg.setMaxIterations(
      static_cast<Eigen::Index>(options.iterations));
  eigenDiagonalPcg.setTolerance(0.0);
  report->eigenDiagonalSetup =
      timeMilliseconds([&] { eigenDiagonalPcg.compute(hessian); },
                       options.warmups, options.repeats);
  if (eigenDiagonalPcg.info() != Eigen::Success) {
    throw std::runtime_error("Eigen diagonal PCG setup failed");
  }
  report->eigenDiagonalPcg = timeMilliseconds(
      [&] {
        solutions->eigenDiagonal = eigenDiagonalPcg.solveWithGuess(rhs, zero);
      },
      options.warmups, options.repeats);

  report->eigenIdentityIterations = eigenIdentityCg.iterations();
  report->eigenDiagonalIterations = eigenDiagonalPcg.iterations();
}

void summarizeFixedPcg(const Options& options, const SparseEigen& hessian,
                       const Vector& rhs, const FixedPcgSolutions& solutions,
                       BenchmarkReport* report) {
  // Compare equivalent solvers and record true normal-equation residuals.
  report->solutionError = relativeError(solutions.legacy, solutions.contiguous);
  report->parallelSolutionError =
      relativeError(solutions.contiguous, solutions.parallel);
  report->sparseSolutionError =
      relativeError(solutions.sparse, solutions.contiguous);
  report->eigenIdentitySolutionError =
      relativeError(solutions.sparse, solutions.eigenIdentity);
  report->eigenDiagonalSolutionError =
      relativeError(solutions.sparseDiagonal, solutions.eigenDiagonal);
  report->eigenFixedIterationCountsMatch =
      report->eigenIdentityIterations ==
          static_cast<Eigen::Index>(options.iterations) &&
      report->eigenDiagonalIterations ==
          static_cast<Eigen::Index>(options.iterations);

  report->contiguousResidual =
      relativeResidual(hessian, rhs, solutions.contiguous);
  report->parallelResidual = relativeResidual(hessian, rhs, solutions.parallel);
  report->sparseDiagonalResidual =
      relativeResidual(hessian, rhs, solutions.sparseDiagonal);
  report->eigenIdentityResidual =
      relativeResidual(hessian, rhs, solutions.eigenIdentity);
  report->eigenDiagonalResidual =
      relativeResidual(hessian, rhs, solutions.eigenDiagonal);
}

void measureFixedPcg(const Options& options,
                     const LegacyGaussianFactorGraphSystem& legacy,
                     const GaussianFactorGraphSystem& contiguous,
                     const GaussianFactorGraphSystem& parallel,
                     const SparseEigen& hessian, const Vector& rhs,
                     BenchmarkReport* report) {
  FixedPcgSolutions solutions;
  measureGtsamFixedPcg(options, legacy, contiguous, parallel, hessian, rhs,
                       &solutions, report);
  measureEigenFixedPcg(options, hessian, rhs, &solutions, report);
  summarizeFixedPcg(options, hessian, rhs, solutions, report);
}

void measureBlockPcg(const Options& options, const GaussianFactorGraph& graph,
                     const KeyInfo& keyInfo, const SparseEigen& hessian,
                     const Vector& rhs, BenchmarkReport* report) {
  // Build equivalent legacy, serial-compiled, and parallel-compiled systems.
  BlockJacobiPreconditioner legacyPreconditioner, compiledPreconditioner;
  legacyPreconditioner.build(graph, keyInfo, {});
  compiledPreconditioner.build(graph, keyInfo, {});
  const LegacyGaussianFactorGraphSystem legacy(graph, legacyPreconditioner,
                                               keyInfo);
  const GaussianFactorGraphSystem contiguous(graph, compiledPreconditioner,
                                             keyInfo, {}, false, 1);
  const GaussianFactorGraphSystem parallel(
      graph, compiledPreconditioner, keyInfo, {}, true, options.gtsamThreads);
  const ConjugateGradientParameters parameters =
      fixedIterationParameters(options);
  const Vector zero = Vector::Zero(hessian.rows());

  // Time fixed-iteration block-Jacobi solves over each graph operator.
  Vector legacySolution, contiguousSolution, parallelSolution;
  report->legacyBlockPcg = timeMilliseconds(
      [&] {
        legacySolution =
            preconditionedConjugateGradient(legacy, zero, parameters);
      },
      options.warmups, options.repeats);
  report->contiguousBlockPcg = timeMilliseconds(
      [&] {
        contiguousSolution =
            preconditionedConjugateGradient(contiguous, zero, parameters);
      },
      options.warmups, options.repeats);
  report->parallelBlockPcg = timeMilliseconds(
      [&] {
        parallelSolution =
            preconditionedConjugateGradient(parallel, zero, parameters);
      },
      options.warmups, options.repeats);

  // Validate solutions against each other and the explicit normal equations.
  report->blockSolutionError =
      relativeError(legacySolution, contiguousSolution);
  report->parallelBlockSolutionError =
      relativeError(contiguousSolution, parallelSolution);
  report->contiguousBlockResidual =
      relativeResidual(hessian, rhs, contiguousSolution);
  report->parallelBlockResidual =
      relativeResidual(hessian, rhs, parallelSolution);
}

template <class SYSTEM>
ConvergenceMeasurement measureGtsamConvergence(
    const SYSTEM& system, const Vector& zero,
    const ConjugateGradientParameters& parameters, const SparseEigen& hessian,
    const Vector& rhs) {
  ConjugateGradientResult<Vector> result;
  const double elapsedMilliseconds =
      1000.0 * gtsam::timing::measureSeconds([&] {
        result = preconditionedConjugateGradientDetailed(system, zero,
                                                         parameters, false);
      });
  return {elapsedMilliseconds, result.stats.iterations,
          result.stats.finalPreconditionedResidualNorm,
          relativeResidual(hessian, rhs, result.solution),
          terminationReason(result.stats.terminationReason)};
}

void measureConvergence(const Options& options,
                        const GaussianFactorGraph& graph,
                        const KeyInfo& keyInfo, const SparseEigen& hessian,
                        const Vector& rhs, BenchmarkReport* report) {
  // Construct graph operators with identical block-Jacobi preconditioners.
  BlockJacobiPreconditioner legacyPreconditioner, compiledPreconditioner;
  legacyPreconditioner.build(graph, keyInfo, {});
  compiledPreconditioner.build(graph, keyInfo, {});
  const LegacyGaussianFactorGraphSystem legacy(graph, legacyPreconditioner,
                                               keyInfo);
  const GaussianFactorGraphSystem contiguous(graph, compiledPreconditioner,
                                             keyInfo, {}, false, 1);
  const GaussianFactorGraphSystem parallel(
      graph, compiledPreconditioner, keyInfo, {}, true, options.gtsamThreads);

  ConjugateGradientParameters parameters;
  parameters.minIterations = 0;
  parameters.maxIterations = 500;
  parameters.reset = 501;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = 1e-3;
  const Vector zero = Vector::Zero(hessian.rows());

  // Measure GTSAM implementations to a shared relative tolerance.
  report->legacyConvergence =
      measureGtsamConvergence(legacy, zero, parameters, hessian, rhs);
  report->contiguousConvergence =
      measureGtsamConvergence(contiguous, zero, parameters, hessian, rhs);
  report->parallelConvergence =
      measureGtsamConvergence(parallel, zero, parameters, hessian, rhs);

  // Measure Eigen's diagonal PCG under the same convergence limit.
  EigenDiagonalPcg eigenPcg;
  eigenPcg.setMaxIterations(
      static_cast<Eigen::Index>(parameters.maxIterations));
  eigenPcg.setTolerance(parameters.epsilon_rel);
  eigenPcg.compute(hessian);
  if (eigenPcg.info() != Eigen::Success) {
    throw std::runtime_error("Eigen convergence PCG setup failed");
  }
  Vector solution;
  const double elapsedMilliseconds =
      1000.0 * gtsam::timing::measureSeconds(
                   [&] { solution = eigenPcg.solveWithGuess(rhs, zero); });
  report->eigenConvergence = {
      elapsedMilliseconds, static_cast<size_t>(eigenPcg.iterations()),
      eigenPcg.error(), relativeResidual(hessian, rhs, solution),
      eigenTerminationReason(eigenPcg.info())};
}

void measureDetailedSolve(const Options& options,
                          const GaussianFactorGraph& graph,
                          BenchmarkReport* report) {
  PCGSolverParameters parameters(
      std::make_shared<BlockJacobiPreconditionerParameters>());
  parameters.minIterations = options.iterations;
  parameters.maxIterations = options.iterations;
  parameters.reset = options.iterations + 1;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = 0.0;
  parameters.parallel = true;
  parameters.numThreads = options.gtsamThreads;
  report->detailed = PCGSolver(parameters).optimizeDetailed(graph, false);
}

void measureBatchFactors(const Options& options, BenchmarkReport* report) {
  const GaussianFactorGraph graph = createBatchGraph();
  const KeyInfo keyInfo(graph);
  DummyPreconditioner dummy;
  dummy.build(graph, keyInfo, {});
  const LegacyGaussianFactorGraphSystem legacy(graph, dummy, keyInfo);
  const GaussianFactorGraphSystem contiguous(graph, dummy, keyInfo, {});
  const Vector x =
      Vector::LinSpaced(static_cast<DenseIndex>(keyInfo.numCols()), -0.5, 0.5);
  Vector legacyProduct, contiguousProduct;
  report->legacyBatchMultiply =
      timeMilliseconds([&] { legacy.multiply(x, legacyProduct); },
                       options.warmups, options.repeats);
  report->contiguousBatchMultiply =
      timeMilliseconds([&] { contiguous.multiply(x, contiguousProduct); },
                       options.warmups, options.repeats);
  report->batchMultiplyError = relativeError(legacyProduct, contiguousProduct);
}

void printConvergenceRow(const std::string& implementation,
                         const std::string& preconditioner,
                         const ConvergenceMeasurement& measurement) {
  std::cout << implementation << ',' << preconditioner << ','
            << measurement.elapsedMilliseconds << ',' << measurement.iterations
            << ',' << measurement.reportedResidual << ','
            << measurement.trueRelativeResidual << ',' << measurement.reason
            << '\n';
}

void printProductRows(const BenchmarkReport& report) {
  const double multiplySpeedup =
      report.legacyMultiply.median / report.contiguousMultiply.median;
  printRow("multiply", "legacy", report.legacyMultiply, 1.0, 0.0);
  printRow("multiply", "contiguous", report.contiguousMultiply, multiplySpeedup,
           report.multiplyError);
  printRow("multiply", "task_scheduler", report.parallelMultiply,
           report.contiguousMultiply.median / report.parallelMultiply.median,
           report.parallelMultiplyError);
  printRow("multiply", "eigen_sparse", report.sparseMultiply,
           report.legacyMultiply.median / report.sparseMultiply.median,
           report.sparseMultiplyError);
  printRow("multiply", "eigen_cg_sparse", report.eigenCgSparseMultiply,
           report.legacyMultiply.median / report.eigenCgSparseMultiply.median,
           report.eigenCgMultiplyError);
}

void printBatchRows(const BenchmarkReport& report) {
  printRow("batch_multiply", "legacy_dense_fallback",
           report.legacyBatchMultiply, 1.0, 0.0);
  printRow(
      "batch_multiply", "contiguous_compact", report.contiguousBatchMultiply,
      report.legacyBatchMultiply.median / report.contiguousBatchMultiply.median,
      report.batchMultiplyError);
}

void printPcgRows(const BenchmarkReport& report) {
  printRow("pcg_fixed_dummy", "legacy", report.legacyPcg, 1.0, 0.0);
  printRow("pcg_fixed_dummy", "contiguous", report.contiguousPcg,
           report.legacyPcg.median / report.contiguousPcg.median,
           report.solutionError);
  printRow("pcg_fixed_dummy", "task_scheduler", report.parallelPcg,
           report.contiguousPcg.median / report.parallelPcg.median,
           report.parallelSolutionError);
  printRow("pcg_fixed_dummy", "eigen_sparse", report.sparsePcg,
           report.legacyPcg.median / report.sparsePcg.median,
           report.sparseSolutionError);
  printRow("pcg_fixed_dummy", "eigen_cg_identity", report.eigenIdentityPcg,
           report.legacyPcg.median / report.eigenIdentityPcg.median,
           report.eigenIdentitySolutionError);
  printRow("pcg_fixed_diagonal", "gtsam_cg_eigen_sparse",
           report.sparseDiagonalPcg, 1.0, 0.0);
  printRow("pcg_fixed_diagonal", "eigen_pcg", report.eigenDiagonalPcg,
           report.sparseDiagonalPcg.median / report.eigenDiagonalPcg.median,
           report.eigenDiagonalSolutionError);
  printRow("pcg_fixed_block_jacobi", "legacy", report.legacyBlockPcg, 1.0, 0.0);
  printRow("pcg_fixed_block_jacobi", "contiguous", report.contiguousBlockPcg,
           report.legacyBlockPcg.median / report.contiguousBlockPcg.median,
           report.blockSolutionError);
  printRow("pcg_fixed_block_jacobi", "task_scheduler", report.parallelBlockPcg,
           report.contiguousBlockPcg.median / report.parallelBlockPcg.median,
           report.parallelBlockSolutionError);
}

void printSetupRows(const BenchmarkReport& report) {
  const TimingStats operatorSetup{
      1000.0 * report.detailed.operatorSetupSeconds,
      1000.0 * report.detailed.operatorSetupSeconds,
      1000.0 * report.detailed.operatorSetupSeconds};
  const TimingStats preconditionerSetup{
      1000.0 * report.detailed.preconditionerSetupSeconds,
      1000.0 * report.detailed.preconditionerSetupSeconds,
      1000.0 * report.detailed.preconditionerSetupSeconds};
  const TimingStats detailedSolve{1000.0 * report.detailed.solveSeconds,
                                  1000.0 * report.detailed.solveSeconds,
                                  1000.0 * report.detailed.solveSeconds};
  printRow("setup", "operator", operatorSetup, 0.0, 0.0);
  printRow("setup", "block_jacobi", preconditionerSetup, 0.0, 0.0);
  printRow("setup", "eigen_normal_equations", report.eigenSystemSetup, 0.0,
           0.0);
  printRow("setup", "eigen_identity", report.eigenIdentitySetup, 0.0, 0.0);
  printRow("setup", "eigen_diagonal", report.eigenDiagonalSetup, 0.0, 0.0);
  printRow("solve", "detailed_block_jacobi", detailedSolve, 0.0, 0.0);
}

void printFixedResidualRows(const Options& options,
                            const BenchmarkReport& report) {
  std::cout << "fixed_implementation,preconditioner,iterations,"
               "true_relative_residual\n"
            << "contiguous,dummy," << options.iterations << ','
            << report.contiguousResidual << '\n'
            << "task_scheduler,dummy," << options.iterations << ','
            << report.parallelResidual << '\n'
            << "contiguous,block_jacobi," << options.iterations << ','
            << report.contiguousBlockResidual << '\n'
            << "task_scheduler,block_jacobi," << options.iterations << ','
            << report.parallelBlockResidual << '\n'
            << "gtsam_cg_eigen_sparse,diagonal," << options.iterations << ','
            << report.sparseDiagonalResidual << '\n'
            << "eigen,identity," << report.eigenIdentityIterations << ','
            << report.eigenIdentityResidual << '\n'
            << "eigen,diagonal," << report.eigenDiagonalIterations << ','
            << report.eigenDiagonalResidual << '\n';
}

void printConvergenceRows(const BenchmarkReport& report) {
  std::cout << "convergence_implementation,preconditioner,elapsed_ms,"
               "iterations,reported_residual,true_relative_residual,reason\n";
  printConvergenceRow("legacy", "block_jacobi", report.legacyConvergence);
  printConvergenceRow("contiguous", "block_jacobi",
                      report.contiguousConvergence);
  printConvergenceRow("task_scheduler", "block_jacobi",
                      report.parallelConvergence);
  printConvergenceRow("eigen", "eigen_diagonal", report.eigenConvergence);
}

void printBenchmarkReport(const Options& options,
                          const GaussianFactorGraph& graph,
                          const KeyInfo& keyInfo,
                          const BenchmarkReport& report) {
  // Emit run metadata before the machine-readable measurement tables.
  std::cout << "dataset," << options.dataset << '\n'
            << "factors," << graph.size() << '\n'
            << "variables," << keyInfo.size() << '\n'
            << "scalars," << keyInfo.numCols() << '\n'
            << "eigen_version," << EIGEN_WORLD_VERSION << '.'
            << EIGEN_MAJOR_VERSION << '.' << EIGEN_MINOR_VERSION << '\n'
#ifdef EIGEN_HAS_OPENMP
            << "eigen_openmp,1\n"
#else
            << "eigen_openmp,0\n"
#endif
            << "eigen_threads," << Eigen::nbThreads() << '\n'
            << "gtsam_threads," << report.gtsamThreads << '\n'
            << "metric,implementation,median_ms,p10_ms,p90_ms,speedup,"
               "relative_error\n";

  printProductRows(report);
  printPcgRows(report);
  printBatchRows(report);
  printSetupRows(report);
  printFixedResidualRows(options, report);
  printConvergenceRows(report);
}

bool benchmarkChecksPass(const BenchmarkReport& report) {
  const double multiplySpeedup =
      report.legacyMultiply.median / report.contiguousMultiply.median;
  const double pcgSpeedup =
      report.legacyPcg.median / report.contiguousPcg.median;
  const double blockPcgSpeedup =
      report.legacyBlockPcg.median / report.contiguousBlockPcg.median;
  return multiplySpeedup >= 8.0 && pcgSpeedup >= 5.0 &&
         blockPcgSpeedup >= 5.0 && report.multiplyError <= 1e-10 &&
         report.solutionError <= 1e-9 && report.blockSolutionError <= 1e-9 &&
         report.batchMultiplyError <= 1e-10 &&
         report.parallelMultiplyError <= 1e-10 &&
         report.parallelSolutionError <= 1e-9 &&
         report.parallelBlockSolutionError <= 1e-9 &&
         report.eigenCgMultiplyError <= 1e-10 &&
         report.eigenIdentitySolutionError <= 1e-9 &&
         report.eigenDiagonalSolutionError <= 1e-9 &&
         report.eigenFixedIterationCountsMatch &&
         report.eigenConvergence.trueRelativeResidual <= 1e-3;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    // Configure the two independent parallel runtimes from command-line input.
    const Options options = parseOptions(argc, argv);
    Eigen::setNbThreads(static_cast<int>(options.eigenThreads));
    if (Eigen::nbThreads() != static_cast<int>(options.eigenThreads)) {
      throw std::runtime_error(
          "Requested Eigen thread count requires an OpenMP-enabled build");
    }

    // Load and linearize the benchmark graph once for every implementation.
    const auto [nonlinearGraph, initial] =
        load2D(findExampleDataFile(options.dataset));
    nonlinearGraph->addPrior(0, initial->at<Pose2>(0),
                             noiseModel::Unit::Create(3));
    const GaussianFactorGraph graph = *nonlinearGraph->linearize(*initial);
    const KeyInfo keyInfo(graph);
    const Vector x = Vector::LinSpaced(
        static_cast<DenseIndex>(keyInfo.numCols()), -1.0, 1.0);

    // Construct equivalent legacy, serial-compiled, and parallel operators.
    DummyPreconditioner dummy;
    dummy.build(graph, keyInfo, {});
    const LegacyGaussianFactorGraphSystem legacy(graph, dummy, keyInfo);
    const GaussianFactorGraphSystem contiguous(graph, dummy, keyInfo, {}, false,
                                               1);
    const GaussianFactorGraphSystem parallel(graph, dummy, keyInfo, {}, true,
                                             options.gtsamThreads);
    if (runProfile(options, parallel, x)) return 0;

    // Collect setup, kernel, fixed-iteration, and convergence measurements.
    BenchmarkReport report;
    report.gtsamThreads = parallel.numThreads();
    EigenLinearSystem eigenSystem;
    report.eigenSystemSetup = timeMilliseconds(
        [&] { eigenSystem = buildEigenLinearSystem(graph, keyInfo); },
        options.warmups, options.repeats);

    measureProducts(options, legacy, contiguous, parallel, x,
                    eigenSystem.hessian, &report);
    measureFixedPcg(options, legacy, contiguous, parallel, eigenSystem.hessian,
                    eigenSystem.rhs, &report);
    measureBlockPcg(options, graph, keyInfo, eigenSystem.hessian,
                    eigenSystem.rhs, &report);
    measureConvergence(options, graph, keyInfo, eigenSystem.hessian,
                       eigenSystem.rhs, &report);
    measureDetailedSolve(options, graph, &report);
    measureBatchFactors(options, &report);

    // Print machine-readable results and optionally enforce the benchmark gate.
    printBenchmarkReport(options, graph, keyInfo, report);

    if (options.check && !benchmarkChecksPass(report)) {
      std::cerr << "PCG performance or correctness gate failed\n";
      return 2;
    }
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
