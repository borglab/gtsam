/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timePCGSolver.cpp
 * @brief Compare legacy and contiguous PCG operator performance.
 * @author Fan Jiang
 */

#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/SparseEigen.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace gtsam;

namespace {

struct Options {
  std::string dataset = "w10000.graph";
  size_t warmups = 5;
  size_t repeats = 30;
  size_t iterations = 20;
  size_t profileIterations = 0;
  bool check = false;
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--dataset" && i + 1 < argc) {
      options.dataset = argv[++i];
    } else if (argument == "--warmup" && i + 1 < argc) {
      options.warmups = std::stoul(argv[++i]);
    } else if (argument == "--repeats" && i + 1 < argc) {
      options.repeats = std::stoul(argv[++i]);
    } else if (argument == "--iterations" && i + 1 < argc) {
      options.iterations = std::stoul(argv[++i]);
    } else if (argument == "--profile" && i + 1 < argc) {
      options.profileIterations = std::stoul(argv[++i]);
    } else if (argument == "--check") {
      options.check = true;
    } else {
      throw std::invalid_argument("Unknown or incomplete option: " + argument);
    }
  }
  if (options.repeats == 0) {
    throw std::invalid_argument("--repeats must be positive");
  }
  return options;
}

struct TimingStats {
  double median = 0.0;
  double p10 = 0.0;
  double p90 = 0.0;
};

template <class FUNCTION>
TimingStats timeMilliseconds(FUNCTION&& function, size_t warmups,
                             size_t repeats) {
  for (size_t i = 0; i < warmups; ++i) function();
  std::vector<double> samples;
  samples.reserve(repeats);
  for (size_t i = 0; i < repeats; ++i) {
    const auto start = std::chrono::steady_clock::now();
    function();
    const auto end = std::chrono::steady_clock::now();
    samples.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
  std::sort(samples.begin(), samples.end());
  const size_t last = samples.size() - 1;
  return {samples[samples.size() / 2], samples[last / 10],
          samples[(9 * last) / 10]};
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

 public:
  EigenSparseSystem(const SparseEigen& hessian, Vector rhs)
      : hessian_(hessian), rhs_(std::move(rhs)) {}

  void residual(const Vector& x, Vector& residual) const {
    residual.noalias() = rhs_ - hessian_ * x;
  }
  void multiply(const Vector& x, Vector& product) const {
    product.noalias() = hessian_ * x;
  }
  void leftPrecondition(const Vector& x, Vector& y) const { y = x; }
  void rightPrecondition(const Vector& x, Vector& y) const { y = x; }
  void scal(double alpha, Vector& x) const { x *= alpha; }
  double dot(const Vector& x, const Vector& y) const { return x.dot(y); }
  void axpy(double alpha, const Vector& x, Vector& y) const { y += alpha * x; }
};

double relativeError(const Vector& expected, const Vector& actual) {
  return (expected - actual).norm() / std::max(1.0, expected.norm());
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

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    const auto [nonlinearGraph, initial] =
        load2D(findExampleDataFile(options.dataset));
    nonlinearGraph->addPrior(0, initial->at<Pose2>(0),
                             noiseModel::Unit::Create(3));
    const GaussianFactorGraph graph = *nonlinearGraph->linearize(*initial);
    const KeyInfo keyInfo(graph);
    const Vector x = Vector::LinSpaced(
        static_cast<DenseIndex>(keyInfo.numCols()), -1.0, 1.0);

    DummyPreconditioner dummy;
    dummy.build(graph, keyInfo, {});
    const LegacyGaussianFactorGraphSystem legacy(graph, dummy, keyInfo);
    const GaussianFactorGraphSystem contiguous(graph, dummy, keyInfo, {});

    if (options.profileIterations != 0) {
      Vector product;
      for (size_t iteration = 0; iteration < options.profileIterations;
           ++iteration) {
        contiguous.multiply(x, product);
      }
      std::cout << "profile_checksum," << std::setprecision(17)
                << product.squaredNorm() << '\n';
      return 0;
    }

    Vector legacyProduct, contiguousProduct;
    legacy.multiply(x, legacyProduct);
    contiguous.multiply(x, contiguousProduct);

    const SparseEigen augmented =
        sparseJacobianEigen(graph, keyInfo.ordering());
    const DenseIndex columns = static_cast<DenseIndex>(keyInfo.numCols());
    const SparseEigen jacobian = augmented.leftCols(columns);
    const SparseEigen hessian = jacobian.transpose() * jacobian;
    const Vector sparseProduct = hessian * x;
    Vector sparseRowRhs = Vector::Zero(augmented.rows());
    for (SparseEigen::InnerIterator entry(augmented, columns); entry; ++entry) {
      sparseRowRhs(entry.row()) = entry.value();
    }
    const EigenSparseSystem sparseSystem(hessian,
                                         jacobian.transpose() * sparseRowRhs);

    const TimingStats legacyMultiply =
        timeMilliseconds([&] { legacy.multiply(x, legacyProduct); },
                         options.warmups, options.repeats);
    const TimingStats contiguousMultiply =
        timeMilliseconds([&] { contiguous.multiply(x, contiguousProduct); },
                         options.warmups, options.repeats);
    Vector timedSparseProduct;
    const TimingStats sparseMultiply =
        timeMilliseconds([&] { timedSparseProduct = hessian * x; },
                         options.warmups, options.repeats);

    ConjugateGradientParameters parameters;
    parameters.minIterations = options.iterations;
    parameters.maxIterations = options.iterations;
    parameters.reset = options.iterations + 1;
    parameters.epsilon_abs = 0.0;
    parameters.epsilon_rel = 0.0;
    const Vector zero =
        Vector::Zero(static_cast<DenseIndex>(keyInfo.numCols()));
    Vector legacySolution, contiguousSolution;
    const TimingStats legacyPcg = timeMilliseconds(
        [&] {
          legacySolution =
              preconditionedConjugateGradient(legacy, zero, parameters);
        },
        options.warmups, options.repeats);
    const TimingStats contiguousPcg = timeMilliseconds(
        [&] {
          contiguousSolution =
              preconditionedConjugateGradient(contiguous, zero, parameters);
        },
        options.warmups, options.repeats);
    Vector sparseSolution;
    const TimingStats sparsePcg = timeMilliseconds(
        [&] {
          sparseSolution =
              preconditionedConjugateGradient(sparseSystem, zero, parameters);
        },
        options.warmups, options.repeats);

    BlockJacobiPreconditioner legacyBlockPreconditioner,
        contiguousBlockPreconditioner;
    legacyBlockPreconditioner.build(graph, keyInfo, {});
    contiguousBlockPreconditioner.build(graph, keyInfo, {});
    const LegacyGaussianFactorGraphSystem legacyBlock(
        graph, legacyBlockPreconditioner, keyInfo);
    const GaussianFactorGraphSystem contiguousBlock(
        graph, contiguousBlockPreconditioner, keyInfo, {});
    Vector legacyBlockSolution, contiguousBlockSolution;
    const TimingStats legacyBlockPcg = timeMilliseconds(
        [&] {
          legacyBlockSolution =
              preconditionedConjugateGradient(legacyBlock, zero, parameters);
        },
        options.warmups, options.repeats);
    const TimingStats contiguousBlockPcg = timeMilliseconds(
        [&] {
          contiguousBlockSolution = preconditionedConjugateGradient(
              contiguousBlock, zero, parameters);
        },
        options.warmups, options.repeats);

    ConjugateGradientParameters convergenceParameters;
    convergenceParameters.minIterations = 0;
    convergenceParameters.maxIterations = 500;
    convergenceParameters.reset = 501;
    convergenceParameters.epsilon_abs = 0.0;
    convergenceParameters.epsilon_rel = 1e-3;
    const auto legacyConvergenceStart = std::chrono::steady_clock::now();
    const auto legacyConvergence = preconditionedConjugateGradientDetailed(
        legacyBlock, zero, convergenceParameters, false);
    const auto legacyConvergenceEnd = std::chrono::steady_clock::now();
    const auto contiguousConvergenceStart = std::chrono::steady_clock::now();
    const auto contiguousConvergence = preconditionedConjugateGradientDetailed(
        contiguousBlock, zero, convergenceParameters, false);
    const auto contiguousConvergenceEnd = std::chrono::steady_clock::now();

    PCGSolverParameters detailedParameters(
        std::make_shared<BlockJacobiPreconditionerParameters>());
    detailedParameters.minIterations = options.iterations;
    detailedParameters.maxIterations = options.iterations;
    detailedParameters.reset = options.iterations + 1;
    detailedParameters.epsilon_abs = 0.0;
    detailedParameters.epsilon_rel = 0.0;
    const PCGSolverResult detailed =
        PCGSolver(detailedParameters).optimizeDetailed(graph, false);

    const double multiplyError =
        std::max(relativeError(legacyProduct, contiguousProduct),
                 relativeError(sparseProduct, contiguousProduct));
    const double solutionError =
        relativeError(legacySolution, contiguousSolution);
    const double blockSolutionError =
        relativeError(legacyBlockSolution, contiguousBlockSolution);
    const double multiplySpeedup =
        legacyMultiply.median / contiguousMultiply.median;
    const double pcgSpeedup = legacyPcg.median / contiguousPcg.median;
    const double blockPcgSpeedup =
        legacyBlockPcg.median / contiguousBlockPcg.median;

    const GaussianFactorGraph batchGraph = createBatchGraph();
    const KeyInfo batchKeyInfo(batchGraph);
    DummyPreconditioner batchDummy;
    batchDummy.build(batchGraph, batchKeyInfo, {});
    const LegacyGaussianFactorGraphSystem legacyBatch(batchGraph, batchDummy,
                                                      batchKeyInfo);
    const GaussianFactorGraphSystem contiguousBatch(batchGraph, batchDummy,
                                                    batchKeyInfo, {});
    const Vector batchX = Vector::LinSpaced(
        static_cast<DenseIndex>(batchKeyInfo.numCols()), -0.5, 0.5);
    Vector legacyBatchProduct, contiguousBatchProduct;
    const TimingStats legacyBatchMultiply = timeMilliseconds(
        [&] { legacyBatch.multiply(batchX, legacyBatchProduct); },
        options.warmups, options.repeats);
    const TimingStats contiguousBatchMultiply = timeMilliseconds(
        [&] { contiguousBatch.multiply(batchX, contiguousBatchProduct); },
        options.warmups, options.repeats);
    const double batchMultiplyError =
        relativeError(legacyBatchProduct, contiguousBatchProduct);
    const double batchMultiplySpeedup =
        legacyBatchMultiply.median / contiguousBatchMultiply.median;

    std::cout << "dataset," << options.dataset << '\n'
              << "factors," << graph.size() << '\n'
              << "variables," << keyInfo.size() << '\n'
              << "scalars," << keyInfo.numCols() << '\n'
              << "metric,implementation,median_ms,p10_ms,p90_ms,speedup,"
                 "relative_error\n";
    printRow("multiply", "legacy", legacyMultiply, 1.0, 0.0);
    printRow("multiply", "contiguous", contiguousMultiply, multiplySpeedup,
             multiplyError);
    printRow("multiply", "eigen_sparse", sparseMultiply,
             legacyMultiply.median / sparseMultiply.median,
             relativeError(sparseProduct, timedSparseProduct));
    printRow("pcg_fixed_dummy", "legacy", legacyPcg, 1.0, 0.0);
    printRow("pcg_fixed_dummy", "contiguous", contiguousPcg, pcgSpeedup,
             solutionError);
    printRow("pcg_fixed_dummy", "eigen_sparse", sparsePcg,
             legacyPcg.median / sparsePcg.median,
             relativeError(sparseSolution, contiguousSolution));
    printRow("pcg_fixed_block_jacobi", "legacy", legacyBlockPcg, 1.0, 0.0);
    printRow("pcg_fixed_block_jacobi", "contiguous", contiguousBlockPcg,
             blockPcgSpeedup, blockSolutionError);
    printRow("batch_multiply", "legacy_dense_fallback", legacyBatchMultiply,
             1.0, 0.0);
    printRow("batch_multiply", "contiguous_compact", contiguousBatchMultiply,
             batchMultiplySpeedup, batchMultiplyError);
    const TimingStats operatorSetup{1000.0 * detailed.operatorSetupSeconds,
                                    1000.0 * detailed.operatorSetupSeconds,
                                    1000.0 * detailed.operatorSetupSeconds};
    const TimingStats preconditionerSetup{
        1000.0 * detailed.preconditionerSetupSeconds,
        1000.0 * detailed.preconditionerSetupSeconds,
        1000.0 * detailed.preconditionerSetupSeconds};
    const TimingStats detailedSolve{1000.0 * detailed.solveSeconds,
                                    1000.0 * detailed.solveSeconds,
                                    1000.0 * detailed.solveSeconds};
    printRow("setup", "operator", operatorSetup, 0.0, 0.0);
    printRow("setup", "block_jacobi", preconditionerSetup, 0.0, 0.0);
    printRow("solve", "detailed_block_jacobi", detailedSolve, 0.0, 0.0);
    std::cout << "convergence_implementation,elapsed_ms,iterations,"
                 "preconditioned_residual,reason\n"
              << "legacy,"
              << std::chrono::duration<double, std::milli>(
                     legacyConvergenceEnd - legacyConvergenceStart)
                     .count()
              << ',' << legacyConvergence.stats.iterations << ','
              << legacyConvergence.stats.finalPreconditionedResidualNorm << ','
              << terminationReason(legacyConvergence.stats.terminationReason)
              << '\n'
              << "contiguous,"
              << std::chrono::duration<double, std::milli>(
                     contiguousConvergenceEnd - contiguousConvergenceStart)
                     .count()
              << ',' << contiguousConvergence.stats.iterations << ','
              << contiguousConvergence.stats.finalPreconditionedResidualNorm
              << ','
              << terminationReason(
                     contiguousConvergence.stats.terminationReason)
              << '\n';

    if (options.check &&
        (multiplySpeedup < 8.0 || pcgSpeedup < 5.0 || blockPcgSpeedup < 5.0 ||
         multiplyError > 1e-10 || solutionError > 1e-9 ||
         blockSolutionError > 1e-9 || batchMultiplyError > 1e-10)) {
      std::cerr << "PCG performance or correctness gate failed\n";
      return 2;
    }
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
