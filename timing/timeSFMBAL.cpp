/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBAL.cpp
 * @brief   time SFM with BAL file,  conventional GeneralSFMFactor
 * @author  Frank Dellaert
 * @date    June 6, 2015
 */

#include "timeSFMBAL.h"

#include <gtsam/base/TaskScheduler.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/SparseEigen.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>

#include <Eigen/IterativeLinearSolvers>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <thread>

namespace {
constexpr const char* kDefaultBenchmarkDataset = "dubrovnik-16-22106-pre";
constexpr const char* kProfileDataset = "dubrovnik-135-90642-pre";

using Camera = PinholeCamera<Cal3Bundler>;
using SfmFactor = GeneralSFMFactor<Camera, Point3>;

std::string usage() {
  return "Usage: timeSFMBAL [--colamd] [--profile] [--camera-batch] "
         "[--cholesky-only] [--profile-point-cholesky] "
         "[--end-to-end-pcg] "
         "[--batch-chunk-size N] "
         "[--benchmark-action-json FILE] [BALfile]";
}

struct TimingRow {
  std::string dataset;
  double regularCholesky = 0.0;
  double regularQr = 0.0;
  double regularSolver = 0.0;
  double batchCholesky = 0.0;
  double batchQr = 0.0;
  double batchSolver = 0.0;
  double cameraFirstCholesky = 0.0;
  double cameraFirstQr = 0.0;
  double cameraFirstSolver = 0.0;
  double cameraBatchCholesky = 0.0;
  double cameraBatchQr = 0.0;
  double cameraBatchSolver = 0.0;
};

struct RunOptions {
  bool profile = false;
  bool benchmarkActionJson = false;
  bool cameraBatch = false;
  bool choleskyOnly = false;
  bool profilePointCholesky = false;
  size_t batchChunkSize = 0;
  std::string benchmarkActionJsonPath;
  std::vector<std::string> filenames;
  bool endToEndPcg = false;
};

struct PointCholeskyProfileRow {
  std::string dataset;
  std::string variant;
  size_t iterations = 0;
  size_t innerIterations = 0;
  size_t nonlinearFactors = 0;
  size_t linearFactors = 0;
  size_t jacobianFactors = 0;
  size_t batchFactors = 0;
  size_t linearRows = 0;
  size_t batchDenseScalarEntries = 0;
  size_t batchStoredScalarEntriesEstimate = 0;
  double initialError = 0.0;
  double finalError = 0.0;
  double linearize = 0.0;
  double hessianDiagonal = 0.0;
  double damp = 0.0;
  double symbolic = 0.0;
  double eliminate = 0.0;
  double backSubstitute = 0.0;
  double deltaError = 0.0;
  double retractAndError = 0.0;

  double solveTotal() const { return symbolic + eliminate + backSubstitute; }

  double total() const {
    return linearize + hessianDiagonal + damp + solveTotal() + deltaError +
           retractAndError;
  }
};

std::string escapeJson(std::string value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += c;
        break;
    }
  }
  return escaped;
}

void writeBenchmarkActionJson(const std::vector<TimingRow>& rows,
                              const std::string& outputPath,
                              bool includeCameraBatch, bool choleskyOnly) {
  std::ofstream out(outputPath);
  if (!out) {
    throw runtime_error("Unable to open benchmark JSON output file: " +
                        outputPath);
  }

  out << "[\n";
  bool first = true;
  const auto appendEntry = [&](const std::string& name, const double value) {
    if (!first) out << ",\n";
    first = false;
    out << "  {\n";
    out << "    \"name\": \"" << escapeJson(name) << "\",\n";
    out << "    \"unit\": \"s\",\n";
    out << "    \"value\": " << std::fixed << std::setprecision(9) << value
        << "\n";
    out << "  }";
  };

  for (const auto& row : rows) {
    appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalCholesky",
                row.regularCholesky);
    if (!choleskyOnly) {
      appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalQR",
                  row.regularQr);
      appendEntry("timeSFMBAL/" + row.dataset + "/MultifrontalSolver",
                  row.regularSolver);
    }
    appendEntry(
        "timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalCholesky",
        row.batchCholesky);
    if (!choleskyOnly) {
      appendEntry("timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalQR",
                  row.batchQr);
      appendEntry(
          "timeSFMBAL/" + row.dataset + "/BatchFactor/MultifrontalSolver",
          row.batchSolver);
    }
    if (!includeCameraBatch) continue;
    appendEntry(
        "timeSFMBAL/" + row.dataset + "/CameraFirst/MultifrontalCholesky",
        row.cameraFirstCholesky);
    if (!choleskyOnly) {
      appendEntry("timeSFMBAL/" + row.dataset + "/CameraFirst/MultifrontalQR",
                  row.cameraFirstQr);
      appendEntry(
          "timeSFMBAL/" + row.dataset + "/CameraFirst/MultifrontalSolver",
          row.cameraFirstSolver);
    }
    appendEntry(
        "timeSFMBAL/" + row.dataset + "/CameraBatchFactor/MultifrontalCholesky",
        row.cameraBatchCholesky);
    if (!choleskyOnly) {
      appendEntry(
          "timeSFMBAL/" + row.dataset + "/CameraBatchFactor/MultifrontalQR",
          row.cameraBatchQr);
      appendEntry(
          "timeSFMBAL/" + row.dataset + "/CameraBatchFactor/MultifrontalSolver",
          row.cameraBatchSolver);
    }
  }
  out << "\n]\n";
}

void writePointCholeskyProfileJson(
    const std::vector<PointCholeskyProfileRow>& rows,
    const std::string& outputPath) {
  std::ofstream out(outputPath);
  if (!out) {
    throw runtime_error("Unable to open profile JSON output file: " +
                        outputPath);
  }

  out << "[\n";
  bool first = true;
  const auto appendEntry = [&](const std::string& name, const std::string& unit,
                               const double value) {
    if (!first) out << ",\n";
    first = false;
    out << "  {\n";
    out << "    \"name\": \"" << escapeJson(name) << "\",\n";
    out << "    \"unit\": \"" << escapeJson(unit) << "\",\n";
    out << "    \"value\": " << std::fixed << std::setprecision(9) << value
        << "\n";
    out << "  }";
  };

  for (const auto& row : rows) {
    const std::string prefix = "timeSFMBAL/" + row.dataset +
                               "/PointFirstCholeskyProfile/" + row.variant +
                               "/";
    appendEntry(prefix + "linearize", "s", row.linearize);
    appendEntry(prefix + "hessianDiagonal", "s", row.hessianDiagonal);
    appendEntry(prefix + "damp", "s", row.damp);
    appendEntry(prefix + "symbolic", "s", row.symbolic);
    appendEntry(prefix + "eliminate", "s", row.eliminate);
    appendEntry(prefix + "backSubstitute", "s", row.backSubstitute);
    appendEntry(prefix + "deltaError", "s", row.deltaError);
    appendEntry(prefix + "retractAndError", "s", row.retractAndError);
    appendEntry(prefix + "total", "s", row.total());
    appendEntry(prefix + "iterations", "count",
                static_cast<double>(row.iterations));
    appendEntry(prefix + "innerIterations", "count",
                static_cast<double>(row.innerIterations));
    appendEntry(prefix + "linearFactors", "count",
                static_cast<double>(row.linearFactors));
    appendEntry(prefix + "batchDenseScalarEntries", "count",
                static_cast<double>(row.batchDenseScalarEntries));
    appendEntry(prefix + "batchStoredScalarEntriesEstimate", "count",
                static_cast<double>(row.batchStoredScalarEntriesEstimate));
  }
  out << "\n]\n";
}

RunOptions parseBalFiles(int argc, char* argv[]) {
  std::string filename;
  bool profile = false;
  bool benchmarkActionJson = false;
  bool cameraBatch = false;
  bool choleskyOnly = false;
  bool profilePointCholesky = false;
  bool endToEndPcg = false;
  bool colamd = false;
  size_t batchChunkSize = 0;
  std::string benchmarkActionJsonPath;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--colamd") == 0) {
      gUseSchur = false;
      colamd = true;
      continue;
    }
    if (strcmp(argv[i], "--profile") == 0) {
      profile = true;
      continue;
    }
    if (strcmp(argv[i], "--camera-batch") == 0) {
      cameraBatch = true;
      continue;
    }
    if (strcmp(argv[i], "--cholesky-only") == 0) {
      choleskyOnly = true;
      continue;
    }
    if (strcmp(argv[i], "--batch-chunk-size") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      batchChunkSize = std::stoul(argv[i]);
      continue;
    }
    if (strcmp(argv[i], "--profile-point-cholesky") == 0) {
      profilePointCholesky = true;
      continue;
    }
    if (strcmp(argv[i], "--end-to-end-pcg") == 0) {
      endToEndPcg = true;
      continue;
    }
    if (strcmp(argv[i], "--benchmark-action-json") == 0) {
      if (++i >= argc || argv[i][0] == '-') {
        throw runtime_error(usage());
      }
      benchmarkActionJson = true;
      benchmarkActionJsonPath = argv[i];
      continue;
    }
    if (argv[i][0] == '-') {
      throw runtime_error(usage());
    }
    if (!filename.empty()) {
      throw runtime_error(usage());
    }
    filename = argv[i];
  }

  if (profile && !filename.empty()) {
    throw runtime_error(usage());
  }
  if (profile && (benchmarkActionJson || cameraBatch || choleskyOnly ||
                  profilePointCholesky)) {
    throw runtime_error(usage());
  }
  if (profilePointCholesky && cameraBatch) {
    throw runtime_error(
        "--profile-point-cholesky profiles the point-first "
        "path; do not combine it with --camera-batch.");
  }
  if (cameraBatch && colamd) {
    throw runtime_error(
        "--camera-batch uses camera-first ordering; do not "
        "combine it with --colamd.");
  }
  if (endToEndPcg &&
      (profile || benchmarkActionJson || cameraBatch || choleskyOnly ||
       profilePointCholesky || batchChunkSize != 0)) {
    throw runtime_error(
        "--end-to-end-pcg is a standalone comparison; combine it only with "
        "--colamd or a BAL file.");
  }

  if (!filename.empty()) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            batchChunkSize,
            benchmarkActionJsonPath,
            {filename},
            endToEndPcg};
  }

  if (profile) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            batchChunkSize,
            benchmarkActionJsonPath,
            {findExampleDataFile(kProfileDataset)},
            endToEndPcg};
  }

  if (profilePointCholesky) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            batchChunkSize,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)},
            endToEndPcg};
  }

  if (benchmarkActionJson) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            batchChunkSize,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)},
            endToEndPcg};
  }

  if (endToEndPcg) {
    return {profile,
            benchmarkActionJson,
            cameraBatch,
            choleskyOnly,
            profilePointCholesky,
            batchChunkSize,
            benchmarkActionJsonPath,
            {findExampleDataFile(kDefaultBenchmarkDataset)},
            endToEndPcg};
  }

  return {profile,
          benchmarkActionJson,
          cameraBatch,
          choleskyOnly,
          profilePointCholesky,
          batchChunkSize,
          benchmarkActionJsonPath,
          {
              findExampleDataFile("dubrovnik-16-22106-pre"),
              findExampleDataFile("dubrovnik-88-64298-pre"),
              findExampleDataFile("dubrovnik-135-90642-pre"),
          },
          endToEndPcg};
}

double runSolver(const NonlinearFactorGraph& graph, const Values& initial,
                 const Ordering& ordering,
                 NonlinearOptimizerParams::LinearSolverType solverType,
                 const std::string& label) {
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setVerbosityLM("SUMMARY");
  params.setRelativeErrorTol(0.01);
  params.linearSolverType = solverType;
  if (solverType == NonlinearOptimizerParams::MULTIFRONTAL_SOLVER) {
    params.multifrontalParams.qrMode = MultifrontalParameters::QRMode::Allow;
  }
  if (gUseSchur) {
    params.setOrdering(ordering);
  }

  auto start = std::chrono::high_resolution_clock::now();
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  lm.optimize();
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed = end - start;
  std::cout << "  " << label << ": " << elapsed.count() << " s\n";
  return elapsed.count();
}

constexpr size_t kEndToEndPcgMaxIterations = 500;
constexpr double kEndToEndPcgRelativeTolerance = 1e-3;

struct LinearSolveStats {
  size_t solves = 0;
  size_t iterations = 0;
  size_t nonConvergedSolves = 0;
};

struct EndToEndResult {
  std::string dataset;
  std::string solver;
  double elapsedSeconds = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t lmIterations = 0;
  size_t lmInnerIterations = 0;
  LinearSolveStats linear;
};

PCGSolverParameters endToEndPcgParameters() {
  PCGSolverParameters parameters(
      std::make_shared<BlockJacobiPreconditionerParameters>());
  parameters.minIterations = 0;
  parameters.maxIterations = kEndToEndPcgMaxIterations;
  parameters.reset = kEndToEndPcgMaxIterations + 1;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = kEndToEndPcgRelativeTolerance;
  parameters.parallel = true;
  parameters.numThreads = 0;
  return parameters;
}

VectorValues solveParallelPcgGraph(const GaussianFactorGraph& graph,
                                   const PCGSolverParameters& parameters,
                                   LinearSolveStats* stats) {
  const PCGSolverResult result =
      PCGSolver(parameters).optimizeDetailed(graph, false);
  ++stats->solves;
  stats->iterations += result.stats.iterations;
  if (!result.stats.converged()) ++stats->nonConvergedSolves;
  if (result.stats.terminationReason ==
      ConjugateGradientTerminationReason::kNumericalBreakdown) {
    throw std::runtime_error("Parallel PCG encountered numerical breakdown");
  }
  return result.solution;
}

class ParallelPcgLevenbergMarquardtOptimizer final
    : public LevenbergMarquardtOptimizer {
  PCGSolverParameters pcgParameters_;
  mutable LinearSolveStats linearStats_;

 public:
  ParallelPcgLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initial,
      const LevenbergMarquardtParams& lmParameters)
      : LevenbergMarquardtOptimizer(graph, initial, lmParameters),
        pcgParameters_(endToEndPcgParameters()) {}

  VectorValues solve(const GaussianFactorGraph& graph,
                     const NonlinearOptimizerParams&) const override {
    return solveParallelPcgGraph(graph, pcgParameters_, &linearStats_);
  }

  const LinearSolveStats& linearStats() const { return linearStats_; }
};

class EigenBlockJacobiPreconditioner {
  std::vector<size_t> dimensions_;
  std::vector<size_t> scalarOffsets_;
  std::vector<Matrix> inverseBlocks_;
  Eigen::ComputationInfo info_ = Eigen::InvalidInput;

 public:
  using StorageIndex = int;
  enum { ColsAtCompileTime = Eigen::Dynamic };
  enum { MaxColsAtCompileTime = Eigen::Dynamic };

  EigenBlockJacobiPreconditioner& setDimensions(
      std::vector<size_t> dimensions) {
    dimensions_ = std::move(dimensions);
    return *this;
  }

  Eigen::Index rows() const {
    return scalarOffsets_.empty()
               ? 0
               : static_cast<Eigen::Index>(scalarOffsets_.back());
  }
  Eigen::Index cols() const { return rows(); }

  template <class MATRIX>
  EigenBlockJacobiPreconditioner& analyzePattern(const MATRIX&) {
    return *this;
  }

  template <class MATRIX>
  EigenBlockJacobiPreconditioner& factorize(const MATRIX& matrix) {
    scalarOffsets_.assign(dimensions_.size() + 1, 0);
    inverseBlocks_.resize(dimensions_.size());
    for (size_t block = 0; block < dimensions_.size(); ++block) {
      const size_t offset = scalarOffsets_[block];
      const size_t dimension = dimensions_[block];
      scalarOffsets_[block + 1] = offset + dimension;

      Matrix diagonalBlock(dimension, dimension);
      for (size_t row = 0; row < dimension; ++row) {
        for (size_t column = 0; column < dimension; ++column) {
          diagonalBlock(static_cast<DenseIndex>(row),
                        static_cast<DenseIndex>(column)) =
              matrix.coeff(static_cast<Eigen::Index>(offset + row),
                           static_cast<Eigen::Index>(offset + column));
        }
      }
      const Eigen::LLT<Matrix> factorization(diagonalBlock);
      if (factorization.info() != Eigen::Success) {
        info_ = Eigen::NumericalIssue;
        return *this;
      }
      inverseBlocks_[block] = factorization.solve(
          Matrix::Identity(static_cast<DenseIndex>(dimension),
                           static_cast<DenseIndex>(dimension)));
    }
    info_ = Eigen::Success;
    return *this;
  }

  template <class MATRIX>
  EigenBlockJacobiPreconditioner& compute(const MATRIX& matrix) {
    return factorize(matrix);
  }

  template <class RHS, class DESTINATION>
  void _solve_impl(const RHS& rhs, DESTINATION& result) const {
    result.resize(rhs.rows());
    for (size_t block = 0; block < dimensions_.size(); ++block) {
      const DenseIndex offset = static_cast<DenseIndex>(scalarOffsets_[block]);
      const DenseIndex dimension = static_cast<DenseIndex>(dimensions_[block]);
      result.segment(offset, dimension).noalias() =
          inverseBlocks_[block] * rhs.segment(offset, dimension);
    }
  }

  template <class RHS>
  const Eigen::Solve<EigenBlockJacobiPreconditioner, RHS> solve(
      const Eigen::MatrixBase<RHS>& rhs) const {
    return Eigen::Solve<EigenBlockJacobiPreconditioner, RHS>(*this,
                                                             rhs.derived());
  }

  Eigen::ComputationInfo info() const { return info_; }
};

VectorValues solveEigenPcgHessian(SparseEigen hessian, const Vector& rhs,
                                  const KeyInfo& keyInfo,
                                  LinearSolveStats* stats) {
  // Include Eigen preconditioner construction and CG in every linear solve.
  Eigen::ConjugateGradient<SparseEigen, Eigen::Lower | Eigen::Upper,
                           EigenBlockJacobiPreconditioner>
      solver;
  solver.setMaxIterations(static_cast<Eigen::Index>(kEndToEndPcgMaxIterations));
  solver.setTolerance(kEndToEndPcgRelativeTolerance);
  solver.preconditioner().setDimensions(keyInfo.colSpec());
  hessian.makeCompressed();
  solver.compute(hessian);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigen PCG normal-equation setup failed");
  }

  const DenseIndex columns = static_cast<DenseIndex>(keyInfo.numCols());
  const Vector solution = solver.solveWithGuess(rhs, Vector::Zero(columns));
  ++stats->solves;
  stats->iterations += static_cast<size_t>(solver.iterations());
  if (solver.info() == Eigen::NoConvergence) {
    ++stats->nonConvergedSolves;
  } else if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Eigen PCG solve failed");
  }
  return buildVectorValues(solution, keyInfo);
}

VectorValues solveEigenPcgGraph(const GaussianFactorGraph& graph,
                                LinearSolveStats* stats) {
  // Materialize the damped normal equations for this LM inner iteration.
  const KeyInfo keyInfo(graph);
  const SparseEigen augmented = sparseJacobianEigen(graph, keyInfo.ordering());
  const DenseIndex columns = static_cast<DenseIndex>(keyInfo.numCols());
  const SparseEigen jacobian = augmented.leftCols(columns);
  Vector rowRhs = Vector::Zero(augmented.rows());
  for (SparseEigen::InnerIterator entry(augmented, columns); entry; ++entry) {
    rowRhs(entry.row()) = entry.value();
  }
  SparseEigen hessian = jacobian.transpose() * jacobian;
  const Vector rhs = jacobian.transpose() * rowRhs;
  return solveEigenPcgHessian(std::move(hessian), rhs, keyInfo, stats);
}

VectorValues solveEigenPcgExplicitHessianGraph(const GaussianFactorGraph& graph,
                                               LinearSolveStats* stats) {
  const KeyInfo keyInfo(graph);
  std::vector<Eigen::Triplet<double, int>> entries;

  // Assemble factor information directly, avoiding Hessian square roots.
  for (const auto& factor : graph) {
    if (!factor) continue;
    const Matrix information = factor->information();
    std::vector<size_t> localToGlobal;
    localToGlobal.reserve(static_cast<size_t>(information.rows()));
    for (auto key = factor->begin(); key != factor->end(); ++key) {
      const auto keyEntry = keyInfo.find(*key);
      if (keyEntry == keyInfo.end()) {
        throw std::runtime_error(
            "Eigen reduced Hessian contains an unknown key");
      }
      const size_t dimension = static_cast<size_t>(factor->getDim(key));
      for (size_t coordinate = 0; coordinate < dimension; ++coordinate) {
        localToGlobal.push_back(keyEntry->second.start + coordinate);
      }
    }
    for (DenseIndex column = 0; column < information.cols(); ++column) {
      for (DenseIndex row = 0; row < information.rows(); ++row) {
        const double value = information(row, column);
        if (value == 0.0) continue;
        entries.emplace_back(
            static_cast<int>(localToGlobal[static_cast<size_t>(row)]),
            static_cast<int>(localToGlobal[static_cast<size_t>(column)]),
            value);
      }
    }
  }

  SparseEigen hessian(static_cast<int>(keyInfo.numCols()),
                      static_cast<int>(keyInfo.numCols()));
  hessian.setFromTriplets(entries.begin(), entries.end());
  const Vector rhs = -graph.gradientAtZero().vector(keyInfo.ordering());
  return solveEigenPcgHessian(std::move(hessian), rhs, keyInfo, stats);
}

class EigenPcgLevenbergMarquardtOptimizer final
    : public LevenbergMarquardtOptimizer {
  mutable LinearSolveStats linearStats_;

 public:
  using LevenbergMarquardtOptimizer::LevenbergMarquardtOptimizer;

  VectorValues solve(const GaussianFactorGraph& graph,
                     const NonlinearOptimizerParams&) const override {
    return solveEigenPcgGraph(graph, &linearStats_);
  }

  const LinearSolveStats& linearStats() const { return linearStats_; }
};

enum class ReducedCameraBackend { kParallelPcg, kEigenPcg };

using ImplicitSfmSchurFactor = RegularImplicitSchurFactor<Camera>;
using CameraJacobianBlock = Matrix29;
using CameraJacobianBlocks =
    std::vector<CameraJacobianBlock,
                Eigen::aligned_allocator<CameraJacobianBlock>>;

struct LandmarkBackSubstitution {
  Key landmark;
  KeyVector cameraKeys;
  CameraJacobianBlocks cameraJacobians;
  Matrix pointJacobian;
  Matrix3 pointCovariance;
  Vector measurementRhs;
  Vector3 pointInformationRhs;
};

struct ReducedCameraSystem {
  std::vector<LandmarkBackSubstitution> landmarks;
  GaussianFactorGraph cameraGraph;
};

struct LandmarkReduction {
  LandmarkBackSubstitution backSubstitution;
  GaussianFactor::shared_ptr cameraFactor;
};

size_t parallelWorkerCount(size_t taskCount) {
  const size_t hardwareThreads = std::thread::hardware_concurrency();
  return std::max<size_t>(
      1, std::min(taskCount, std::max<size_t>(1, hardwareThreads)));
}

std::optional<Key> measurementCameraKey(const GaussianFactor& factor,
                                        Key landmark) {
  std::optional<Key> cameraKey;
  for (const Key key : factor.keys()) {
    if (key == landmark) continue;
    if (cameraKey) {
      throw std::runtime_error(
          "Reduced-camera BAL benchmark requires binary measurements");
    }
    cameraKey = key;
  }
  return cameraKey;
}

void appendLandmarkMeasurement(
    Key landmark, Key cameraKey, const GaussianFactor& factor,
    LandmarkBackSubstitution* backSubstitution,
    std::vector<Matrix23, Eigen::aligned_allocator<Matrix23>>* pointJacobians,
    std::vector<Vector2, Eigen::aligned_allocator<Vector2>>* measurementRhs) {
  if (std::find(backSubstitution->cameraKeys.begin(),
                backSubstitution->cameraKeys.end(),
                cameraKey) != backSubstitution->cameraKeys.end()) {
    throw std::runtime_error(
        "Reduced-camera BAL benchmark found duplicate camera observations");
  }

  const auto [matrix, rhs] = factor.jacobian();
  if (matrix.rows() != 2 || rhs.size() != 2) {
    throw std::runtime_error(
        "Reduced-camera BAL benchmark requires 2D measurements");
  }
  CameraJacobianBlock cameraJacobian;
  Matrix23 pointJacobian;
  DenseIndex columnOffset = 0;
  for (auto key = factor.begin(); key != factor.end(); ++key) {
    const DenseIndex dimension = factor.getDim(key);
    if (*key == landmark) {
      if (dimension != 3) {
        throw std::runtime_error(
            "Reduced-camera BAL benchmark requires 3D landmarks");
      }
      pointJacobian = matrix.block<2, 3>(0, columnOffset);
    } else {
      if (dimension != CameraJacobianBlock::ColsAtCompileTime) {
        throw std::runtime_error(
            "Reduced-camera BAL benchmark found a non-camera variable");
      }
      cameraJacobian = matrix.block<2, 9>(0, columnOffset);
    }
    columnOffset += dimension;
  }
  backSubstitution->cameraKeys.push_back(cameraKey);
  backSubstitution->cameraJacobians.push_back(cameraJacobian);
  pointJacobians->push_back(pointJacobian);
  measurementRhs->push_back(rhs);
}

void stackLandmarkMeasurements(
    const std::vector<Matrix23, Eigen::aligned_allocator<Matrix23>>&
        pointJacobians,
    const std::vector<Vector2, Eigen::aligned_allocator<Vector2>>& rhsBlocks,
    LandmarkBackSubstitution* backSubstitution) {
  const size_t measurementCount = backSubstitution->cameraKeys.size();
  backSubstitution->pointJacobian.resize(2 * measurementCount, 3);
  backSubstitution->measurementRhs.resize(2 * measurementCount);
  for (size_t index = 0; index < measurementCount; ++index) {
    backSubstitution->pointJacobian.block<2, 3>(2 * index, 0) =
        pointJacobians[index];
    backSubstitution->measurementRhs.segment<2>(2 * index) = rhsBlocks[index];
  }
}

GaussianFactor::shared_ptr createReducedCameraFactor(
    const LandmarkBackSubstitution& landmark, ReducedCameraBackend backend) {
  if (backend == ReducedCameraBackend::kParallelPcg) {
    return std::make_shared<ImplicitSfmSchurFactor>(
        landmark.cameraKeys, landmark.cameraJacobians, landmark.pointJacobian,
        landmark.pointCovariance, landmark.measurementRhs);
  }
  const SymmetricBlockMatrix augmentedHessian =
      CameraSet<Camera>::SchurComplement<3, 9>(
          landmark.cameraJacobians, landmark.pointJacobian,
          landmark.pointCovariance, landmark.measurementRhs);
  return std::make_shared<HessianFactor>(landmark.cameraKeys, augmentedHessian);
}

LandmarkReduction reduceLandmark(Key landmark, const GaussianFactorGraph& graph,
                                 ReducedCameraBackend backend) {
  LandmarkBackSubstitution backSubstitution;
  backSubstitution.landmark = landmark;
  backSubstitution.pointInformationRhs = Vector3::Zero();
  Matrix3 pointInformation = Matrix3::Zero();
  std::vector<Matrix23, Eigen::aligned_allocator<Matrix23>> pointJacobians;
  std::vector<Vector2, Eigen::aligned_allocator<Vector2>> measurementRhs;

  // Extract one whitened 2D measurement block for each observing camera.
  for (const auto& factor : graph) {
    const std::optional<Key> cameraKey =
        measurementCameraKey(*factor, landmark);
    if (!cameraKey) {
      pointInformation += factor->information();
      const VectorValues gradient = factor->gradientAtZero();
      if (gradient.exists(landmark)) {
        backSubstitution.pointInformationRhs -= gradient.at(landmark);
      }
      continue;
    }
    appendLandmarkMeasurement(landmark, *cameraKey, *factor, &backSubstitution,
                              &pointJacobians, &measurementRhs);
  }

  stackLandmarkMeasurements(pointJacobians, measurementRhs, &backSubstitution);

  // Point damping changes the inverse point Hessian used by both solvers.
  const Matrix3 pointHessian = backSubstitution.pointJacobian.transpose() *
                                   backSubstitution.pointJacobian +
                               pointInformation;
  const Eigen::LLT<Matrix3> pointFactorization(pointHessian);
  if (pointFactorization.info() != Eigen::Success) {
    throw std::runtime_error(
        "Reduced-camera BAL benchmark found a singular landmark block");
  }
  backSubstitution.pointCovariance =
      pointFactorization.solve(Matrix3::Identity());
  if (!backSubstitution.pointInformationRhs.isZero()) {
    throw std::runtime_error(
        "Reduced-camera BAL benchmark does not support landmark priors");
  }

  GaussianFactor::shared_ptr cameraFactor =
      createReducedCameraFactor(backSubstitution, backend);
  return {std::move(backSubstitution), std::move(cameraFactor)};
}

ReducedCameraSystem buildReducedCameraSystemParallel(
    const GaussianFactorGraph& graph, ReducedCameraBackend backend) {
  std::map<Key, GaussianFactorGraph> factorsByLandmark;
  ReducedCameraSystem reduced;

  // Partition the damped linear graph into independent landmark cliques.
  for (const auto& factor : graph) {
    if (!factor) continue;
    bool hasLandmark = false;
    Key landmark = 0;
    for (const Key key : factor->keys()) {
      if (symbolChr(key) != 'p') continue;
      if (hasLandmark && landmark != key) {
        throw std::runtime_error(
            "Reduced-camera BAL benchmark found a factor coupling landmarks");
      }
      hasLandmark = true;
      landmark = key;
    }
    if (hasLandmark) {
      factorsByLandmark[landmark].push_back(factor);
    } else {
      reduced.cameraGraph.push_back(factor);
    }
  }

  std::vector<Key> landmarks;
  std::vector<const GaussianFactorGraph*> landmarkGraphs;
  landmarks.reserve(factorsByLandmark.size());
  landmarkGraphs.reserve(factorsByLandmark.size());
  for (const auto& [landmark, factors] : factorsByLandmark) {
    landmarks.push_back(landmark);
    landmarkGraphs.push_back(&factors);
  }

  std::vector<LandmarkReduction> landmarkReductions(landmarks.size());
  std::atomic<size_t> nextLandmark{0};
  const size_t workerCount = parallelWorkerCount(landmarks.size());
  TaskScheduler<void> scheduler(workerCount);
  for (size_t worker = 0; worker < workerCount; ++worker) {
    scheduler.enqueue([&] {
      while (true) {
        const size_t index = nextLandmark.fetch_add(1);
        if (index >= landmarks.size()) return;
        landmarkReductions[index] =
            reduceLandmark(landmarks[index], *landmarkGraphs[index], backend);
      }
    });
  }
  scheduler.waitForAllTasks();

  // Preserve deterministic key order when assembling both camera graphs.
  reduced.landmarks.reserve(landmarkReductions.size());
  for (LandmarkReduction& reduction : landmarkReductions) {
    reduced.landmarks.push_back(std::move(reduction.backSubstitution));
    reduced.cameraGraph.push_back(std::move(reduction.cameraFactor));
  }
  return reduced;
}

VectorValues backSubstituteLandmarksParallel(
    const std::vector<LandmarkBackSubstitution>& landmarks,
    const VectorValues& cameraSolution) {
  std::vector<Vector3, Eigen::aligned_allocator<Vector3>> landmarkSolutions(
      landmarks.size());
  std::atomic<size_t> nextLandmark{0};
  const size_t workerCount = parallelWorkerCount(landmarks.size());
  TaskScheduler<void> scheduler(workerCount);
  for (size_t worker = 0; worker < workerCount; ++worker) {
    scheduler.enqueue([&] {
      while (true) {
        const size_t index = nextLandmark.fetch_add(1);
        if (index >= landmarks.size()) return;
        const LandmarkBackSubstitution& landmark = landmarks[index];
        Vector residual = landmark.measurementRhs;
        for (size_t camera = 0; camera < landmark.cameraKeys.size(); ++camera) {
          residual.segment<2>(2 * camera).noalias() -=
              landmark.cameraJacobians[camera] *
              cameraSolution.at(landmark.cameraKeys[camera]);
        }
        landmarkSolutions[index] =
            landmark.pointCovariance *
            (landmark.pointJacobian.transpose() * residual +
             landmark.pointInformationRhs);
      }
    });
  }
  scheduler.waitForAllTasks();

  VectorValues solution = cameraSolution;
  for (size_t index = 0; index < landmarks.size(); ++index) {
    solution.insert(landmarks[index].landmark, landmarkSolutions[index]);
  }
  return solution;
}

class ReducedCameraPcgLevenbergMarquardtOptimizer final
    : public LevenbergMarquardtOptimizer {
  ReducedCameraBackend backend_;
  PCGSolverParameters pcgParameters_;
  mutable LinearSolveStats linearStats_;

 public:
  ReducedCameraPcgLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initial,
      const LevenbergMarquardtParams& lmParameters,
      ReducedCameraBackend backend)
      : LevenbergMarquardtOptimizer(graph, initial, lmParameters),
        backend_(backend),
        pcgParameters_(endToEndPcgParameters()) {}

  VectorValues solve(const GaussianFactorGraph& graph,
                     const NonlinearOptimizerParams&) const override {
    // Eliminate all points, solve the camera Schur complement, then recover
    // independent point updates from the cached per-track Jacobians.
    const ReducedCameraSystem reduced =
        buildReducedCameraSystemParallel(graph, backend_);
    const VectorValues cameraSolution =
        backend_ == ReducedCameraBackend::kParallelPcg
            ? solveParallelPcgGraph(reduced.cameraGraph, pcgParameters_,
                                    &linearStats_)
            : solveEigenPcgExplicitHessianGraph(reduced.cameraGraph,
                                                &linearStats_);
    return backSubstituteLandmarksParallel(reduced.landmarks, cameraSolution);
  }

  const LinearSolveStats& linearStats() const { return linearStats_; }
};

LevenbergMarquardtParams endToEndParameters(
    const Ordering& ordering,
    NonlinearOptimizerParams::LinearSolverType solverType) {
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtParams::SetCeresDefaults(&parameters);
  parameters.setVerbosityLM("SILENT");
  parameters.setRelativeErrorTol(0.01);
  parameters.linearSolverType = solverType;
  if (gUseSchur) parameters.setOrdering(ordering);
  return parameters;
}

EndToEndResult runDirectEndToEnd(const std::string& dataset,
                                 const NonlinearFactorGraph& graph,
                                 const Values& initial,
                                 const Ordering& ordering) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = "MultifrontalCholesky";
  result.initialError = graph.error(initial);
  const auto parameters = endToEndParameters(
      ordering, NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);

  const auto start = std::chrono::high_resolution_clock::now();
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  optimizer.optimize();
  const auto end = std::chrono::high_resolution_clock::now();

  result.elapsedSeconds = std::chrono::duration<double>(end - start).count();
  result.finalError = optimizer.error();
  result.lmIterations = optimizer.iterations();
  result.lmInnerIterations =
      static_cast<size_t>(optimizer.getInnerIterations());
  return result;
}

EndToEndResult runParallelPcgEndToEnd(const std::string& dataset,
                                      const NonlinearFactorGraph& graph,
                                      const Values& initial,
                                      const Ordering& ordering) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = "ParallelPCG(BlockJacobi)";
  result.initialError = graph.error(initial);
  const auto parameters =
      endToEndParameters(ordering, NonlinearOptimizerParams::Iterative);

  const auto start = std::chrono::high_resolution_clock::now();
  ParallelPcgLevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  optimizer.optimize();
  const auto end = std::chrono::high_resolution_clock::now();

  result.elapsedSeconds = std::chrono::duration<double>(end - start).count();
  result.finalError = optimizer.error();
  result.lmIterations = optimizer.iterations();
  result.lmInnerIterations =
      static_cast<size_t>(optimizer.getInnerIterations());
  result.linear = optimizer.linearStats();
  return result;
}

EndToEndResult runEigenPcgEndToEnd(const std::string& dataset,
                                   const NonlinearFactorGraph& graph,
                                   const Values& initial,
                                   const Ordering& ordering) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = "EigenPCG(ExplicitH,BlockJacobi)";
  result.initialError = graph.error(initial);
  const auto parameters =
      endToEndParameters(ordering, NonlinearOptimizerParams::Iterative);

  const auto start = std::chrono::high_resolution_clock::now();
  EigenPcgLevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  optimizer.optimize();
  const auto end = std::chrono::high_resolution_clock::now();

  result.elapsedSeconds = std::chrono::duration<double>(end - start).count();
  result.finalError = optimizer.error();
  result.lmIterations = optimizer.iterations();
  result.lmInnerIterations =
      static_cast<size_t>(optimizer.getInnerIterations());
  result.linear = optimizer.linearStats();
  return result;
}

EndToEndResult runReducedCameraPcgEndToEnd(const std::string& dataset,
                                           const NonlinearFactorGraph& graph,
                                           const Values& initial,
                                           const Ordering& ordering,
                                           ReducedCameraBackend backend) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = backend == ReducedCameraBackend::kParallelPcg
                      ? "ReducedParallelPCG(ImplicitSchur,BlockJacobi)"
                      : "ReducedEigenPCG(ExplicitH,BlockJacobi)";
  result.initialError = graph.error(initial);
  const auto parameters =
      endToEndParameters(ordering, NonlinearOptimizerParams::Iterative);

  const auto start = std::chrono::high_resolution_clock::now();
  ReducedCameraPcgLevenbergMarquardtOptimizer optimizer(graph, initial,
                                                        parameters, backend);
  optimizer.optimize();
  const auto end = std::chrono::high_resolution_clock::now();

  result.elapsedSeconds = std::chrono::duration<double>(end - start).count();
  result.finalError = optimizer.error();
  result.lmIterations = optimizer.iterations();
  result.lmInnerIterations =
      static_cast<size_t>(optimizer.getInnerIterations());
  result.linear = optimizer.linearStats();
  return result;
}

void printEndToEndSummary(const std::vector<EndToEndResult>& results) {
  std::map<std::string, std::map<std::string, double>> timings;
  for (const EndToEndResult& result : results) {
    timings[result.dataset][result.solver] = result.elapsedSeconds;
  }

  std::cout << std::fixed << std::setprecision(6)
            << "\n| Dataset | Cholesky s | Full Parallel PCG s | Full Eigen "
               "PCG s | Reduced Parallel PCG s | Reduced Eigen PCG s |\n"
            << "| --- | ---: | ---: | ---: | ---: | ---: |\n";
  for (const auto& [dataset, datasetTimings] : timings) {
    std::cout << "| " << dataset << " | "
              << datasetTimings.at("MultifrontalCholesky") << " | "
              << datasetTimings.at("ParallelPCG(BlockJacobi)") << " | "
              << datasetTimings.at("EigenPCG(ExplicitH,BlockJacobi)") << " | "
              << datasetTimings.at(
                     "ReducedParallelPCG(ImplicitSchur,BlockJacobi)")
              << " | "
              << datasetTimings.at("ReducedEigenPCG(ExplicitH,BlockJacobi)")
              << " |\n";
  }
}

void printEndToEndResults(const std::vector<EndToEndResult>& results) {
  printEndToEndSummary(results);
  std::map<std::string, double> directSeconds;
  for (const EndToEndResult& result : results) {
    if (result.solver == "MultifrontalCholesky") {
      directSeconds[result.dataset] = result.elapsedSeconds;
    }
  }

  std::cout << std::fixed << std::setprecision(6)
            << "\n| Dataset | Solver | Total s | vs. Cholesky | Initial error "
               "| Final error | LM iterations | LM inner iterations | Linear "
               "solves | Avg. CG iterations | Non-converged CG solves |\n"
            << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- "
               "| --- |\n";
  for (const EndToEndResult& result : results) {
    const double averageCgIterations =
        result.linear.solves == 0
            ? 0.0
            : static_cast<double>(result.linear.iterations) /
                  static_cast<double>(result.linear.solves);
    std::cout << "| " << result.dataset << " | " << result.solver << " | "
              << result.elapsedSeconds << " | "
              << directSeconds.at(result.dataset) / result.elapsedSeconds
              << "x | " << result.initialError << " | " << result.finalError
              << " | " << result.lmIterations << " | "
              << result.lmInnerIterations << " | ";
    if (result.linear.solves == 0) {
      std::cout << "n/a | n/a | n/a |\n";
    } else {
      std::cout << result.linear.solves << " | " << averageCgIterations << " | "
                << result.linear.nonConvergedSolves << " |\n";
    }
  }
}

void runEndToEndPcgComparison(const std::vector<std::string>& filenames) {
  std::vector<EndToEndResult> results;
  for (const std::string& filename : filenames) {
    const std::string dataset =
        std::filesystem::path(filename).filename().string();
    std::cout << "\nEnd-to-end PCG comparison for BAL file: " << filename
              << std::endl;
    const SfmData db = SfmData::FromBalFile(filename);
    const NonlinearFactorGraph graph = buildGeneralSfmGraph(db);
    const Values initial = buildGeneralSfmInitial(db);
    const Ordering ordering = createSchurOrdering(db, false);

    results.push_back(runDirectEndToEnd(dataset, graph, initial, ordering));
    results.push_back(
        runParallelPcgEndToEnd(dataset, graph, initial, ordering));
    results.push_back(runEigenPcgEndToEnd(dataset, graph, initial, ordering));
    results.push_back(runReducedCameraPcgEndToEnd(
        dataset, graph, initial, ordering, ReducedCameraBackend::kParallelPcg));
    results.push_back(runReducedCameraPcgEndToEnd(
        dataset, graph, initial, ordering, ReducedCameraBackend::kEigenPcg));
  }
  printEndToEndResults(results);
}

NonlinearFactorGraph buildBatchSfmGraph(const SfmData& db,
                                        bool useHessianFactor,
                                        size_t chunkSize) {
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < db.numberTracks(); j++) {
    const auto& measurementsForTrack = db.tracks[j].measurements;
    if (measurementsForTrack.size() < 2) continue;

    const size_t nMeasurements = measurementsForTrack.size();
    const size_t effectiveChunkSize =
        (chunkSize == 0) ? nMeasurements : std::min(chunkSize, nMeasurements);
    if (effectiveChunkSize == 0) continue;

    if (effectiveChunkSize >= nMeasurements) {
      std::map<Key, Point2> measurements;
      for (const SfmMeasurement& measurement : measurementsForTrack) {
        measurements[C(measurement.first)] = measurement.second;
      }

      auto batch = std::make_shared<BatchFactor<SfmFactor, 2>>(
          measurements, P(j), gNoiseModel);
      batch->setUseHessianFactor(useHessianFactor);
      graph.add(batch);
      continue;
    }

    for (size_t start = 0; start < nMeasurements; start += effectiveChunkSize) {
      const size_t end = std::min(start + effectiveChunkSize, nMeasurements);
      std::map<Key, Point2> measurements;
      for (size_t i = start; i < end; ++i) {
        const SfmMeasurement& measurement = measurementsForTrack[i];
        measurements[C(measurement.first)] = measurement.second;
      }
      auto batch = std::make_shared<BatchFactor<SfmFactor, 2>>(
          measurements, P(j), gNoiseModel);
      batch->setUseHessianFactor(useHessianFactor);
      graph.add(batch);
    }
  }
  return graph;
}

NonlinearFactorGraph buildCameraBatchSfmGraph(const SfmData& db) {
  NonlinearFactorGraph graph;
  std::vector<std::map<Key, Point2>> measurementsByCamera(db.numberCameras());

  for (size_t j = 0; j < db.numberTracks(); j++) {
    const auto& measurementsForTrack = db.tracks[j].measurements;
    if (measurementsForTrack.size() < 2) continue;

    for (const SfmMeasurement& measurement : measurementsForTrack) {
      measurementsByCamera[measurement.first][P(j)] = measurement.second;
    }
  }

  for (size_t i = 0; i < measurementsByCamera.size(); ++i) {
    const auto& measurements = measurementsByCamera[i];
    if (measurements.empty()) continue;

    graph.add(std::make_shared<BatchFactor<SfmFactor, 2>>(C(i), measurements,
                                                          gNoiseModel));
  }
  return graph;
}

Ordering createCameraFirstOrdering(const SfmData& db) {
  Ordering ordering;
  for (size_t i = 0; i < db.numberCameras(); i++) ordering.push_back(C(i));
  for (size_t j = 0; j < db.numberTracks(); j++) ordering.push_back(P(j));
  return ordering;
}

double speedup(double baseline, double candidate) {
  return candidate > 0.0 ? (baseline / candidate) : 0.0;
}

template <typename Callable>
double measureSeconds(Callable&& callable) {
  const auto start = std::chrono::high_resolution_clock::now();
  callable();
  const auto end = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double>(end - start).count();
}

void collectLinearStats(const GaussianFactorGraph& linear,
                        PointCholeskyProfileRow* row) {
  row->linearFactors = linear.size();
  for (const auto& factor : linear) {
    if (!factor) continue;
    if (const auto batch =
            std::dynamic_pointer_cast<BatchJacobianFactorBase>(factor)) {
      row->batchFactors++;
      const size_t rows = batch->rows();
      row->linearRows += rows;

      size_t denseColumns = 1;  // RHS column.
      for (auto it = batch->begin(); it != batch->end(); ++it) {
        denseColumns += static_cast<size_t>(batch->getDim(it));
      }
      row->batchDenseScalarEntries += rows * denseColumns;

      // SFM factors have 2 residual rows and touch one 9D camera plus one 3D
      // point per measurement. This estimates the compact row-sparse payload.
      constexpr size_t kSfmStoredScalarsPerMeasurement = 2 * (9 + 3 + 1);
      row->batchStoredScalarEntriesEstimate +=
          (rows / 2) * kSfmStoredScalarsPerMeasurement;
    } else if (const auto jacobian =
                   std::dynamic_pointer_cast<JacobianFactor>(factor)) {
      row->jacobianFactors++;
      row->linearRows += jacobian->rows();
    }
  }
}

PointCholeskyProfileRow profilePointFirstCholeskyVariant(
    const std::string& dataset, const std::string& variant,
    const NonlinearFactorGraph& graph, const Values& initial,
    const Ordering& ordering) {
  PointCholeskyProfileRow row;
  row.dataset = dataset;
  row.variant = variant;
  row.nonlinearFactors = graph.size();

  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setVerbosityLM("SILENT");
  params.setRelativeErrorTol(0.01);
  params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;
  params.setOrdering(ordering);

  Values values = initial;
  double currentError = graph.error(values);
  row.initialError = currentError;
  row.finalError = currentError;
  double lambda = params.lambdaInitial;
  double currentFactor = params.lambdaFactor;
  std::optional<IndexedJunctionTree> indexedJunctionTree;

  for (size_t iteration = 0; iteration < params.maxIterations; ++iteration) {
    GaussianFactorGraph::shared_ptr linear;
    row.linearize += measureSeconds([&] { linear = graph.linearize(values); });
    if (iteration == 0) collectLinearStats(*linear, &row);

    VectorValues sqrtHessianDiagonal;
    row.hessianDiagonal += measureSeconds([&] {
      sqrtHessianDiagonal = linear->hessianDiagonal();
      for (auto& [key, value] : sqrtHessianDiagonal) {
        value = value.cwiseMax(params.dampingParams.minDiagonal)
                    .cwiseMin(params.dampingParams.maxDiagonal)
                    .cwiseSqrt();
      }
    });

    bool stepSuccessful = false;
    bool stopSearchingLambda = false;
    double acceptedError = currentError;
    Values acceptedValues;

    while (!stepSuccessful && !stopSearchingLambda) {
      row.innerIterations++;
      GaussianFactorGraph damped;
      row.damp += measureSeconds([&] {
        damped = *linear;
        damped.reserve(damped.size() + sqrtHessianDiagonal.size());
        const double sigma = 1.0 / std::sqrt(lambda);
        for (const auto& keyVector : sqrtHessianDiagonal) {
          const Key key = keyVector.first;
          const Vector& diagonal = keyVector.second;
          const size_t dim = static_cast<size_t>(diagonal.size());
          Matrix A = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(diagonal);
          Vector b = Vector::Zero(dim);
          damped.emplace_shared<JacobianFactor>(
              key, A, b, noiseModel::Isotropic::Sigma(dim, sigma));
        }
      });

      if (!indexedJunctionTree) {
        row.symbolic += measureSeconds([&] {
          indexedJunctionTree.emplace(
              damped.buildIndexedJunctionTree(ordering));
        });
      }

      std::shared_ptr<GaussianBayesTree> bayesTree;
      row.eliminate += measureSeconds([&] {
        bayesTree = damped.eliminateMultifrontal(*indexedJunctionTree,
                                                 EliminatePreferCholesky);
      });

      VectorValues delta;
      row.backSubstitute +=
          measureSeconds([&] { delta = bayesTree->optimize(); });

      double linearizedCostChange = 0.0;
      row.deltaError += measureSeconds([&] {
        double oldLinearizedError = 0.0, newLinearizedError = 0.0;
        linearizedCostChange =
            linear->deltaError(delta, &oldLinearizedError, &newLinearizedError);
      });

      double newError = std::numeric_limits<double>::infinity();
      Values newValues;
      row.retractAndError += measureSeconds([&] {
        newValues = values.retract(delta);
        newError = graph.error(newValues);
      });

      const double costChange = currentError - newError;
      double modelFidelity = 0.0;
      if (linearizedCostChange >= 0.0 &&
          linearizedCostChange >
              std::numeric_limits<double>::epsilon() * currentError) {
        modelFidelity = costChange / linearizedCostChange;
        stepSuccessful = modelFidelity > params.minModelFidelity;
      }

      const double minAbsoluteTolerance =
          params.relativeErrorTol * currentError;
      stopSearchingLambda = std::abs(costChange) < minAbsoluteTolerance;

      if (stepSuccessful) {
        if (params.useFixedLambdaFactor) {
          lambda /= currentFactor;
        } else {
          lambda *=
              std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * modelFidelity - 1.0, 3));
          currentFactor = 2.0 * currentFactor;
        }
        lambda = std::max(params.lambdaLowerBound, lambda);
        acceptedValues = std::move(newValues);
        acceptedError = newError;
      } else if (!stopSearchingLambda) {
        lambda *= currentFactor;
        if (!params.useFixedLambdaFactor) currentFactor *= 2.0;
        if (lambda >= params.lambdaUpperBound) {
          stopSearchingLambda = true;
        }
      }
    }

    row.iterations++;
    const double previousError = currentError;
    if (stepSuccessful) {
      values = std::move(acceptedValues);
      currentError = acceptedError;
      row.finalError = currentError;
    }

    if (std::abs(previousError - currentError) <
        params.relativeErrorTol * previousError) {
      break;
    }
    if (!stepSuccessful) break;
  }

  return row;
}

std::vector<PointCholeskyProfileRow> profilePointFirstCholesky(
    const std::string& filename, size_t batchChunkSize) {
  const std::string dataset =
      std::filesystem::path(filename).filename().string();
  std::cout << "\nProfiling point-first Cholesky for BAL file: " << filename
            << std::endl;
  const SfmData db = SfmData::FromBalFile(filename);
  const Values initial = buildGeneralSfmInitial(db);
  const Ordering ordering = createSchurOrdering(db, false);

  const NonlinearFactorGraph regularGraph = buildGeneralSfmGraph(db);
  const NonlinearFactorGraph batchGraph =
      buildBatchSfmGraph(db, false, batchChunkSize);

  std::vector<PointCholeskyProfileRow> rows;
  rows.push_back(profilePointFirstCholeskyVariant(
      dataset, "Regular", regularGraph, initial, ordering));
  rows.push_back(profilePointFirstCholeskyVariant(
      dataset, "PointBatch", batchGraph, initial, ordering));

  std::cout
      << "\n| Dataset | Variant | Nonlinear factors | Linear factors | "
         "Jacobian factors | Batch factors | Rows | Iterations | Inner its | "
         "Linearize s | Hessian diagonal s | Damping s | Symbolic s | "
         "Eliminate s | Back-substitute s | Delta error s | "
         "Retract+error s | Total s |\n";
  std::cout
      << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
         "--- | --- | --- | --- | --- | --- | --- |\n";
  std::cout << std::fixed << std::setprecision(6);
  for (const auto& row : rows) {
    std::cout << "| " << row.dataset << " | " << row.variant << " | "
              << row.nonlinearFactors << " | " << row.linearFactors << " | "
              << row.jacobianFactors << " | " << row.batchFactors << " | "
              << row.linearRows << " | " << row.iterations << " | "
              << row.innerIterations << " | " << row.linearize << " | "
              << row.hessianDiagonal << " | " << row.damp << " | "
              << row.symbolic << " | " << row.eliminate << " | "
              << row.backSubstitute << " | " << row.deltaError << " | "
              << row.retractAndError << " | " << row.total() << " |\n";
  }

  std::cout << "\n| Dataset | Variant | Batch dense scalar entries | "
               "Estimated stored scalar entries | Dense/stored ratio |\n";
  std::cout << "| --- | --- | --- | --- | --- |\n";
  for (const auto& row : rows) {
    const double ratio =
        row.batchStoredScalarEntriesEstimate > 0
            ? static_cast<double>(row.batchDenseScalarEntries) /
                  static_cast<double>(row.batchStoredScalarEntriesEstimate)
            : 0.0;
    std::cout << "| " << row.dataset << " | " << row.variant << " | "
              << row.batchDenseScalarEntries << " | "
              << row.batchStoredScalarEntriesEstimate << " | " << ratio
              << "x |\n";
  }

  return rows;
}
}  // namespace

int main(int argc, char* argv[]) {
  const auto options = parseBalFiles(argc, argv);

  if (options.endToEndPcg) {
    runEndToEndPcgComparison(options.filenames);
    return 0;
  }

  if (options.profilePointCholesky) {
    std::vector<PointCholeskyProfileRow> profileRows;
    for (const auto& filename : options.filenames) {
      std::vector<PointCholeskyProfileRow> rows =
          profilePointFirstCholesky(filename, options.batchChunkSize);
      profileRows.insert(profileRows.end(), rows.begin(), rows.end());
    }
    if (options.benchmarkActionJson) {
      writePointCholeskyProfileJson(profileRows,
                                    options.benchmarkActionJsonPath);
      std::cout << "\nWrote point-first Cholesky profile JSON to "
                << options.benchmarkActionJsonPath << "\n";
    }
    return 0;
  }

  std::vector<TimingRow> rows;

  for (const auto& filename : options.filenames) {
    const std::string dataset =
        std::filesystem::path(filename).filename().string();
    std::cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = SfmData::FromBalFile(filename);

    NonlinearFactorGraph graph = buildGeneralSfmGraph(db);
    Values initial = buildGeneralSfmInitial(db);

    Ordering ordering;
    Ordering cameraFirstOrdering;
    if (gUseSchur) {
      ordering = createSchurOrdering(db, false);
      if (options.cameraBatch) {
        cameraFirstOrdering = createCameraFirstOrdering(db);
      }
    }

    double newTime = 0.0;
    if (!options.choleskyOnly) {
      newTime = runSolver(graph, initial, ordering,
                          NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                          "MultifrontalSolver");
    }
    double legacyTime = 0.0;
    double legacyQrTime = 0.0;
    double batchLegacyTime = 0.0;
    double batchQrTime = 0.0;
    double batchTime = 0.0;
    double cameraFirstLegacyTime = 0.0;
    double cameraFirstQrTime = 0.0;
    double cameraFirstSolverTime = 0.0;
    double cameraBatchLegacyTime = 0.0;
    double cameraBatchQrTime = 0.0;
    double cameraBatchSolverTime = 0.0;
    if (!options.profile) {
      legacyTime = runSolver(graph, initial, ordering,
                             NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                             "MultifrontalCholesky");
      if (!options.choleskyOnly) {
        legacyQrTime = runSolver(graph, initial, ordering,
                                 NonlinearOptimizerParams::MULTIFRONTAL_QR,
                                 "MultifrontalQR");
      }
      const NonlinearFactorGraph batchGraph =
          buildBatchSfmGraph(db, false, options.batchChunkSize);
      batchLegacyTime =
          runSolver(batchGraph, initial, ordering,
                    NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                    "BatchFactor/MultifrontalCholesky");
      if (!options.choleskyOnly) {
        batchQrTime = runSolver(batchGraph, initial, ordering,
                                NonlinearOptimizerParams::MULTIFRONTAL_QR,
                                "BatchFactor/MultifrontalQR");
        batchTime = runSolver(batchGraph, initial, ordering,
                              NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                              "BatchFactor/MultifrontalSolver");
      }

      if (options.cameraBatch) {
        const NonlinearFactorGraph cameraBatchGraph =
            buildCameraBatchSfmGraph(db);
        cameraBatchLegacyTime =
            runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraBatchFactor/MultifrontalCholesky");
        if (!options.choleskyOnly) {
          cameraBatchQrTime =
              runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_QR,
                        "CameraBatchFactor/MultifrontalQR");
          cameraBatchSolverTime =
              runSolver(cameraBatchGraph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                        "CameraBatchFactor/MultifrontalSolver");
        }

        cameraFirstLegacyTime =
            runSolver(graph, initial, cameraFirstOrdering,
                      NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY,
                      "CameraFirst/MultifrontalCholesky");
        if (!options.choleskyOnly) {
          cameraFirstQrTime =
              runSolver(graph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_QR,
                        "CameraFirst/MultifrontalQR");
          cameraFirstSolverTime =
              runSolver(graph, initial, cameraFirstOrdering,
                        NonlinearOptimizerParams::MULTIFRONTAL_SOLVER,
                        "CameraFirst/MultifrontalSolver");
        }
      }
    }

    if (!options.profile) {
      rows.push_back({dataset, legacyTime, legacyQrTime, newTime,
                      batchLegacyTime, batchQrTime, batchTime,
                      cameraFirstLegacyTime, cameraFirstQrTime,
                      cameraFirstSolverTime, cameraBatchLegacyTime,
                      cameraBatchQrTime, cameraBatchSolverTime});
    }
  }
  if (!options.profile) {
    std::cout << std::fixed << std::setprecision(3);
    if (options.choleskyOnly) {
      std::cout << "\n| Dataset | Regular Cholesky s | "
                   "Batch Cholesky s | Cholesky Speedup |\n";
      std::cout << "| --- | --- | --- | --- |\n";
      for (const auto& row : rows) {
        std::cout << "| " << row.dataset << " | " << row.regularCholesky
                  << " | " << row.batchCholesky << " | "
                  << speedup(row.regularCholesky, row.batchCholesky) << "x |\n";
      }
    } else {
      std::cout << "\n| Dataset | Regular Cholesky s | Regular QR s | "
                   "Regular Solver s | Batch Cholesky s | Batch QR s | "
                   "Batch Solver s | Batch Solver Speedup |\n";
      std::cout << "| --- | --- | --- | --- | --- | --- | --- | --- |\n";
      for (const auto& row : rows) {
        std::cout << "| " << row.dataset << " | " << row.regularCholesky
                  << " | " << row.regularQr << " | " << row.regularSolver
                  << " | " << row.batchCholesky << " | " << row.batchQr << " | "
                  << row.batchSolver << " | "
                  << speedup(row.regularSolver, row.batchSolver) << "x |\n";
      }
    }

    if (options.cameraBatch) {
      if (options.choleskyOnly) {
        std::cout << "\n| Dataset | Camera-first Cholesky s | "
                     "Camera-batch Cholesky s | Cholesky Speedup |\n";
        std::cout << "| --- | --- | --- | --- |\n";
        for (const auto& row : rows) {
          std::cout << "| " << row.dataset << " | " << row.cameraFirstCholesky
                    << " | " << row.cameraBatchCholesky << " | "
                    << speedup(row.cameraFirstCholesky, row.cameraBatchCholesky)
                    << "x |\n";
        }
      } else {
        std::cout
            << "\n| Dataset | Camera-first Cholesky s | Camera-first QR s | "
               "Camera-first Solver s | Camera-batch Cholesky s | "
               "Camera-batch QR s | Camera-batch Solver s | Cholesky Speedup | "
               "QR Speedup | Solver Speedup |\n";
        std::cout << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | "
                     "--- |\n";
        for (const auto& row : rows) {
          std::cout << "| " << row.dataset << " | " << row.cameraFirstCholesky
                    << " | " << row.cameraFirstQr << " | "
                    << row.cameraFirstSolver << " | " << row.cameraBatchCholesky
                    << " | " << row.cameraBatchQr << " | "
                    << row.cameraBatchSolver << " | "
                    << speedup(row.cameraFirstCholesky, row.cameraBatchCholesky)
                    << "x | " << speedup(row.cameraFirstQr, row.cameraBatchQr)
                    << "x | "
                    << speedup(row.cameraFirstSolver, row.cameraBatchSolver)
                    << "x |\n";
        }
      }
    }
  }

  if (options.benchmarkActionJson) {
    if (rows.empty()) {
      throw runtime_error("No benchmark rows found to write.");
    }
    writeBenchmarkActionJson(rows, options.benchmarkActionJsonPath,
                             options.cameraBatch, options.choleskyOnly);
    std::cout << "\nWrote benchmark-action JSON to "
              << options.benchmarkActionJsonPath << "\n";
  }
  return 0;
}
