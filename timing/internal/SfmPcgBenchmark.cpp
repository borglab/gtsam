/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmPcgBenchmark.cpp
 * @brief End-to-end full and reduced-camera PCG benchmarks for BAL.
 */

#include "SfmPcgBenchmark.h"

#include <gtsam/base/TaskScheduler.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/SparseEigen.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>

#include <Eigen/IterativeLinearSolvers>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <thread>

#include "TimingUtils.h"

using namespace gtsam;
using symbol_shorthand::P;

namespace gtsam::timing::bal {
namespace {

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
    NonlinearOptimizerParams::LinearSolverType solverType, bool useSchur) {
  BalBenchmarkConfig config;
  config.useSchur = useSchur;
  LevenbergMarquardtParams parameters =
      makeLevenbergMarquardtParams(config, &ordering, "SILENT");
  parameters.linearSolverType = solverType;
  return parameters;
}

EndToEndResult runDirectEndToEnd(const std::string& dataset,
                                 const NonlinearFactorGraph& graph,
                                 const Values& initial,
                                 const Ordering& ordering, bool useSchur) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = "MultifrontalCholesky";
  result.initialError = graph.error(initial);
  const auto parameters = endToEndParameters(
      ordering, NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY, useSchur);

  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  result.elapsedSeconds = measureSeconds([&] { optimizer.optimize(); });
  result.finalError = optimizer.error();
  result.lmIterations = optimizer.iterations();
  result.lmInnerIterations =
      static_cast<size_t>(optimizer.getInnerIterations());
  return result;
}

EndToEndResult runParallelPcgEndToEnd(const std::string& dataset,
                                      const NonlinearFactorGraph& graph,
                                      const Values& initial,
                                      const Ordering& ordering, bool useSchur) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = "ParallelPCG(BlockJacobi)";
  result.initialError = graph.error(initial);
  const auto parameters = endToEndParameters(
      ordering, NonlinearOptimizerParams::Iterative, useSchur);

  ParallelPcgLevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  result.elapsedSeconds = measureSeconds([&] { optimizer.optimize(); });
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
                                   const Ordering& ordering, bool useSchur) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = "EigenPCG(ExplicitH,BlockJacobi)";
  result.initialError = graph.error(initial);
  const auto parameters = endToEndParameters(
      ordering, NonlinearOptimizerParams::Iterative, useSchur);

  EigenPcgLevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  result.elapsedSeconds = measureSeconds([&] { optimizer.optimize(); });
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
                                           bool useSchur,
                                           ReducedCameraBackend backend) {
  EndToEndResult result;
  result.dataset = dataset;
  result.solver = backend == ReducedCameraBackend::kParallelPcg
                      ? "ReducedParallelPCG(ImplicitSchur,BlockJacobi)"
                      : "ReducedEigenPCG(ExplicitH,BlockJacobi)";
  result.initialError = graph.error(initial);
  const auto parameters = endToEndParameters(
      ordering, NonlinearOptimizerParams::Iterative, useSchur);

  ReducedCameraPcgLevenbergMarquardtOptimizer optimizer(graph, initial,
                                                        parameters, backend);
  result.elapsedSeconds = measureSeconds([&] { optimizer.optimize(); });
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

void runEndToEndPcgComparisonImpl(const std::vector<std::string>& filenames,
                                  const BalBenchmarkConfig& config) {
  std::vector<EndToEndResult> results;
  for (const std::string& filename : filenames) {
    const std::string dataset =
        std::filesystem::path(filename).filename().string();
    std::cout << "\nEnd-to-end PCG comparison for BAL file: " << filename
              << std::endl;
    const SfmData db = loadDataset(filename);
    const NonlinearFactorGraph graph = buildGeneralSfmGraph(db, config);
    const Values initial = buildGeneralSfmInitial(db);
    const Ordering ordering = createSchurOrdering(db, false);

    results.push_back(
        runDirectEndToEnd(dataset, graph, initial, ordering, config.useSchur));
    results.push_back(runParallelPcgEndToEnd(dataset, graph, initial, ordering,
                                             config.useSchur));
    results.push_back(runEigenPcgEndToEnd(dataset, graph, initial, ordering,
                                          config.useSchur));
    results.push_back(runReducedCameraPcgEndToEnd(
        dataset, graph, initial, ordering, config.useSchur,
        ReducedCameraBackend::kParallelPcg));
    results.push_back(runReducedCameraPcgEndToEnd(
        dataset, graph, initial, ordering, config.useSchur,
        ReducedCameraBackend::kEigenPcg));
  }
  printEndToEndResults(results);
}

}  // namespace

void runEndToEndPcgComparison(const std::vector<std::string>& filenames,
                              const BalBenchmarkConfig& config) {
  runEndToEndPcgComparisonImpl(filenames, config);
}

}  // namespace gtsam::timing::bal
