/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SparseLevenbergMarquardt.h
 * @brief   Levenberg-Marquardt with a persistent CUDA sparse linear system
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/cuda/LinearSolver.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace gtsam::cuda {

class SparseLevenbergMarquardtOptimizer;
struct SparseLevenbergMarquardtResult;

/// CUDA-specific controls layered on the standard LM parameter set.
class GTSAM_EXPORT SparseLevenbergMarquardtParams
    : public LevenbergMarquardtParams {
 public:
  using OptimizerType = SparseLevenbergMarquardtOptimizer;

  // The CUDA path honors LM damping, trust-region, termination, error,
  // iteration-hook, and attempt-trace controls. A supplied GTSAM Ordering is
  // expanded from variable keys to scalar columns and passed to cuDSS.

  bool fallbackOnUnsupported = true;
  bool collectTiming = false;
  bool collectAttemptTrace = false;
  bool validateStructureEveryIteration = false;

  // The shared backend configuration used by both general and SFM CUDA LM.
  // DenseCholesky is rejected because the general frontend produces sparse
  // or operator systems only. Ordering is meaningful only for cuDSS.
  LinearSolverOptions linear{gtsam::cuda::LinearSolverType::Cudss};
  PcgOptions pcg;
};

/**
 * Batch Levenberg-Marquardt optimizer with a persistent CUDA sparse system.
 *
 * The graph topology is fixed at construction. `optimize()` may be called once
 * and retains the final values and diagnostics for subsequent inspection.
 *
 * The diagnostics types reachable through `result()` are defined below the
 * class, since inspecting them is not part of ordinary use.
 */
class GTSAM_EXPORT SparseLevenbergMarquardtOptimizer {
 public:
  /// Construct the symbolic plan and validate the requested CUDA backend.
  SparseLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const SparseLevenbergMarquardtParams& params =
          SparseLevenbergMarquardtParams());
  ~SparseLevenbergMarquardtOptimizer();

  SparseLevenbergMarquardtOptimizer(
      const SparseLevenbergMarquardtOptimizer&) = delete;
  SparseLevenbergMarquardtOptimizer& operator=(
      const SparseLevenbergMarquardtOptimizer&) = delete;

  /// Run batch optimization and return the retained final values.
  const Values& optimize();
  /// Return the current values.
  const Values& values() const;
  /// Return the nonlinear error at the current values.
  double error() const;
  /// Return the immutable parameter set supplied at construction.
  const SparseLevenbergMarquardtParams& params() const;
  /// Return backend, termination, transfer, and timing diagnostics.
  const SparseLevenbergMarquardtResult& result() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Backend that produced the final optimization result.
enum class SparseLevenbergMarquardtBackend { Device, CpuFallback };

/// Reason optimization stopped.
enum class SparseLevenbergMarquardtTerminationReason {
  None,
  ErrorThreshold,
  Converged,
  MaxIterations,
  SmallCostChange,
  LambdaUpperBound,
};

/// Reason the CUDA path transferred control to the CPU optimizer.
enum class SparseLevenbergMarquardtFallbackReason {
  None,
  RuntimeUnavailable,
  ToolkitUnsupported,
  CudssUnavailable,
  PlanIncompatible,
  DirectJacobianUnsupported,
};

/// Cause reported when direct CUDA Jacobian construction cannot continue.
enum class DirectJacobianFailure {
  None,
  StructuralMismatch,
  UnsupportedGaussianFactor,
  ConstrainedFactor,
  NonFiniteValues,
};

/// Location and description of a direct Jacobian construction failure.
struct DirectJacobianStatus {
  DirectJacobianFailure failure = DirectJacobianFailure::None;
  size_t factorIndex = std::numeric_limits<size_t>::max();
  std::string detail;

  bool ok() const { return failure == DirectJacobianFailure::None; }
};

/// Sparse system dimensions recorded after plan construction.
struct SparseLevenbergMarquardtSystemSize {
  size_t factors = 0;
  size_t jacobianRows = 0;
  size_t jacobianColumns = 0;
  size_t jacobianNonzeros = 0;
  size_t normalNonzeros = 0;
};

/// Logical host/device traffic accumulated by an optimization.
struct SparseLevenbergMarquardtTransferCounts {
  size_t patternH2dBytes = 0;
  size_t numericH2dBytes = 0;
  size_t setupD2hBytes = 0;
  size_t attemptD2hBytes = 0;
  size_t pcgD2hBytes = 0;

  size_t totalH2dBytes() const { return patternH2dBytes + numericH2dBytes; }
  size_t totalD2hBytes() const {
    return setupD2hBytes + attemptD2hBytes + pcgD2hBytes;
  }
};

/**
 * Cumulative stage timings in seconds.
 *
 * Some fields intentionally overlap: totalWall contains the CUDA prefix;
 * persistentSetupWall contains deviceInitializeWall; pattern, structure, and
 * setup-D2H device stages overlap deviceInitializeWall; worker CPU sums
 * overlap factorLinearizationAndPackingWall; and the mandatory cuDSS
 * DATA_INFO host boundary overlaps cudssFactorAndSolve. PCG convergence D2H
 * timing overlaps pcgSolve. Timing fields are therefore not an exclusive
 * partition and must not be blindly summed.
 */
struct SparseLevenbergMarquardtStageTimings {
  double totalWall = 0.0;
  double initialError = 0.0;
  double plan = 0.0;
  double persistentSetupWall = 0.0;
  double deviceInitializeWall = 0.0;
  double patternH2d = 0.0;
  double structureSetup = 0.0;
  double setupD2h = 0.0;
  double hostZero = 0.0;
  double factorLinearizationAndPackingWall = 0.0;
  double factorLinearizationCpuSum = 0.0;
  double csrPackingCpuSum = 0.0;
  double numericH2d = 0.0;
  double transposeUpdate = 0.0;
  double normalJtJ = 0.0;
  double normalJtb = 0.0;
  double diagonalExtraction = 0.0;
  double oldModelError = 0.0;
  double dampingPreparation = 0.0;
  double dampingApplication = 0.0;
  double cudssAnalysis = 0.0;
  double cudssFactorAndSolve = 0.0;
  double cudssDataInfoBoundaryWall = 0.0;
  double pcgPreconditionerBuild = 0.0;
  double pcgSolve = 0.0;
  double pcgD2h = 0.0;
  double newModelError = 0.0;
  double attemptD2h = 0.0;
  double attemptHostBuild = 0.0;
  double retract = 0.0;
  double nonlinearTrialError = 0.0;

};

/// Diagnostics for one LM lambda attempt.
struct SparseLevenbergMarquardtAttemptRecord {
  size_t acceptedIterationsBeforeAttempt = 0;
  size_t attempt = 0;
  double lambda = 0.0;
  double linearizedChange = 0.0;
  double nonlinearChange = 0.0;
  double modelFidelity = 0.0;
  size_t pcgIterations = 0;
  bool pcgSolve = false;
  bool pcgConverged = false;
  bool pcgBreakdown = false;
  bool accepted = false;
};

/// Final values, diagnostics, and optional profile for one optimization.
struct SparseLevenbergMarquardtResult {
  // On CpuFallback, CUDA counters, system/transfer data, timings,
  // attemptTrace, and finalLambda describe the attempted CUDA prefix.
  // values() and finalError describe the complete CPU solve restarted from
  // the original initial values.
  SparseLevenbergMarquardtBackend backend = SparseLevenbergMarquardtBackend::Device;
  SparseLevenbergMarquardtFallbackReason fallbackReason =
      SparseLevenbergMarquardtFallbackReason::None;
  DirectJacobianStatus fallbackStatus;
  std::string fallbackDetail;
  SparseLevenbergMarquardtTerminationReason termination =
      SparseLevenbergMarquardtTerminationReason::None;
  size_t outerLinearizations = 0;
  size_t iterations = 0;
  size_t lambdaAttempts = 0;
  size_t acceptedSteps = 0;
  size_t cudssAnalyses = 0;
  size_t pcgIterationsTotal = 0;
  size_t pcgSolves = 0;
  size_t pcgMaxIterationHits = 0;
  double initialError = 0.0;
  double finalError = 0.0;
  double finalLambda = 0.0;
  SparseLevenbergMarquardtSystemSize systemSize;
  SparseLevenbergMarquardtTransferCounts transfers;
  SparseLevenbergMarquardtStageTimings timings;
  LinearSolveStats linearSolveStats;
  std::vector<int> appliedScalarPermutation;
  std::vector<SparseLevenbergMarquardtAttemptRecord> attemptTrace;
};

}  // namespace gtsam::cuda
