#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace gtsam::cuda {

class CudaSparseLevenbergMarquardtOptimizer;

class GTSAM_EXPORT CudaSparseLevenbergMarquardtParams
    : public LevenbergMarquardtParams {
 public:
  using OptimizerType = CudaSparseLevenbergMarquardtOptimizer;

  // The CUDA path honors LM damping, trust-region, termination, error,
  // iteration-hook, and attempt-trace controls. CPU ordering/linear-solver
  // selection, verbosity, and CSV logging apply only if execution falls back
  // to ordinary CPU LM; cuDSS chooses the CUDA sparse ordering internally.

  bool fallbackOnUnsupported = true;
  bool collectTiming = true;
  bool collectAttemptTrace = false;
  bool validateStructureEveryIteration = false;
};

enum class CudaSparseLmBackend { Cuda, CpuFallback };

enum class CudaSparseLmTerminationReason {
  None,
  ErrorThreshold,
  Converged,
  MaxIterations,
  SmallCostChange,
  LambdaUpperBound,
};

enum class CudaSparseLmFallbackReason {
  None,
  CudaUnavailable,
  CudaToolkitUnsupported,
  CudssUnavailable,
  PlanIncompatible,
  DirectJacobianUnsupported,
};

struct CudaSparseLmSystemSize {
  size_t factors = 0;
  size_t jacobianRows = 0;
  size_t jacobianColumns = 0;
  size_t jacobianNonzeros = 0;
  size_t normalNonzeros = 0;
};

struct CudaSparseLmTransferCounts {
  size_t patternH2dBytes = 0;
  size_t numericH2dBytes = 0;
  size_t setupD2hBytes = 0;
  size_t attemptD2hBytes = 0;

  size_t totalH2dBytes() const { return patternH2dBytes + numericH2dBytes; }
  size_t totalD2hBytes() const { return setupD2hBytes + attemptD2hBytes; }
};

/**
 * Cumulative stage timings in seconds.
 *
 * Some fields intentionally overlap: totalWall contains the CUDA prefix;
 * persistentSetupWall contains deviceInitializeWall; pattern, structure, and
 * setup-D2H device stages overlap deviceInitializeWall; worker CPU sums
 * overlap factorLinearizationAndPackingWall; and the mandatory cuDSS
 * DATA_INFO host boundary overlaps cudssFactorAndSolve. The compatibility
 * aggregate fields at the end are aliases assembled from detailed stages and
 * must not be added to those stages again.
 */
struct CudaSparseLmStageTimings {
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
  double newModelError = 0.0;
  double attemptD2h = 0.0;
  double attemptHostBuild = 0.0;
  double retract = 0.0;
  double nonlinearTrialError = 0.0;

  // Compatibility aggregates retained for existing callers.
  double upload = 0.0;
  double normalEquations = 0.0;
  double damping = 0.0;
  double modelError = 0.0;
  double deltaDownload = 0.0;
};

struct CudaSparseLmAttemptRecord {
  size_t acceptedIterationsBeforeAttempt = 0;
  size_t attempt = 0;
  double lambda = 0.0;
  double linearizedChange = 0.0;
  double nonlinearChange = 0.0;
  double modelFidelity = 0.0;
  bool accepted = false;
};

struct CudaSparseLevenbergMarquardtResult {
  // On CpuFallback, CUDA counters, system/transfer data, timings,
  // attemptTrace, and finalLambda describe the attempted CUDA prefix.
  // values() and finalError describe the complete CPU solve restarted from
  // the original initial values.
  CudaSparseLmBackend backend = CudaSparseLmBackend::Cuda;
  CudaSparseLmFallbackReason fallbackReason = CudaSparseLmFallbackReason::None;
  DirectJacobianStatus fallbackStatus;
  std::string fallbackDetail;
  CudaSparseLmTerminationReason termination =
      CudaSparseLmTerminationReason::None;
  size_t outerLinearizations = 0;
  size_t iterations = 0;
  size_t lambdaAttempts = 0;
  size_t acceptedSteps = 0;
  size_t cudssAnalyses = 0;
  double initialError = 0.0;
  double finalError = 0.0;
  double finalLambda = 0.0;
  CudaSparseLmSystemSize systemSize;
  CudaSparseLmTransferCounts transfers;
  CudaSparseLmStageTimings timings;
  std::vector<CudaSparseLmAttemptRecord> attemptTrace;
};

class GTSAM_EXPORT CudaSparseLevenbergMarquardtOptimizer {
 public:
  CudaSparseLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const CudaSparseLevenbergMarquardtParams& params = {});
  ~CudaSparseLevenbergMarquardtOptimizer();

  CudaSparseLevenbergMarquardtOptimizer(
      const CudaSparseLevenbergMarquardtOptimizer&) = delete;
  CudaSparseLevenbergMarquardtOptimizer& operator=(
      const CudaSparseLevenbergMarquardtOptimizer&) = delete;

  const Values& optimize();
  const Values& values() const;
  double error() const;
  const CudaSparseLevenbergMarquardtParams& params() const;
  const CudaSparseLevenbergMarquardtResult& result() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
