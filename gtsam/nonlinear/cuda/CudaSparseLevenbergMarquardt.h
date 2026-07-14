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

struct CudaSparseLmStageTimings {
  double plan = 0.0;
  double hostZero = 0.0;
  double factorLinearizationAndPackingWall = 0.0;
  double factorLinearizationCpuSum = 0.0;
  double csrPackingCpuSum = 0.0;
  double upload = 0.0;
  double transposeUpdate = 0.0;
  double normalEquations = 0.0;
  double cudssAnalysis = 0.0;
  double damping = 0.0;
  double cudssFactorAndSolve = 0.0;
  double modelError = 0.0;
  double deltaDownload = 0.0;
  double retract = 0.0;
  double nonlinearTrialError = 0.0;
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
  // On CpuFallback, CUDA counters, timings, attemptTrace, and finalLambda
  // describe the attempted CUDA prefix. values() and finalError describe the
  // complete CPU solve restarted from the original initial values.
  CudaSparseLmBackend backend = CudaSparseLmBackend::Cuda;
  CudaSparseLmFallbackReason fallbackReason =
      CudaSparseLmFallbackReason::None;
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
