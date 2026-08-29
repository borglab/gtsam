//*************************************************************************
// General CUDA nonlinear optimization
//*************************************************************************

namespace gtsam {

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace cuda {

#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>

class SparseLevenbergMarquardtParams
    : gtsam::LevenbergMarquardtParams {
  SparseLevenbergMarquardtParams();

  bool fallbackOnUnsupported;
  bool collectTiming;
  bool collectAttemptTrace;
  bool validateStructureEveryIteration;
  gtsam::cuda::LinearSolverOptions linear;
  gtsam::cuda::PcgOptions pcg;
};

enum class SparseLevenbergMarquardtBackend {
  Device,
  CpuFallback
};

enum class SparseLevenbergMarquardtTerminationReason {
  None,
  ErrorThreshold,
  Converged,
  MaxIterations,
  SmallCostChange,
  LambdaUpperBound
};

enum class SparseLevenbergMarquardtFallbackReason {
  None,
  RuntimeUnavailable,
  ToolkitUnsupported,
  CudssUnavailable,
  PlanIncompatible,
  DirectJacobianUnsupported
};

enum class DirectJacobianFailure {
  None,
  StructuralMismatch,
  UnsupportedGaussianFactor,
  ConstrainedFactor,
  NonFiniteValues
};

class DirectJacobianStatus {
  DirectJacobianStatus();

  gtsam::cuda::DirectJacobianFailure failure;
  size_t factorIndex;
  string detail;

  bool ok() const;
};

class SparseLevenbergMarquardtResult {
  SparseLevenbergMarquardtResult();

  gtsam::cuda::SparseLevenbergMarquardtBackend backend;
  gtsam::cuda::SparseLevenbergMarquardtFallbackReason fallbackReason;
  gtsam::cuda::DirectJacobianStatus fallbackStatus;
  string fallbackDetail;
  gtsam::cuda::SparseLevenbergMarquardtTerminationReason termination;
  size_t outerLinearizations;
  size_t iterations;
  size_t lambdaAttempts;
  size_t acceptedSteps;
  size_t cudssAnalyses;
  size_t pcgIterationsTotal;
  size_t pcgSolves;
  size_t pcgMaxIterationHits;
  double initialError;
  double finalError;
  double finalLambda;
};

class SparseLevenbergMarquardtOptimizer {
  SparseLevenbergMarquardtOptimizer(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues,
      const gtsam::cuda::SparseLevenbergMarquardtParams& params =
          gtsam::cuda::SparseLevenbergMarquardtParams());

  const gtsam::Values& optimize();
  const gtsam::Values& values() const;
  double error() const;
  const gtsam::cuda::SparseLevenbergMarquardtParams& params() const;
  const gtsam::cuda::SparseLevenbergMarquardtResult& result() const;
};

}  // namespace cuda
}  // namespace gtsam
