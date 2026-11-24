//*************************************************************************
// General CUDA nonlinear optimization
//*************************************************************************

namespace gtsam {

#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ParameterTag.h>
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

#include <gtsam/nonlinear/cuda/BatchedFactorGroupHook.h>

class BatchedFactorGroupHook {
  BatchedFactorGroupHook();
  BatchedFactorGroupHook(
      const gtsam::cuda::BatchedFactorGroupStageFunction& stageFunction,
      const gtsam::cuda::BatchedFactorGroupComputeBatchFunction&
          computeBatchFunction);
};

#include <gtsam/nonlinear/cuda/BatchedBlockTridiagonalLevenbergMarquardt.h>

class BatchedBlockTridiagonalLevenbergMarquardtItemResult {
  BatchedBlockTridiagonalLevenbergMarquardtItemResult();

  double initialError;
  double finalError;
  double finalLambda;
  size_t acceptedIterations;
  size_t lambdaAttempts;
  bool converged;
};

class BatchedBlockTridiagonalLevenbergMarquardtOptimizer {
  // Two overloads, not one with a defaulted trailing vector-of-custom-type
  // parameter: pybind's py::arg(...) = {} default-value mechanism cannot
  // deduce a std::vector<BatchedFactorGroupHook> from a bare {} (confirmed
  // by a real regression -- the single-constructor-with-default form
  // compiled silently but made factorGroupHooks a required argument at
  // the Python level). Matches CustomFactor's own precedent for an
  // optional trailing callback-ish parameter in this file.
  BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
      const std::vector<gtsam::NonlinearFactorGraph>& graphs,
      const std::vector<gtsam::Values>& initialValues, int numBlocks,
      int blockDim,
      const gtsam::LevenbergMarquardtParams& params =
          gtsam::LevenbergMarquardtParams(),
      const gtsam::cuda::BatchedSolverOptions& deviceOptions =
          gtsam::cuda::BatchedSolverOptions());
  BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
      const std::vector<gtsam::NonlinearFactorGraph>& graphs,
      const std::vector<gtsam::Values>& initialValues, int numBlocks,
      int blockDim, const gtsam::LevenbergMarquardtParams& params,
      const gtsam::cuda::BatchedSolverOptions& deviceOptions,
      const std::vector<gtsam::cuda::BatchedFactorGroupHook>&
          factorGroupHooks);

  const std::vector<gtsam::Values>& optimize();
  const std::vector<gtsam::Values>& polish(int numSteps);
  const std::vector<gtsam::Values>& values() const;
  const std::vector<gtsam::cuda::BatchedBlockTridiagonalLevenbergMarquardtItemResult>&
  results() const;
};

#include <gtsam/nonlinear/cuda/BatchedBlockTridiagonalDeviceGradients.h>

class BatchedBlockTridiagonalDeviceGradients {
  BatchedBlockTridiagonalDeviceGradients(
      const std::vector<gtsam::NonlinearFactorGraph>& graphs,
      const std::vector<gtsam::Values>& solutions, int numBlocks, int blockDim,
      const gtsam::cuda::BatchedSolverOptions& options =
          gtsam::cuda::BatchedSolverOptions());
  std::vector<gtsam::VectorValues> adjoint(
      const std::vector<gtsam::VectorValues>& stateCotangents) const;
  std::vector<std::optional<gtsam::Vector>> parameterGradient(
      const std::vector<gtsam::VectorValues>& stateCotangents,
      const gtsam::ParameterTag& parameter) const;
  std::vector<gtsam::VectorValues> pushforward(
      const std::vector<std::vector<gtsam::Vector>>& factorResidualDirections)
      const;
  std::vector<std::optional<gtsam::Vector>> noiseGradient(
      const std::vector<gtsam::VectorValues>& stateCotangents,
      const gtsam::SharedNoiseModel model) const;
  int batchSize() const;
  int dimension() const;
};

}  // namespace cuda

#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/GncOptimizer.h>

// Each interface is parsed independently. Keep these template declarations
// consistent with nonlinear.i; the loss and scheduler enums are registered there.
template <PARAMS>
virtual class GncParams {
  GncParams(const PARAMS& baseOptimizerParams);
  GncParams();
  PARAMS baseOptimizerParams;
  gtsam::GncLossType lossType;
  size_t maxIterations;
  double lambdaStep;
  double relativeCostTol;
  double weightsTol;
  double lambdaMax;
  gtsam::This::Verbosity verbosity;
  gtsam::This::IndexVector knownInliers;
  gtsam::This::IndexVector knownOutliers;
  bool allowNonNoiseModelFactors;
  gtsam::GncScheduler scheduler;

  void setLossType(const gtsam::GncLossType type);
  void setMaxIterations(const size_t maxIter);
  void setLambdaStep(const double step);
  void setRelativeCostTol(double value);
  void setWeightsTol(double value);
  void setVerbosityGNC(const gtsam::This::Verbosity value);
  void setKnownInliers(const gtsam::This::IndexVector& knownIn);
  void setKnownOutliers(const gtsam::This::IndexVector& knownOut);
  void setAllowNonNoiseModelFactors(bool allow);
  void setScheduler(const gtsam::GncScheduler s);
  void print(const string& str = "GncParams: ") const;

  enum Verbosity { SILENT, SUMMARY, LAMBDA, WEIGHTS, VALUES };
};

template <PARAMS>
virtual class GncOptimizer {
  GncOptimizer(const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& initialValues, const PARAMS& params);
  void setInlierCostThresholds(const double inth);
  const gtsam::Vector& getInlierCostThresholds() const;
  void setInlierCostThresholdsAtProbability(const double alpha);
  void setWeights(const gtsam::Vector w);
  const gtsam::Vector& getWeights() const;
  const PARAMS& getParams() const;
  gtsam::Values optimize();
};

typedef gtsam::GncParams<gtsam::cuda::SparseLevenbergMarquardtParams>
    GncCudaSparseLMParams;
typedef gtsam::GncOptimizer<
    gtsam::GncParams<gtsam::cuda::SparseLevenbergMarquardtParams>>
    GncCudaSparseLMOptimizer;

}  // namespace gtsam
