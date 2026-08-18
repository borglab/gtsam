#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/SfmTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace gtsam::cuda {

class SfmLevenbergMarquardtOptimizer;

/** Parameters for CUDA-resident SFM Levenberg-Marquardt optimization. */
class GTSAM_EXPORT SfmLevenbergMarquardtParams
    : public LevenbergMarquardtParams {
 public:
  using OptimizerType = SfmLevenbergMarquardtOptimizer;

  bool enableDetailedProfiling;
  /// Numerical backend for the reduced Schur-complement system.
  LinearSolverOptions linear{
      gtsam::cuda::LinearSolverType::DenseCholesky};
  /// The inherited optional Ordering contains camera keys only.
  PcgOptions pcg;

  /// Creates parameters using GTSAM's legacy LM defaults.
  SfmLevenbergMarquardtParams();

  /// Returns parameters initialized with GTSAM's legacy LM defaults.
  static SfmLevenbergMarquardtParams legacyDefaults();
  /// Returns parameters initialized with GTSAM's Ceres-style LM defaults.
  static SfmLevenbergMarquardtParams ceresDefaults();

  /// Returns the numerical backend for the reduced Schur system.
  gtsam::cuda::LinearSolverType getLinearSolver() const {
    return linear.backend;
  }
  /// Sets the numerical backend for the reduced Schur system.
  void setLinearSolver(gtsam::cuda::LinearSolverType solver) {
    linear.backend = solver;
  }
  /// Returns the lower clamp applied to damping diagonals.
  double getMinDiagonal() const { return dampingParams.minDiagonal; }
  /// Returns the upper clamp applied to damping diagonals.
  double getMaxDiagonal() const { return dampingParams.maxDiagonal; }
  /// Sets the lower clamp applied to damping diagonals.
  void setMinDiagonal(double value) { dampingParams.minDiagonal = value; }
  /// Sets the upper clamp applied to damping diagonals.
  void setMaxDiagonal(double value) { dampingParams.maxDiagonal = value; }
  /// Prints this parameter set.
  void print(const std::string& str = "") const override;
  /// Tests numerical equality with another parameter set.
  bool equals(const SfmLevenbergMarquardtParams& other,
              double tol = 1e-9) const;
};

struct SfmLevenbergMarquardtAttemptProfile {
  int iteration = 0;
  int attempt = 0;
  double lambda = 0.0;
  double totalElapsed = 0.0;
  double denseSchurSolveElapsed = 0.0;
  double normalEquationsElapsed = 0.0;
  double addDampingElapsed = 0.0;
  double cudssAnalyzeElapsed = 0.0;
  double cudssSolveElapsed = 0.0;
  double linearizedErrorElapsed = 0.0;
  double applyDeltaElapsed = 0.0;
  double trialErrorElapsed = 0.0;
  double lambdaUpdateElapsed = 0.0;
  double oldLinearizedError = 0.0;
  double newLinearizedError = 0.0;
  double linearizedCostChange = 0.0;
  double trialError = 0.0;
  double costChange = 0.0;
  double modelFidelity = 0.0;
  bool attemptedTrial = false;
  bool stepSuccessful = false;
  bool accepted = false;
  bool stopSearchingLambda = false;
  bool lambdaUpperBoundReached = false;
  bool terminated = false;
};

struct SfmLevenbergMarquardtIterationProfile {
  int iteration = 0;
  double startError = 0.0;
  double endError = 0.0;
  double startLambda = 0.0;
  double endLambda = 0.0;
  double totalElapsed = 0.0;
  double dampingDiagonalElapsed = 0.0;
  /** Once-per-attempt reduced-system construction. */
  double normalEquationsElapsed = 0.0;
  double acceptTrialElapsed = 0.0;
  bool acceptedStep = false;
  bool terminated = false;
  std::vector<SfmLevenbergMarquardtAttemptProfile> attemptProfiles;
};

struct SfmLevenbergMarquardtResult {
  LinearSolverType linearBackend = gtsam::cuda::LinearSolverType::DenseCholesky;
  LinearSystemKind linearSystemKind = LinearSystemKind::Dense;
  size_t linearSystemDimension = 0;
  size_t linearSystemNonzeros = 0;
  double initialError = 0.0;
  double finalError = 0.0;
  double totalMeasuredElapsed = 0.0;
  double graphConversionElapsed = 0.0;
  double graphBackendCallElapsed = 0.0;
  double graphConvertedDataDestructionElapsed = 0.0;
  double graphValueMergeElapsed = 0.0;
  double setupElapsed = 0.0;
  double solveLoopElapsed = 0.0;
  double contextElapsed = 0.0;
  double packValuesElapsed = 0.0;
  double packValuesHostBuildElapsed = 0.0;
  double packValuesDeviceAllocElapsed = 0.0;
  double packValuesH2dCopyElapsed = 0.0;
  size_t packValuesH2dBytes = 0;
  double allocateTrialElapsed = 0.0;
  double projectionBatchElapsed = 0.0;
  double projectionBatchHostBuildElapsed = 0.0;
  double projectionBatchDeviceAllocElapsed = 0.0;
  double projectionBatchH2dCopyElapsed = 0.0;
  size_t projectionBatchH2dBytes = 0;
  double initialErrorElapsed = 0.0;
  double denseSchurSolverConstructionElapsed = 0.0;
  double firstCudssAnalyzeElapsed = 0.0;
  double downloadElapsed = 0.0;
  double downloadHostAllocElapsed = 0.0;
  double downloadD2hCopyElapsed = 0.0;
  double downloadValuesBuildElapsed = 0.0;
  size_t downloadD2hBytes = 0;
  double totalH2dCopyElapsed = 0.0;
  size_t totalH2dBytes = 0;
  double totalD2hCopyElapsed = 0.0;
  size_t totalD2hBytes = 0;
  double dampingDiagonalElapsed = 0.0;
  double denseSchurSolveElapsed = 0.0;
  double normalEquationsElapsed = 0.0;
  double addDampingElapsed = 0.0;
  double cudssAnalyzeElapsed = 0.0;
  double cudssSolveElapsed = 0.0;
  double linearizedErrorElapsed = 0.0;
  double applyDeltaElapsed = 0.0;
  double trialErrorElapsed = 0.0;
  double acceptTrialElapsed = 0.0;
  double lambdaUpdateElapsed = 0.0;
  int iterations = 0;
  int innerIterations = 0;
  int acceptedSteps = 0;
  double finalLambda = 0.0;
  LinearSolveStats linearSolveStats;
  std::vector<int> appliedScalarPermutation;
  std::vector<SfmLevenbergMarquardtIterationProfile> iterationProfiles;
  Values optimizedValues;
};

struct SfmFactorGraphData {
  SfmData data;
  std::vector<Key> cameraKeys;
  std::vector<Key> pointKeys;
  std::vector<std::vector<SfmSqrtInfo2>> sqrtInfoByTrack;
  std::vector<std::vector<SfmRobustModel>> robustModelsByTrack;
  bool hasNonUnitNoise = false;
  bool hasRobustNoise = false;
};

/** Converts a supported general factor graph into packed SFM input data. */
GTSAM_EXPORT SfmFactorGraphData convertGeneralSfmGraph(
    const NonlinearFactorGraph& graph, const Values& initialValues);

/** Optimizes SFM data and downloads the optimized values. */
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfm(
    const SfmData& data,
    const SfmLevenbergMarquardtParams& params);

/** Optimizes SFM data without downloading the final device values. */
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfmWithoutValueDownload(
    const SfmData& data,
    const SfmLevenbergMarquardtParams& params);

/** Optimizes SFM data using explicit camera and point keys. */
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfm(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params);

/** Optimizes keyed SFM data without downloading final device values. */
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfmWithoutValueDownload(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params);

/** Batch optimizer adapter for the CUDA-resident SFM implementation. */
class GTSAM_EXPORT SfmLevenbergMarquardtOptimizer {
 public:
  /// Creates an optimizer for a supported factor graph and initial values.
  SfmLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const SfmLevenbergMarquardtParams& params =
          SfmLevenbergMarquardtParams());

  /// Runs optimization and returns the final values.
  const Values& optimize();

  /// Returns the latest optimized values, or the initial values before optimize().
  const Values& values() const { return values_; }
  /// Returns the nonlinear objective at the current values.
  double error() const { return graph_.error(values_); }
  /// Returns the number of accepted nonlinear iterations in the latest run.
  size_t iterations() const { return static_cast<size_t>(result_.iterations); }

  /// Returns the optimizer parameters.
  const SfmLevenbergMarquardtParams& params() const { return params_; }
  /// Returns profiling and convergence results from the latest run.
  const SfmLevenbergMarquardtResult& result() const { return result_; }

 private:
  NonlinearFactorGraph graph_;
  Values values_;
  SfmLevenbergMarquardtParams params_;
  SfmLevenbergMarquardtResult result_;
};

}  // namespace gtsam::cuda
