#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/linear/cuda/PcgSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/SfmTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace gtsam::cuda {

/// Mathematical system formulation used by the SFM optimizer.
enum class SfmSystemFormulation { Schur, FullNormal };

class SfmLevenbergMarquardtOptimizer;

/** Parameters for CUDA-resident SFM Levenberg-Marquardt optimization. */
class GTSAM_EXPORT SfmLevenbergMarquardtParams
    : public LevenbergMarquardtParams {
 public:
  using OptimizerType = SfmLevenbergMarquardtOptimizer;

  bool enableDetailedProfiling;
  /** The only solver configuration state: mathematical formulation and
   * numerical backend are independent axes. */
  SfmSystemFormulation formulation = SfmSystemFormulation::Schur;
  LinearSolverOptions linear{
      gtsam::cuda::LinearSolverType::DenseCholesky, false};
  /** The inherited optional Ordering is camera-only for Schur and contains
   * camera plus point keys for full normal systems. */
  PcgOptions pcg;

  /// Creates parameters using GTSAM's legacy LM defaults.
  SfmLevenbergMarquardtParams();

  /// Returns parameters initialized with GTSAM's legacy LM defaults.
  static SfmLevenbergMarquardtParams legacyDefaults();
  /// Returns parameters initialized with GTSAM's Ceres-style LM defaults.
  static SfmLevenbergMarquardtParams ceresDefaults();

  /// Returns the combined formulation/backend solver name.
  std::string getLinearSolver() const;
  /// Sets formulation and backend from a combined solver name.
  void setLinearSolver(const std::string& solver);
  /// Returns the mathematical formulation name.
  std::string getFormulation() const;
  /// Sets the mathematical formulation independently of the backend.
  void setFormulation(const std::string& formulationName);
  /// Returns the numerical backend name.
  std::string getLinearSolverBackend() const;
  /// Sets the numerical backend independently of the formulation.
  void setLinearSolverBackend(const std::string& solverName);
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
  /** Once-per-outer-iteration full-normal system construction. */
  double normalEquationsElapsed = 0.0;
  double acceptTrialElapsed = 0.0;
  bool acceptedStep = false;
  bool terminated = false;
  std::vector<SfmLevenbergMarquardtAttemptProfile> attemptProfiles;
};

struct SfmLevenbergMarquardtResult {
  SfmSystemFormulation formulation = SfmSystemFormulation::Schur;
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
  double cudssSolverConstructionElapsed = 0.0;
  double denseSchurSolverConstructionElapsed = 0.0;
  double csrStructureElapsed = 0.0;
  double uploadPatternElapsed = 0.0;
  double uploadPatternDeviceAllocElapsed = 0.0;
  double uploadPatternH2dCopyElapsed = 0.0;
  size_t uploadPatternH2dBytes = 0;
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

/** NonlinearOptimizer adapter for the CUDA-resident SFM implementation. */
class GTSAM_EXPORT SfmLevenbergMarquardtOptimizer
    : public NonlinearOptimizer {
 public:
  /// Creates an optimizer for a supported factor graph and initial values.
  SfmLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const SfmLevenbergMarquardtParams& params =
          SfmLevenbergMarquardtParams());

  /// Runs optimization and returns the final values.
  const Values& optimize() override;

  /// Executes one nonlinear iteration.
  GaussianFactorGraph::shared_ptr iterate() override;

  /// Returns the optimizer parameters.
  const SfmLevenbergMarquardtParams& params() const { return params_; }
  /// Returns profiling and convergence results from the latest run.
  const SfmLevenbergMarquardtResult& result() const { return result_; }

 protected:
  const NonlinearOptimizerParams& _params() const override { return params_; }

 private:
  SfmLevenbergMarquardtParams params_;
  SfmLevenbergMarquardtResult result_;
};

}  // namespace gtsam::cuda
