#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>
#include <gtsam/linear/cuda/CudaPcgSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace gtsam::cuda {

enum class CudaSfmSystemFormulation { Schur, FullNormal };

class CudaSfmLevenbergMarquardtOptimizer;

class GTSAM_EXPORT CudaSfmLevenbergMarquardtParams
    : public LevenbergMarquardtParams {
 public:
  using OptimizerType = CudaSfmLevenbergMarquardtOptimizer;

  bool enableDetailedProfiling;
  /** The only solver configuration state: mathematical formulation and
   * numerical backend are independent axes. */
  CudaSfmSystemFormulation formulation = CudaSfmSystemFormulation::Schur;
  CudaLinearSolverOptions linear{CudaLinearSolverType::DenseCholesky, false};
  /** The inherited optional Ordering is camera-only for Schur and contains
   * camera plus point keys for full normal systems. */
  CudaPcgOptions pcg;

  CudaSfmLevenbergMarquardtParams();

  static CudaSfmLevenbergMarquardtParams LegacyDefaults();
  static CudaSfmLevenbergMarquardtParams CeresDefaults();

  std::string getLinearSolver() const;
  void setLinearSolver(const std::string& solver);
  std::string getFormulation() const;
  void setFormulation(const std::string& formulationName);
  std::string getCudaLinearSolver() const;
  void setCudaLinearSolver(const std::string& solverName);
  double getMinDiagonal() const { return dampingParams.minDiagonal; }
  double getMaxDiagonal() const { return dampingParams.maxDiagonal; }
  void setMinDiagonal(double value) { dampingParams.minDiagonal = value; }
  void setMaxDiagonal(double value) { dampingParams.maxDiagonal = value; }
  void print(const std::string& str = "") const override;
  bool equals(const CudaSfmLevenbergMarquardtParams& other,
              double tol = 1e-9) const;
};

struct CudaSfmLmAttemptProfile {
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

struct CudaSfmLmIterationProfile {
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
  std::vector<CudaSfmLmAttemptProfile> attemptProfiles;
};

struct CudaSfmLevenbergMarquardtResult {
  CudaSfmSystemFormulation formulation = CudaSfmSystemFormulation::Schur;
  CudaLinearSolverType linearBackend = CudaLinearSolverType::DenseCholesky;
  CudaLinearSystemKind linearSystemKind = CudaLinearSystemKind::Dense;
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
  CudaLinearSolveStats linearSolveStats;
  std::vector<int> appliedScalarPermutation;
  std::vector<CudaSfmLmIterationProfile> iterationProfiles;
  Values optimizedValues;
};

struct CudaSfmFactorGraphData {
  SfmData data;
  std::vector<Key> cameraKeys;
  std::vector<Key> pointKeys;
  std::vector<std::vector<CudaSfmSqrtInfo2>> sqrtInfoByTrack;
  std::vector<std::vector<CudaSfmRobustModel>> robustModelsByTrack;
  bool hasNonUnitNoise = false;
  bool hasRobustNoise = false;
};

GTSAM_EXPORT CudaSfmFactorGraphData ConvertGeneralSfmGraphToCudaSfmData(
    const NonlinearFactorGraph& graph, const Values& initialValues);

GTSAM_EXPORT CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data,
    const CudaSfmLevenbergMarquardtParams& params);

GTSAM_EXPORT CudaSfmLevenbergMarquardtResult OptimizeCudaSfmWithoutValueDownload(
    const SfmData& data,
    const CudaSfmLevenbergMarquardtParams& params);

GTSAM_EXPORT CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params);

GTSAM_EXPORT CudaSfmLevenbergMarquardtResult OptimizeCudaSfmWithoutValueDownload(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params);

class GTSAM_EXPORT CudaSfmLevenbergMarquardtOptimizer
    : public NonlinearOptimizer {
 public:
  CudaSfmLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const CudaSfmLevenbergMarquardtParams& params =
          CudaSfmLevenbergMarquardtParams());

  const Values& optimize() override;

  GaussianFactorGraph::shared_ptr iterate() override;

  const CudaSfmLevenbergMarquardtParams& params() const { return params_; }
  const CudaSfmLevenbergMarquardtResult& result() const { return result_; }

 protected:
  const NonlinearOptimizerParams& _params() const override { return params_; }

 private:
  CudaSfmLevenbergMarquardtParams params_;
  CudaSfmLevenbergMarquardtResult result_;
};

}  // namespace gtsam::cuda
