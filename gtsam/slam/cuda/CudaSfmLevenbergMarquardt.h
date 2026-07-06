#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace gtsam::cuda {

enum class CudaSfmLinearSolverType {
  DenseSchur,
  CudssFullNormal,
};

class CudaSfmLevenbergMarquardtOptimizer;

class GTSAM_EXPORT CudaSfmLevenbergMarquardtParams {
 public:
  using OptimizerType = CudaSfmLevenbergMarquardtOptimizer;

  int maxIterations;
  double lambdaInitial;
  double lambdaFactor;
  double lambdaUpperBound;
  double lambdaLowerBound;
  double relativeErrorTol;
  double absoluteErrorTol;
  double errorTol;
  double minModelFidelity;
  bool useFixedLambdaFactor;
  bool diagonalDamping;
  double minDiagonal;
  double maxDiagonal;
  CudaSfmLinearSolverType linearSolver = CudaSfmLinearSolverType::DenseSchur;

  CudaSfmLevenbergMarquardtParams();

  static CudaSfmLevenbergMarquardtParams LegacyDefaults();
  static CudaSfmLevenbergMarquardtParams CeresDefaults();

  std::string getLinearSolver() const;
  void setLinearSolver(const std::string& solver);
  void print(const std::string& str = "") const;
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
  double acceptTrialElapsed = 0.0;
  bool acceptedStep = false;
  bool terminated = false;
  std::vector<CudaSfmLmAttemptProfile> attemptProfiles;
};

struct CudaSfmLevenbergMarquardtResult {
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
  const NonlinearOptimizerParams& _params() const override {
    return baseParams_;
  }

 private:
  CudaSfmLevenbergMarquardtParams params_;
  NonlinearOptimizerParams baseParams_;
  CudaSfmLevenbergMarquardtResult result_;
};

}  // namespace gtsam::cuda
