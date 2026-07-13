#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>

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

struct CudaSfmLevenbergMarquardtResult {
  double initialError = 0.0;
  double finalError = 0.0;
  double totalMeasuredElapsed = 0.0;
  double setupElapsed = 0.0;
  double solveLoopElapsed = 0.0;
  double contextElapsed = 0.0;
  double packValuesElapsed = 0.0;
  double allocateTrialElapsed = 0.0;
  double projectionBatchElapsed = 0.0;
  double initialErrorElapsed = 0.0;
  double cudssSolverConstructionElapsed = 0.0;
  double denseSchurSolverConstructionElapsed = 0.0;
  double csrStructureElapsed = 0.0;
  double uploadPatternElapsed = 0.0;
  double firstCudssAnalyzeElapsed = 0.0;
  double downloadElapsed = 0.0;
  int iterations = 0;
  int innerIterations = 0;
  int acceptedSteps = 0;
  double finalLambda = 0.0;
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
