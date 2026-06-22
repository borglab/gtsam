#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>

#include <vector>

namespace gtsam::cuda {

enum class CudaSfmLinearSolverType {
  DenseSchur,
  CudssFullNormal,
};

struct CudaSfmLevenbergMarquardtParams {
  int maxIterations = 20;
  double initialLambda = 1e-3;
  double lambdaUpFactor = 10.0;
  double lambdaDownFactor = 0.1;
  double lambdaUpperBound = 1e5;
  double lambdaLowerBound = 0.0;
  double relativeErrorTol = 1e-5;
  double absoluteErrorTol = 1e-5;
  double errorTol = 0.0;
  double minModelFidelity = 1e-3;
  bool useFixedLambdaFactor = true;
  bool diagonalDamping = false;
  double minDiagonal = 1e-6;
  double maxDiagonal = 1e32;
  CudaSfmLinearSolverType linearSolver = CudaSfmLinearSolverType::DenseSchur;
  bool downloadOptimizedValues = true;
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
};

CudaSfmFactorGraphData ConvertGeneralSfmGraphToCudaSfmData(
    const NonlinearFactorGraph& graph, const Values& initialValues);

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const CudaSfmLevenbergMarquardtParams& params);

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params);

class CudaSfmLevenbergMarquardtOptimizer : public NonlinearOptimizer {
 public:
  CudaSfmLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams());
  CudaSfmLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const LevenbergMarquardtParams& params,
      CudaSfmLinearSolverType linearSolver);

  const Values& optimize() override;

  GaussianFactorGraph::shared_ptr iterate() override;

  const LevenbergMarquardtParams& params() const { return params_; }
  const CudaSfmLevenbergMarquardtResult& result() const { return result_; }

 protected:
  const NonlinearOptimizerParams& _params() const override { return params_; }

 private:
  LevenbergMarquardtParams params_;
  CudaSfmLevenbergMarquardtParams cudaParams_;
  CudaSfmLevenbergMarquardtResult result_;
};

}  // namespace gtsam::cuda
