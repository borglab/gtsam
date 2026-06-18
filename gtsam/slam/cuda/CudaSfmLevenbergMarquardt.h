#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>

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
  double relativeErrorTol = 1e-5;
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
  int acceptedSteps = 0;
  Values optimizedValues;
};

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const CudaSfmLevenbergMarquardtParams& params);

}  // namespace gtsam::cuda
