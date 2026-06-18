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
  double solveLoopElapsed = 0.0;
  int iterations = 0;
  int acceptedSteps = 0;
  Values optimizedValues;
};

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const CudaSfmLevenbergMarquardtParams& params);

}  // namespace gtsam::cuda
