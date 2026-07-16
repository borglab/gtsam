//*************************************************************************
// CUDA SFM
//*************************************************************************

namespace gtsam {

#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>

namespace cuda {

#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>

enum class CudaSfmLinearSolverType {
  DenseSchur,
  CudssFullNormal
};

class CudaSfmLevenbergMarquardtParams {
  CudaSfmLevenbergMarquardtParams();

  static gtsam::cuda::CudaSfmLevenbergMarquardtParams LegacyDefaults();
  static gtsam::cuda::CudaSfmLevenbergMarquardtParams CeresDefaults();

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
  gtsam::cuda::CudaSfmLinearSolverType linearSolver;

  string getLinearSolver() const;
  void setLinearSolver(const string& solver);
  void print(const string& str = "") const;
};

class CudaSfmLevenbergMarquardtResult {
  CudaSfmLevenbergMarquardtResult();

  double initialError;
  double finalError;
  double totalMeasuredElapsed;
  double setupElapsed;
  double solveLoopElapsed;
  double contextElapsed;
  double packValuesElapsed;
  double allocateTrialElapsed;
  double projectionBatchElapsed;
  double initialErrorElapsed;
  double cudssSolverConstructionElapsed;
  double denseSchurSolverConstructionElapsed;
  double csrStructureElapsed;
  double uploadPatternElapsed;
  double firstCudssAnalyzeElapsed;
  double downloadElapsed;
  int iterations;
  int innerIterations;
  int acceptedSteps;
  double finalLambda;
  gtsam::Values optimizedValues;
};

gtsam::cuda::CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const gtsam::SfmData& data,
    const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params);

gtsam::cuda::CudaSfmLevenbergMarquardtResult
OptimizeCudaSfmWithoutValueDownload(
    const gtsam::SfmData& data,
    const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params);

virtual class CudaSfmLevenbergMarquardtOptimizer
    : gtsam::NonlinearOptimizer {
  CudaSfmLevenbergMarquardtOptimizer(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues,
      const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params =
          gtsam::cuda::CudaSfmLevenbergMarquardtParams());

  const gtsam::cuda::CudaSfmLevenbergMarquardtParams& params() const;
  const gtsam::cuda::CudaSfmLevenbergMarquardtResult& result() const;
};

}  // namespace cuda
}  // namespace gtsam
