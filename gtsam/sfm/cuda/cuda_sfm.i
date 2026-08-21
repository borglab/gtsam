//*************************************************************************
// CUDA SFM
//*************************************************************************

namespace gtsam {

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/sfm/SfmEliminationMode.h>

namespace cuda {

#include <gtsam/sfm/cuda/SfmLevenbergMarquardt.h>

enum class LinearSolverType {
  DenseCholesky,
  Cudss,
  Pcg
};

class SfmLevenbergMarquardtParams
    : gtsam::LevenbergMarquardtParams {
  SfmLevenbergMarquardtParams();

  static gtsam::cuda::SfmLevenbergMarquardtParams legacyDefaults();
  static gtsam::cuda::SfmLevenbergMarquardtParams ceresDefaults();

  bool enableDetailedProfiling;
  gtsam::cuda::LinearSolverType getLinearSolver() const;
  void setLinearSolver(gtsam::cuda::LinearSolverType solver);
  gtsam::SfmEliminationMode getEliminationMode() const;
  void setEliminationMode(gtsam::SfmEliminationMode mode);
  double getMinDiagonal() const;
  double getMaxDiagonal() const;
  void setMinDiagonal(double value);
  void setMaxDiagonal(double value);
  void print(const string& str = "") const;
};

class SfmLevenbergMarquardtResult {
  SfmLevenbergMarquardtResult();

  gtsam::SfmEliminationMode eliminationMode;
  double initialError;
  double finalError;
  double totalMeasuredElapsed;
  double setupElapsed;
  double solveLoopElapsed;
  double contextElapsed;
  double packValuesElapsed;
  double packValuesHostBuildElapsed;
  double packValuesDeviceAllocElapsed;
  double packValuesH2dCopyElapsed;
  size_t packValuesH2dBytes;
  double allocateTrialElapsed;
  double projectionBatchElapsed;
  double projectionBatchHostBuildElapsed;
  double projectionBatchDeviceAllocElapsed;
  double projectionBatchH2dCopyElapsed;
  size_t projectionBatchH2dBytes;
  double initialErrorElapsed;
  double denseSchurSolverConstructionElapsed;
  double firstCudssAnalyzeElapsed;
  double downloadElapsed;
  double downloadHostAllocElapsed;
  double downloadD2hCopyElapsed;
  double downloadValuesBuildElapsed;
  size_t downloadD2hBytes;
  double totalH2dCopyElapsed;
  size_t totalH2dBytes;
  double totalD2hCopyElapsed;
  size_t totalD2hBytes;
  int iterations;
  int innerIterations;
  int acceptedSteps;
  double finalLambda;
  gtsam::Values optimizedValues;
};

gtsam::cuda::SfmLevenbergMarquardtResult optimizeSfm(
    const gtsam::SfmData& data,
    const gtsam::cuda::SfmLevenbergMarquardtParams& params);

gtsam::cuda::SfmLevenbergMarquardtResult
optimizeSfmWithoutValueDownload(
    const gtsam::SfmData& data,
    const gtsam::cuda::SfmLevenbergMarquardtParams& params);

class SfmLevenbergMarquardtOptimizer {
  SfmLevenbergMarquardtOptimizer(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues,
      const gtsam::cuda::SfmLevenbergMarquardtParams& params =
          gtsam::cuda::SfmLevenbergMarquardtParams());

  const gtsam::cuda::SfmLevenbergMarquardtParams& params() const;
  const gtsam::cuda::SfmLevenbergMarquardtResult& result() const;
  const gtsam::Values& optimize();
  const gtsam::Values& values() const;
  double error() const;
  size_t iterations() const;
};

}  // namespace cuda
}  // namespace gtsam
