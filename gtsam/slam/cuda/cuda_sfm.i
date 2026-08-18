//*************************************************************************
// CUDA SFM
//*************************************************************************

namespace gtsam {

#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>

namespace cuda {

#include <gtsam/slam/cuda/SfmLevenbergMarquardt.h>

enum class SfmSystemFormulation {
  Schur,
  FullNormal
};

class SfmLevenbergMarquardtParams
    : gtsam::LevenbergMarquardtParams {
  SfmLevenbergMarquardtParams();

  static gtsam::cuda::SfmLevenbergMarquardtParams legacyDefaults();
  static gtsam::cuda::SfmLevenbergMarquardtParams ceresDefaults();

  bool enableDetailedProfiling;
  gtsam::cuda::SfmSystemFormulation formulation;

  string getLinearSolver() const;
  void setLinearSolver(const string& solver);
  string getFormulation() const;
  void setFormulation(const string& formulationName);
  string getLinearSolverBackend() const;
  void setLinearSolverBackend(const string& solverName);
  double getMinDiagonal() const;
  double getMaxDiagonal() const;
  void setMinDiagonal(double value);
  void setMaxDiagonal(double value);
  void print(const string& str = "") const;
};

class SfmLevenbergMarquardtResult {
  SfmLevenbergMarquardtResult();

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
  double cudssSolverConstructionElapsed;
  double denseSchurSolverConstructionElapsed;
  double csrStructureElapsed;
  double uploadPatternElapsed;
  double uploadPatternDeviceAllocElapsed;
  double uploadPatternH2dCopyElapsed;
  size_t uploadPatternH2dBytes;
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

virtual class SfmLevenbergMarquardtOptimizer
    : gtsam::NonlinearOptimizer {
  SfmLevenbergMarquardtOptimizer(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues,
      const gtsam::cuda::SfmLevenbergMarquardtParams& params =
          gtsam::cuda::SfmLevenbergMarquardtParams());

  const gtsam::cuda::SfmLevenbergMarquardtParams& params() const;
  const gtsam::cuda::SfmLevenbergMarquardtResult& result() const;
};

}  // namespace cuda
}  // namespace gtsam
