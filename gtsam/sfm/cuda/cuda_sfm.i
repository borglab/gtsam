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

#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/GncOptimizer.h>

// Each interface is parsed independently. Keep these template declarations
// consistent with nonlinear.i; the loss and scheduler enums are registered there.
template <PARAMS>
virtual class GncParams {
  GncParams(const PARAMS& baseOptimizerParams);
  GncParams();
  PARAMS baseOptimizerParams;
  gtsam::GncLossType lossType;
  size_t maxIterations;
  double lambdaStep;
  double relativeCostTol;
  double weightsTol;
  double lambdaMax;
  gtsam::This::Verbosity verbosity;
  gtsam::This::IndexVector knownInliers;
  gtsam::This::IndexVector knownOutliers;
  bool allowNonNoiseModelFactors;
  gtsam::GncScheduler scheduler;

  void setLossType(const gtsam::GncLossType type);
  void setMaxIterations(const size_t maxIter);
  void setLambdaStep(const double step);
  void setRelativeCostTol(double value);
  void setWeightsTol(double value);
  void setVerbosityGNC(const gtsam::This::Verbosity value);
  void setKnownInliers(const gtsam::This::IndexVector& knownIn);
  void setKnownOutliers(const gtsam::This::IndexVector& knownOut);
  void setAllowNonNoiseModelFactors(bool allow);
  void setScheduler(const gtsam::GncScheduler s);
  void print(const string& str = "GncParams: ") const;

  enum Verbosity { SILENT, SUMMARY, LAMBDA, WEIGHTS, VALUES };
};

template <PARAMS>
virtual class GncOptimizer {
  GncOptimizer(const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& initialValues, const PARAMS& params);
  void setInlierCostThresholds(const double inth);
  const gtsam::Vector& getInlierCostThresholds() const;
  void setInlierCostThresholdsAtProbability(const double alpha);
  void setWeights(const gtsam::Vector w);
  const gtsam::Vector& getWeights() const;
  const PARAMS& getParams() const;
  gtsam::Values optimize();
};

typedef gtsam::GncParams<gtsam::cuda::SfmLevenbergMarquardtParams>
    GncCudaSfmLMParams;
typedef gtsam::GncOptimizer<
    gtsam::GncParams<gtsam::cuda::SfmLevenbergMarquardtParams>>
    GncCudaSfmLMOptimizer;

}  // namespace gtsam
