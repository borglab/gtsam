//*************************************************************************
// GNC with CUDA inner optimizers. Compiled only when GTSAM_ENABLE_CUDA is on.
//*************************************************************************

namespace gtsam {

#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>
#include <gtsam/sfm/cuda/SfmLevenbergMarquardt.h>

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

typedef gtsam::GncParams<gtsam::cuda::SparseLevenbergMarquardtParams>
    GncCudaSparseLMParams;
typedef gtsam::GncOptimizer<
    gtsam::GncParams<gtsam::cuda::SparseLevenbergMarquardtParams>>
    GncCudaSparseLMOptimizer;

typedef gtsam::GncParams<gtsam::cuda::SfmLevenbergMarquardtParams>
    GncCudaSfmLMParams;
typedef gtsam::GncOptimizer<
    gtsam::GncParams<gtsam::cuda::SfmLevenbergMarquardtParams>>
    GncCudaSfmLMOptimizer;

}  // namespace gtsam
