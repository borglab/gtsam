//*************************************************************************
// sam
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

// #####

#include <gtsam/sam/RangeFactor.h>
template <POSE, POINT>
virtual class RangeFactor : gtsam::NoiseModelFactor {
  RangeFactor(gtsam::Key key1, gtsam::Key key2, double measured,
              const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;

  const double measured() const;
};

// between points:
typedef gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> RangeFactor2;
typedef gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> RangeFactor3;

// between pose and point:
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> RangeFactor2D;
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> RangeFactorPose2;

// between poses:
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> RangeFactor3D;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> RangeFactorPose3;

// more specialized types:
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>
    RangeFactorCalibratedCameraPoint;
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>
    RangeFactorSimpleCameraPoint;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>
    RangeFactorCalibratedCamera;
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>,
                           gtsam::PinholeCamera<gtsam::Cal3_S2>>
    RangeFactorSimpleCamera;

#include <gtsam/sam/RangeFactor.h>
template <POSE, POINT>
virtual class RangeFactorWithTransform : gtsam::NoiseModelFactor {
  RangeFactorWithTransform(gtsam::Key key1, gtsam::Key key2, double measured,
                           const gtsam::noiseModel::Base* noiseModel,
                           const POSE& body_T_sensor);

  // enabling serialization functionality
  void serialize() const;

  // Use `double` instead of template since that is all we need.
  const double measured() const;
};

typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>
    RangeFactorWithTransform2D;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>
    RangeFactorWithTransform3D;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>
    RangeFactorWithTransformPose2;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>
    RangeFactorWithTransformPose3;

#include <gtsam/sam/RangeFactor.h>
template <POSE, POINT>
virtual class RangeFactorWithTransformBias : gtsam::NoiseModelFactor {
  RangeFactorWithTransformBias(gtsam::Key key1, gtsam::Key key2,
                               gtsam::Key key3, double measured,
                               const gtsam::noiseModel::Base* noiseModel,
                               const POSE& body_T_sensor);

  // enabling serialization functionality
  void serialize() const;

  const double measured() const;
};

typedef gtsam::RangeFactorWithTransformBias<gtsam::Pose2, gtsam::Point2>
    RangeFactorWithTransformBias2D;
typedef gtsam::RangeFactorWithTransformBias<gtsam::Pose3, gtsam::Point3>
    RangeFactorWithTransformBias3D;

#include <gtsam/sam/BearingFactor.h>
template <POSE, POINT, BEARING>
virtual class BearingFactor : gtsam::NoiseModelFactor {
  BearingFactor(gtsam::Key key1, gtsam::Key key2, const BEARING& measured,
                const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;

  const BEARING& measured() const;
};

typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>
    BearingFactor2D;
typedef gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>
    BearingFactor3D;
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>
    BearingFactorPose2;

#include <gtsam/sam/BearingRangeFactor.h>
template <POSE, POINT, BEARING, RANGE>
virtual class BearingRangeFactor : gtsam::NoiseModelFactor {
  BearingRangeFactor(gtsam::Key poseKey, gtsam::Key pointKey,
                     const BEARING& measuredBearing, const RANGE& measuredRange,
                     const gtsam::noiseModel::Base* noiseModel);

  gtsam::BearingRange<POSE, POINT, BEARING, RANGE> measured() const;

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2,
                                  double>
    BearingRangeFactor2D;
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2,
                                  double>
    BearingRangeFactorPose2;
typedef gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3,
                                  double>
    BearingRangeFactor3D;
typedef gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3,
                                  double>
    BearingRangeFactorPose3;

#include <gtsam/sam/RISAMGraduationScheduler.h>
class GraduationScheduler {
  GraduationScheduler();
  GraduationScheduler(
      const gtsam::GraduationScheduler::MuUpdateStrategy& mu_update_strat,
      double mu_init_increment = 0.2, double mu_init = 0.0);

  double muInit() const;
  double updateMu(const double& mu, const double& residual,
                  const size_t& update_count) const;
  double updateMuInit(const double& mu_init, const bool is_inlier) const;
  bool isMuConverged(const double& mu) const;

  static double muUpdateMcGann2023(const double& mu, const double& residual,
                                   const size_t& update_count);
  static double muUpdateStable(const double& mu, const double& residual,
                               const size_t& update_count);
};

/** Graduated factors. The instantiation lists mirror those of the factors they
 * wrap, so every factor available in Python has a graduated counterpart.
 */
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/Point1.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/SphericalCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/sam/RISAMGraduatedFactor.h>
#include <gtsam/slam/BetweenFactor.h>
template <FACTOR_TYPE = {gtsam::BetweenFactor<double>,
                         gtsam::BetweenFactor<gtsam::Vector>,
                         gtsam::BetweenFactor<gtsam::Point2>,
                         gtsam::BetweenFactor<gtsam::Point3>,
                         gtsam::BetweenFactor<gtsam::Rot2>,
                         gtsam::BetweenFactor<gtsam::SO3>,
                         gtsam::BetweenFactor<gtsam::SO4>,
                         gtsam::BetweenFactor<gtsam::SL4>,
                         gtsam::BetweenFactor<gtsam::Rot3>,
                         gtsam::BetweenFactor<gtsam::Pose2>,
                         gtsam::BetweenFactor<gtsam::Pose3>,
                         gtsam::BetweenFactor<gtsam::Similarity2>,
                         gtsam::BetweenFactor<gtsam::Similarity3>,
                         gtsam::BetweenFactor<gtsam::Gal3>,
                         gtsam::BetweenFactor<gtsam::NavState>,
                         gtsam::BetweenFactor<gtsam::Se23>,
                         gtsam::BetweenFactor<gtsam::ExtendedPose3d>,
                         gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>,
                         gtsam::BetweenFactor<gtsam::SOn>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2,
                         const FACTOR_TYPE::T& relativePose,
                         const gtsam::noiseModel::Base* noiseModel);

  FACTOR_TYPE::T measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

#include <gtsam/nonlinear/PriorFactor.h>
template <FACTOR_TYPE = {gtsam::PriorFactor<double>,
                         gtsam::PriorFactor<gtsam::Vector>,
                         gtsam::PriorFactor<gtsam::Point1>,
                         gtsam::PriorFactor<gtsam::Vector6>,
                         gtsam::PriorFactor<gtsam::Point2>,
                         gtsam::PriorFactor<gtsam::StereoPoint2>,
                         gtsam::PriorFactor<gtsam::Point3>,
                         gtsam::PriorFactor<gtsam::Gal3>,
                         gtsam::PriorFactor<gtsam::Se23>,
                         gtsam::PriorFactor<gtsam::ExtendedPose3d>,
                         gtsam::PriorFactor<gtsam::Rot2>,
                         gtsam::PriorFactor<gtsam::SO3>,
                         gtsam::PriorFactor<gtsam::SO4>,
                         gtsam::PriorFactor<gtsam::SOn>,
                         gtsam::PriorFactor<gtsam::SL4>,
                         gtsam::PriorFactor<gtsam::Rot3>,
                         gtsam::PriorFactor<gtsam::Pose2>,
                         gtsam::PriorFactor<gtsam::Pose3>,
                         gtsam::PriorFactor<gtsam::Similarity2>,
                         gtsam::PriorFactor<gtsam::Similarity3>,
                         gtsam::PriorFactor<gtsam::Unit3>,
                         gtsam::PriorFactor<gtsam::Cal3_S2>,
                         gtsam::PriorFactor<gtsam::Cal3DS2>,
                         gtsam::PriorFactor<gtsam::Cal3Bundler>,
                         gtsam::PriorFactor<gtsam::Cal3Fisheye>,
                         gtsam::PriorFactor<gtsam::Cal3Unified>,
                         gtsam::PriorFactor<gtsam::CalibratedCamera>,
                         gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>,
                         gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>,
                         gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>,
                         gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>,
                         gtsam::PriorFactor<gtsam::SphericalCamera>,
                         gtsam::PriorFactor<gtsam::NavState>,
                         gtsam::PriorFactor<gtsam::imuBias::ConstantBias>,
                         gtsam::PriorFactor<gtsam::EssentialMatrix>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key, const FACTOR_TYPE::T& prior,
                         const gtsam::noiseModel::Base* noiseModel);

  FACTOR_TYPE::T prior() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

#include <gtsam/sam/RangeFactor.h>
template <FACTOR_TYPE = {gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>,
                         gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>,
                         gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>,
                         gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>,
                         gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>,
                         gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>,
                         gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>,
                         gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>,
                         gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>,
                         gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2, double measured,
                         const gtsam::noiseModel::Base* noiseModel);

  double measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

/// RangeFactorWithTransform keeps its pose type private, so the body_T_sensor
/// type is spelled out and the 2D and 3D instantiations are split.
template <FACTOR_TYPE = {gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>,
                         gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2, double measured,
                         const gtsam::noiseModel::Base* noiseModel,
                         const gtsam::Pose2& body_T_sensor);

  double measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

template <FACTOR_TYPE = {gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>,
                         gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2, double measured,
                         const gtsam::noiseModel::Base* noiseModel,
                         const gtsam::Pose3& body_T_sensor);

  double measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

template <FACTOR_TYPE = {gtsam::RangeFactorWithTransformBias<gtsam::Pose2, gtsam::Point2>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2, gtsam::Key key3,
                         double measured,
                         const gtsam::noiseModel::Base* noiseModel,
                         const gtsam::Pose2& body_T_sensor);

  double measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

template <FACTOR_TYPE = {gtsam::RangeFactorWithTransformBias<gtsam::Pose3, gtsam::Point3>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2, gtsam::Key key3,
                         double measured,
                         const gtsam::noiseModel::Base* noiseModel,
                         const gtsam::Pose3& body_T_sensor);

  double measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

#include <gtsam/sam/BearingFactor.h>
template <FACTOR_TYPE = {gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2>,
                         gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2,
                         const gtsam::Rot2& measured,
                         const gtsam::noiseModel::Base* noiseModel);

  gtsam::Rot2 measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

template <FACTOR_TYPE = {gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key key1, gtsam::Key key2,
                         const gtsam::Unit3& measured,
                         const gtsam::noiseModel::Base* noiseModel);

  gtsam::Unit3 measured() const;
  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

/// BearingRangeFactor keeps its measurement typedef private, so measured() is
/// not exposed and the 2D and 3D instantiations are split.
#include <gtsam/sam/BearingRangeFactor.h>
template <FACTOR_TYPE = {gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>,
                         gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key poseKey, gtsam::Key pointKey,
                         const gtsam::Rot2& measuredBearing,
                         const double& measuredRange,
                         const gtsam::noiseModel::Base* noiseModel);

  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

template <FACTOR_TYPE = {gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>,
                         gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>}>
virtual class GenericGraduatedFactor : gtsam::NoiseModelFactor {
  GenericGraduatedFactor(const gtsam::noiseModel::mEstimator::Base* loss,
                         const gtsam::GraduationScheduler* scheduler,
                         gtsam::Key poseKey, gtsam::Key pointKey,
                         const gtsam::Unit3& measuredBearing,
                         const double& measuredRange,
                         const gtsam::noiseModel::Base* noiseModel);

  double residual(const gtsam::Values& current_estimate) const;
  double robustLoss(const gtsam::Values& current_estimate) const;
  gtsam::noiseModel::mEstimator::Base* loss() const;
  gtsam::GraduationScheduler* scheduler() const;
  gtsam::NonlinearFactor* cloneUngraduated() const;
};

#include <gtsam/sam/RISAM.h>
class RISAMParams {
  RISAMParams();
  RISAMParams(const gtsam::ISAM2Params& isam2_params, bool increment_outlier_mu,
              double outlier_mu_chisq_upper_bound,
              double outlier_mu_chisq_lower_bound,
              double outlier_mu_avg_var_convergence_thresh,
              size_t number_extra_iters);

  gtsam::ISAM2Params isam2_params;
  bool increment_outlier_mu;
  double outlier_mu_chisq_upper_bound;
  double outlier_mu_chisq_lower_bound;
  double outlier_mu_avg_var_convergence_thresh;
  size_t number_extra_iters;
};

class RISAMUpdateResult {
  gtsam::ISAM2Result isam2_result;
  std::set<gtsam::Key> involved_variables;
  std::set<gtsam::Key> affected_variables;
  std::set<gtsam::FactorIndex> convexified_factors;
};

class RISAM {
  RISAM(const gtsam::RISAMParams& params);

  gtsam::RISAMUpdateResult update();
  gtsam::RISAMUpdateResult update(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& new_theta);
  gtsam::RISAMUpdateResult update(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& new_theta,
      const std::optional<std::set<gtsam::Key>> extra_gnc_involved_keys);
  gtsam::RISAMUpdateResult update(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& new_theta,
      const std::optional<std::set<gtsam::Key>> extra_gnc_involved_keys,
      const gtsam::FactorIndices& remove_factor_indices);
  gtsam::RISAMUpdateResult update(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& new_theta,
      const std::optional<std::set<gtsam::Key>> extra_gnc_involved_keys,
      const gtsam::FactorIndices& remove_factor_indices,
      const std::optional<gtsam::KeyGroupMap>& constrained_keys,
      const std::optional<gtsam::KeyList>& no_relin_keys,
      const std::optional<gtsam::KeyList>& extra_reelim_keys,
      bool force_relinearize = false);
  gtsam::RISAMUpdateResult update(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& new_theta,
      const std::optional<std::set<gtsam::Key>> extra_gnc_involved_keys,
      const gtsam::ISAM2UpdateParams& update_params);

  gtsam::Values calculateEstimate();
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
  std::set<size_t> getOutliers(double chi2_outlier_thresh);
};

}  // namespace gtsam
