//*************************************************************************
// nonlinear_continuous_time: Continuous-time Gaussian-process factors, kept
// separate from nonlinear.i to reduce wrapper build memory.
//*************************************************************************

namespace gtsam {

// Headers needed by classes separated from the original module.
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
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
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/GraphvizFormatting.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

//*************************************************************************
// Continuous-time Gaussian-process factor types
//*************************************************************************
#include <gtsam/nonlinear/WnoaStateData.h>
class StateData {
  StateData();
  StateData(gtsam::Key pose_in, gtsam::Key velocity_in, double time_in);
  gtsam::Key pose;
  gtsam::Key velocity;
  double time;
};

#include <gtsam/nonlinear/WnoaFactor.h>
template <POSE = {gtsam::Point1, gtsam::Point2, gtsam::Point3, gtsam::Pose2,
                  gtsam::Pose3}>
virtual class WnoaMotionFactor : gtsam::NoiseModelFactor {
  WnoaMotionFactor(const gtsam::StateData& state_k,
                   const gtsam::StateData& state_kp1,
                   const gtsam::Vector& q_psd_diag);
  gtsam::Vector evaluateError(const POSE& p1, const gtsam::This::Velocity& v1,
                              const POSE& p2, const gtsam::This::Velocity& v2,
                              gtsam::OptionalMatrixType Hp1 = nullptr,
                              gtsam::OptionalMatrixType Hv1 = nullptr,
                              gtsam::OptionalMatrixType Hp2 = nullptr,
                              gtsam::OptionalMatrixType Hv2 = nullptr) const;
};

#include <gtsam/nonlinear/WnoaInterpFactor.h>
template <POSE = {gtsam::Point1, gtsam::Point2, gtsam::Point3, gtsam::Pose2,
                  gtsam::Pose3}>
virtual class WnoaInterpFactor : gtsam::NoiseModelFactor {
  WnoaInterpFactor(const gtsam::NoiseModelFactor::shared_ptr inner_factor,
                   const std::set<gtsam::StateData> estimated_states,
                   const std::set<gtsam::StateData> interp_states,
                   const gtsam::Vector q_psd_diag,
                   const bool fixed_noise_model = false,
                   const bool precomp_interp_mats = true);
};

// Dummy Wrapper for ExpressionFactorGraph to enable inheritance for
// WnoaFactorGraph
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
virtual class ExpressionFactorGraph : gtsam::NonlinearFactorGraph {};

#include <gtsam/nonlinear/WnoaFactorGraph.h>
template <POSE = {gtsam::Point1, gtsam::Point2, gtsam::Point3, gtsam::Pose2,
                  gtsam::Pose3}>
class WnoaFactorGraph : gtsam::ExpressionFactorGraph {
  WnoaFactorGraph(
      std::unordered_map<gtsam::StateData,
                         std::pair<gtsam::StateData, gtsam::StateData>>
          interp_map,
      const gtsam::Vector q_psd_diag, bool fixed_noise_model = false);
};

template <POSE = {gtsam::Point1, gtsam::Point2, gtsam::Point3, gtsam::Pose2,
                  gtsam::Pose3}>
gtsam::NonlinearFactorGraph interpolateFactorGraph(
    const gtsam::NonlinearFactorGraph& graph,
    const std::set<gtsam::StateData>& estimated_states,
    const std::set<gtsam::StateData>& interp_states, gtsam::Vector q_psd_diag,
    bool fixed_noise = false);

template <POSE = {gtsam::Point1, gtsam::Point2, gtsam::Point3, gtsam::Pose2,
                  gtsam::Pose3}>
gtsam::WnoaFactorGraph<POSE> interpolateWnoaFactorGraph(
    const gtsam::NonlinearFactorGraph& graph,
    const std::set<gtsam::StateData>& estimated_states,
    const std::set<gtsam::StateData>& interp_states, gtsam::Vector q_psd_diag,
    bool fixed_noise = false);

template <POSE = {gtsam::Point1, gtsam::Point2, gtsam::Point3, gtsam::Pose2,
                  gtsam::Pose3}>
gtsam::Values updateInterpValues(
    const gtsam::NonlinearFactorGraph& interp_graph,
    const gtsam::Values& values, const std::set<gtsam::StateData>& estim_states,
    const std::set<gtsam::StateData>& interp_states,
    const gtsam::Vector q_psd_diag);

template <POSE = {gtsam::Point1, gtsam::Point2, gtsam::Point3, gtsam::Pose2,
                  gtsam::Pose3}>
std::pair<gtsam::Values, gtsam::InterpCovarianceMap>
updateInterpValuesWithCovariance(
    const gtsam::NonlinearFactorGraph& interp_graph,
    const gtsam::Values& values, const std::set<gtsam::StateData>& estim_states,
    const std::set<gtsam::StateData>& interp_states,
    const gtsam::Vector q_psd_diag);

//*************************************************************************
// Nonlinear factor types
//*************************************************************************
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/nonlinear/PriorFactor.h>
template <T = {double,
               gtsam::Vector,
               gtsam::Point1,
               gtsam::Vector6,
               gtsam::Point2,
               gtsam::StereoPoint2,
               gtsam::Point3,
               gtsam::Gal3,
               gtsam::Se23,
               gtsam::ExtendedPose3d,
               gtsam::Rot2,
               gtsam::SO3,
               gtsam::SO4,
               gtsam::SOn,
               gtsam::SL4,
               gtsam::Rot3,
               gtsam::Pose2,
               gtsam::Pose3,
               gtsam::Similarity2,
               gtsam::Similarity3,
               gtsam::Unit3,
               gtsam::Cal3_S2,
               gtsam::Cal3DS2,
               gtsam::Cal3Bundler,
               gtsam::Cal3Fisheye,
               gtsam::Cal3Unified,
               gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::SphericalCamera,
               gtsam::NavState,
               gtsam::imuBias::ConstantBias,
               gtsam::EssentialMatrix}>
virtual class PriorFactor : gtsam::NoiseModelFactor {
  PriorFactor(gtsam::Key key, const T& prior,
              const gtsam::noiseModel::Base* noiseModel = nullptr);
  const T& prior() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/ExtendedPriorFactor.h>
template <T = {double,
               gtsam::Vector,
               gtsam::Point2,
               gtsam::StereoPoint2,
               gtsam::Point3,
               gtsam::Gal3,
               gtsam::Rot2,
               gtsam::SO3,
               gtsam::SO4,
               gtsam::SOn,
               gtsam::SL4,
               gtsam::Rot3,
               gtsam::Pose2,
               gtsam::Pose3,
               gtsam::Similarity2,
               gtsam::Similarity3,
               gtsam::Unit3,
               gtsam::Cal3_S2,
               gtsam::Cal3DS2,
               gtsam::Cal3Bundler,
               gtsam::Cal3Fisheye,
               gtsam::Cal3Unified,
               gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::NavState,
               gtsam::imuBias::ConstantBias}>
virtual class ExtendedPriorFactor : gtsam::NoiseModelFactor {
  ExtendedPriorFactor(gtsam::Key key, const T& origin,
                      const gtsam::SharedNoiseModel& noiseModel);
  ExtendedPriorFactor(gtsam::Key key, const T& origin,
                      const gtsam::Vector& mean,
                      const gtsam::SharedNoiseModel& noiseModel);
  ExtendedPriorFactor(gtsam::Key key, const T& origin,
                      const gtsam::Matrix& covariance);
  ExtendedPriorFactor(gtsam::Key key, const T& origin,
                      const gtsam::Vector& mean,
                      const gtsam::Matrix& covariance);
  const T& origin() const;
  // Optional tangent space mean (may be empty / None)
  const std::optional<gtsam::Vector>& mean() const;
  std::optional<gtsam::Matrix> covariance(const string& method = "<unknown>",
                                          bool throwOnFailure = false) const;
  std::optional<std::pair<gtsam::Vector, gtsam::Matrix>> gaussian(
      const string& method = "<unknown>", bool throwOnFailure = false) const;

  // T-versions (vs. values)
  double error(const T& x) const;
  double likelihood(const T& x) const;
  gtsam::Vector evaluateError(const T& x) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/ConcentratedGaussian.h>
template <T = {double, gtsam::Vector, gtsam::Point2, gtsam::StereoPoint2,
               gtsam::Point3, gtsam::Gal3, gtsam::Rot2, gtsam::SO3, gtsam::Rot3,
               gtsam::Pose2, gtsam::Pose3}>
virtual class ConcentratedGaussian : gtsam::ExtendedPriorFactor<T> {
  ConcentratedGaussian();
  // Constructors mirroring header (origin terminology)
  ConcentratedGaussian(
      gtsam::Key key, const T& origin,
      const gtsam::noiseModel::Gaussian::shared_ptr& noiseModel);
  ConcentratedGaussian(
      gtsam::Key key, const T& origin, const gtsam::Vector& mean,
      const gtsam::noiseModel::Gaussian::shared_ptr& noiseModel);
  ConcentratedGaussian(gtsam::Key key, const T& origin,
                       const gtsam::Matrix& covariance);
  ConcentratedGaussian(gtsam::Key key, const T& origin,
                       const gtsam::Vector& mean,
                       const gtsam::Matrix& covariance);
  // Return element corresponding to mean, with optional Jacobian
  T retractMean() const;
  T retractMean(gtsam::Matrix& xHm) const;
  // Normalization constant (negative log) and log-probability helpers
  double negLogConstant() const;
  double logProbability(const T& x) const;
  double logProbability(const gtsam::Values& values) const;
  double evaluate(const T& x) const;
  double evaluate(const gtsam::Values& values) const;
  // Chart transport / reset operations
  This reset() const;
  This transportTo(const T& x_hat) const;
  // Fusion operator
  This operator*(const This& other) const;
};

#include <gtsam/nonlinear/VectorNormFactor.h>
template <N = {3}>
virtual class VectorNormFactor : gtsam::NoiseModelFactor {
  VectorNormFactor(gtsam::Key key, double norm,
                   const gtsam::noiseModel::Base* model);

  // Standard Interface
  double norm() const;
  gtsam::Vector evaluateError(const gtsam::Vector3& v) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/FixedLagSmoother.h>
// This class is not available in python, just use a dictionary
class FixedLagSmootherKeyTimestampMapValue {
  FixedLagSmootherKeyTimestampMapValue(gtsam::Key key, double timestamp);
  FixedLagSmootherKeyTimestampMapValue(
      const gtsam::FixedLagSmootherKeyTimestampMapValue& other);
};

// This class is not available in python, just use a dictionary
class FixedLagSmootherKeyTimestampMap {
  FixedLagSmootherKeyTimestampMap();
  FixedLagSmootherKeyTimestampMap(
      const gtsam::FixedLagSmootherKeyTimestampMap& other);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  double at(const gtsam::Key key) const;
  void insert(const gtsam::FixedLagSmootherKeyTimestampMapValue& value);
};

class FixedLagSmootherResult {
  size_t getIterations() const;
  size_t getIntermediateSteps() const;
  size_t getNonlinearVariables() const;
  size_t getLinearVariables() const;
  double getError() const;
  gtsam::FactorIndices getMarginalFactorIndices() const;
  gtsam::FactorIndices getDeletedFactorIndices() const;
  gtsam::KeySet getKeysOfDeletedNodes() const;
  gtsam::KeySet getExpiredPendingKeys() const;
  void print() const;
};

virtual class FixedLagSmoother {
  void print(const string& s = "FixedLagSmoother:\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::FixedLagSmoother& rhs, double tol = 1e-9) const;

  const gtsam::FixedLagSmootherKeyTimestampMap& timestamps() const;
  double smootherLag() const;
  void setSmootherLag(double smootherLag);
  const gtsam::KeySet& retainedKeys() const;

  gtsam::FixedLagSmootherResult update(
      const gtsam::NonlinearFactorGraph& newFactors =
          gtsam::NonlinearFactorGraph(),
      const gtsam::Values& newTheta = gtsam::Values(),
      const gtsam::FixedLagSmootherKeyTimestampMap& timestamps =
          gtsam::FixedLagSmootherKeyTimestampMap(),
      const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices());
  gtsam::Values calculateEstimate() const;
  gtsam::Values calculateEstimate(const gtsam::KeyVector& keys) const;
};

#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
virtual class BatchFixedLagSmoother : gtsam::FixedLagSmoother {
  BatchFixedLagSmoother();
  BatchFixedLagSmoother(double smootherLag);
  BatchFixedLagSmoother(double smootherLag,
                        const gtsam::LevenbergMarquardtParams& parameters);

  const gtsam::LevenbergMarquardtParams& params() const;

  const gtsam::NonlinearFactorGraph& getFactors() const;
  const gtsam::Values& getLinearizationPoint() const;
  const gtsam::Ordering& getOrdering() const;
  const gtsam::VectorValues& getDelta() const;

  gtsam::FixedLagSmootherResult update(
      const gtsam::NonlinearFactorGraph& newFactors =
          gtsam::NonlinearFactorGraph(),
      const gtsam::Values& newTheta = gtsam::Values(),
      const gtsam::FixedLagSmootherKeyTimestampMap& timestamps =
          gtsam::FixedLagSmootherKeyTimestampMap(),
      const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices());

  gtsam::FixedLagSmootherResult update(
      const gtsam::NonlinearFactorGraph& newFactors,
      const gtsam::Values& newTheta,
      const gtsam::FixedLagSmootherKeyTimestampMap& timestamps,
      const gtsam::FactorIndices& factorsToRemove,
      const gtsam::KeySet& keysToRetain,
      const gtsam::KeySet& keysToRelease = gtsam::KeySet());

  template <VALUE = {double, gtsam::Point2, gtsam::Rot2, gtsam::Pose2,
                     gtsam::Point3, gtsam::Rot3, gtsam::Pose3, gtsam::NavState,
                     gtsam::SL4, gtsam::Similarity2, gtsam::Similarity3,
                     gtsam::Cal3_S2, gtsam::Cal3DS2,
                     gtsam::imuBias::ConstantBias, gtsam::Vector,
                     gtsam::Matrix}>
  VALUE calculateEstimate(gtsam::Key key) const;
};

#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
virtual class IncrementalFixedLagSmoother : gtsam::FixedLagSmoother {
  IncrementalFixedLagSmoother();
  IncrementalFixedLagSmoother(double smootherLag);
  IncrementalFixedLagSmoother(double smootherLag,
                              const gtsam::ISAM2Params& parameters);

  gtsam::Matrix marginalCovariance(gtsam::Key key) const;
  const gtsam::ISAM2Params& params() const;

  const gtsam::NonlinearFactorGraph& getFactors() const;
  const gtsam::Values& getLinearizationPoint() const;
  const gtsam::VectorValues& getDelta() const;
  const gtsam::ISAM2& getISAM2() const;
  const gtsam::ISAM2Result& getISAM2Result() const;

  gtsam::FixedLagSmootherResult update(
      const gtsam::NonlinearFactorGraph& newFactors =
          gtsam::NonlinearFactorGraph(),
      const gtsam::Values& newTheta = gtsam::Values(),
      const gtsam::FixedLagSmootherKeyTimestampMap& timestamps =
          gtsam::FixedLagSmootherKeyTimestampMap(),
      const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices());

  gtsam::FixedLagSmootherResult update(
      const gtsam::NonlinearFactorGraph& newFactors,
      const gtsam::Values& newTheta,
      const gtsam::FixedLagSmootherKeyTimestampMap& timestamps,
      const gtsam::FactorIndices& factorsToRemove,
      const gtsam::KeySet& keysToRetain,
      const gtsam::KeySet& keysToRelease = gtsam::KeySet());

  // Mirrors gtsam::ISAM2::calculateEstimate<VALUE>, which this forwards to.
  template <VALUE = {double,
                     gtsam::Point2,
                     gtsam::Rot2,
                     gtsam::Pose2,
                     gtsam::Point3,
                     gtsam::Gal3,
                     gtsam::Rot3,
                     gtsam::Pose3,
                     gtsam::NavState,
                     gtsam::SL4,
                     gtsam::Similarity2,
                     gtsam::Similarity3,
                     gtsam::Cal3_S2,
                     gtsam::Cal3DS2,
                     gtsam::Cal3f,
                     gtsam::Cal3Bundler,
                     gtsam::imuBias::ConstantBias,
                     gtsam::EssentialMatrix,
                     gtsam::FundamentalMatrix,
                     gtsam::SimpleFundamentalMatrix,
                     gtsam::PinholeCamera<gtsam::Cal3_S2>,
                     gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                     gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                     gtsam::PinholeCamera<gtsam::Cal3Unified>,
                     gtsam::Vector,
                     gtsam::Matrix}>
  VALUE calculateEstimate(gtsam::Key key) const;
};

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
template <T = {gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::Rot3,
               gtsam::Pose2, gtsam::Pose3, gtsam::Gal3, gtsam::SL4,
               gtsam::Similarity2, gtsam::Similarity3, gtsam::NavState,
               gtsam::imuBias::ConstantBias}>
virtual class ExtendedKalmanFilter {
  ExtendedKalmanFilter(gtsam::Key key_initial, const T& x_initial,
                       const gtsam::noiseModel::Gaussian* P_initial);

  T predict(const gtsam::NoiseModelFactor& motionFactor);
  T update(const gtsam::NoiseModelFactor& measurementFactor);

  const gtsam::JacobianFactor::shared_ptr Density() const;
};

}  // namespace gtsam
