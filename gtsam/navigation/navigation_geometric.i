//*************************************************************************
// navigation_geometric: Lie-group and equivariant filters, kept separate from
// navigation.i to reduce wrapper build memory.
//*************************************************************************

namespace gtsam {

// Headers needed by classes separated from the original module.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/BarometricFactor.h>
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/CombinedImuFactorWithGravity.h>
#include <gtsam/navigation/ConstantVelocityFactor.h>
#include <gtsam/navigation/DopplerFactor.h>
#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuFactorWithGravity.h>
#include <gtsam/navigation/MagFactor.h>
#include <gtsam/navigation/MagPoseFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/navigation/ScenarioRunner.h>
// ---------------------------------------------------------------------------
// EKF classes
#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/ManifoldEKF.h>
template <M = {gtsam::Unit3, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3,
               gtsam::NavState, gtsam::Gal3}>
virtual class ManifoldEKF {
  // Constructors
  ManifoldEKF(const M& X0, const gtsam::This::Covariance& P0);

  // Accessors
  const M& state() const;
  const gtsam::This::Covariance& covariance() const;
  size_t dimension() const;

  // Predict with provided next state and Jacobian
  void predict(const M& X_next, const gtsam::This::Jacobian& F,
               const gtsam::This::Covariance& Q);

  // Only vector-based measurements are supported in wrapper
  void updateWithVector(const gtsam::Vector& prediction, const gtsam::Matrix& H,
                        const gtsam::Vector& z, const gtsam::Matrix& R,
                        bool performReset = true);
};

#include <gtsam/navigation/LieGroupEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState,
               gtsam::Gal3}>
virtual class LieGroupEKF : gtsam::ManifoldEKF<G> {
  // Constructors
  LieGroupEKF(const G& X0, const gtsam::This::Covariance& P0);

  // Increment-based predict (precomputed increment and Jacobian)
  void predictWithCompose(const G& U, const gtsam::This::Jacobian& J_UX,
                          const gtsam::This::Covariance& Q);
};

#include <gtsam/navigation/LeftLinearEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState,
               gtsam::Gal3}>
virtual class LeftLinearEKF : gtsam::LieGroupEKF<G> {
  // Constructors
  LeftLinearEKF(const G& X0, const gtsam::This::Covariance& P0);
};

#include <gtsam/navigation/InvariantEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState,
               gtsam::Gal3}>
virtual class InvariantEKF : gtsam::LeftLinearEKF<G> {
  // Constructors
  InvariantEKF(const G& X0, const gtsam::This::Covariance& P0);

  // Left-invariant predict APIs
  void predict(const G& U, const gtsam::This::Covariance& Q);
  void predict(const G& W, const G& U, const gtsam::This::Covariance& Q);
  void predict(const gtsam::This::TangentVector& u, double dt,
               const gtsam::This::Covariance& Q);
};

// ---------------------------------------------------------------------------
// ABC Equivariant Filter
#include <gtsam_unstable/geometry/ABCEquivariantFilter.h>
namespace abc {
template <N = {1, 2, 3}>
class AbcEquivariantFilter {
  // Constructors
  AbcEquivariantFilter();
  AbcEquivariantFilter(const gtsam::Matrix6& Sigma0);

  // Predict and update methods
  void predict(const gtsam::Vector3& omega,
               const gtsam::Matrix6& inputCovariance, double dt);
  void update(const gtsam::Unit3& y, const gtsam::Unit3& d,
              const gtsam::Matrix3& R, int cal_idx);

  // Accessors
  gtsam::Rot3 attitude() const;
  gtsam::Vector3 bias() const;
  gtsam::Rot3 calibration(size_t i) const;
};
}  // namespace abc

// Specialized NavState IMU EKF
#include <gtsam/navigation/NavStateImuEKF.h>
class NavStateImuEKF : gtsam::LeftLinearEKF<gtsam::NavState> {
  // Constructors
  NavStateImuEKF(const gtsam::NavState& X0, const gtsam::Matrix9& P0,
                 const gtsam::PreintegrationParams* params);

  // Accessors
  const gtsam::Matrix9& processNoise() const;
  const gtsam::Vector3& gravity() const;
  const std::shared_ptr<gtsam::PreintegrationParams>& params() const;

  // Static methods
  static gtsam::NavState Gravity(const gtsam::Vector3& n_gravity, double dt);
  static gtsam::NavState Imu(const gtsam::Vector3& omega_b,
                             const gtsam::Vector3& f_b, double dt);
  static gtsam::NavState Dynamics(const gtsam::Vector3& n_gravity,
                                  const gtsam::NavState& X,
                                  const gtsam::Vector3& omega_b,
                                  const gtsam::Vector3& f_b, double dt,
                                  gtsam::OptionalJacobian<9, 9> A = nullptr);

  // Predict using IMU measurements
  void predict(const gtsam::Vector3& omega_b, const gtsam::Vector3& f_b,
               double dt);
};

#include <gtsam/navigation/Gal3ImuEKF.h>
class Gal3ImuEKF : gtsam::InvariantEKF<gtsam::Gal3> {
  enum Mode { NO_TIME, TRACK_TIME_NO_COVARIANCE, TRACK_TIME_WITH_COVARIANCE };
  // Constructors
  Gal3ImuEKF(const gtsam::Gal3& X0, const gtsam::Gal3ImuEKF::Covariance& P0,
             const gtsam::PreintegrationParams*
                 params);  // mode = TRACK_TIME_NO_COVARIANCE
  Gal3ImuEKF(const gtsam::Gal3& X0, const gtsam::Gal3ImuEKF::Covariance& P0,
             const gtsam::PreintegrationParams* params,
             gtsam::Gal3ImuEKF::Mode mode);

  // Accessors
  const gtsam::Gal3ImuEKF::Covariance& processNoise() const;
  const gtsam::Vector3& gravity() const;
  const std::shared_ptr<gtsam::PreintegrationParams>& params() const;

  // Static methods
  static gtsam::Gal3 Gravity(const gtsam::Vector3& g_n, double dt);
  static gtsam::Gal3 TimeZeroingGravity(const gtsam::Vector3& g_n, double dt);
  static gtsam::Gal3 CompensatedGravity(const gtsam::Vector3& g_n, double dt,
                                        double t_k);
  static gtsam::Gal3 Imu(const gtsam::Vector3& omega_b,
                         const gtsam::Vector3& f_b, double dt);
  static gtsam::Gal3 Dynamics(const gtsam::Vector3& n_gravity,
                              const gtsam::Gal3& X,
                              const gtsam::Vector3& omega_b,
                              const gtsam::Vector3& f_b, double dt,
                              gtsam::Gal3ImuEKF::Mode mode,
                              gtsam::OptionalJacobian<10, 10> A = nullptr);

  // Predict using IMU measurements
  void predict(const gtsam::Vector3& omega_b, const gtsam::Vector3& f_b,
               double dt);
};

#include <gtsam/navigation/LeggedEstimator.h>
class ContactMeasurement {
  ContactMeasurement();
  size_t foot;
  gtsam::Vector3 bodyPoint;
  bool touchdown;
};

class LeggedEstimatorParams {
  LeggedEstimatorParams();
  std::shared_ptr<gtsam::PreintegrationParams> preintegrationParams;
  gtsam::Pose3 body_P_imu;
  double footholdProcessSigma;
  double footholdInitSigma;
  gtsam::Matrix3 contactCovariance;
  double heightPriorSigma;
  bool useRobustContactNoise;
  double robustContactHuberK;
  gtsam::imuBias::ConstantBias imuBias;
  double biasAccRandomWalkSigma;
  double biasOmegaRandomWalkSigma;
  bool useFullContactInitialization;
  bool marginalizeLeavingFoot;
};

virtual class LeggedEstimator {
  void turnHeightPriorOn(double terrainHeight);
  void turnHeightPriorOff();
  void predict(const gtsam::Vector3& omegaBody,
               const gtsam::Vector3& specificForceBody, double dt);
  void processContacts(
      const std::vector<gtsam::ContactMeasurement>& activeContacts);
  gtsam::ExtendedPose3d estimate() const;
  gtsam::imuBias::ConstantBias estimateBias() const;
};

class LeggedInvariantEKF : gtsam::LeggedEstimator {
  LeggedInvariantEKF(const gtsam::NavState& navState0,
                     const gtsam::Matrix& footholds0, const gtsam::Matrix& P0,
                     const gtsam::LeggedEstimatorParams& params,
                     const std::vector<std::string>& footNames);
  // LeggedEstimator is the second C++ base. Binding these through that base
  // misadjusts `this` at runtime because the first base is not wrapped.
  void turnHeightPriorOn(double terrainHeight);
  void turnHeightPriorOff();
  void predict(const gtsam::Vector3& omegaBody,
               const gtsam::Vector3& specificForceBody, double dt);
  void processContacts(
      const std::vector<gtsam::ContactMeasurement>& activeContacts);
  gtsam::ExtendedPose3d estimate() const;
  gtsam::imuBias::ConstantBias estimateBias() const;
  gtsam::Matrix covariance() const;
  size_t numFeet() const;
};

class LeggedInvariantIEKF : gtsam::LeggedInvariantEKF {
  LeggedInvariantIEKF(const gtsam::NavState& navState0,
                      const gtsam::Matrix& footholds0, const gtsam::Matrix& P0,
                      const gtsam::LeggedEstimatorParams& params,
                      const std::vector<std::string>& footNames);
};

class LeggedFixedLagSmoother : gtsam::LeggedEstimator {
  LeggedFixedLagSmoother(const gtsam::NavState& navState0,
                         const gtsam::Matrix& footholds0,
                         const gtsam::Matrix9& baseCovariance0,
                         const gtsam::LeggedEstimatorParams& params,
                         double lagSeconds,
                         const std::vector<std::string>& footNames);
  size_t numFeet() const;
};

class LeggedCombinedFixedLagSmoother : gtsam::LeggedEstimator {
  LeggedCombinedFixedLagSmoother(const gtsam::NavState& navState0,
                                 const gtsam::Matrix& footholds0,
                                 const gtsam::Matrix9& baseCovariance0,
                                 const gtsam::LeggedEstimatorParams& params,
                                 double lagSeconds,
                                 const std::vector<std::string>& footNames);
  size_t numFeet() const;
};
}  // namespace gtsam