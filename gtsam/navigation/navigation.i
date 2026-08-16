//*************************************************************************
// Navigation
//*************************************************************************

namespace gtsam {

namespace imuBias {
#include <gtsam/navigation/ImuBias.h>

class ConstantBias {
  // Constructors
  ConstantBias();
  ConstantBias(const gtsam::Vector3& biasAcc,
               const gtsam::Vector3& biasGyro);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::imuBias::ConstantBias& expected, double tol) const;

  // Group
  static gtsam::imuBias::ConstantBias Identity();

  // Operator Overloads
  gtsam::imuBias::ConstantBias operator-() const;
  gtsam::imuBias::ConstantBias operator+(const gtsam::imuBias::ConstantBias& b) const;
  gtsam::imuBias::ConstantBias operator-(const gtsam::imuBias::ConstantBias& b) const;

  // Standard Interface
  gtsam::Vector6 vector() const;
  const gtsam::Vector3& accelerometer() const;
  const gtsam::Vector3& gyroscope() const;
  gtsam::Vector3 correctAccelerometer(
      const gtsam::Vector3& measurement,
      gtsam::OptionalJacobian<3, 6> H1 = nullptr,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;
  gtsam::Vector3 correctGyroscope(
      const gtsam::Vector3& measurement,
      gtsam::OptionalJacobian<3, 6> H1 = nullptr,
      gtsam::OptionalJacobian<3, 3> H2 = nullptr) const;

  // Manifold
  gtsam::imuBias::ConstantBias retract(const gtsam::Vector6& v) const;
  gtsam::Vector6 localCoordinates(
      const gtsam::imuBias::ConstantBias& b) const;

  // enabling serialization functionality
  void serialize() const;
};

}///\namespace imuBias

#include <gtsam/navigation/NavState.h>
class NavState {
  // Constructors
  NavState();
  NavState(const gtsam::Rot3& R, const gtsam::Point3& t,
           const gtsam::Vector3& v);
  NavState(const gtsam::Pose3& pose, const gtsam::Vector3& v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::NavState& other, double tol) const;

  // Access
  const gtsam::Rot3& attitude(
      gtsam::OptionalJacobian<3, 9> H = nullptr) const;
  gtsam::Point3 position(
      gtsam::OptionalJacobian<3, 9> H = nullptr) const;
  gtsam::Vector3 velocity(
      gtsam::OptionalJacobian<3, 9> H = nullptr) const;
  gtsam::Vector3 bodyVelocity(
      gtsam::OptionalJacobian<3, 9> H = nullptr) const;
  const gtsam::Pose3 pose() const;

  // Standard Interface
  double range(const gtsam::Point3& point,
               gtsam::OptionalJacobian<1, 9> Hself = nullptr,
               gtsam::OptionalJacobian<1, 3> Hpoint = nullptr) const;
  gtsam::Unit3 bearing(const gtsam::Point3& point,
                       gtsam::OptionalJacobian<2, 9> Hself = nullptr,
                       gtsam::OptionalJacobian<2, 3> Hpoint = nullptr) const;

  // Group
  static gtsam::NavState Identity();
  gtsam::NavState inverse() const;
  gtsam::NavState compose(const gtsam::NavState& p2) const;
  gtsam::NavState between(const gtsam::NavState& p2) const;

  // Operator Overloads
  gtsam::NavState operator*(const gtsam::NavState& p2) const;

  // Manifold
  gtsam::NavState retract(
      const gtsam::Vector9& v,
      gtsam::OptionalJacobian<9, 9> H1 = nullptr,
      gtsam::OptionalJacobian<9, 9> H2 = nullptr) const;
  gtsam::Vector9 localCoordinates(
      const gtsam::NavState& g,
      gtsam::OptionalJacobian<9, 9> H1 = nullptr,
      gtsam::OptionalJacobian<9, 9> H2 = nullptr) const;

  // Lie Group
  static gtsam::NavState Expmap(
      const gtsam::Vector9& xi,
      gtsam::OptionalJacobian<9, 9> Hxi = nullptr);
  static gtsam::Vector9 Logmap(
      const gtsam::NavState& pose,
      gtsam::OptionalJacobian<9, 9> Hpose = nullptr);
  gtsam::NavState expmap(const gtsam::Vector9& v) const;
  gtsam::NavState expmap(const gtsam::Vector9& v,
                        gtsam::OptionalJacobian<9, 9> H1,
                        gtsam::OptionalJacobian<9, 9> H2 = nullptr) const;
  gtsam::Vector9 logmap(const gtsam::NavState& p) const;
  gtsam::Vector9 logmap(
      const gtsam::NavState& p, gtsam::OptionalJacobian<9, 9> H1,
      gtsam::OptionalJacobian<9, 9> H2 = nullptr) const;

  // Matrix Lie Group
  gtsam::Matrix9 AdjointMap() const;
  gtsam::Vector9 Adjoint(
      const gtsam::Vector9& xi_b,
      gtsam::OptionalJacobian<9, 9> H_this = nullptr,
      gtsam::OptionalJacobian<9, 9> H_xi = nullptr) const;
  gtsam::Vector9 AdjointTranspose(
      const gtsam::Vector9& x,
      gtsam::OptionalJacobian<9, 9> H_this = nullptr,
      gtsam::OptionalJacobian<9, 9> H_x = nullptr) const;
  static gtsam::Matrix9 adjointMap(const gtsam::Vector9& xi);
  static gtsam::Vector9 adjoint(
      const gtsam::Vector9& xi, const gtsam::Vector9& y,
      gtsam::OptionalJacobian<9, 9> Hxi = nullptr,
      gtsam::OptionalJacobian<9, 9> H_y = nullptr);
  static gtsam::Vector9 adjointTranspose(
      const gtsam::Vector9& xi, const gtsam::Vector9& y,
      gtsam::OptionalJacobian<9, 9> Hxi = nullptr,
      gtsam::OptionalJacobian<9, 9> H_y = nullptr);
  gtsam::NavState::Vector25 vec(
      gtsam::OptionalJacobian<25, 9> H = nullptr) const;
  gtsam::Matrix5 matrix() const;
  static gtsam::Matrix5 Hat(const gtsam::Vector9& xi);
  static gtsam::Vector9 Vee(const gtsam::Matrix5& X);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/PreintegratedRotation.h>
virtual class PreintegratedRotationParams {
  PreintegratedRotationParams();

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegratedRotationParams& other,
              double tol) const;

  void setGyroscopeCovariance(const gtsam::Matrix3& cov);
  void setOmegaCoriolis(const gtsam::Vector3& omega);
  void setBodyPSensor(const gtsam::Pose3& pose);

  const gtsam::Matrix3& getGyroscopeCovariance() const;

  std::optional<gtsam::Vector3> getOmegaCoriolis() const;
  std::optional<gtsam::Pose3> getBodyPSensor() const;

  // enabling serialization functionality
  void serialize() const;
};

class PreintegratedRotation {
  // Constructors
  PreintegratedRotation(const gtsam::PreintegratedRotationParams* params);

  // Standard Interface
  void resetIntegration();
  void integrateGyroMeasurement(
      const gtsam::Vector3& measuredOmega, const gtsam::Vector3& biasHat,
      double deltaT, gtsam::OptionalJacobian<3, 3> F = nullptr);
  gtsam::Rot3 biascorrectedDeltaRij(
      const gtsam::Vector3& biasOmegaIncr,
      gtsam::OptionalJacobian<3, 3> H = nullptr) const;
  gtsam::Vector3 integrateCoriolis(
      const gtsam::Rot3& rot_i,
      gtsam::OptionalJacobian<3, 3> H = nullptr) const;

  // Access instance variables
  const double& deltaTij() const;
  const gtsam::Rot3& deltaRij() const;
  const gtsam::Matrix3& delRdelBiasOmega() const;

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegratedRotation& other, double tol) const;

  // enabling serialization functionality
  void serialize() const;
};

gtsam::Rot3 integrateSequentialRotations(
    const gtsam::Vector& times, gtsam::ConstMatrixView measuredOmegas,
    const gtsam::Vector3& biasHat = gtsam::Vector3::Zero(),
    const gtsam::Rot3& body_R_sensor = gtsam::Rot3());

gtsam::Rot3 integrateSingleSpeedConing(
    const gtsam::Vector& times, gtsam::ConstMatrixView measuredOmegas,
    const gtsam::Vector3& biasHat = gtsam::Vector3::Zero(),
    const gtsam::Rot3& body_R_sensor = gtsam::Rot3());

#include <gtsam/navigation/PreintegrationParams.h>
virtual class PreintegrationParams : gtsam::PreintegratedRotationParams {
  PreintegrationParams(const gtsam::Vector3& n_gravity);

  gtsam::Vector3 n_gravity;

  static gtsam::PreintegrationParams* MakeSharedD(double g = 9.81);
  static gtsam::PreintegrationParams* MakeSharedU(double g = 9.81);

  void setAccelerometerCovariance(const gtsam::Matrix3& cov);
  void setIntegrationCovariance(const gtsam::Matrix3& cov);
  void setUse2ndOrderCoriolis(bool flag);

  const gtsam::Matrix3& getAccelerometerCovariance() const;
  const gtsam::Matrix3& getIntegrationCovariance() const;
  bool   getUse2ndOrderCoriolis()     const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/ImuFactor.h>
class PreintegratedImuMeasurements {
  // Constructors
  PreintegratedImuMeasurements(const gtsam::PreintegrationParams* params);
  PreintegratedImuMeasurements(const gtsam::PreintegrationParams* params,
      const gtsam::imuBias::ConstantBias& bias);

  // Testable
  void print(string s = "") const;
  bool equals(
      const gtsam::PreintegratedImuMeasurementsT<gtsam::TangentPreintegration>&
          expected,
      double tol) const;

  // Standard Interface
  void integrateMeasurement(
      const gtsam::Vector3& measuredAcc,
      const gtsam::Vector3& measuredOmega, double deltaT);
  void resetIntegration();
  void resetIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& biasHat);

  gtsam::Matrix preintMeasCov() const;
  const gtsam::Vector9& preintegrated() const;
  double deltaTij() const;
  gtsam::Rot3 deltaRij() const;
  gtsam::Vector3 deltaPij() const;
  gtsam::Vector3 deltaVij() const;
  gtsam::Vector so3TangentAt(double t) const;
  gtsam::Matrix deskewPoints(gtsam::ConstMatrixView points,
      const gtsam::Vector3& velocity_i = gtsam::Vector3::Zero()) const;
  gtsam::Matrix deskewPointsAtTimes(gtsam::ConstMatrixView points,
      const gtsam::Vector& times,
      const gtsam::Vector3& velocity_i = gtsam::Vector3::Zero()) const;
  const gtsam::imuBias::ConstantBias& biasHat() const;
  gtsam::Vector6 biasHatVector() const;
  gtsam::NavState predict(const gtsam::NavState& state_i,
      const gtsam::imuBias::ConstantBias& bias,
      gtsam::OptionalJacobian<9, 9> H1 = nullptr,
      gtsam::OptionalJacobian<9, 6> H2 = nullptr) const;

  // enabling serialization functionality
  void serialize() const;
};

virtual class ImuFactor: gtsam::NonlinearFactor {
  ImuFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key pose_j, gtsam::Key vel_j,
      gtsam::Key bias,
      const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements);

  // Standard Interface
  const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i,
      const gtsam::Vector3& vel_i,
      const gtsam::Pose3& pose_j, const gtsam::Vector3& vel_j,
      const gtsam::imuBias::ConstantBias& bias) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class ImuFactor2: gtsam::NonlinearFactor {
  ImuFactor2();
  ImuFactor2(gtsam::Key state_i, gtsam::Key state_j,
      gtsam::Key bias,
      const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements);

  // Standard Interface
  const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::NavState& state_i,
                              const gtsam::NavState& state_j,
                              const gtsam::imuBias::ConstantBias& bias_i,
                              gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr,
                              gtsam::OptionalMatrixType H3 = nullptr) const;

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/ImuFactorWithGravity.h>
template <PIM, GRAVITY>
virtual class ImuFactorWithGravityT : gtsam::NonlinearFactor {
  ImuFactorWithGravityT(gtsam::Key pose_i, gtsam::Key vel_i,
      gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key bias, gtsam::Key gravity,
      const PIM& preintegratedMeasurements);
  ImuFactorWithGravityT(gtsam::Key pose_i, gtsam::Key vel_i,
      gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key bias, gtsam::Key gravity,
      const PIM& preintegratedMeasurements, double gravityMagnitude);

  // Standard Interface
  PIM preintegratedMeasurements() const;
  double gravityMagnitude() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, gtsam::Vector vel_i,
      const gtsam::Pose3& pose_j, gtsam::Vector vel_j,
      const gtsam::imuBias::ConstantBias& bias_i, const GRAVITY& gravity);

  // enable serialization functionality
  void serialize() const;
};

// Gravity direction as an optimized Unit3 with fixed magnitude:
typedef gtsam::ImuFactorWithGravityT<gtsam::PreintegratedImuMeasurements,
                                     gtsam::Unit3>
    ImuFactorWithGravityDirection;
// Gravity as a free Point3 vector, direction and magnitude both optimized:
typedef gtsam::ImuFactorWithGravityT<gtsam::PreintegratedImuMeasurements,
                                     gtsam::Point3>
    ImuFactorWithGravityVector;

#include <gtsam/navigation/CombinedImuFactor.h>
virtual class PreintegrationCombinedParams : gtsam::PreintegrationParams {
  PreintegrationCombinedParams(const gtsam::Vector3& n_gravity);

  static gtsam::PreintegrationCombinedParams* MakeSharedD(double g = 9.81);
  static gtsam::PreintegrationCombinedParams* MakeSharedU(double g = 9.81);

  void setBiasAccCovariance(const gtsam::Matrix3& cov);
  void setBiasOmegaCovariance(const gtsam::Matrix3& cov);
  
  const gtsam::Matrix3& getBiasAccCovariance() const;
  const gtsam::Matrix3& getBiasOmegaCovariance() const;

  // enabling serialization functionality
  void serialize() const;
};

class PreintegratedCombinedMeasurements {
  // Constructors
  PreintegratedCombinedMeasurements(
      const gtsam::PreintegrationCombinedParams* params);
  PreintegratedCombinedMeasurements(
      const gtsam::PreintegrationCombinedParams* params,
      const gtsam::imuBias::ConstantBias& bias);
  // Testable
  void print(string s = "Preintegrated Measurements:") const;
  bool equals(
      const gtsam::PreintegratedCombinedMeasurementsT<
          gtsam::TangentPreintegration>& expected,
      double tol) const;

  // Standard Interface
  void integrateMeasurement(
      const gtsam::Vector3& measuredAcc,
      const gtsam::Vector3& measuredOmega, double deltaT);
  void resetIntegration();
  void resetIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& biasHat);

  gtsam::Matrix preintMeasCov() const;
  double deltaTij() const;
  gtsam::Rot3 deltaRij() const;
  gtsam::Vector3 deltaPij() const;
  gtsam::Vector3 deltaVij() const;
  gtsam::Vector so3TangentAt(double t) const;
  gtsam::Matrix deskewPoints(gtsam::ConstMatrixView points,
      const gtsam::Vector3& velocity_i = gtsam::Vector3::Zero()) const;
  gtsam::Matrix deskewPointsAtTimes(gtsam::ConstMatrixView points,
      const gtsam::Vector& times,
      const gtsam::Vector3& velocity_i = gtsam::Vector3::Zero()) const;
  const gtsam::imuBias::ConstantBias& biasHat() const;
  gtsam::Vector6 biasHatVector() const;
  gtsam::NavState predict(const gtsam::NavState& state_i,
      const gtsam::imuBias::ConstantBias& bias,
      gtsam::OptionalJacobian<9, 9> H1 = nullptr,
      gtsam::OptionalJacobian<9, 6> H2 = nullptr) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class CombinedImuFactor: gtsam::NoiseModelFactor {
  CombinedImuFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key pose_j, gtsam::Key vel_j,
      gtsam::Key bias_i, gtsam::Key bias_j,
      const gtsam::PreintegratedCombinedMeasurements& CombinedPreintegratedMeasurements);

  // Standard Interface
  const gtsam::PreintegratedCombinedMeasurements&
  preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i,
      const gtsam::Vector3& vel_i,
      const gtsam::Pose3& pose_j, const gtsam::Vector3& vel_j,
      const gtsam::imuBias::ConstantBias& bias_i,
      const gtsam::imuBias::ConstantBias& bias_j,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr,
      gtsam::OptionalMatrixType H5 = nullptr,
      gtsam::OptionalMatrixType H6 = nullptr) const;

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/CombinedImuFactorWithGravity.h>
template <PIM, GRAVITY>
virtual class CombinedImuFactorWithGravityT : gtsam::NoiseModelFactor {
  CombinedImuFactorWithGravityT(gtsam::Key pose_i, gtsam::Key vel_i,
      gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key bias_i, gtsam::Key bias_j,
      gtsam::Key gravity, const PIM& preintegratedMeasurements);
  CombinedImuFactorWithGravityT(gtsam::Key pose_i, gtsam::Key vel_i,
      gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key bias_i, gtsam::Key bias_j,
      gtsam::Key gravity, const PIM& preintegratedMeasurements,
      double gravityMagnitude);

  // Standard Interface
  PIM preintegratedMeasurements() const;
  double gravityMagnitude() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, gtsam::Vector vel_i,
      const gtsam::Pose3& pose_j, gtsam::Vector vel_j,
      const gtsam::imuBias::ConstantBias& bias_i,
      const gtsam::imuBias::ConstantBias& bias_j, const GRAVITY& gravity);

  // enable serialization functionality
  void serialize() const;
};

// Gravity direction as an optimized Unit3 with fixed magnitude:
typedef gtsam::CombinedImuFactorWithGravityT<
    gtsam::PreintegratedCombinedMeasurements, gtsam::Unit3>
    CombinedImuFactorWithGravityDirection;
// Gravity as a free Point3 vector, direction and magnitude both optimized:
typedef gtsam::CombinedImuFactorWithGravityT<
    gtsam::PreintegratedCombinedMeasurements, gtsam::Point3>
    CombinedImuFactorWithGravityVector;

#include <gtsam/navigation/AHRSFactor.h>
class PreintegratedAhrsMeasurements {
  // Standard Constructor
  PreintegratedAhrsMeasurements(
      const gtsam::PreintegratedRotationParams* params,
      const gtsam::Vector3& biasHat = gtsam::Vector3::Zero());
  PreintegratedAhrsMeasurements(
      const gtsam::PreintegratedRotationParams* p,
      const gtsam::Vector3& bias_hat, double deltaTij,
                                const gtsam::Rot3& deltaRij,
                                const gtsam::Matrix3& delRdelBiasOmega,
                                const gtsam::Matrix3& preint_meas_cov);
  PreintegratedAhrsMeasurements(const gtsam::PreintegratedAhrsMeasurements& rhs);

  // Testable
  void print(string s = "Preintegrated Measurements: ") const;
  bool equals(const gtsam::PreintegratedAhrsMeasurements& expected,
              double tol) const;

  // get Data
  const gtsam::Rot3& deltaRij() const;
  const double& deltaTij() const;
  const gtsam::Vector3& biasHat() const;
  const gtsam::Matrix3& preintMeasCov() const;

  // Standard Interface
  void integrateMeasurement(const gtsam::Vector3& measuredOmega,
                            double deltaT);
  void resetIntegration() ;

  // enable serialization functionality
  void serialize() const;
};

virtual class AHRSFactor : gtsam::NonlinearFactor {
  AHRSFactor(gtsam::Key rot_i, gtsam::Key rot_j, gtsam::Key bias,
    const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements);

  // Standard Interface
  const gtsam::PreintegratedAhrsMeasurements&
  preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Rot3& rot_i, const gtsam::Rot3& rot_j,
      const gtsam::Vector3& bias) const;
  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/AttitudeFactor.h>
template<VALUE = {gtsam::Rot3, gtsam::Pose3, gtsam::NavState, gtsam::Gal3,
                  gtsam::Se23, gtsam::ExtendedPose3d}>
virtual class AttitudeFactor : gtsam::NoiseModelFactor {
  AttitudeFactor(gtsam::Key key, const gtsam::Unit3& nRef,
                 const gtsam::noiseModel::Diagonal* model,
                 const gtsam::Unit3& bMeasured);
  AttitudeFactor(gtsam::Key key, const gtsam::Unit3& nRef,
                 const gtsam::noiseModel::Diagonal* model);
  AttitudeFactor();
  const gtsam::Unit3& nRef() const;
  const gtsam::Unit3& bMeasured() const;
  gtsam::Vector evaluateError(const VALUE& value) const;

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/GPSFactor.h>
virtual class GPSFactor : gtsam::NonlinearFactor{
  GPSFactor(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  const gtsam::Point3& measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& nTb) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactorArm : gtsam::NonlinearFactor{
  GPSFactorArm(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::Point3& leverArm, 
            const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  const gtsam::Point3& measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& nTb) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactorArmCalib : gtsam::NonlinearFactor{
  GPSFactorArmCalib(gtsam::Key key1, gtsam::Key key2, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  const gtsam::Point3& measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& nTb,
                              const gtsam::Point3& leverArm) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactor2 : gtsam::NonlinearFactor {
  GPSFactor2(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  const gtsam::Point3& measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::NavState& nTb) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactor2Arm : gtsam::NonlinearFactor{
  GPSFactor2Arm(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::Point3& leverArm, 
            const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  const gtsam::Point3& measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::NavState& nTb) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactor2ArmCalib : gtsam::NonlinearFactor{
  GPSFactor2ArmCalib(gtsam::Key key1, gtsam::Key key2, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  const gtsam::Point3& measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::NavState& nTb,
                              const gtsam::Point3& leverArm) const;

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/PseudorangeFactor.h>
virtual class PseudorangeFactor : gtsam::NonlinearFactor {
  PseudorangeFactor(gtsam::Key receiverPositionKey,
                    gtsam::Key receiverClockBiasKey, double measuredPseudorange,
                    const gtsam::Point3& satellitePosition,
                    double satelliteClockBias,
                    const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  gtsam::Vector evaluateError(const gtsam::Point3& receiverPosition,
                              const double& receiverClockBias) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class UndifferencedPseudorangeFactor : gtsam::NonlinearFactor {
  UndifferencedPseudorangeFactor(gtsam::Key receiverPositionKey,
                              gtsam::Key receiverClockBiasKey,
                              gtsam::Key tropoZenithWetKey,
                              gtsam::Key slantIonoKey,
                              double measuredPseudorange,
                              const gtsam::Point3& satellitePosition,
                              double tropoWetMapping, double ionoCoefficient,
                              double satelliteClockBias,
                              const gtsam::noiseModel::Base* model);


  gtsam::Vector evaluateError(const gtsam::Point3& receiverPosition,
                              const double& receiverClockBias,
                              const double& tropoZenithWet,
                              const double& slantIono) const;
  double tropoMapping() const;
  double ionoCoefficient() const;

  void serialize() const;
};

virtual class UndifferencedPseudorangeFactorArm : gtsam::NonlinearFactor {
  UndifferencedPseudorangeFactorArm(gtsam::Key poseKey,
                                 gtsam::Key receiverClockBiasKey,
                                 gtsam::Key tropoZenithWetKey,
                                 gtsam::Key slantIonoKey,
                                 double measuredPseudorange,
                                 const gtsam::Point3& satellitePosition,
                                 const gtsam::Point3& leverArm,
                                 double tropoWetMapping, double ionoCoefficient,
                                 double satelliteClockBias,
                                 const gtsam::noiseModel::Base* model);
  UndifferencedPseudorangeFactorArm(gtsam::Key poseKey,
                                 gtsam::Key receiverClockBiasKey,
                                 gtsam::Key tropoZenithWetKey,
                                 gtsam::Key slantIonoKey,
                                 double measuredPseudorange,
                                 const gtsam::Point3& satellitePosition,
                                 const gtsam::Point3& leverArm,
                                 const gtsam::Pose3& ecef_T_nav,
                                 double tropoWetMapping, double ionoCoefficient,
                                 double satelliteClockBias,
                                 const gtsam::noiseModel::Base* model);


  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const double& receiverClockBias,
                              const double& tropoZenithWet,
                              const double& slantIono) const;
  const gtsam::Point3& leverArm() const;
  double tropoMapping() const;
  double ionoCoefficient() const;

  void serialize() const;
};

virtual class DifferentialPseudorangeFactor : gtsam::NonlinearFactor {
  DifferentialPseudorangeFactor(gtsam::Key receiverPositionKey,
                                gtsam::Key receiverClockBiasKey,
                                gtsam::Key differentialCorrectionKey,
                                double measuredPseudorange,
                                const gtsam::Point3& satellitePosition,
                                double satelliteClockBias,
                                const gtsam::noiseModel::Base* model);


  gtsam::Vector evaluateError(const gtsam::Point3& receiverPosition,
                              const double& receiverClockBias,
                              const double& differentialCorrection) const;

  void serialize() const;
};

virtual class PseudorangeFactorArm : gtsam::NonlinearFactor {
  PseudorangeFactorArm(gtsam::Key poseKey,
                        gtsam::Key receiverClockBiasKey,
                        double measuredPseudorange,
                        const gtsam::Point3& satellitePosition,
                        const gtsam::Point3& leverArm,
                        double satelliteClockBias,
                        const gtsam::noiseModel::Base* model);
  PseudorangeFactorArm(gtsam::Key poseKey,
                        gtsam::Key receiverClockBiasKey,
                        double measuredPseudorange,
                        const gtsam::Point3& satellitePosition,
                        const gtsam::Point3& leverArm,
                        const gtsam::Pose3& ecef_T_nav,
                        double satelliteClockBias,
                        const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const double& receiverClockBias) const;
  const gtsam::Point3& leverArm() const;

  // enable serialization functionality
  void serialize() const;
};

virtual class DifferentialPseudorangeFactorArm : gtsam::NonlinearFactor {
  DifferentialPseudorangeFactorArm(gtsam::Key poseKey,
                        gtsam::Key receiverClockBiasKey,
                        gtsam::Key differentialCorrectionKey,
                        double measuredPseudorange,
                        const gtsam::Point3& satellitePosition,
                        const gtsam::Point3& leverArm,
                        double satelliteClockBias,
                        const gtsam::noiseModel::Base* model);
  DifferentialPseudorangeFactorArm(gtsam::Key poseKey,
                        gtsam::Key receiverClockBiasKey,
                        gtsam::Key differentialCorrectionKey,
                        double measuredPseudorange,
                        const gtsam::Point3& satellitePosition,
                        const gtsam::Point3& leverArm,
                        const gtsam::Pose3& ecef_T_nav,
                        double satelliteClockBias,
                        const gtsam::noiseModel::Base* model);


  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const double& receiverClockBias,
                              const double& differentialCorrection) const;
  const gtsam::Point3& leverArm() const;

  void serialize() const;
};

virtual class DoubleDifferencePseudorangeFactor : gtsam::NonlinearFactor {
  DoubleDifferencePseudorangeFactor(gtsam::Key positionKey,
                      double prRovRef, double prBaseRef,
                      double prRovTarget, double prBaseTarget,
                      const gtsam::Point3& satRefRov, const gtsam::Point3& satTargetRov,
                      const gtsam::Point3& satRefBase, const gtsam::Point3& satTargetBase,
                      const gtsam::Point3& basePos,
                      const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Point3& pos) const;
  void serialize() const;
};

virtual class DoubleDifferencePseudorangeFactorArm : gtsam::NonlinearFactor {
  DoubleDifferencePseudorangeFactorArm(gtsam::Key poseKey,
                         double prRovRef, double prBaseRef,
                         double prRovTarget, double prBaseTarget,
                         const gtsam::Point3& satRefRov, const gtsam::Point3& satTargetRov,
                         const gtsam::Point3& satRefBase, const gtsam::Point3& satTargetBase,
                         const gtsam::Point3& basePos, const gtsam::Point3& leverArm,
                         const gtsam::noiseModel::Base* model);
  DoubleDifferencePseudorangeFactorArm(gtsam::Key poseKey,
                         double prRovRef, double prBaseRef,
                         double prRovTarget, double prBaseTarget,
                         const gtsam::Point3& satRefRov, const gtsam::Point3& satTargetRov,
                         const gtsam::Point3& satRefBase, const gtsam::Point3& satTargetBase,
                         const gtsam::Point3& basePos, const gtsam::Point3& leverArm,
                         const gtsam::Pose3& ecef_T_nav,
                         const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Pose3& pose) const;
  const gtsam::Point3& leverArm() const;
  void serialize() const;
};

#include <gtsam/navigation/CarrierPhaseFactor.h>
virtual class CarrierPhaseFactor : gtsam::NonlinearFactor {
  CarrierPhaseFactor(gtsam::Key receiverPositionKey,
                      gtsam::Key receiverClockBiasKey,
                      gtsam::Key ambiguityKey,
                      double measuredCarrierPhaseMeters,
                      const gtsam::Point3& satellitePosition,
                      double satelliteClockBias,
                      const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  gtsam::Vector evaluateError(const gtsam::Point3& receiverPosition,
                              const double& receiverClockBias,
                              const double& ambiguity) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class UndifferencedCarrierPhaseFactor : gtsam::NonlinearFactor {
  UndifferencedCarrierPhaseFactor(gtsam::Key receiverPositionKey,
                               gtsam::Key receiverClockBiasKey,
                               gtsam::Key tropoZenithWetKey,
                               gtsam::Key slantIonoKey, gtsam::Key ambiguityKey,
                               double measuredCarrierPhaseMeters,
                               const gtsam::Point3& satellitePosition,
                               double tropoWetMapping, double ionoCoefficient,
                               double lambda, double satelliteClockBias,
                               const gtsam::noiseModel::Base* model);


  gtsam::Vector evaluateError(const gtsam::Point3& receiverPosition,
                              const double& receiverClockBias,
                              const double& tropoZenithWet,
                              const double& slantIono,
                              const double& ambiguity) const;
  double tropoMapping() const;
  double ionoCoefficient() const;
  double wavelength() const;

  void serialize() const;
};

virtual class UndifferencedCarrierPhaseFactorArm : gtsam::NonlinearFactor {
  UndifferencedCarrierPhaseFactorArm(gtsam::Key poseKey,
                                  gtsam::Key receiverClockBiasKey,
                                  gtsam::Key tropoZenithWetKey,
                                  gtsam::Key slantIonoKey,
                                  gtsam::Key ambiguityKey,
                                  double measuredCarrierPhaseMeters,
                                  const gtsam::Point3& satellitePosition,
                                  const gtsam::Point3& leverArm,
                                  double tropoWetMapping, double ionoCoefficient,
                                  double lambda, double satelliteClockBias,
                                  const gtsam::noiseModel::Base* model);
  UndifferencedCarrierPhaseFactorArm(gtsam::Key poseKey,
                                  gtsam::Key receiverClockBiasKey,
                                  gtsam::Key tropoZenithWetKey,
                                  gtsam::Key slantIonoKey,
                                  gtsam::Key ambiguityKey,
                                  double measuredCarrierPhaseMeters,
                                  const gtsam::Point3& satellitePosition,
                                  const gtsam::Point3& leverArm,
                                  const gtsam::Pose3& ecef_T_nav,
                                  double tropoWetMapping, double ionoCoefficient,
                                  double lambda, double satelliteClockBias,
                                  const gtsam::noiseModel::Base* model);


  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const double& receiverClockBias,
                              const double& tropoZenithWet,
                              const double& slantIono,
                              const double& ambiguity) const;
  const gtsam::Point3& leverArm() const;
  double tropoMapping() const;
  double ionoCoefficient() const;
  double wavelength() const;

  void serialize() const;
};

virtual class CarrierPhaseFactorArm : gtsam::NonlinearFactor {
  CarrierPhaseFactorArm(gtsam::Key poseKey,
                         gtsam::Key receiverClockBiasKey,
                         gtsam::Key ambiguityKey,
                         double measuredCarrierPhaseMeters,
                         const gtsam::Point3& satellitePosition,
                         const gtsam::Point3& leverArm,
                         double satelliteClockBias,
                         const gtsam::noiseModel::Base* model);
  CarrierPhaseFactorArm(gtsam::Key poseKey,
                         gtsam::Key receiverClockBiasKey,
                         gtsam::Key ambiguityKey,
                         double measuredCarrierPhaseMeters,
                         const gtsam::Point3& satellitePosition,
                         const gtsam::Point3& leverArm,
                         const gtsam::Pose3& ecef_T_nav,
                         double satelliteClockBias,
                         const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const double& receiverClockBias,
                              const double& ambiguity) const;
  const gtsam::Point3& leverArm() const;

  // enable serialization functionality
  void serialize() const;
};


virtual class DoubleDifferenceCarrierPhaseFactor : gtsam::NonlinearFactor {
  DoubleDifferenceCarrierPhaseFactor(gtsam::Key positionKey, gtsam::Key ambRefKey,
                       gtsam::Key ambTargetKey,
                       double cpRovRefMeters, double cpBaseRefMeters,
                       double cpRovTargetMeters, double cpBaseTargetMeters,
                       const gtsam::Point3& satRefRov, const gtsam::Point3& satTargetRov,
                       const gtsam::Point3& satRefBase, const gtsam::Point3& satTargetBase,
                       const gtsam::Point3& basePos, double lam,
                       const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Point3& pos, const double& ambRef, const double& ambTarget) const;
  void serialize() const;
};

virtual class DoubleDifferenceCarrierPhaseFactorArm : gtsam::NonlinearFactor {
  DoubleDifferenceCarrierPhaseFactorArm(gtsam::Key poseKey, gtsam::Key ambRefKey, gtsam::Key ambTargetKey,
                          double cpRovRefMeters, double cpBaseRefMeters,
                          double cpRovTargetMeters, double cpBaseTargetMeters,
                          const gtsam::Point3& satRefRov, const gtsam::Point3& satTargetRov,
                          const gtsam::Point3& satRefBase, const gtsam::Point3& satTargetBase,
                          const gtsam::Point3& basePos, double lam,
                          const gtsam::Point3& leverArm,
                          const gtsam::noiseModel::Base* model);
  DoubleDifferenceCarrierPhaseFactorArm(gtsam::Key poseKey, gtsam::Key ambRefKey, gtsam::Key ambTargetKey,
                          double cpRovRefMeters, double cpBaseRefMeters,
                          double cpRovTargetMeters, double cpBaseTargetMeters,
                          const gtsam::Point3& satRefRov, const gtsam::Point3& satTargetRov,
                          const gtsam::Point3& satRefBase, const gtsam::Point3& satTargetBase,
                          const gtsam::Point3& basePos, double lam,
                          const gtsam::Point3& leverArm, const gtsam::Pose3& ecef_T_nav,
                          const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const double& ambRef, const double& ambTarget) const;
  const gtsam::Point3& leverArm() const;
  void serialize() const;
};

#include <gtsam/navigation/DopplerFactor.h>
virtual class DopplerFactor : gtsam::NonlinearFactor {
  DopplerFactor(gtsam::Key velocityKey, gtsam::Key clockBiasPrevKey,
                gtsam::Key clockBiasCurrKey, double measuredDoppler,
                double wavelength, const gtsam::Point3& satellitePosition,
                const gtsam::Point3& satelliteVelocity,
                const gtsam::Point3& receiverPosition, double dt,
                double satelliteClockDrift,
                const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const gtsam::Vector3& velocity,
                              const double& clockBiasPrev,
                              const double& clockBiasCurr) const;
  double measuredRangeRate() const;
  const gtsam::Point3& lineOfSight() const;
  double dt() const;
  void serialize() const;
};

virtual class DopplerFactorArm : gtsam::NonlinearFactor {
  DopplerFactorArm(gtsam::Key poseKey, gtsam::Key velocityKey,
                   gtsam::Key clockBiasPrevKey, gtsam::Key clockBiasCurrKey,
                   double measuredDoppler, double wavelength,
                   const gtsam::Point3& satellitePosition,
                   const gtsam::Point3& satelliteVelocity,
                   const gtsam::Point3& receiverPosition,
                   const gtsam::Point3& leverArm,
                   const gtsam::Point3& angularVelocity, double dt,
                   double satelliteClockDrift,
                   const gtsam::noiseModel::Base* model);
  DopplerFactorArm(gtsam::Key poseKey, gtsam::Key velocityKey,
                   gtsam::Key clockBiasPrevKey, gtsam::Key clockBiasCurrKey,
                   double measuredDoppler, double wavelength,
                   const gtsam::Point3& satellitePosition,
                   const gtsam::Point3& satelliteVelocity,
                   const gtsam::Point3& receiverPosition,
                   const gtsam::Point3& leverArm, const gtsam::Pose3& ecef_T_nav,
                   const gtsam::Point3& angularVelocity, double dt,
                   double satelliteClockDrift,
                   const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const gtsam::Vector3& velocity,
                              const double& clockBiasPrev,
                              const double& clockBiasCurr) const;
  double measuredRangeRate() const;
  const gtsam::Point3& lineOfSight() const;
  double dt() const;
  const gtsam::Point3& leverArm() const;
  void serialize() const;
};

#include <gtsam/navigation/BarometricFactor.h>
virtual class BarometricFactor : gtsam::NonlinearFactor {
  BarometricFactor();
  BarometricFactor(gtsam::Key key, gtsam::Key baroKey, const double& baroIn,
                   const gtsam::noiseModel::Base* model);

  // Testable

  // Standard Interface
  const double& measurementIn() const;
  double heightOut(double n) const;
  double baroOut(const double& meters) const;
  gtsam::Vector evaluateError(const gtsam::Pose3& p, const double& b) const;

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/ConstantVelocityFactor.h>
class ConstantVelocityFactor : gtsam::NonlinearFactor {
  ConstantVelocityFactor(gtsam::Key i, gtsam::Key j, double dt, const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::NavState &x1, const gtsam::NavState &x2) const;
};

#include <gtsam/navigation/MagFactor.h>

class MagFactor: gtsam::NonlinearFactor {
  MagFactor(gtsam::Key key, const gtsam::Point3& measured, double scale,
      const gtsam::Unit3& direction, const gtsam::Point3& bias,
      const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Rot2& nRb,
                              gtsam::OptionalMatrixType H = nullptr) const;
};

class MagFactor1: gtsam::NonlinearFactor {
  MagFactor1(gtsam::Key key, const gtsam::Point3& measured, double scale,
      const gtsam::Unit3& direction, const gtsam::Point3& bias,
      const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Rot3& nRb,
                              gtsam::OptionalMatrixType H = nullptr) const;
};

#include <gtsam/geometry/Pose2.h>
#include <gtsam/navigation/MagPoseFactor.h>
template <POSE = {gtsam::Pose2, gtsam::Pose3}>
virtual class MagPoseFactor : gtsam::NoiseModelFactor {
  MagPoseFactor(gtsam::Key pose_key,
    const POSE::Translation& measured, double scale,
    const POSE::Translation& direction, const POSE::Translation& bias,
    const gtsam::noiseModel::Base* noiseModel);
  MagPoseFactor(gtsam::Key pose_key,
      const POSE::Translation& measured, double scale,
      const POSE::Translation& direction, const POSE::Translation& bias,
      const gtsam::noiseModel::Base* noiseModel, const POSE& body_P_sensor);
    gtsam::Vector evaluateError(const POSE& nPb,
                                gtsam::OptionalMatrixType H = nullptr) const;
};

#include <gtsam/navigation/Scenario.h>
virtual class Scenario {
  gtsam::Pose3 pose(double t) const;
  gtsam::Vector3 omega_b(double t) const;
  gtsam::Vector3 velocity_n(double t) const;
  gtsam::Vector3 acceleration_n(double t) const;
  gtsam::Rot3 rotation(double t) const;
  gtsam::NavState navState(double t) const;
  gtsam::Gal3 gal3(double t) const;
  gtsam::Vector3 velocity_b(double t) const;
  gtsam::Vector3 acceleration_b(double t) const;
};

virtual class ConstantTwistScenario : gtsam::Scenario {
  ConstantTwistScenario(const gtsam::Vector3& w,
                        const gtsam::Vector3& v);
  ConstantTwistScenario(const gtsam::Vector3& w,
                        const gtsam::Vector3& v,
                        const gtsam::Pose3& nTb0);
};

virtual class AcceleratingScenario : gtsam::Scenario {
  AcceleratingScenario(const gtsam::Rot3& nRb, const gtsam::Point3& p0,
                       const gtsam::Vector3& v0,
                       const gtsam::Vector3& a_n,
                       const gtsam::Vector3& omega_b = gtsam::Vector3::Zero());
};

virtual class DiscreteScenario : gtsam::Scenario {
  DiscreteScenario(const std::map<double, gtsam::Pose3>& poses,
                   const std::map<double, gtsam::Vector3>& angularVelocities_b,
                   const std::map<double, gtsam::Vector3>& velocities_n,
                   const std::map<double, gtsam::Vector3>& accelerations_n);

  static gtsam::DiscreteScenario FromCSV(const std::string& csv_filepath);
};

#include <gtsam/navigation/ScenarioRunner.h>
class ScenarioRunner {
  ScenarioRunner(const gtsam::Scenario& scenario,
                 const gtsam::PreintegrationParams* p,
                 double imuSampleTime,
                 const gtsam::imuBias::ConstantBias& bias);
  const gtsam::Vector3& gravity_n() const;
  gtsam::Vector3 actualAngularVelocity(double t) const;
  gtsam::Vector3 actualSpecificForce(double t) const;
  gtsam::Vector3 measuredAngularVelocity(double t) const;
  gtsam::Vector3 measuredSpecificForce(double t) const;
  double imuSampleTime() const;
  gtsam::PreintegratedImuMeasurements integrate(
      double T,
      const gtsam::imuBias::ConstantBias& estimatedBias =
          gtsam::imuBias::ConstantBias(),
      bool corrupted = false) const;
  gtsam::NavState predict(
      const gtsam::PreintegratedImuMeasurements& pim,
      const gtsam::imuBias::ConstantBias& estimatedBias =
          gtsam::imuBias::ConstantBias()) const;
  gtsam::Matrix9 estimateCovariance(
      double T, size_t N = 1000,
      const gtsam::imuBias::ConstantBias& estimatedBias =
          gtsam::imuBias::ConstantBias()) const;
  gtsam::Matrix6 estimateNoiseCovariance(size_t N = 1000) const;
};

// ---------------------------------------------------------------------------
// EKF classes
#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/ManifoldEKF.h>
template <M = {gtsam::Unit3, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
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
                        const gtsam::Vector& z, const gtsam::Matrix& R, bool performReset = true);
};

#include <gtsam/navigation/LieGroupEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
virtual class LieGroupEKF : gtsam::ManifoldEKF<G> {
  // Constructors
  LieGroupEKF(const G& X0, const gtsam::This::Covariance& P0);
  
  // Increment-based predict (precomputed increment and Jacobian)
  void predictWithCompose(const G& U,
                          const gtsam::This::Jacobian& J_UX,
                          const gtsam::This::Covariance& Q);
};

#include <gtsam/navigation/LeftLinearEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
virtual class LeftLinearEKF : gtsam::LieGroupEKF<G> {
  // Constructors
  LeftLinearEKF(const G& X0, const gtsam::This::Covariance& P0);
};

#include <gtsam/navigation/InvariantEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
virtual class InvariantEKF : gtsam::LeftLinearEKF<G> {
  // Constructors
  InvariantEKF(const G& X0, const gtsam::This::Covariance& P0);

  // Left-invariant predict APIs
  void predict(const G& U, const gtsam::This::Covariance& Q);
  void predict(const G& W, const G& U,
               const gtsam::This::Covariance& Q);
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
  void predict(const gtsam::Vector3& omega, const gtsam::Matrix6& inputCovariance, double dt);
  void update(const gtsam::Unit3& y, const gtsam::Unit3& d, const gtsam::Matrix3& R, int cal_idx);

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
  static gtsam::NavState Dynamics(
      const gtsam::Vector3& n_gravity, const gtsam::NavState& X,
      const gtsam::Vector3& omega_b, const gtsam::Vector3& f_b, double dt,
      gtsam::OptionalJacobian<9, 9> A = nullptr);
  
  // Predict using IMU measurements
  void predict(const gtsam::Vector3& omega_b,
               const gtsam::Vector3& f_b, double dt);
};

#include <gtsam/navigation/Gal3ImuEKF.h>
class Gal3ImuEKF : gtsam::InvariantEKF<gtsam::Gal3> {
  enum Mode { NO_TIME, TRACK_TIME_NO_COVARIANCE, TRACK_TIME_WITH_COVARIANCE };
  // Constructors
  Gal3ImuEKF(const gtsam::Gal3& X0,
             const gtsam::Gal3ImuEKF::Covariance& P0,
             const gtsam::PreintegrationParams* params); // mode = TRACK_TIME_NO_COVARIANCE
  Gal3ImuEKF(const gtsam::Gal3& X0,
             const gtsam::Gal3ImuEKF::Covariance& P0,
             const gtsam::PreintegrationParams* params,
             gtsam::Gal3ImuEKF::Mode mode);

  // Accessors
  const gtsam::Gal3ImuEKF::Covariance& processNoise() const;
  const gtsam::Vector3& gravity() const;
  const std::shared_ptr<gtsam::PreintegrationParams>& params() const;

  // Static methods
  static gtsam::Gal3 Gravity(const gtsam::Vector3& g_n, double dt);
  static gtsam::Gal3 TimeZeroingGravity(const gtsam::Vector3& g_n, double dt);
  static gtsam::Gal3 CompensatedGravity(
      const gtsam::Vector3& g_n, double dt, double t_k);
  static gtsam::Gal3 Imu(const gtsam::Vector3& omega_b,
                         const gtsam::Vector3& f_b, double dt);
  static gtsam::Gal3 Dynamics(const gtsam::Vector3& n_gravity,
                              const gtsam::Gal3& X,
                              const gtsam::Vector3& omega_b,
                              const gtsam::Vector3& f_b, double dt,
                              gtsam::Gal3ImuEKF::Mode mode,
                              gtsam::OptionalJacobian<10, 10> A = nullptr);

  // Predict using IMU measurements
  void predict(const gtsam::Vector3& omega_b,
               const gtsam::Vector3& f_b, double dt);
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
                     const gtsam::Matrix& footholds0,
                     const gtsam::Matrix& P0,
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
                      const gtsam::Matrix& footholds0,
                      const gtsam::Matrix& P0,
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
  LeggedCombinedFixedLagSmoother(
      const gtsam::NavState& navState0,
      const gtsam::Matrix& footholds0,
      const gtsam::Matrix9& baseCovariance0,
      const gtsam::LeggedEstimatorParams& params, double lagSeconds,
      const std::vector<std::string>& footNames);
  size_t numFeet() const;
};
}
