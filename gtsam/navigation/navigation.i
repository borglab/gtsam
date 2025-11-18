//*************************************************************************
// Navigation
//*************************************************************************

namespace gtsam {

namespace imuBias {
#include <gtsam/navigation/ImuBias.h>

class ConstantBias {
  // Constructors
  ConstantBias();
  ConstantBias(gtsam::Vector biasAcc, gtsam::Vector biasGyro);

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
  gtsam::Vector vector() const;
  gtsam::Vector accelerometer() const;
  gtsam::Vector gyroscope() const;
  gtsam::Vector correctAccelerometer(gtsam::Vector measurement) const;
  gtsam::Vector correctGyroscope(gtsam::Vector measurement) const;

  // Manifold
  gtsam::imuBias::ConstantBias retract(const gtsam::Vector& v) const;
  gtsam::Vector localCoordinates(const gtsam::imuBias::ConstantBias& b) const;

  // enabling serialization functionality
  void serialize() const;
};

}///\namespace imuBias

#include <gtsam/navigation/NavState.h>
class NavState {
  // Constructors
  NavState();
  NavState(const gtsam::Rot3& R, const gtsam::Point3& t, gtsam::Vector v);
  NavState(const gtsam::Pose3& pose, gtsam::Vector v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::NavState& other, double tol) const;

  // Access
  gtsam::Rot3 attitude() const;
  gtsam::Point3 position() const;
  gtsam::Vector velocity() const;
  gtsam::Pose3 pose() const;

  // Manifold
  gtsam::NavState retract(const gtsam::Vector& v) const;
  gtsam::Vector localCoordinates(const gtsam::NavState& g) const;

  // Lie Group
  static gtsam::NavState Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::NavState& p);
  static gtsam::NavState Expmap(gtsam::Vector xi, Eigen::Ref<Eigen::MatrixXd> Hxi);
  static gtsam::Vector Logmap(const gtsam::NavState& pose, Eigen::Ref<Eigen::MatrixXd> Hpose);
  gtsam::NavState expmap(gtsam::Vector v);
  gtsam::NavState expmap(gtsam::Vector v, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2);
  gtsam::Vector logmap(const gtsam::NavState& p);
  gtsam::Vector logmap(const gtsam::NavState& p, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2);
  gtsam::Matrix AdjointMap() const;
  gtsam::Vector Adjoint(gtsam::Vector xi_b) const;
  static gtsam::Matrix Hat(const gtsam::Vector& xi);
  static gtsam::Vector Vee(const gtsam::Matrix& X);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/PreintegratedRotation.h>
virtual class PreintegratedRotationParams {
  PreintegratedRotationParams();

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegratedRotationParams& other, double tol);

  void setGyroscopeCovariance(gtsam::Matrix cov);
  void setOmegaCoriolis(gtsam::Vector omega);
  void setBodyPSensor(const gtsam::Pose3& pose);

  gtsam::Matrix getGyroscopeCovariance() const;

  std::optional<gtsam::Vector> getOmegaCoriolis() const;
  std::optional<gtsam::Pose3> getBodyPSensor() const;

  // enabling serialization functionality
  void serialize() const;
};

class PreintegratedRotation {
  // Constructors
  PreintegratedRotation(const gtsam::PreintegratedRotationParams* params);

  // Standard Interface
  void resetIntegration();
  void integrateGyroMeasurement(const gtsam::Vector&  measuredOmega, const gtsam::Vector& biasHat, double deltaT);
  gtsam::Rot3 biascorrectedDeltaRij(const gtsam::Vector& biasOmegaIncr) const;
  gtsam::Vector integrateCoriolis(const gtsam::Rot3& rot_i) const;

  // Access instance variables
  double deltaTij() const;
  gtsam::Rot3 deltaRij() const;
  gtsam::Matrix delRdelBiasOmega() const;

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegratedRotation& other, double tol) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/PreintegrationParams.h>
virtual class PreintegrationParams : gtsam::PreintegratedRotationParams {
  PreintegrationParams(gtsam::Vector n_gravity);

  gtsam::Vector n_gravity;

  static gtsam::PreintegrationParams* MakeSharedD(double g);
  static gtsam::PreintegrationParams* MakeSharedU(double g);
  static gtsam::PreintegrationParams* MakeSharedD();  // default g = 9.81
  static gtsam::PreintegrationParams* MakeSharedU();  // default g = 9.81

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegrationParams& other, double tol);

  void setAccelerometerCovariance(gtsam::Matrix cov);
  void setIntegrationCovariance(gtsam::Matrix cov);
  void setUse2ndOrderCoriolis(bool flag);

  gtsam::Matrix getAccelerometerCovariance() const;
  gtsam::Matrix getIntegrationCovariance()   const;
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
  bool equals(const gtsam::PreintegratedImuMeasurements& expected, double tol);

  // Standard Interface
  void integrateMeasurement(gtsam::Vector measuredAcc, gtsam::Vector measuredOmega,
      double deltaT);
  void resetIntegration();
  void resetIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& biasHat);

  gtsam::Matrix preintMeasCov() const;
  gtsam::Vector preintegrated() const;
  double deltaTij() const;
  gtsam::Rot3 deltaRij() const;
  gtsam::Vector deltaPij() const;
  gtsam::Vector deltaVij() const;
  gtsam::imuBias::ConstantBias biasHat() const;
  gtsam::Vector biasHatVector() const;
  gtsam::NavState predict(const gtsam::NavState& state_i,
      const gtsam::imuBias::ConstantBias& bias) const;

  // enabling serialization functionality
  void serialize() const;
};

virtual class ImuFactor: gtsam::NonlinearFactor {
  ImuFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key pose_j, gtsam::Key vel_j,
      gtsam::Key bias,
      const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedImuMeasurements preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, gtsam::Vector vel_i,
      const gtsam::Pose3& pose_j, gtsam::Vector vel_j,
      const gtsam::imuBias::ConstantBias& bias);

  // enable serialization functionality
  void serialize() const;
};

virtual class ImuFactor2: gtsam::NonlinearFactor {
  ImuFactor2();
  ImuFactor2(gtsam::Key state_i, gtsam::Key state_j,
      gtsam::Key bias,
      const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedImuMeasurements preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::NavState& state_i,
                              gtsam::NavState& state_j,
                              const gtsam::imuBias::ConstantBias& bias_i);

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/CombinedImuFactor.h>
virtual class PreintegrationCombinedParams : gtsam::PreintegrationParams {
  PreintegrationCombinedParams(gtsam::Vector n_gravity);

  static gtsam::PreintegrationCombinedParams* MakeSharedD(double g);
  static gtsam::PreintegrationCombinedParams* MakeSharedU(double g);
  static gtsam::PreintegrationCombinedParams* MakeSharedD();  // default g = 9.81
  static gtsam::PreintegrationCombinedParams* MakeSharedU();  // default g = 9.81

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegrationCombinedParams& other, double tol);

  void setBiasAccCovariance(gtsam::Matrix cov);
  void setBiasOmegaCovariance(gtsam::Matrix cov);
  
  gtsam::Matrix getBiasAccCovariance() const ;
  gtsam::Matrix getBiasOmegaCovariance() const ;

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
  bool equals(const gtsam::PreintegratedCombinedMeasurements& expected,
              double tol);

  // Standard Interface
  void integrateMeasurement(gtsam::Vector measuredAcc,
                            gtsam::Vector measuredOmega, double deltaT);
  void resetIntegration();
  void resetIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& biasHat);

  gtsam::Matrix preintMeasCov() const;
  double deltaTij() const;
  gtsam::Rot3 deltaRij() const;
  gtsam::Vector deltaPij() const;
  gtsam::Vector deltaVij() const;
  gtsam::imuBias::ConstantBias biasHat() const;
  gtsam::Vector biasHatVector() const;
  gtsam::NavState predict(const gtsam::NavState& state_i,
                          const gtsam::imuBias::ConstantBias& bias) const;

  // enable serialization functionality
  void serialize() const;
};

virtual class CombinedImuFactor: gtsam::NoiseModelFactor {
  CombinedImuFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key pose_j, gtsam::Key vel_j,
      gtsam::Key bias_i, gtsam::Key bias_j,
      const gtsam::PreintegratedCombinedMeasurements& CombinedPreintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedCombinedMeasurements preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, gtsam::Vector vel_i,
      const gtsam::Pose3& pose_j, gtsam::Vector vel_j,
      const gtsam::imuBias::ConstantBias& bias_i,
      const gtsam::imuBias::ConstantBias& bias_j);

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/AHRSFactor.h>
class PreintegratedAhrsMeasurements {
  // Standard Constructor
  PreintegratedAhrsMeasurements(const gtsam::PreintegrationParams* params,
                                const gtsam::Vector& biasHat);
  PreintegratedAhrsMeasurements(const gtsam::PreintegrationParams* p,
                                const gtsam::Vector& bias_hat, double deltaTij,
                                const gtsam::Rot3& deltaRij,
                                const gtsam::Matrix& delRdelBiasOmega,
                                const gtsam::Matrix& preint_meas_cov);
  PreintegratedAhrsMeasurements(const gtsam::PreintegratedAhrsMeasurements& rhs);

  // Testable
  void print(string s = "Preintegrated Measurements: ") const;
  bool equals(const gtsam::PreintegratedAhrsMeasurements& expected, double tol);

  // get Data
  gtsam::Rot3 deltaRij() const;
  double deltaTij() const;
  gtsam::Vector biasHat() const;

  // Standard Interface
  void integrateMeasurement(gtsam::Vector measuredOmega, double deltaT);
  void resetIntegration() ;

  // enable serialization functionality
  void serialize() const;
};

virtual class AHRSFactor : gtsam::NonlinearFactor {
  AHRSFactor(gtsam::Key rot_i, gtsam::Key rot_j, gtsam::Key bias,
    const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedAhrsMeasurements preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Rot3& rot_i, const gtsam::Rot3& rot_j,
      gtsam::Vector bias) const;
  gtsam::Rot3 predict(const gtsam::Rot3& rot_i, gtsam::Vector bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements,
      gtsam::Vector omegaCoriolis) const;

  // enable serialization functionality
  void serialize() const;

  // deprecated:
  AHRSFactor(gtsam::Key rot_i, gtsam::Key rot_j, gtsam::Key bias,
    const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements, gtsam::Vector omegaCoriolis);
  AHRSFactor(gtsam::Key rot_i, gtsam::Key rot_j, gtsam::Key bias,
    const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements, gtsam::Vector omegaCoriolis,
    const gtsam::Pose3& body_P_sensor);
};

#include <gtsam/navigation/AttitudeFactor.h>
virtual class Rot3AttitudeFactor : gtsam::NoiseModelFactor {
  Rot3AttitudeFactor(gtsam::Key key, const gtsam::Unit3& nRef, const gtsam::noiseModel::Diagonal* model,
      const gtsam::Unit3& bMeasured);
  Rot3AttitudeFactor(gtsam::Key key, const gtsam::Unit3& nRef, const gtsam::noiseModel::Diagonal* model);
  Rot3AttitudeFactor();
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nRef() const;
  gtsam::Unit3 bMeasured() const;
  gtsam::Vector evaluateError(const gtsam::Rot3& nRb);

  // enable serialization functionality
  void serialize() const;
};

virtual class Pose3AttitudeFactor : gtsam::NoiseModelFactor {
  Pose3AttitudeFactor(gtsam::Key key, const gtsam::Unit3& nRef,
                      const gtsam::noiseModel::Diagonal* model,
                      const gtsam::Unit3& bMeasured);
  Pose3AttitudeFactor(gtsam::Key key, const gtsam::Unit3& nRef,
                      const gtsam::noiseModel::Diagonal* model);
  Pose3AttitudeFactor();
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nRef() const;
  gtsam::Unit3 bMeasured() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& nTb);

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/GPSFactor.h>
virtual class GPSFactor : gtsam::NonlinearFactor{
  GPSFactor(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& nTb);

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactorArm : gtsam::NonlinearFactor{
  GPSFactorArm(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::Point3& leverArm, 
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& nTb);

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactorArmCalib : gtsam::NonlinearFactor{
  GPSFactorArmCalib(gtsam::Key key1, gtsam::Key key2, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& nTb, const gtsam::Point3& leverArm);

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactor2 : gtsam::NonlinearFactor {
  GPSFactor2(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::NavState& nTb);

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactor2Arm : gtsam::NonlinearFactor{
  GPSFactor2Arm(gtsam::Key key, const gtsam::Point3& gpsIn,
            const gtsam::Point3& leverArm, 
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::NavState& nTb);

  // enable serialization functionality
  void serialize() const;
};

virtual class GPSFactor2ArmCalib : gtsam::NonlinearFactor{
  GPSFactor2ArmCalib(gtsam::Key key1, gtsam::Key key2, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
  gtsam::Vector evaluateError(const gtsam::NavState& nTb, const gtsam::Point3& leverArm);

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/BarometricFactor.h>
virtual class BarometricFactor : gtsam::NonlinearFactor {
  BarometricFactor();
  BarometricFactor(gtsam::Key key, gtsam::Key baroKey, const double& baroIn,
                   const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  const double& measurementIn() const;
  double heightOut(double n) const;
  double baroOut(const double& meters) const;
  gtsam::Vector evaluateError(const gtsam::Pose3& p, double b);

  // enable serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/ConstantVelocityFactor.h>
class ConstantVelocityFactor : gtsam::NonlinearFactor {
  ConstantVelocityFactor(size_t i, size_t j, double dt, const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::NavState &x1, const gtsam::NavState &x2) const;
};

#include <gtsam/navigation/MagFactor.h>

class MagFactor: gtsam::NonlinearFactor {
  MagFactor(gtsam::Key key, const gtsam::Point3& measured, double scale,
      const gtsam::Unit3& direction, const gtsam::Point3& bias,
      const gtsam::noiseModel::Base* model);
  Vector evaluateError(const gtsam::Rot2& nRb);
};

class MagFactor1: gtsam::NonlinearFactor {
  MagFactor1(gtsam::Key key, const gtsam::Point3& measured, double scale,
      const gtsam::Unit3& direction, const gtsam::Point3& bias,
      const gtsam::noiseModel::Base* model);
  Vector evaluateError(const gtsam::Rot3& nRb);
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
    Vector evaluateError(const POSE& nRb);
};

#include <gtsam/navigation/Scenario.h>
virtual class Scenario {
  gtsam::Pose3 pose(double t) const;
  gtsam::Vector omega_b(double t) const;
  gtsam::Vector velocity_n(double t) const;
  gtsam::Vector acceleration_n(double t) const;
  gtsam::Rot3 rotation(double t) const;
  gtsam::NavState navState(double t) const;
  gtsam::Vector velocity_b(double t) const;
  gtsam::Vector acceleration_b(double t) const;
};

virtual class ConstantTwistScenario : gtsam::Scenario {
  ConstantTwistScenario(gtsam::Vector w, gtsam::Vector v);
  ConstantTwistScenario(gtsam::Vector w, gtsam::Vector v,
                        const gtsam::Pose3& nTb0);
};

virtual class AcceleratingScenario : gtsam::Scenario {
  AcceleratingScenario(const gtsam::Rot3& nRb, const gtsam::Point3& p0,
                       gtsam::Vector v0, gtsam::Vector a_n,
                       gtsam::Vector omega_b);
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
  gtsam::Vector gravity_n() const;
  gtsam::Vector actualAngularVelocity(double t) const;
  gtsam::Vector actualSpecificForce(double t) const;
  gtsam::Vector measuredAngularVelocity(double t) const;
  gtsam::Vector measuredSpecificForce(double t) const;
  double imuSampleTime() const;
  gtsam::PreintegratedImuMeasurements integrate(
      double T, const gtsam::imuBias::ConstantBias& estimatedBias,
      bool corrupted) const;
  gtsam::NavState predict(
      const gtsam::PreintegratedImuMeasurements& pim,
      const gtsam::imuBias::ConstantBias& estimatedBias) const;
  gtsam::Matrix estimateCovariance(
      double T, size_t N,
      const gtsam::imuBias::ConstantBias& estimatedBias) const;
  gtsam::Matrix estimateNoiseCovariance(size_t N) const;
};

// ---------------------------------------------------------------------------
// EKF classes
#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/ManifoldEKF.h>
template <M = {gtsam::Unit3, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
virtual class ManifoldEKF {
  // Constructors
  ManifoldEKF(const M& X0, gtsam::Matrix P0);

  // Accessors
  M state() const;
  gtsam::Matrix covariance() const;
  int dimension() const;

  // Predict with provided next state and Jacobian
  void predict(const M& X_next, gtsam::Matrix F, gtsam::Matrix Q);

  // Only vector-based measurements are supported in wrapper
  void updateWithVector(const gtsam::Vector& prediction, const gtsam::Matrix& H,
                        const gtsam::Vector& z, const gtsam::Matrix& R);
};

#include <gtsam/navigation/LieGroupEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
virtual class LieGroupEKF : gtsam::ManifoldEKF<G> {
  // Constructors
  LieGroupEKF(const G& X0, gtsam::Matrix P0);
  
  // Increment-based predict (precomputed increment and Jacobian)
  void predictWithCompose(const G& U, gtsam::Matrix J_UX, gtsam::Matrix Q);
};

#include <gtsam/navigation/LeftLinearEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
virtual class LeftLinearEKF : gtsam::LieGroupEKF<G> {
  // Constructors
  LeftLinearEKF(const G& X0, gtsam::Matrix P0);
};

#include <gtsam/navigation/InvariantEKF.h>
template <G = {gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::NavState, gtsam::Gal3}>
virtual class InvariantEKF : gtsam::LeftLinearEKF<G> {
  // Constructors
  InvariantEKF(const G& X0, gtsam::Matrix P0);

  // Left-invariant predict APIs
  void predict(const G& U, gtsam::Matrix Q);
  void predict(const gtsam::Vector& u, double dt, gtsam::Matrix Q);
};

// Specialized NavState IMU EKF
#include <gtsam/navigation/NavStateImuEKF.h>
class NavStateImuEKF : gtsam::LeftLinearEKF<gtsam::NavState> {
  // Constructors
  NavStateImuEKF(const gtsam::NavState& X0, gtsam::Matrix P0,
                 const gtsam::PreintegrationParams* params);

  // Accessors
  gtsam::Matrix processNoise() const;
  gtsam::Vector gravity() const;
  const gtsam::PreintegrationParams* params() const;

  // Static methods
  static gtsam::NavState Gravity(const gtsam::Vector& n_gravity, double dt);
  static gtsam::NavState Imu(const gtsam::Vector& omega_b, const gtsam::Vector& f_b, double dt);
  static gtsam::NavState Dynamics(const gtsam::Vector& n_gravity, const gtsam::NavState& X,
                                   const gtsam::Vector& omega_b, const gtsam::Vector& f_b,
                                   double dt);
  
  // Predict using IMU measurements
  void predict(const gtsam::Vector& omega_b, const gtsam::Vector& f_b, double dt);
};

#include <gtsam/navigation/Gal3ImuEKF.h>
class Gal3ImuEKF : gtsam::InvariantEKF<gtsam::Gal3> {
  enum Mode { NO_TIME, TRACK_TIME_NO_COVARIANCE, TRACK_TIME_WITH_COVARIANCE };
  // Constructors
  Gal3ImuEKF(const gtsam::Gal3& X0, gtsam::Matrix P0,
             const gtsam::PreintegrationParams* params); // mode = TRACK_TIME_NO_COVARIANCE
  Gal3ImuEKF(const gtsam::Gal3& X0, gtsam::Matrix P0,
             const gtsam::PreintegrationParams* params,
             gtsam::Gal3ImuEKF::Mode mode);

  // Accessors
  gtsam::Matrix processNoise() const;
  gtsam::Vector gravity() const;
  const gtsam::PreintegrationParams* params() const;

  // Static methods
  static gtsam::Gal3 Gravity(const gtsam::Vector& g_n, double dt);
  static gtsam::Gal3 TimeZeroingGravity(const gtsam::Vector& g_n, double dt);
  static gtsam::Gal3 CompensatedGravity(const gtsam::Vector& g_n, double dt, double t_k);
  static gtsam::Gal3 Imu(const gtsam::Vector& omega_b, const gtsam::Vector& f_b, double dt);
  static gtsam::Gal3 Dynamics(const gtsam::Vector& n_gravity,
                              const gtsam::Gal3& X,
                              const gtsam::Vector& omega_b,
                              const gtsam::Vector& f_b, double dt); // mode = TRACK_TIME_NO_COVARIANCE
  static gtsam::Gal3 Dynamics(const gtsam::Vector& n_gravity,
                              const gtsam::Gal3& X,
                              const gtsam::Vector& omega_b,
                              const gtsam::Vector& f_b, double dt,
                              gtsam::Gal3ImuEKF::Mode mode);

  // Predict using IMU measurements
  void predict(const gtsam::Vector& omega_b, const gtsam::Vector& f_b, double dt);
};
}
