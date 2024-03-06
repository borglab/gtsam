//*************************************************************************
// Navigation
//*************************************************************************

namespace gtsam {

namespace imuBias {
#include <gtsam/navigation/ImuBias.h>

class ConstantBias {
  // Constructors
  ConstantBias();
  ConstantBias(const gtsam::Vector& biasAcc, const gtsam::Vector& biasGyro);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::imuBias::ConstantBias& expected, double tol) const;

  // Group
  static gtsam::imuBias::ConstantBias Identity();

  // Operator Overloads
  gtsam::imuBias::ConstantBias operator-() const;
  gtsam::imuBias::ConstantBias operator+(
      const gtsam::imuBias::ConstantBias& b) const;
  gtsam::imuBias::ConstantBias operator-(
      const gtsam::imuBias::ConstantBias& b) const;

  // Standard Interface
  gtsam::Vector vector() const;
  gtsam::Vector accelerometer() const;
  gtsam::Vector gyroscope() const;
  gtsam::Vector correctAccelerometer(const gtsam::Vector& measurement) const;
  gtsam::Vector correctGyroscope(const gtsam::Vector& measurement) const;

  // enabling serialization functionality
  void serialize() const;
};

}  // namespace imuBias

#include <gtsam/navigation/NavState.h>
class NavState {
  // Constructors
  NavState();
  NavState(const gtsam::Rot3& R, const gtsam::Point3& t,
           const gtsam::Vector& v);
  NavState(const gtsam::Pose3& pose, const gtsam::Vector& v);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::NavState& expected, double tol) const;

  // Access
  gtsam::Rot3 attitude() const;
  gtsam::Point3 position() const;
  gtsam::Vector velocity() const;
  gtsam::Pose3 pose() const;

  gtsam::NavState retract(const gtsam::Vector& x) const;
  gtsam::Vector localCoordinates(const gtsam::NavState& g) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/navigation/PreintegratedRotation.h>
virtual class PreintegratedRotationParams {
  PreintegratedRotationParams();

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegratedRotationParams& expected, double tol);

  void setGyroscopeCovariance(const gtsam::Matrix& cov);
  void setOmegaCoriolis(const gtsam::Vector& omega);
  void setBodyPSensor(const gtsam::Pose3& pose);

  gtsam::Matrix getGyroscopeCovariance() const;

  std::optional<gtsam::Vector> getOmegaCoriolis() const;
  std::optional<gtsam::Pose3> getBodyPSensor() const;
};

#include <gtsam/navigation/PreintegrationParams.h>
virtual class PreintegrationParams : gtsam::PreintegratedRotationParams {
  PreintegrationParams(const gtsam::Vector& n_gravity);

  gtsam::Vector n_gravity;

  static gtsam::PreintegrationParams* MakeSharedD(double g);
  static gtsam::PreintegrationParams* MakeSharedU(double g);
  static gtsam::PreintegrationParams* MakeSharedD();  // default g = 9.81
  static gtsam::PreintegrationParams* MakeSharedU();  // default g = 9.81

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegrationParams& expected, double tol);

  void setAccelerometerCovariance(const gtsam::Matrix& cov);
  void setIntegrationCovariance(const gtsam::Matrix& cov);
  void setUse2ndOrderCoriolis(bool flag);

  gtsam::Matrix getAccelerometerCovariance() const;
  gtsam::Matrix getIntegrationCovariance() const;
  bool getUse2ndOrderCoriolis() const;

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
  void integrateMeasurement(const gtsam::Vector& measuredAcc,
                            const gtsam::Vector& measuredOmega, double deltaT);
  void integrateMeasurements(const gtsam::Matrix& measuredAccs,
                             const gtsam::Matrix& measuredOmegas,
                             const gtsam::Vector& dts);
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

virtual class ImuFactor : gtsam::NonlinearFactor {
  ImuFactor(
      size_t pose_i, size_t vel_i, size_t pose_j, size_t vel_j, size_t bias,
      const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedImuMeasurements preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i,
                              const gtsam::Vector& vel_i,
                              const gtsam::Pose3& pose_j,
                              const gtsam::Vector& vel_j,
                              const gtsam::imuBias::ConstantBias& bias);
};

#include <gtsam/navigation/CombinedImuFactor.h>
virtual class PreintegrationCombinedParams : gtsam::PreintegrationParams {
  PreintegrationCombinedParams(const gtsam::Vector& n_gravity);

  static gtsam::PreintegrationCombinedParams* MakeSharedD(double g);
  static gtsam::PreintegrationCombinedParams* MakeSharedU(double g);
  static gtsam::PreintegrationCombinedParams*
  MakeSharedD();  // default g = 9.81
  static gtsam::PreintegrationCombinedParams*
  MakeSharedU();  // default g = 9.81

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::PreintegrationCombinedParams& expected, double tol);

  void setBiasAccCovariance(const gtsam::Matrix& cov);
  void setBiasOmegaCovariance(const gtsam::Matrix& cov);
  void setBiasAccOmegaInit(const gtsam::Matrix& cov);

  gtsam::Matrix getBiasAccCovariance() const;
  gtsam::Matrix getBiasOmegaCovariance() const;
  gtsam::Matrix getBiasAccOmegaInit() const;
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
  void integrateMeasurement(const gtsam::Vector& measuredAcc,
                            const gtsam::Vector& measuredOmega, double deltaT);
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
};

virtual class CombinedImuFactor : gtsam::NonlinearFactor {
  CombinedImuFactor(size_t pose_i, size_t vel_i, size_t pose_j, size_t vel_j,
                    size_t bias_i, size_t bias_j,
                    const gtsam::PreintegratedCombinedMeasurements&
                        CombinedPreintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedCombinedMeasurements preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i,
                              const gtsam::Vector& vel_i,
                              const gtsam::Pose3& pose_j,
                              const gtsam::Vector& vel_j,
                              const gtsam::imuBias::ConstantBias& bias_i,
                              const gtsam::imuBias::ConstantBias& bias_j);
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
  PreintegratedAhrsMeasurements(
      const gtsam::PreintegratedAhrsMeasurements& rhs);

  // Testable
  void print(string s = "Preintegrated Measurements: ") const;
  bool equals(const gtsam::PreintegratedAhrsMeasurements& expected, double tol);

  // get Data
  gtsam::Rot3 deltaRij() const;
  double deltaTij() const;
  gtsam::Vector biasHat() const;

  // Standard Interface
  void integrateMeasurement(const gtsam::Vector& measuredOmega, double deltaT);
  void resetIntegration();
};

virtual class AHRSFactor : gtsam::NonlinearFactor {
  AHRSFactor(
      size_t rot_i, size_t rot_j, size_t bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements,
      const gtsam::Vector& omegaCoriolis);
  AHRSFactor(
      size_t rot_i, size_t rot_j, size_t bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements,
      const gtsam::Vector& omegaCoriolis, const gtsam::Pose3& body_P_sensor);

  // Standard Interface
  gtsam::PreintegratedAhrsMeasurements preintegratedMeasurements() const;
  gtsam::Vector evaluateError(const gtsam::Rot3& rot_i,
                              const gtsam::Rot3& rot_j,
                              const gtsam::Vector& bias) const;
  gtsam::Rot3 predict(
      const gtsam::Rot3& rot_i, const gtsam::Vector& bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements,
      const gtsam::Vector& omegaCoriolis) const;
};

#include <gtsam/navigation/AttitudeFactor.h>
// virtual class AttitudeFactor : gtsam::NonlinearFactor {
//   AttitudeFactor(const Unit3& nZ, const Unit3& bRef);
//   AttitudeFactor();
// };
virtual class Rot3AttitudeFactor : gtsam::NonlinearFactor {
  Rot3AttitudeFactor(size_t key, const gtsam::Unit3& nZ,
                     const gtsam::noiseModel::Diagonal* model,
                     const gtsam::Unit3& bRef);
  Rot3AttitudeFactor(size_t key, const gtsam::Unit3& nZ,
                     const gtsam::noiseModel::Diagonal* model);
  Rot3AttitudeFactor();
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nZ() const;
  gtsam::Unit3 bRef() const;
};

virtual class Pose3AttitudeFactor : gtsam::NonlinearFactor {
  Pose3AttitudeFactor(size_t key, const gtsam::Unit3& nZ,
                      const gtsam::noiseModel::Diagonal* model,
                      const gtsam::Unit3& bRef);
  Pose3AttitudeFactor(size_t key, const gtsam::Unit3& nZ,
                      const gtsam::noiseModel::Diagonal* model);
  Pose3AttitudeFactor();
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nZ() const;
  gtsam::Unit3 bRef() const;
};

#include <gtsam/navigation/GPSFactor.h>
virtual class GPSFactor : gtsam::NonlinearFactor {
  GPSFactor(size_t key, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
};

virtual class GPSFactor2 : gtsam::NonlinearFactor {
  GPSFactor2(size_t key, const gtsam::Point3& gpsIn,
             const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
};

#include <gtsam/navigation/BarometricFactor.h>
virtual class BarometricFactor : gtsam::NonlinearFactor {
  BarometricFactor();
  BarometricFactor(size_t key, size_t baroKey, const double& baroIn,
                   const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol);

  // Standard Interface
  const double& measurementIn() const;
  double heightOut(double n) const;
  double baroOut(const double& meters) const;
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
  ConstantTwistScenario(const gtsam::Vector& w, const gtsam::Vector& v);
  ConstantTwistScenario(const gtsam::Vector& w, const gtsam::Vector& v,
                        const gtsam::Pose3& nTb0);
};

virtual class AcceleratingScenario : gtsam::Scenario {
  AcceleratingScenario(const gtsam::Rot3& nRb, const gtsam::Point3& p0,
                       const gtsam::Vector& v0, const gtsam::Vector& a_n,
                       const gtsam::Vector& omega_b);
};

#include <gtsam/navigation/ScenarioRunner.h>
class ScenarioRunner {
  ScenarioRunner(const gtsam::Scenario& scenario,
                 const gtsam::PreintegrationParams* p, double imuSampleTime,
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

}  // namespace gtsam
