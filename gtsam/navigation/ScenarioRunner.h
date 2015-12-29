/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ScenarioRunner.h
 * @brief   Simple class to test navigation scenarios
 * @author  Frank Dellaert
 */

#pragma once
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/linear/Sampler.h>

namespace gtsam {

// Convert covariance to diagonal noise model, if possible, otherwise throw
static noiseModel::Diagonal::shared_ptr Diagonal(const Matrix& covariance) {
  bool smart = true;
  auto model = noiseModel::Gaussian::Covariance(covariance, smart);
  auto diagonal = boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (!diagonal)
    throw std::invalid_argument("ScenarioRunner::Diagonal: not a diagonal");
  return diagonal;
}

class GaussianBayesNet;

/**
 * Class that integrates state estimate on the manifold.
 * We integrate zeta = [theta, position, velocity]
 * See ImuFactor.lyx for the relevant math.
 */
class PreintegratedMeasurements2 {
 public:
  typedef ImuFactor::PreintegratedMeasurements::Params Params;
  typedef boost::shared_ptr<GaussianBayesNet> SharedBayesNet;

 private:
  const boost::shared_ptr<Params> p_;
  const SharedDiagonal accelerometerNoiseModel_, gyroscopeNoiseModel_;
  const imuBias::ConstantBias estimatedBias_;

  size_t k_;         ///< index/count of measurements integrated
  double deltaTij_;  ///< sum of time increments

  /// posterior on current iterate, stored as a Bayes net P(zeta|bias_delta):
  SharedBayesNet posterior_k_;

 public:
  PreintegratedMeasurements2(
      const boost::shared_ptr<Params>& p,
      const imuBias::ConstantBias& estimatedBias = imuBias::ConstantBias())
      : p_(p),
        accelerometerNoiseModel_(Diagonal(p->accelerometerCovariance)),
        gyroscopeNoiseModel_(Diagonal(p->gyroscopeCovariance)),
        estimatedBias_(estimatedBias),
        k_(0),
        deltaTij_(0.0) {}

  SharedDiagonal discreteAccelerometerNoiseModel(double dt) const;
  SharedDiagonal discreteGyroscopeNoiseModel(double dt) const;

  /**
   * Add a single IMU measurement to the preintegration.
   * @param measuredAcc Measured acceleration (in body frame)
   * @param measuredOmega Measured angular velocity (in body frame)
   * @param dt Time interval between this and the last IMU measurement
   */
  void integrateMeasurement(const Vector3& measuredAcc,
                            const Vector3& measuredOmega, double dt);

  /// Predict state at time j
  NavState predict(const NavState& state_i, const imuBias::ConstantBias& bias_i,
                   OptionalJacobian<9, 9> H1 = boost::none,
                   OptionalJacobian<9, 6> H2 = boost::none) const;

  /// Return Gaussian noise model on prediction
  SharedGaussian noiseModel() const;

  /// @deprecated: Explicitly calculate covariance
  Matrix9 preintMeasCov() const;

 private:
  // estimate zeta given estimated biases
  // calculates conditional mean of P(zeta|bias_delta)
  Vector9 currentEstimate() const;

  // estimate theta given estimated biases
  Vector3 currentTheta() const;

  // We obtain discrete-time noise models by dividing the continuous-time
  // covariances by dt:

  // initialize posterior with first (corrected) IMU measurement
  SharedBayesNet initPosterior(const Vector3& correctedAcc,
                               const Vector3& correctedOmega, double dt) const;

  // integrate
  SharedBayesNet integrateCorrected(const Vector3& correctedAcc,
                                    const Vector3& correctedOmega,
                                    double dt) const;
};

/*
 *  Simple class to test navigation scenarios.
 *  Takes a trajectory scenario as input, and can generate IMU measurements
 */
class ScenarioRunner {
 public:
  typedef boost::shared_ptr<PreintegratedMeasurements2::Params> SharedParams;

 private:
  const Scenario* scenario_;
  const SharedParams p_;
  const double imuSampleTime_, sqrt_dt_;
  const imuBias::ConstantBias bias_;

  // Create two samplers for acceleration and omega noise
  mutable Sampler gyroSampler_, accSampler_;

 public:
  ScenarioRunner(const Scenario* scenario, const SharedParams& p,
                 double imuSampleTime = 1.0 / 100.0,
                 const imuBias::ConstantBias& bias = imuBias::ConstantBias())
      : scenario_(scenario),
        p_(p),
        imuSampleTime_(imuSampleTime),
        sqrt_dt_(std::sqrt(imuSampleTime)),
        bias_(bias),
        // NOTE(duy): random seeds that work well:
        gyroSampler_(Diagonal(p->gyroscopeCovariance), 10),
        accSampler_(Diagonal(p->accelerometerCovariance), 29284) {}

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  const Vector3& gravity_n() const { return p_->n_gravity; }

  // A gyro simply measures angular velocity in body frame
  Vector3 actual_omega_b(double t) const { return scenario_->omega_b(t); }

  // An accelerometer measures acceleration in body, but not gravity
  Vector3 actual_specific_force_b(double t) const {
    Rot3 bRn = scenario_->rotation(t).transpose();
    return scenario_->acceleration_b(t) - bRn * gravity_n();
  }

  // versions corrupted by bias and noise
  Vector3 measured_omega_b(double t) const {
    return actual_omega_b(t) + bias_.gyroscope() +
           gyroSampler_.sample() / sqrt_dt_;
  }
  Vector3 measured_specific_force_b(double t) const {
    return actual_specific_force_b(t) + bias_.accelerometer() +
           accSampler_.sample() / sqrt_dt_;
  }

  const double& imuSampleTime() const { return imuSampleTime_; }

  /// Integrate measurements for T seconds into a PIM
  PreintegratedMeasurements2 integrate(
      double T,
      const imuBias::ConstantBias& estimatedBias = imuBias::ConstantBias(),
      bool corrupted = false) const;

  /// Predict predict given a PIM
  NavState predict(const PreintegratedMeasurements2& pim,
                   const imuBias::ConstantBias& estimatedBias =
                       imuBias::ConstantBias()) const;

  /// Compute a Monte Carlo estimate of the predict covariance using N samples
  Matrix9 estimateCovariance(double T, size_t N = 1000,
                             const imuBias::ConstantBias& estimatedBias =
                                 imuBias::ConstantBias()) const;

  /// Estimate covariance of sampled noise for sanity-check
  Matrix6 estimateNoiseCovariance(size_t N = 1000) const;
};

}  // namespace gtsam
