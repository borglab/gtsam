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
#include <gtsam/linear/Sampler.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>

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

/*
 *  Simple class to test navigation scenarios.
 *  Takes a trajectory scenario as input, and can generate IMU measurements
 */
class GTSAM_EXPORT ScenarioRunner {
 public:
  typedef imuBias::ConstantBias Bias;
  typedef std::shared_ptr<PreintegrationParams> SharedParams;

 private:
  const Scenario& scenario_;
  const SharedParams p_;
  const double imuSampleTime_, sqrt_dt_;
  const Bias estimatedBias_;

  // Create two samplers for acceleration and omega noise
  Sampler gyroSampler_, accSampler_;

 public:
  ScenarioRunner(const Scenario& scenario, const SharedParams& p,
                 double imuSampleTime = 1.0 / 100.0, const Bias& bias = Bias())
      : scenario_(scenario),
        p_(p),
        imuSampleTime_(imuSampleTime),
        sqrt_dt_(std::sqrt(imuSampleTime)),
        estimatedBias_(bias),
        // NOTE(duy): random seeds that work well:
        gyroSampler_(Diagonal(p->gyroscopeCovariance), 10),
        accSampler_(Diagonal(p->accelerometerCovariance), 29284) {}

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  const Vector3& gravity_n() const { return p_->n_gravity; }

  const Scenario& scenario() const { return scenario_; }

  // A gyro simply measures angular velocity in body frame
  Vector3 actualAngularVelocity(double t) const { return scenario_.omega_b(t); }

  // An accelerometer measures acceleration in body, but not gravity
  Vector3 actualSpecificForce(double t) const {
    Rot3 bRn(scenario_.rotation(t).transpose());
    return scenario_.acceleration_b(t) - bRn * gravity_n();
  }

  // versions corrupted by bias and noise
  Vector3 measuredAngularVelocity(double t) const {
    return actualAngularVelocity(t) + estimatedBias_.gyroscope() +
           gyroSampler_.sample() / sqrt_dt_;
  }
  Vector3 measuredSpecificForce(double t) const {
    return actualSpecificForce(t) + estimatedBias_.accelerometer() +
           accSampler_.sample() / sqrt_dt_;
  }

  const double& imuSampleTime() const { return imuSampleTime_; }

  /// Integrate measurements for T seconds into a PIM
  PreintegratedImuMeasurements integrate(double T,
                                         const Bias& estimatedBias = Bias(),
                                         bool corrupted = false) const;

  /// Predict predict given a PIM
  NavState predict(const PreintegratedImuMeasurements& pim,
                   const Bias& estimatedBias = Bias()) const;

  /// Compute a Monte Carlo estimate of the predict covariance using N samples
  Matrix9 estimateCovariance(double T, size_t N = 1000,
                             const Bias& estimatedBias = Bias()) const;

  /// Estimate covariance of sampled noise for sanity-check
  Matrix6 estimateNoiseCovariance(size_t N = 1000) const;
};

/*
 *  Simple class to test navigation scenarios with CombinedImuMeasurements.
 *  Takes a trajectory scenario as input, and can generate IMU measurements
 */
class GTSAM_EXPORT CombinedScenarioRunner : public ScenarioRunner {
 public:
  typedef std::shared_ptr<PreintegrationCombinedParams> SharedParams;

 private:
  const SharedParams p_;
  const Bias estimatedBias_;

 public:
  CombinedScenarioRunner(const Scenario& scenario, const SharedParams& p,
                         double imuSampleTime = 1.0 / 100.0,
                         const Bias& bias = Bias())
      : ScenarioRunner(scenario, static_cast<ScenarioRunner::SharedParams>(p),
                       imuSampleTime, bias),
        p_(p),
        estimatedBias_(bias) {}

  /// Integrate measurements for T seconds into a PIM
  PreintegratedCombinedMeasurements integrate(
      double T, const Bias& estimatedBias = Bias(),
      bool corrupted = false) const;

  /// Predict predict given a PIM
  NavState predict(const PreintegratedCombinedMeasurements& pim,
                   const Bias& estimatedBias = Bias()) const;

  /// Compute a Monte Carlo estimate of the predict covariance using N samples
  Eigen::Matrix<double, 15, 15> estimateCovariance(
      double T, size_t N = 1000, const Bias& estimatedBias = Bias()) const;
};

}  // namespace gtsam
