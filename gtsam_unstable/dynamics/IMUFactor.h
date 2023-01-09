/**
 * @file IMUFactor.h
 * @brief Factor to express an IMU measurement between dynamic poses
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>

#include <boost/bind/bind.hpp>

namespace gtsam {

/**
 * Class that represents integrating IMU measurements over time for dynamic systems
 * Templated to allow for different key types, but variables all
 * assumed to be PoseRTV
 */
template<class POSE>
class IMUFactor : public NoiseModelFactorN<POSE, POSE> {
public:
  typedef NoiseModelFactorN<POSE, POSE> Base;
  typedef IMUFactor<POSE> This;

protected:

  /** measurements from the IMU */
  Vector3 accel_, gyro_;
  double dt_; /// time between measurements

public:
  using Base::evaluateError;

  /** Standard constructor */
  IMUFactor(const Vector3& accel, const Vector3& gyro,
      double dt, const Key& key1, const Key& key2, const SharedNoiseModel& model)
  : Base(model, key1, key2), accel_(accel), gyro_(gyro), dt_(dt) {}

  /** Full IMU vector specification */
  IMUFactor(const Vector6& imu_vector,
      double dt, const Key& key1, const Key& key2, const SharedNoiseModel& model)
  : Base(model, key1, key2), accel_(imu_vector.head(3)), gyro_(imu_vector.tail(3)), dt_(dt) {}

  ~IMUFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& e, double tol = 1e-9) const override {
    const This* const f = dynamic_cast<const This*>(&e);
    return f && Base::equals(e) &&
        equal_with_abs_tol(accel_, f->accel_, tol) &&
        equal_with_abs_tol(gyro_, f->gyro_, tol) &&
        std::abs(dt_ - f->dt_) < tol;
  }

  void print(const std::string& s="", const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override {
    std::string a = "IMUFactor: " + s;
    Base::print(a, formatter);
    gtsam::print((Vector)accel_, "accel");
    gtsam::print((Vector)gyro_, "gyro");
    std::cout << "dt: " << dt_ << std::endl;
  }

  // access
  const Vector3& gyro() const { return gyro_; }
  const Vector3& accel() const { return accel_; }
  Vector6 z() const { return (Vector6() << accel_, gyro_).finished(); }

  /**
   * Error evaluation with optional derivatives - calculates
   *  z - h(x1,x2)
   */
  Vector evaluateError(const PoseRTV& x1, const PoseRTV& x2,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {
    const Vector6 meas = z();
    if (H1) *H1 = numericalDerivative21<Vector6, PoseRTV, PoseRTV>(
        std::bind(This::predict_proxy, std::placeholders::_1, std::placeholders::_2, dt_, meas), x1, x2, 1e-5);
    if (H2) *H2 = numericalDerivative22<Vector6, PoseRTV, PoseRTV>(
        std::bind(This::predict_proxy, std::placeholders::_1, std::placeholders::_2, dt_, meas), x1, x2, 1e-5);
    return predict_proxy(x1, x2, dt_, meas);
  }

  /** dummy version that fails for non-dynamic poses */
  virtual Vector evaluateError(const Pose3& x1, const Pose3& x2,
      OptionalMatrixType H1, OptionalMatrixType H2) const {
    assert(false); // no corresponding factor here
    return Vector6::Zero();
  }

private:
  /** copy of the measurement function formulated for numerical derivatives */
  static Vector6 predict_proxy(const PoseRTV& x1, const PoseRTV& x2,
      double dt, const Vector6& meas) {
    Vector6 hx = x1.imuPrediction(x2, dt);
    return meas - hx;
  }
};

} // \namespace gtsam
