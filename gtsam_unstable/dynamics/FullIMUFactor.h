/**
 * @file FullIMUFactor.h
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
 * This factor has dimension 9, with a built-in constraint for velocity modeling
 *
 * Templated to allow for different key types, but variables all
 * assumed to be PoseRTV
 */
template<class POSE>
class FullIMUFactor : public NoiseModelFactorN<POSE, POSE> {

public:
  typedef NoiseModelFactorN<POSE, POSE> Base;
  typedef FullIMUFactor<POSE> This;

protected:

  /** measurements from the IMU */
  Vector3 accel_, gyro_;
  double dt_; /// time between measurements

public:

  /** Standard constructor */
  FullIMUFactor(const Vector3& accel, const Vector3& gyro,
      double dt, const Key& key1, const Key& key2, const SharedNoiseModel& model)
  : Base(model, key1, key2), accel_(accel), gyro_(gyro), dt_(dt) {
    assert(model->dim() == 9);
  }

  /** Single IMU vector - imu = [accel, gyro] */
  FullIMUFactor(const Vector6& imu,
      double dt, const Key& key1, const Key& key2, const SharedNoiseModel& model)
  : Base(model, key1, key2), accel_(imu.head(3)), gyro_(imu.tail(3)), dt_(dt) {
    assert(imu.size() == 6);
    assert(model->dim() == 9);
  }

  ~FullIMUFactor() override {}

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
    std::string a = "FullIMUFactor: " + s;
    Base::print(a, formatter);
    gtsam::print((Vector)accel_, "accel");
    gtsam::print((Vector)gyro_, "gyro");
    std::cout << "dt: " << dt_ << std::endl;
  }

  // access
  const Vector3& gyro() const { return gyro_; }
  const Vector3& accel() const { return accel_; }
  Vector6 z() const { return (Vector(6) << accel_, gyro_).finished(); }

  /**
   * Error evaluation with optional derivatives - calculates
   *  z - h(x1,x2)
   */
  Vector evaluateError(const PoseRTV& x1, const PoseRTV& x2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const override {
    Vector9 z;
    z.head(3).operator=(accel_); // Strange syntax to work around ambiguous operator error with clang
    z.segment(3, 3).operator=(gyro_); // Strange syntax to work around ambiguous operator error with clang
    z.tail(3).operator=(x2.t()); // Strange syntax to work around ambiguous operator error with clang
    if (H1) *H1 = numericalDerivative21<Vector9, PoseRTV, PoseRTV>(
        std::bind(This::predict_proxy, std::placeholders::_1, std::placeholders::_2, dt_), x1, x2, 1e-5);
    if (H2) *H2 = numericalDerivative22<Vector9, PoseRTV, PoseRTV>(
        std::bind(This::predict_proxy, std::placeholders::_1, std::placeholders::_2, dt_), x1, x2, 1e-5);
    return z - predict_proxy(x1, x2, dt_);
  }

  /** dummy version that fails for non-dynamic poses */
  virtual Vector evaluateError(const Pose3& x1, const Pose3& x2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const {
    assert(false);
    return Vector6::Zero();
  }

private:

  /** copy of the measurement function formulated for numerical derivatives */
  static Vector9 predict_proxy(const PoseRTV& x1, const PoseRTV& x2, double dt) {
    Vector9 hx;
    hx.head(6).operator=(x1.imuPrediction(x2, dt)); // Strange syntax to work around ambiguous operator error with clang
    hx.tail(3).operator=(x1.translationIntegration(x2, dt)); // Strange syntax to work around ambiguous operator error with clang
    return hx;
  }
};

} // \namespace gtsam
