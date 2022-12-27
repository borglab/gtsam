/**
 * @file VelocityConstraint.h
 * @brief Constraint enforcing the relationship between pose and velocity
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>

#include <boost/bind/bind.hpp>

namespace gtsam {

namespace dynamics {

/** controls which model to use for numerical integration to use for constraints */
typedef enum {
  TRAPEZOIDAL, // Constant acceleration
  EULER_START, // Constant velocity, using starting velocity
  EULER_END    // Constant velocity, using ending velocity
} IntegrationMode;

}

/**
 * Constraint to enforce dynamics between the velocities and poses, using
 * a prediction based on a numerical integration flag.
 *
 * NOTE: this approximation is insufficient for large timesteps, but is accurate
 * if timesteps are small.
 */
class VelocityConstraint : public gtsam::NoiseModelFactorN<PoseRTV,PoseRTV> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(VelocityConstraint, 2);

public:
  typedef gtsam::NoiseModelFactorN<PoseRTV,PoseRTV> Base;

protected:

  double dt_;   /// time difference between frames in seconds
  dynamics::IntegrationMode integration_mode_;  ///< Numerical integration control

public:

  /**
   * Creates a constraint relating the given variables with fully constrained noise model
   */
  VelocityConstraint(Key key1, Key key2, const dynamics::IntegrationMode& mode,
      double dt, double mu = 1000)
  : Base(noiseModel::Constrained::All(3, mu), key1, key2), dt_(dt), integration_mode_(mode) {}

  /**
     * Creates a constraint relating the given variables with fully constrained noise model
     * Uses the default Trapezoidal integrator
     */
    VelocityConstraint(Key key1, Key key2, double dt, double mu = 1000)
    : Base(noiseModel::Constrained::All(3, mu), key1, key2),
      dt_(dt), integration_mode_(dynamics::TRAPEZOIDAL) {}

  /**
   * Creates a constraint relating the given variables with arbitrary noise model
   */
  VelocityConstraint(Key key1, Key key2, const dynamics::IntegrationMode& mode,
      double dt, const gtsam::SharedNoiseModel& model)
  : Base(model, key1, key2), dt_(dt), integration_mode_(mode) {}

  /**
   * Creates a constraint relating the given variables with arbitrary noise model
   * Uses the default Trapezoidal integrator
   */
  VelocityConstraint(Key key1, Key key2, double dt, const gtsam::SharedNoiseModel& model)
  : Base(model, key1, key2), dt_(dt), integration_mode_(dynamics::TRAPEZOIDAL) {}

  ~VelocityConstraint() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new VelocityConstraint(*this))); }

  /**
   * Calculates the error for trapezoidal model given
   */
  gtsam::Vector evaluateError(const PoseRTV& x1, const PoseRTV& x2,
      boost::optional<gtsam::Matrix&> H1=boost::none,
      boost::optional<gtsam::Matrix&> H2=boost::none) const override {
    if (H1) *H1 = gtsam::numericalDerivative21<gtsam::Vector,PoseRTV,PoseRTV>(
        std::bind(VelocityConstraint::evaluateError_, std::placeholders::_1,
            std::placeholders::_2, dt_, integration_mode_), x1, x2, 1e-5);
    if (H2) *H2 = gtsam::numericalDerivative22<gtsam::Vector,PoseRTV,PoseRTV>(
        std::bind(VelocityConstraint::evaluateError_, std::placeholders::_1,
            std::placeholders::_2, dt_, integration_mode_), x1, x2, 1e-5);
    return evaluateError_(x1, x2, dt_, integration_mode_);
  }

  void print(const std::string& s = "", const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const override {
    std::string a = "VelocityConstraint: " + s;
    Base::print(a, formatter);
    switch(integration_mode_) {
    case dynamics::TRAPEZOIDAL: std::cout << "Integration: Trapezoidal\n"; break;
    case dynamics::EULER_START: std::cout << "Integration: Euler (start)\n"; break;
    case dynamics::EULER_END: std::cout << "Integration: Euler (end)\n"; break;
    default: std::cout << "Integration: Unknown\n" << std::endl; break;
    }
    std::cout << "dt: " << dt_ << std::endl;
  }

private:
  static gtsam::Vector evaluateError_(const PoseRTV& x1, const PoseRTV& x2,
      double dt, const dynamics::IntegrationMode& mode) {

    const Velocity3& v1 = x1.v(), v2 = x2.v();
    const Point3& p1 = x1.t(), p2 = x2.t();
    Point3 hx(0,0,0);
    switch(mode) {
    case dynamics::TRAPEZOIDAL: hx = p1 + Point3((v1 + v2) * dt *0.5); break;
    case dynamics::EULER_START: hx = p1 + Point3(v1 * dt); break;
    case dynamics::EULER_END  : hx = p1 + Point3(v2 * dt); break;
    default: assert(false); break;
    }
    return p2 - hx;
  }
};

} // \namespace gtsam
