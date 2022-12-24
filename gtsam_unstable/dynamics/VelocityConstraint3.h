/**
 * @file VelocityConstraint3.h
 * @brief A simple 3-way factor constraining double poses and velocity
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class VelocityConstraint3 : public NoiseModelFactorN<double, double, double> {
public:

protected:
  typedef NoiseModelFactorN<double, double, double> Base;

  /** default constructor to allow for serialization */
  VelocityConstraint3() {}

  double dt_;

public:

  typedef boost::shared_ptr<VelocityConstraint3 > shared_ptr;

  ///TODO: comment
  VelocityConstraint3(Key key1, Key key2, Key velKey, double dt, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(1, std::abs(mu)), key1, key2, velKey), dt_(dt) {}
  ~VelocityConstraint3() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new VelocityConstraint3(*this))); }

  /** x1 + v*dt - x2 = 0, with optional derivatives */
  Vector evaluateError(const double& x1, const double& x2, const double& v,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const override {
    const size_t p = 1;
    if (H1) *H1 = Matrix::Identity(p,p);
    if (H2) *H2 = -Matrix::Identity(p,p);
    if (H3) *H3 = Matrix::Identity(p,p)*dt_;
    return (Vector(1) << x1+v*dt_-x2).finished();
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
        boost::serialization::base_object<Base>(*this));
  }
}; // \VelocityConstraint3

}
