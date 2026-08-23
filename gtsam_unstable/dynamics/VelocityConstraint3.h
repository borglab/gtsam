/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file VelocityConstraint3.h
 * @brief A simple 3-way factor constraining double poses and velocity
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/config.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtsam {

/**
 * Three-way scalar velocity constraint.
 * @deprecated This experimental dynamics factor has no maintained replacement.
 */
class VelocityConstraint3
    : public NoiseModelFactorT<Vector1, double, double, double> {
public:

protected:
  typedef NoiseModelFactorT<Vector1, double, double, double> Base;

  /** default constructor to allow for serialization */
  VelocityConstraint3() {}

  double dt_;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  typedef std::shared_ptr<VelocityConstraint3 > shared_ptr;

  ///TODO: comment
  VelocityConstraint3(Key key1, Key key2, Key velKey, double dt, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(1, std::abs(mu)), key1, key2, velKey), dt_(dt) {}
  ~VelocityConstraint3() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new VelocityConstraint3(*this))); }

  /** x1 + v*dt - x2 = 0, with optional derivatives */
  Vector1 evaluateError(const double& x1, const double& x2, const double& v,
                        OptionalMatrixType H1, OptionalMatrixType H2,
                        OptionalMatrixType H3) const override {
    const size_t p = 1;
    if (H1) *H1 = Matrix::Identity(p,p);
    if (H2) *H2 = -Matrix::Identity(p,p);
    if (H3) *H3 = Matrix::Identity(p,p)*dt_;
    return Vector{{x1 + v * dt_ - x2}};
  }

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
        boost::serialization::base_object<Base>(*this));
  }
#endif
}; // \VelocityConstraint3

}

#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43
