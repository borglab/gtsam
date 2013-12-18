/*
 * @file RotateFactor.cpp
 * @brief RotateFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Sphere2.h>

namespace gtsam {

/**
 * Factor that evaluates distance between two rotated directions
 */
class RotateFactor: public NoiseModelFactor1<Rot3> {

  Sphere2 p_, z_; ///< Predicted and measured directions, p = iRc * z

  typedef NoiseModelFactor1<Rot3> Base;

public:

  /// Constructor
  RotateFactor(Key key, const Sphere2& p, const Sphere2& z,
      const SharedNoiseModel& model) :
      Base(model, key), p_(p), z_(z) {
  }

  /// print
  virtual void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    Base::print(s);
    p_.print("p");
    z_.print("z");
  }

  /// vector of errors returns 2D vector
  Vector evaluateError(const Rot3& R,
      boost::optional<Matrix&> H = boost::none) const {
    Sphere2 q = R * z_;
    Vector e = p_.error(q, H);
    if (H) {
      Matrix DR;
      Sphere2::Rotate(R, z_, DR);
      *H = (*H) * DR;
    }
    return e;
  }

  /// Obsolete: vector of errors returns 1D vector
  Vector evaluateError1(const Rot3& R,
      boost::optional<Matrix&> H = boost::none) const {
    Sphere2 q = R * z_;
    double e = p_.distance(q, H);
    if (H) {
      Matrix DR;
      Sphere2::Rotate(R, z_, DR);
      *H = (*H) * DR;
    }
    return (Vector(1) << e);
  }

};
} // gtsam

