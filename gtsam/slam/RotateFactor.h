/*
 * @file RotateFactor.cpp
 * @brief RotateFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot3.h>

namespace gtsam {

/**
 * Factor on unknown rotation iRC that relates two incremental rotations
 *   c1Rc2 = iRc' * i1Ri2 * iRc
 * Which we can write (see doc/math.lyx)
 *   e^[z] = iRc' * e^[p] * iRc = e^([iRc'*p])
 * with z and p measured and predicted angular velocities, and hence
 *   p = iRc * z
 */
class RotateFactor: public NoiseModelFactor1<Rot3> {

  Point3 p_, z_; ///< Predicted and measured directions, p = iRc * z

  typedef NoiseModelFactor1<Rot3> Base;
  typedef RotateFactor This;

public:

  /// Constructor
  RotateFactor(Key key, const Rot3& P, const Rot3& Z,
      const SharedNoiseModel& model) :
      Base(model, key), p_(Rot3::Logmap(P)), z_(Rot3::Logmap(Z)) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// print
  virtual void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "RotateFactor:" << std::endl;
    p_.print("p");
    z_.print("z");
  }

  /// vector of errors returns 2D vector
  Vector evaluateError(const Rot3& R,
      boost::optional<Matrix&> H = boost::none) const {
    // predict p_ as q = R*z_, derivative H will be filled if not none
    Point3 q = R.rotate(z_,H);
    // error is just difference, and note derivative of that wrpt q is I3
    return (Vector(3) << q.x()-p_.x(), q.y()-p_.y(), q.z()-p_.z()).finished();
  }

};

/**
 * Factor on unknown rotation R that relates two directions p_i = iRc * z_c
 * Directions provide less constraints than a full rotation
 */
class RotateDirectionsFactor: public NoiseModelFactor1<Rot3> {

  Unit3 p_, z_; ///< Predicted and measured directions, p = iRc * z

  typedef NoiseModelFactor1<Rot3> Base;
  typedef RotateDirectionsFactor This;

public:

  /// Constructor
  RotateDirectionsFactor(Key key, const Unit3& p, const Unit3& z,
      const SharedNoiseModel& model) :
      Base(model, key), p_(p), z_(z) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// print
  virtual void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "RotateDirectionsFactor:" << std::endl;
    p_.print("p");
    z_.print("z");
  }

  /// vector of errors returns 2D vector
  Vector evaluateError(const Rot3& R,
      boost::optional<Matrix&> H = boost::none) const {
    Unit3 q = R * z_;
    Vector e = p_.error(q, H);
    if (H) {
      Matrix DR;
      R.rotate(z_, DR);
      *H = (*H) * DR;
    }
    return e;
  }

};
} // gtsam

