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
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// print
  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s);
    std::cout << "RotateFactor:]\n";
    std::cout << "p: " << p_.transpose() << std::endl;
    std::cout << "z: " << z_.transpose() << std::endl;
  }

  /// vector of errors returns 2D vector
  Vector evaluateError(const Rot3& R,
      boost::optional<Matrix&> H = boost::none) const override {
    // predict p_ as q = R*z_, derivative H will be filled if not none
    Point3 q = R.rotate(z_,H);
    // error is just difference, and note derivative of that wrpt q is I3
    return (Vector(3) << q.x()-p_.x(), q.y()-p_.y(), q.z()-p_.z()).finished();
  }

};

/**
 * Factor on unknown rotation iRc that relates two directions c
 * Directions provide less constraints than a full rotation
 */
class RotateDirectionsFactor: public NoiseModelFactor1<Rot3> {

  Unit3 i_p_, c_z_; ///< Predicted and measured directions, i_p = iRc * c_z

  typedef NoiseModelFactor1<Rot3> Base;
  typedef RotateDirectionsFactor This;

public:

  /// Constructor
  RotateDirectionsFactor(Key key, const Unit3& i_p, const Unit3& c_z,
      const SharedNoiseModel& model) :
      Base(model, key), i_p_(i_p), c_z_(c_z) {
  }

  /// Initialize rotation iRc such that i_p = iRc * c_z
  static Rot3 Initialize(const Unit3& i_p, const Unit3& c_z) {
    gtsam::Quaternion iRc;
    // setFromTwoVectors sets iRc to (a) quaternion which transform c_z into i_p
    iRc.setFromTwoVectors(c_z.unitVector(), i_p.unitVector());
    return Rot3(iRc);
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// print
  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s);
    std::cout << "RotateDirectionsFactor:" << std::endl;
    i_p_.print("p");
    c_z_.print("z");
  }

  /// vector of errors returns 2D vector
  Vector evaluateError(const Rot3& iRc, boost::optional<Matrix&> H = boost::none) const override {
    Unit3 i_q = iRc * c_z_;
    Vector error = i_p_.error(i_q, H);
    if (H) {
      Matrix DR;
      iRc.rotate(c_z_, DR);
      *H = (*H) * DR;
    }
    return error;
  }

  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

