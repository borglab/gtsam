/**
 * @file VelocityConstraint3.h
 * @brief A simple 3-way factor constraining LieScalar poses and velocity
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/base/LieScalar.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class VelocityConstraint3 : public NoiseModelFactor3<LieScalar, LieScalar, LieScalar> {
public:

protected:
  typedef NoiseModelFactor3<LieScalar, LieScalar, LieScalar> Base;

  /** default constructor to allow for serialization */
  VelocityConstraint3() {}

  double dt_;

public:

  typedef boost::shared_ptr<VelocityConstraint3 > shared_ptr;

  ///TODO: comment
  VelocityConstraint3(Key key1, Key key2, Key velKey, double dt, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(LieScalar::Dim(), fabs(mu)), key1, key2, velKey), dt_(dt) {}
  virtual ~VelocityConstraint3() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new VelocityConstraint3(*this))); }

  /** g(x) with optional derivative2 */
  Vector evaluateError(const LieScalar& x1, const LieScalar& x2, const LieScalar& v,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const {
    const size_t p = LieScalar::Dim();
    if (H1) *H1 = eye(p);
    if (H2) *H2 = -eye(p);
    if (H3) *H3 = eye(p)*dt_;
    return x2.localCoordinates(x1.compose(LieScalar(v*dt_)));
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
        boost::serialization::base_object<Base>(*this));
  }
}; // \VelocityConstraint3

}
