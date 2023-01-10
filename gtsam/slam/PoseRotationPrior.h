/**
 * @file PoseRotationPrior.h
 *
 * @brief Implements a prior on the rotation component of a pose
 *
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>


namespace gtsam {

template<class POSE>
class PoseRotationPrior : public NoiseModelFactorN<POSE> {
public:

  typedef PoseRotationPrior<POSE> This;
  typedef NoiseModelFactorN<POSE> Base;
  typedef POSE Pose;
  typedef typename POSE::Translation Translation;
  typedef typename POSE::Rotation Rotation;


  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  GTSAM_CONCEPT_POSE_TYPE(Pose)
  GTSAM_CONCEPT_GROUP_TYPE(Pose)
  GTSAM_CONCEPT_LIE_TYPE(Rotation)

  // Get dimensions of pose and rotation type at compile time
  static const int xDim = FixedDimension<POSE>::value;
  static const int rDim = FixedDimension<typename POSE::Rotation>::value;

protected:

  Rotation measured_;

public:

  /** default constructor - only use for serialization */
  PoseRotationPrior() {}

  /** standard constructor */
  PoseRotationPrior(Key key, const Rotation& rot_z, const SharedNoiseModel& model)
  : Base(model, key), measured_(rot_z) {}

  /** Constructor that pulls the translation from an incoming POSE */
  PoseRotationPrior(Key key, const POSE& pose_z, const SharedNoiseModel& model)
  : Base(model, key), measured_(pose_z.rotation()) {}

  ~PoseRotationPrior() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  // access
  const Rotation& measured() const { return measured_; }

  // testable

  /** equals specialized to this factor */
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != nullptr && Base::equals(*e, tol) && measured_.equals(e->measured_, tol);
  }

  /** print contents */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s + "PoseRotationPrior", keyFormatter);
    measured_.print("Measured Rotation");
  }

  /** h(x)-z */
  Vector evaluateError(const Pose& pose, OptionalMatrixType H) const override {
    const Rotation& newR = pose.rotation();
    if (H) {
      *H = Matrix::Zero(rDim, xDim);
      std::pair<size_t, size_t> rotInterval = POSE::rotationInterval();
      (*H).middleCols(rotInterval.first, rDim).setIdentity(rDim, rDim);
    }

    return measured_.localCoordinates(newR);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
};

} // \namespace gtsam




