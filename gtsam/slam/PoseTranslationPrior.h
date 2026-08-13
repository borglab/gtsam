/**
 * @file PoseTranslationPrior.h
 *
 * @brief Implements a prior on the translation component of a pose
 *
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtsam {

/**
 * A prior on the translation part of a pose
 */
template<class POSE>
class PoseTranslationPrior : public NoiseModelFactorN<POSE> {
public:
  typedef PoseTranslationPrior<POSE> This;
  typedef NoiseModelFactorN<POSE> Base;
  typedef POSE Pose;
  typedef typename POSE::Translation Translation;
  typedef typename POSE::Rotation Rotation;
  
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  GTSAM_CONCEPT_POSE_TYPE(Pose)
  GTSAM_CONCEPT_GROUP_TYPE(Pose)
  GTSAM_CONCEPT_LIE_TYPE(Translation)

protected:

  Translation measured_;

public:

  /** default constructor - only use for serialization */
  PoseTranslationPrior() {}

  /** standard constructor */
  PoseTranslationPrior(Key key, const Translation& measured, const noiseModel::Base::shared_ptr& model)
  : Base(model, key), measured_(measured) {
  }

  /** Constructor that pulls the translation from an incoming POSE */
  PoseTranslationPrior(Key key, const POSE& pose_z, const noiseModel::Base::shared_ptr& model)
  : Base(model, key), measured_(pose_z.translation()) {
  }

  ~PoseTranslationPrior() override {}

  const Translation& measured() const { return measured_; }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** h(x)-z */
  Vector evaluateError(const Pose& pose, OptionalMatrixType H) const override {
    Matrix Htranslation;
    const Translation& newTrans = pose.translation(H ? &Htranslation : nullptr);
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
    if constexpr (internal::HasLocalJacobians<Translation>::value) {
      if (H) {
        typename traits<Translation>::ChartJacobian::Jacobian Hlocal;
        const Vector error = traits<Translation>::Local(
            measured_, newTrans, OptionalNone, &Hlocal);
        *H = Hlocal * Htranslation;
        return error;
      }
    }
#endif
    if (H) *H = Htranslation;

    return traits<Translation>::Local(measured_, newTrans);
  }

  /** equals specialized to this factor */
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != nullptr && Base::equals(*e, tol) && traits<Translation>::Equals(measured_, e->measured_, tol);
  }

  /** print contents */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s + "PoseTranslationPrior", keyFormatter);
    traits<Translation>::Print(measured_, "Measured Translation");
  }

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
#endif

};

} // \namespace gtsam


