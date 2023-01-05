/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2014, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   EssentialMatrixConstraint.h
 *  @author Frank Dellaert
 *  @author Pablo Alcantarilla
 *  @date   Jan 5, 2014
 **/

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>

namespace gtsam {

/**
 * Binary factor between two Pose3 variables induced by an EssentialMatrix measurement
 * @ingroup slam
 */
class GTSAM_EXPORT EssentialMatrixConstraint: public NoiseModelFactorN<Pose3, Pose3> {

private:

  typedef EssentialMatrixConstraint This;
  typedef NoiseModelFactorN<Pose3, Pose3> Base;

  EssentialMatrix measuredE_; /** The measurement is an essential matrix */

public:

  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<EssentialMatrixConstraint> shared_ptr;

  /** default constructor - only use for serialization */
  EssentialMatrixConstraint() {
  }

  /**
   *  Constructor
   *  @param key1 key for first pose
   *  @param key2 key for second pose
   *  @param measuredE measured EssentialMatrix
   *  @param model noise model, 5D !
   */
  EssentialMatrixConstraint(Key key1, Key key2,
      const EssentialMatrix& measuredE, const SharedNoiseModel& model) :
      Base(model, key1, key2), measuredE_(measuredE) {
  }

  ~EssentialMatrixConstraint() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed for Testable */

  /** print */
  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** equals */
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector evaluateError(const Pose3& p1, const Pose3& p2,
      OptionalMatrixType Hp1 = OptionalNone, //
      OptionalMatrixType Hp2 = OptionalNone) const override;

  /** return the measured */
  const EssentialMatrix& measured() const {
    return measuredE_;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measuredE_);
  }

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
// \class EssentialMatrixConstraint

}/// namespace gtsam
