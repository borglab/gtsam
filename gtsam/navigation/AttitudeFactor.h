/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   AttitudeFactor.h
 *  @author Frank Dellaert
 *  @brief  Header file for Attitude factor
 *  @date   January 28, 2014
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Sphere2.h>

namespace gtsam {

/**
 * Prior on the attitude of a Pose3.
 * Example:
 * - measurement is direction of gravity in body frame bF
 * - reference is direction of gravity in navigation frame nG
 * This factor will give zero error if nRb * bF == nG
 * @addtogroup Navigation
 */
class AttitudeFactor: public NoiseModelFactor1<Pose3> {

private:

  typedef NoiseModelFactor1<Pose3> Base;

  Sphere2 nZ_, bRef_; ///< Position measurement in

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<AttitudeFactor> shared_ptr;

  /// Typedef to this class
  typedef AttitudeFactor This;

  /** default constructor - only use for serialization */
  AttitudeFactor() {
  }

  virtual ~AttitudeFactor() {
  }

  /**
   * @brief Constructor
   * @param key of the Pose3 variable that will be constrained
   * @param nZ measured direction in navigation frame
   * @param model Gaussian noise model
   * @param bRef reference direction in body frame (default Z-axis)
   */
  AttitudeFactor(Key key, const Sphere2& nZ,
      const SharedNoiseModel& model, const Sphere2& bRef=Sphere2(0,0,1)) :
      Base(model, key), nZ_(nZ), bRef_(bRef) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed for Testable */

  /** print */
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const;

  /** equals */
  virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector evaluateError(const Pose3& p,
      boost::optional<Matrix&> H = boost::none) const;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nZ_);
    ar & BOOST_SERIALIZATION_NVP(bRef_);
  }
};

} /// namespace gtsam

