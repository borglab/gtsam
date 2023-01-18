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
#include <gtsam/geometry/Unit3.h>

namespace gtsam {

/**
 * Base class for prior on attitude
 * Example:
 * - measurement is direction of gravity in body frame bF
 * - reference is direction of gravity in navigation frame nG
 * This factor will give zero error if nRb * bF == nG
 * @ingroup navigation
 */
class AttitudeFactor {

protected:

  Unit3 nZ_, bRef_; ///< Position measurement in

public:

  /** default constructor - only use for serialization */
  AttitudeFactor() {
  }

  /**
   * @brief Constructor
   * @param nZ measured direction in navigation frame
   * @param bRef reference direction in body frame (default Z-axis in NED frame, i.e., [0; 0; 1])
   */
  AttitudeFactor(const Unit3& nZ, const Unit3& bRef = Unit3(0, 0, 1)) :
      nZ_(nZ), bRef_(bRef) {
  }

  /** vector of errors */
  Vector attitudeError(const Rot3& p,
      OptionalJacobian<2,3> H = {}) const;

  const Unit3& nZ() const {
    return nZ_;
  }
  const Unit3& bRef() const {
    return bRef_;
  }

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("nZ_",  nZ_);
    ar & boost::serialization::make_nvp("bRef_", bRef_);
  }
#endif
};

/**
 * Version of AttitudeFactor for Rot3
 * @ingroup navigation
 */
class GTSAM_EXPORT Rot3AttitudeFactor: public NoiseModelFactorN<Rot3>, public AttitudeFactor {

  typedef NoiseModelFactorN<Rot3> Base;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<Rot3AttitudeFactor> shared_ptr;

  /// Typedef to this class
  typedef Rot3AttitudeFactor This;

  /** default constructor - only use for serialization */
  Rot3AttitudeFactor() {
  }

  ~Rot3AttitudeFactor() override {
  }

  /**
   * @brief Constructor
   * @param key of the Rot3 variable that will be constrained
   * @param nZ measured direction in navigation frame
   * @param model Gaussian noise model
   * @param bRef reference direction in body frame (default Z-axis)
   */
  Rot3AttitudeFactor(Key key, const Unit3& nZ, const SharedNoiseModel& model,
      const Unit3& bRef = Unit3(0, 0, 1)) :
      Base(model, key), AttitudeFactor(nZ, bRef) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /** equals */
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /** vector of errors */
  Vector evaluateError(const Rot3& nRb, OptionalMatrixType H) const override {
    return attitudeError(nRb, H);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
    ar & boost::serialization::make_nvp("AttitudeFactor",
        boost::serialization::base_object<AttitudeFactor>(*this));
  }

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

/// traits
template<> struct traits<Rot3AttitudeFactor> : public Testable<Rot3AttitudeFactor> {};

/**
 * Version of AttitudeFactor for Pose3
 * @ingroup navigation
 */
class GTSAM_EXPORT Pose3AttitudeFactor: public NoiseModelFactorN<Pose3>,
    public AttitudeFactor {

  typedef NoiseModelFactorN<Pose3> Base;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<Pose3AttitudeFactor> shared_ptr;

  /// Typedef to this class
  typedef Pose3AttitudeFactor This;

  /** default constructor - only use for serialization */
  Pose3AttitudeFactor() {
  }

  ~Pose3AttitudeFactor() override {
  }

  /**
   * @brief Constructor
   * @param key of the Pose3 variable that will be constrained
   * @param nZ measured direction in navigation frame
   * @param model Gaussian noise model
   * @param bRef reference direction in body frame (default Z-axis)
   */
  Pose3AttitudeFactor(Key key, const Unit3& nZ, const SharedNoiseModel& model,
      const Unit3& bRef = Unit3(0, 0, 1)) :
      Base(model, key), AttitudeFactor(nZ, bRef) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /** equals */
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /** vector of errors */
  Vector evaluateError(const Pose3& nTb, OptionalMatrixType H) const override {
    Vector e = attitudeError(nTb.rotation(), H);
    if (H) {
      Matrix H23 = *H;
      *H = Matrix::Zero(2,6);
      H->block<2,3>(0,0) = H23;
    }
    return e;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
    ar & boost::serialization::make_nvp("AttitudeFactor",
        boost::serialization::base_object<AttitudeFactor>(*this));
  }

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

/// traits
template<> struct traits<Pose3AttitudeFactor> : public Testable<Pose3AttitudeFactor> {};

} /// namespace gtsam

