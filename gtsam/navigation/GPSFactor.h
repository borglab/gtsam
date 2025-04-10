/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactor.h
 *  @author Frank Dellaert
 *  @brief  Header file for GPS factor
 *  @date   January 22, 2014
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * Prior on position in a Cartesian frame.
 * If there exists a non-zero lever arm between body frame and GPS
 * antenna, instead use GPSFactorArm.
 * Possibilities include:
 *   ENU: East-North-Up navigation frame at some local origin
 *   NED: North-East-Down navigation frame at some local origin
 *   ECEF: Earth-centered Earth-fixed, origin at Earth's center
 * See Farrell08book or e.g. http://www.dirsig.org/docs/new/coordinates.html
 * @ingroup navigation
 */
class GTSAM_EXPORT GPSFactor: public NoiseModelFactorN<Pose3> {

private:

  typedef NoiseModelFactorN<Pose3> Base;

  Point3 nT_; ///< Position measurement in cartesian coordinates (navigation frame)

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<GPSFactor> shared_ptr;

  /// Typedef to this class
  typedef GPSFactor This;

  /** default constructor - only use for serialization */
  GPSFactor(): nT_(0, 0, 0) {}

  ~GPSFactor() override {}

  /**
   * @brief Constructor from a measurement in a Cartesian frame.
   * Use GeographicLib to convert from geographic (latitude and longitude) coordinates
   * @param key of the Pose3 variable that will be constrained
   * @param gpsIn measurement already in correct coordinates
   * @param model Gaussian noise model
   */
  GPSFactor(Key key, const Point3& gpsIn, const SharedNoiseModel& model) :
      Base(model, key), nT_(gpsIn) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& nTb, OptionalMatrixType H) const override;

  /// return the measurement, in the navigation frame
  inline const Point3 & measurementIn() const {
    return nT_;
  }

  /**
   *  Convenience function to estimate state at time t, given two GPS
   *  readings (in local NED Cartesian frame) bracketing t
   *  Assumes roll is zero, calculates yaw and pitch from NED1->NED2 vector.
   */
  static std::pair<Pose3, Vector3> EstimateState(double t1, const Point3& NED1,
      double t2, const Point3& NED2, double timestamp);

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
#endif
};

/**
 * Version of GPSFactor (for Pose3) with lever arm between GPS and Body frame.
 * Because the actual location of the antenna depends on both position and
 * attitude, providing a lever arm makes components of the attitude observable
 * and accounts for position measurement vs state discrepancies while turning.
 * @ingroup navigation
 */
class GTSAM_EXPORT GPSFactorArm: public NoiseModelFactorN<Pose3> {

private:

  typedef NoiseModelFactorN<Pose3> Base;

  Point3 nT_;  ///< Position measurement in cartesian coordinates (navigation frame)
  Point3 bL_;  ///< bL_ is a lever arm in the body frame, denoting the 3D
               ///< position of the GPS antenna in the body frame

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<GPSFactorArm> shared_ptr;

  /// Typedef to this class
  typedef GPSFactorArm This;

  /// default constructor - only use for serialization
  GPSFactorArm():nT_(0, 0, 0), bL_(0, 0, 0) {}

  ~GPSFactorArm() override {}

  /** Constructor from a measurement in a Cartesian frame.
   * @param key       key of the Pose3 variable related to this measurement
   * @param gpsIn     gps measurement, in Cartesian navigation frame
   * @param leverArm  translation from the body frame origin to the gps antenna, in body frame
   * @param model     Gaussian noise model
  */
  GPSFactorArm(Key key, const Point3& gpsIn, const Point3& leverArm, const SharedNoiseModel& model) :
      Base(model, key), nT_(gpsIn), bL_(leverArm) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& nTb, OptionalMatrixType H) const override;

  /// return the measurement, in the navigation frame
  inline const Point3 & measurementIn() const {
    return nT_;
  }

  /// return the lever arm, a position in the body frame
  inline const Point3 & leverArm() const {
    return bL_;
  }

};

/**
 * Version of GPSFactorArm (for Pose3) with unknown lever arm between GPS and
 * Body frame. Because the actual location of the antenna depends on both
 * position and attitude, providing a lever arm makes components of the attitude
 * observable and accounts for position measurement vs state discrepancies while
 * turning.
 * @ingroup navigation
 */
class GTSAM_EXPORT GPSFactorArmCalib: public NoiseModelFactorN<Pose3, Point3> {

private:

  typedef NoiseModelFactorN<Pose3, Point3> Base;

  Point3 nT_;  ///< Position measurement in cartesian coordinates (navigation frame)

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<GPSFactorArmCalib> shared_ptr;

  /// Typedef to this class
  typedef GPSFactorArmCalib This;

  /// default constructor - only use for serialization
  GPSFactorArmCalib() : nT_(0, 0, 0) {}

  ~GPSFactorArmCalib() override {}

  /** Constructor from a measurement in a Cartesian frame.
   * @param key1      key of the Pose3 variable related to this measurement
   * @param key2      key of the Point3 variable related to the lever arm
   * @param gpsIn     gps measurement, in Cartesian navigation frame
   * @param leverArm  translation from the body frame origin to the gps antenna, in body frame
   * @param model     Gaussian noise model
  */
  GPSFactorArmCalib(Key key1, Key key2, const Point3& gpsIn, const SharedNoiseModel& model) :
      Base(model, key1, key2), nT_(gpsIn) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& nTb, const Point3& bL, OptionalMatrixType H1,
                       OptionalMatrixType H2) const override;

  /// return the measurement, in the navigation frame
  inline const Point3 & measurementIn() const {
    return nT_;
  }
};

/**
 * Version of GPSFactor for NavState, assuming zero lever arm between body frame
 * and GPS. If there exists a non-zero lever arm between body frame and GPS
 * antenna, instead use GPSFactor2Arm.
 * @ingroup navigation
 */
class GTSAM_EXPORT GPSFactor2: public NoiseModelFactorN<NavState> {

private:

  typedef NoiseModelFactorN<NavState> Base;

  Point3 nT_; ///< Position measurement in cartesian coordinates (navigation frame)

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<GPSFactor2> shared_ptr;

  /// Typedef to this class
  typedef GPSFactor2 This;

  /// default constructor - only use for serialization
  GPSFactor2():nT_(0, 0, 0) {}

  ~GPSFactor2() override {}

  /** Constructor from a measurement in a Cartesian frame.
   * @param key       key of the NavState variable related to this measurement
   * @param gpsIn     gps measurement, in Cartesian navigation frame
   * @param model     Gaussian noise model
  */
  GPSFactor2(Key key, const Point3& gpsIn, const SharedNoiseModel& model) :
      Base(model, key), nT_(gpsIn) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const NavState& nTb, OptionalMatrixType H) const override;

  /// return the measurement, in the navigation frame
  inline const Point3 & measurementIn() const {
    return nT_;
  }

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
#endif
};

/**
 * Version of GPSFactor2 with lever arm between GPS and Body frame.
 * Because the actual location of the antenna depends on both position and
 * attitude, providing a lever arm makes components of the attitude observable
 * and accounts for position measurement vs state discrepancies while turning.
 * @ingroup navigation
 */
class GTSAM_EXPORT GPSFactor2Arm: public NoiseModelFactorN<NavState> {

private:

  typedef NoiseModelFactorN<NavState> Base;

  Point3 nT_;  ///< Position measurement in cartesian coordinates (navigation frame)
  Point3 bL_;  ///< bL_ is a lever arm in the body frame, denoting the 3D
               ///< position of the GPS antenna in the body frame

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<GPSFactor2Arm> shared_ptr;

  /// Typedef to this class
  typedef GPSFactor2Arm This;

  /// default constructor - only use for serialization
  GPSFactor2Arm():nT_(0, 0, 0), bL_(0, 0, 0) {}

  ~GPSFactor2Arm() override {}

  /** Constructor from a measurement in a Cartesian frame.
   * @param key       key of the NavState variable related to this measurement
   * @param gpsIn     gps measurement, in Cartesian navigation frame
   * @param leverArm  translation from the body frame origin to the gps antenna, in body frame
   * @param model     noise model for the factor's residual
  */
  GPSFactor2Arm(Key key, const Point3& gpsIn, const Point3& leverArm, const SharedNoiseModel& model) :
      Base(model, key), nT_(gpsIn), bL_(leverArm) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const NavState& nTb, OptionalMatrixType H) const override;

  /// return the measurement, in the navigation frame
  inline const Point3 & measurementIn() const {
    return nT_;
  }

  /// return the lever arm, a position in the body frame
  inline const Point3 & leverArm() const {
    return bL_;
  }

};

/**
 * Version of GPSFactor2Arm for an unknown lever arm between GPS and Body frame.
 * Because the actual location of the antenna depends on both position and
 * attitude, providing a lever arm makes components of the attitude observable
 * and accounts for position measurement vs state discrepancies while turning.
 * @ingroup navigation
 */
class GTSAM_EXPORT GPSFactor2ArmCalib: public NoiseModelFactorN<NavState, Point3> {

private:

  typedef NoiseModelFactorN<NavState, Point3> Base;

  Point3 nT_; ///< Position measurement in cartesian coordinates (navigation frame)

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<GPSFactor2ArmCalib> shared_ptr;

  /// Typedef to this class
  typedef GPSFactor2ArmCalib This;

  /// default constructor - only use for serialization
  GPSFactor2ArmCalib():nT_(0, 0, 0) {}

  ~GPSFactor2ArmCalib() override {}

  /** Constructor from a measurement in a Cartesian frame.
   * @param key1      key of the NavState variable related to this measurement
   * @param key2      key of the Point3 variable related to the lever arm
   * @param gpsIn     gps measurement, in Cartesian navigation frame
   * @param model     Gaussian noise model
  */
  GPSFactor2ArmCalib(Key key1, Key key2, const Point3& gpsIn, const SharedNoiseModel& model) :
      Base(model, key1, key2), nT_(gpsIn) {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const NavState& nTb, const Point3& bL,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override;

  /// return the measurement, in the navigation frame
  inline const Point3 & measurementIn() const {
    return nT_;
  }
};

} /// namespace gtsam
