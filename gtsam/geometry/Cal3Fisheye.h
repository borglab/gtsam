/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Fisheye.h
 * @brief Calibration of a fisheye camera, calculations in base class Cal3Fisheye_Base
 * @date Apr 8, 2020
 * @author ghaggin
 */

#pragma once

#include <gtsam/geometry/Cal3Fisheye_Base.h>

namespace gtsam {

/**
 * @brief Calibration of a fisheye camera that also supports
 * Lie-group behaviors for optimization.
 * \sa Cal3Fisheye_Base
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3Fisheye : public Cal3Fisheye_Base {

  typedef Cal3Fisheye_Base Base;

public:

  enum { dimension = 9 };

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3Fisheye() : Base() {}

  Cal3Fisheye(const double fx, const double fy, const double s, const double u0, const double v0,
      const double k1, const double k2, const double k3, const double k4) :
        Base(fx, fy, s, u0, v0, k1, k2, k3, k4) {}

  virtual ~Cal3Fisheye() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  Cal3Fisheye(const Vector &v) : Base(v) {}

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  virtual void print(const std::string& s = "") const ;

  /// assert equality up to a tolerance
  bool equals(const Cal3Fisheye& K, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3Fisheye retract(const Vector& d) const ;

  /// Given a different calibration, calculate update to obtain it
  Vector localCoordinates(const Cal3Fisheye& T2) const ;

  /// Return dimensions of calibration manifold object
  virtual size_t dim() const { return 9 ; } //TODO: make a final dimension variable (also, usually size_t in other classes e.g. Pose2)

  /// Return dimensions of calibration manifold object
  static size_t Dim() { return 9; }  //TODO: make a final dimension variable

  /// @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  virtual boost::shared_ptr<Base> clone() const {
    return boost::shared_ptr<Base>(new Cal3Fisheye(*this));
  }

  /// @}


private:

  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/)
  {
    ar & boost::serialization::make_nvp("Cal3Fisheye",
        boost::serialization::base_object<Cal3Fisheye_Base>(*this));
  }

  /// @}

};

template<>
struct traits<Cal3Fisheye> : public internal::Manifold<Cal3Fisheye> {};

template<>
struct traits<const Cal3Fisheye> : public internal::Manifold<Cal3Fisheye> {};

}

