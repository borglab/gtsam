/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Bundler.h
 * @brief Calibration used by Bundler
 * @date Sep 25, 2010
 * @author Yong Dian Jian
 */

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief Calibration used by Bundler
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3Bundler : public DerivedValue<Cal3Bundler> {

private:
  double f_; ///< focal length
  double k1_, k2_;///< radial distortion

public:

  Matrix K() const;///< Standard 3*3 calibration matrix
  Vector k() const;///< Radial distortion parameters

  Vector vector() const;

  /// @name Standard Constructors
  /// @{

  /// Default constructor
  Cal3Bundler();

  /// Constructor
  Cal3Bundler(const double f, const double k1, const double k2);

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// Construct from vector
  Cal3Bundler(const Vector &v);

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const Cal3Bundler& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * convert intrinsic coordinates xy to image coordinates uv
   * @param p point in intrinsic coordinates
   * @param Dcal optional 2*3 Jacobian wrpt CalBundler parameters
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
   * @return point in image coordinates
   */
  Point2 uncalibrate(const Point2& p,
      boost::optional<Matrix&> Dcal = boost::none,
      boost::optional<Matrix&> Dp = boost::none) const;

  /// 2*2 Jacobian of uncalibrate with respect to intrinsic coordinates
  Matrix D2d_intrinsic(const Point2& p) const;

  /// 2*3 Jacobian of uncalibrate wrpt CalBundler parameters
  Matrix D2d_calibration(const Point2& p) const;

  /// 2*5 Jacobian of uncalibrate wrpt both intrinsic and calibration
  Matrix D2d_intrinsic_calibration(const Point2& p) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Update calibration with tangent space delta
  Cal3Bundler retract(const Vector& d) const;

  /// Calculate local coordinates to another calibration
  Vector localCoordinates(const Cal3Bundler& T2) const;

  /// dimensionality
  virtual size_t dim() const {return 3;}

  /// dimensionality
  static size_t Dim() {return 3;}

private:

  /// @}
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & boost::serialization::make_nvp("Cal3Bundler",
        boost::serialization::base_object<Value>(*this));
    ar & BOOST_SERIALIZATION_NVP(f_);
    ar & BOOST_SERIALIZATION_NVP(k1_);
    ar & BOOST_SERIALIZATION_NVP(k2_);
  }

  /// @}

};

} // namespace gtsam
