/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2.h
 * @brief Calibration of a camera with radial distortion, calculations in base
 * class Cal3DS2_Base
 * @date Feb 28, 2010
 * @author ydjian
 * @autho Varun Agrawal
 */

#pragma once

#include <gtsam/geometry/Cal3DS2_Base.h>
#include <memory>

namespace gtsam {

/**
 * @brief Calibration of a camera with radial distortion that also supports
 * Lie-group behaviors for optimization.
 * \sa Cal3DS2_Base
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3DS2 : public Cal3DS2_Base {
  using Base = Cal3DS2_Base;

 public:
  enum { dimension = 9 };

  ///< shared pointer to stereo calibration object
  using shared_ptr = std::shared_ptr<Cal3DS2>;

  /// @name Standard Constructors
  /// @{

  /// Default Constructor with only unit focal length
  Cal3DS2() = default;

  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1,
          double k2, double p1 = 0.0, double p2 = 0.0, double tol = 1e-5)
      : Base(fx, fy, s, u0, v0, k1, k2, p1, p2, tol) {}

  ~Cal3DS2() override {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  Cal3DS2(const Vector9& v) : Base(v) {}

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3DS2& cal);

  /// print with optional string
  void print(const std::string& s = "") const override;

  /// assert equality up to a tolerance
  bool equals(const Cal3DS2& K, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Given delta vector, update calibration
  Cal3DS2 retract(const Vector& d) const;

  /// Given a different calibration, calculate update to obtain it
  Vector localCoordinates(const Cal3DS2& T2) const;

  /// Return dimensions of calibration manifold object
  size_t dim() const override { return Dim(); }

  /// Return dimensions of calibration manifold object
  inline static size_t Dim() { return dimension; }

  /// @}
  /// @name Clone
  /// @{

  /// @return a deep copy of this object
  std::shared_ptr<Base> clone() const override {
    return std::shared_ptr<Base>(new Cal3DS2(*this));
  }

  /// @}

 private:
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3DS2", boost::serialization::base_object<Cal3DS2_Base>(*this));
  }
#endif

  /// @}
};

template <>
struct traits<Cal3DS2> : public internal::Manifold<Cal3DS2> {};

template <>
struct traits<const Cal3DS2> : public internal::Manifold<Cal3DS2> {};
}
