/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3.h
 * @brief  Common code for all Calibration models.
 * @author Varun Agrawal
 */

/**
 * @ingroup geometry
 */

#pragma once

#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * Function which makes use of the Implicit Function Theorem to compute the
 * Jacobians of `calibrate` using `uncalibrate`.
 * This is useful when there are iterative operations in the `calibrate`
 * function which make computing jacobians difficult.
 *
 * Given f(pi, pn) = uncalibrate(pn) - pi, and g(pi) = calibrate, we can
 * easily compute the Jacobians:
 * df/pi = -I (pn and pi are independent args)
 * Dp = -inv(H_uncal_pn) * df/pi = -inv(H_uncal_pn) * (-I) = inv(H_uncal_pn)
 * Dcal = -inv(H_uncal_pn) * df/K = -inv(H_uncal_pn) * H_uncal_K
 *
 * @tparam Cal Calibration model.
 * @tparam Dim The number of parameters in the calibration model.
 * @param p Calibrated point.
 * @param Dcal optional 2*p Jacobian wrpt `p` Cal3DS2 parameters.
 * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates.
 */
template <typename Cal, size_t Dim>
void calibrateJacobians(const Cal& calibration, const Point2& pn,
                        OptionalJacobian<2, Dim> Dcal = {},
                        OptionalJacobian<2, 2> Dp = {}) {
  if (Dcal || Dp) {
    Eigen::Matrix<double, 2, Dim> H_uncal_K;
    Matrix22 H_uncal_pn, H_uncal_pn_inv;

    // Compute uncalibrate Jacobians
    calibration.uncalibrate(pn, Dcal ? &H_uncal_K : nullptr, H_uncal_pn);

    H_uncal_pn_inv = H_uncal_pn.inverse();

    if (Dp) *Dp = H_uncal_pn_inv;
    if (Dcal) *Dcal = -H_uncal_pn_inv * H_uncal_K;
  }
}

/**
 * @brief Common base class for all calibration models.
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3 {
 protected:
  double fx_ = 1.0f, fy_ = 1.0f;  ///< focal length
  double s_ = 0.0f;               ///< skew
  double u0_ = 0.0f, v0_ = 0.0f;  ///< principal point

 public:
  enum { dimension = 5 };
  ///< shared pointer to calibration object
  using shared_ptr = std::shared_ptr<Cal3>;

  /// @name Standard Constructors
  /// @{

  /// Create a default calibration that leaves coordinates unchanged
  Cal3() = default;

  /// constructor from doubles
  Cal3(double fx, double fy, double s, double u0, double v0)
      : fx_(fx), fy_(fy), s_(s), u0_(u0), v0_(v0) {}

  /// constructor from vector
  Cal3(const Vector5& d)
      : fx_(d(0)), fy_(d(1)), s_(d(2)), u0_(d(3)), v0_(d(4)) {}

  /**
   * Easy constructor, takes fov in degrees, asssumes zero skew, unit aspect
   * @param fov field of view in degrees
   * @param w image width
   * @param h image height
   */
  Cal3(double fov, int w, int h);

  /// Virtual destructor
  virtual ~Cal3() {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  /**
   * Load calibration parameters from `calibration_info.txt` file located in
   * `path` directory.
   *
   * The contents of calibration file should be the 5 parameters in order:
   * `fx, fy, s, u0, v0`
   *
   * @param path path to directory containing `calibration_info.txt`.
   */
  Cal3(const std::string& path);

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3& cal);

  /// print with optional string
  virtual void print(const std::string& s = "") const;

  /// Check if equal up to specified tolerance
  bool equals(const Cal3& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// focal length x
  inline double fx() const { return fx_; }

  /// focal length y
  inline double fy() const { return fy_; }

  /// aspect ratio
  inline double aspectRatio() const { return fx_ / fy_; }

  /// skew
  inline double skew() const { return s_; }

  /// image center in x
  inline double px() const { return u0_; }

  /// image center in y
  inline double py() const { return v0_; }

  /// return the principal point
  Point2 principalPoint() const { return Point2(u0_, v0_); }

  /// vectorized form (column-wise)
  Vector5 vector() const {
    Vector5 v;
    v << fx_, fy_, s_, u0_, v0_;
    return v;
  }

  /// return calibration matrix K
  virtual Matrix3 K() const {
    Matrix3 K;
    K << fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
    return K;
  }

  /// Return inverted calibration matrix inv(K)
  Matrix3 inverse() const;

  /// return DOF, dimensionality of tangent space
  inline virtual size_t dim() const { return Dim(); }

  /// return DOF, dimensionality of tangent space
  inline static size_t Dim() { return dimension; }

  /// @}
  /// @name Advanced Interface
  /// @{

 private:
  /// Serialization function
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(fx_);
    ar& BOOST_SERIALIZATION_NVP(fy_);
    ar& BOOST_SERIALIZATION_NVP(s_);
    ar& BOOST_SERIALIZATION_NVP(u0_);
    ar& BOOST_SERIALIZATION_NVP(v0_);
  }
#endif

  /// @}
};

}  // \ namespace gtsam
