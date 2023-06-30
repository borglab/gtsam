/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Similarity2.h
 * @brief  Implementation of Similarity2 transform
 * @author John Lambert, Varun Agrawal
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/dllexport.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>

namespace gtsam {

// Forward declarations
class Pose2;

/**
 * 2D similarity transform
 */
class GTSAM_EXPORT Similarity2 : public LieGroup<Similarity2, 4> {
  /// @name Pose Concept
  /// @{
  typedef Rot2 Rotation;
  typedef Point2 Translation;
  /// @}

 private:
  Rot2 R_;
  Point2 t_;
  double s_;

 public:
  /// @name Constructors
  /// @{

  /// Default constructor
  Similarity2();

  /// Construct pure scaling
  Similarity2(double s);

  /// Construct from GTSAM types
  Similarity2(const Rot2& R, const Point2& t, double s);

  /// Construct from Eigen types
  Similarity2(const Matrix2& R, const Vector2& t, double s);

  /// Construct from matrix [R t; 0 s^-1]
  Similarity2(const Matrix3& T);

  /// @}
  /// @name Testable
  /// @{

  /// Compare with tolerance
  bool equals(const Similarity2& sim, double tol) const;

  /// Exact equality
  bool operator==(const Similarity2& other) const;

  /// Print with optional string
  void print(const std::string& s) const;

  friend std::ostream& operator<<(std::ostream& os, const Similarity2& p);

  /// @}
  /// @name Group
  /// @{

  /// Return an identity transform
  static Similarity2 Identity();

  /// Composition
  Similarity2 operator*(const Similarity2& S) const;

  /// Return the inverse
  Similarity2 inverse() const;

  /// @}
  /// @name Group action on Point2
  /// @{

  /// Action on a point p is s*(R*p+t)
  Point2 transformFrom(const Point2& p) const;

  /**
   * Action on a pose T.
   * |Rs  ts|   |R t|   |Rs*R Rs*t+ts|
   * |0  1/s| * |0 1| = | 0      1/s |, the result is still a Sim2 object.
   * To retrieve a Pose2, we normalized the scale value into 1.
   * |Rs*R Rs*t+ts|   |Rs*R s(Rs*t+ts)|
   * | 0      1/s | = |  0       1    |
   *
   * This group action satisfies the compatibility condition.
   * For more details, refer to: https://en.wikipedia.org/wiki/Group_action
   */
  Pose2 transformFrom(const Pose2& T) const;

  /* syntactic sugar for transformFrom */
  Point2 operator*(const Point2& p) const;

  /**
   *  Create Similarity2 by aligning at least two point pairs
   */
  static Similarity2 Align(const Point2Pairs& abPointPairs);

  /**
   * Create the Similarity2 object that aligns at least two pose pairs.
   * Each pair is of the form (aTi, bTi).
   * Given a list of pairs in frame a, and a list of pairs in frame b,
   Align()
   * will compute the best-fit Similarity2 aSb transformation to align them.
   * First, the rotation aRb will be computed as the average (Karcher mean)
   of
   * many estimates aRb (from each pair). Afterwards, the scale factor will
   be computed
   * using the algorithm described here:
   * http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf
   */
  static Similarity2 Align(const Pose2Pairs& abPosePairs);

  /// @}
  /// @name Lie Group
  /// @{

  /**
   * Log map at the identity
   * \f$ [t_x, t_y, \delta, \lambda] \f$
   */
  static Vector4 Logmap(const Similarity2& S,  //
                        OptionalJacobian<4, 4> Hm = {});

  /// Exponential map at the identity
  static Similarity2 Expmap(const Vector4& v,  //
                            OptionalJacobian<4, 4> Hm = {});

  /// Chart at the origin
  struct GTSAM_EXPORT ChartAtOrigin {
    static Similarity2 Retract(const Vector4& v,
                               ChartJacobian H = {}) {
      return Similarity2::Expmap(v, H);
    }
    static Vector4 Local(const Similarity2& other,
                         ChartJacobian H = {}) {
      return Similarity2::Logmap(other, H);
    }
  };

  /// Project from one tangent space to another
  Matrix4 AdjointMap() const;

  using LieGroup<Similarity2, 4>::inverse;

  /// @}
  /// @name Standard interface
  /// @{

  /// Calculate 4*4 matrix group equivalent
  Matrix3 matrix() const;

  /// Return a GTSAM rotation
  Rot2 rotation() const { return R_; }

  /// Return a GTSAM translation
  Point2 translation() const { return t_; }

  /// Return the scale
  double scale() const { return s_; }

  /// Dimensionality of tangent space = 4 DOF - used to autodetect sizes
  inline static size_t Dim() { return 4; }

  /// Dimensionality of tangent space = 4 DOF
  inline size_t dim() const { return 4; }

  /// @}
};

template <>
struct traits<Similarity2> : public internal::LieGroup<Similarity2> {};

template <>
struct traits<const Similarity2> : public internal::LieGroup<Similarity2> {};

}  // namespace gtsam
