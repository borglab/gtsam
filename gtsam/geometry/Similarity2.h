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
 * @author John Lambert
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
class Similarity2 : public LieGroup<Similarity2, 4> {
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
  GTSAM_EXPORT Similarity2();

  /// Construct pure scaling
  GTSAM_EXPORT Similarity2(double s);

  /// Construct from GTSAM types
  GTSAM_EXPORT Similarity2(const Rot2& R, const Point2& t, double s);

  /// Construct from Eigen types
  GTSAM_EXPORT Similarity2(const Matrix2& R, const Vector2& t, double s);

  /// Construct from matrix [R t; 0 s^-1]
  GTSAM_EXPORT Similarity2(const Matrix3& T);

  /// @}
  /// @name Testable
  /// @{

  /// Compare with tolerance
  GTSAM_EXPORT bool equals(const Similarity2& sim, double tol) const;

  /// Exact equality
  GTSAM_EXPORT bool operator==(const Similarity2& other) const;

  /// Print with optional string
  GTSAM_EXPORT void print(const std::string& s) const;

  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Similarity2& p);

  /// @}
  /// @name Group
  /// @{

  /// Return an identity transform
  GTSAM_EXPORT static Similarity2 identity();

  /// Composition
  GTSAM_EXPORT Similarity2 operator*(const Similarity2& S) const;

  /// Return the inverse
  GTSAM_EXPORT Similarity2 inverse() const;

  /// @}
  /// @name Group action on Point2
  /// @{

  /// Action on a point p is s*(R*p+t)
  GTSAM_EXPORT Point2 transformFrom(const Point2& p) const;

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
  GTSAM_EXPORT Pose2 transformFrom(const Pose2& T) const;

  /* syntactic sugar for transformFrom */
  GTSAM_EXPORT Point2 operator*(const Point2& p) const;

  /**
   *  Create Similarity2 by aligning at least two point pairs
   */
  GTSAM_EXPORT static Similarity2 Align(const Point2Pairs& abPointPairs);

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
  GTSAM_EXPORT static Similarity2 Align(
      const std::vector<Pose2Pair>& abPosePairs);

  /// @}
  /// @name Lie Group
  /// @{

  using LieGroup<Similarity2, 4>::inverse;

  /// @}
  /// @name Standard interface
  /// @{

  /// Calculate 4*4 matrix group equivalent
  GTSAM_EXPORT Matrix3 matrix() const;

  /// Return a GTSAM rotation
  Rot2 rotation() const { return R_; }

  /// Return a GTSAM translation
  Point2 translation() const { return t_; }

  /// Return the scale
  double scale() const { return s_; }

  /// Convert to a rigid body pose (R, s*t)
  GTSAM_EXPORT operator Pose2() const;

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