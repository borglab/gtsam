/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Similarity3.h
 * @brief  Implementation of Similarity3 transform
 * @author Paul Drews
 * @author John Lambert
 */

#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/dllexport.h>


namespace gtsam {

// Forward declarations
class Pose3;

/**
 * 3D similarity transform
 */
class Similarity3: public LieGroup<Similarity3, 7> {

  /// @name Pose Concept
  /// @{
  typedef Rot3 Rotation;
  typedef Point3 Translation;
  /// @}

private:
  Rot3 R_;
  Point3 t_;
  double s_;

public:

  /// @name Constructors
  /// @{

  /// Default constructor
  GTSAM_EXPORT Similarity3();

  /// Construct pure scaling
  GTSAM_EXPORT Similarity3(double s);

  /// Construct from GTSAM types
  GTSAM_EXPORT Similarity3(const Rot3& R, const Point3& t, double s);

  /// Construct from Eigen types
  GTSAM_EXPORT Similarity3(const Matrix3& R, const Vector3& t, double s);

  /// Construct from matrix [R t; 0 s^-1]
  GTSAM_EXPORT Similarity3(const Matrix4& T);

  /// @}
  /// @name Testable
  /// @{

  /// Compare with tolerance
  GTSAM_EXPORT bool equals(const Similarity3& sim, double tol) const;

  /// Exact equality
  GTSAM_EXPORT bool operator==(const Similarity3& other) const;

  /// Print with optional string
  GTSAM_EXPORT void print(const std::string& s) const;

  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Similarity3& p);

  /// @}
  /// @name Group
  /// @{

  /// Return an identity transform
  GTSAM_EXPORT static Similarity3 identity();

  /// Composition
  GTSAM_EXPORT Similarity3 operator*(const Similarity3& S) const;

  /// Return the inverse
  GTSAM_EXPORT Similarity3 inverse() const;

  /// @}
  /// @name Group action on Point3
  /// @{

  /// Action on a point p is s*(R*p+t)
  GTSAM_EXPORT Point3 transformFrom(const Point3& p, //
      OptionalJacobian<3, 7> H1 = boost::none, //
      OptionalJacobian<3, 3> H2 = boost::none) const;

  /** 
   * Action on a pose T.
   * |Rs  ts|   |R t|   |Rs*R Rs*t+ts| 
   * |0  1/s| * |0 1| = | 0      1/s |, the result is still a Sim3 object.
   * To retrieve a Pose3, we normalized the scale value into 1.
   * |Rs*R Rs*t+ts|   |Rs*R s(Rs*t+ts)|
   * | 0      1/s | = |  0       1    |
   * 
   * This group action satisfies the compatibility condition. 
   * For more details, refer to: https://en.wikipedia.org/wiki/Group_action
   */
  GTSAM_EXPORT Pose3 transformFrom(const Pose3& T) const;

  /** syntactic sugar for transformFrom */
  GTSAM_EXPORT Point3 operator*(const Point3& p) const;

  /**
   *  Create Similarity3 by aligning at least three point pairs
   */
  GTSAM_EXPORT static Similarity3 Align(const std::vector<Point3Pair>& abPointPairs);
  
  /**
   * Create the Similarity3 object that aligns at least two pose pairs.
   * Each pair is of the form (aTi, bTi).
   * Given a list of pairs in frame a, and a list of pairs in frame b, Align()
   * will compute the best-fit Similarity3 aSb transformation to align them.
   * First, the rotation aRb will be computed as the average (Karcher mean) of
   * many estimates aRb (from each pair). Afterwards, the scale factor will be computed
   * using the algorithm described here:
   * http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf
   */
  GTSAM_EXPORT static Similarity3 Align(const std::vector<Pose3Pair>& abPosePairs);

  /// @}
  /// @name Lie Group
  /// @{

  /** Log map at the identity
   * \f$ [R_x,R_y,R_z, t_x, t_y, t_z, \lambda] \f$
   */
  GTSAM_EXPORT static Vector7 Logmap(const Similarity3& s, //
      OptionalJacobian<7, 7> Hm = boost::none);

  /** Exponential map at the identity
   */
  GTSAM_EXPORT static Similarity3 Expmap(const Vector7& v, //
      OptionalJacobian<7, 7> Hm = boost::none);

  /// Chart at the origin
  struct ChartAtOrigin {
    static Similarity3 Retract(const Vector7& v, ChartJacobian H = boost::none) {
      return Similarity3::Expmap(v, H);
    }
    static Vector7 Local(const Similarity3& other, ChartJacobian H = boost::none) {
      return Similarity3::Logmap(other, H);
    }
  };

  using LieGroup<Similarity3, 7>::inverse;

  /**
   * wedge for Similarity3:
   * @param xi 7-dim twist (w,u,lambda) where
   * @return 4*4 element of Lie algebra that can be exponentiated
   * TODO(frank): rename to Hat, make part of traits
   */
  GTSAM_EXPORT static Matrix4 wedge(const Vector7& xi);

  /// Project from one tangent space to another
  GTSAM_EXPORT  Matrix7 AdjointMap() const;

  /// @}
  /// @name Standard interface
  /// @{

  /// Calculate 4*4 matrix group equivalent
  GTSAM_EXPORT const Matrix4 matrix() const;

  /// Return a GTSAM rotation
  const Rot3& rotation() const {
    return R_;
  }

  /// Return a GTSAM translation
  const Point3& translation() const {
    return t_;
  }

  /// Return the scale
  double scale() const {
    return s_;
  }

  /// Convert to a rigid body pose (R, s*t)
  /// TODO(frank): why is this here? Red flag! Definitely don't have it as a cast.
  GTSAM_EXPORT operator Pose3() const;

  /// Dimensionality of tangent space = 7 DOF - used to autodetect sizes
  inline static size_t Dim() {
    return 7;
  }

  /// Dimensionality of tangent space = 7 DOF
  inline size_t dim() const {
    return 7;
  }

  /// @}
  /// @name Helper functions
  /// @{

private:
  /// Calculate expmap and logmap coefficients.
  static Matrix3 GetV(Vector3 w, double lambda);

  /// @}
};

template<>
inline Matrix wedge<Similarity3>(const Vector& xi) {
  return Similarity3::wedge(xi);
}

template<>
struct traits<Similarity3> : public internal::LieGroup<Similarity3> {};

template<>
struct traits<const Similarity3> : public internal::LieGroup<Similarity3> {};

} // namespace gtsam
