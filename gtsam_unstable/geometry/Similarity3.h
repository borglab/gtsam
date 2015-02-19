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
 */

#ifndef GTSAM_UNSTABLE_GEOMETRY_SIMILARITY3_H_
#define GTSAM_UNSTABLE_GEOMETRY_SIMILARITY3_H_


#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

/**
 * 3D similarity transform
 */
class Similarity3: public LieGroup<Similarity3, 7> {

  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Translation;

private:
  Rot3 R_;
  Point3 t_;
  double s_;

public:

  Similarity3();

  /// Construct pure scaling
  Similarity3(double s);

  /// Construct from GTSAM types
  Similarity3(const Rot3& R, const Point3& t, double s);

  /// Construct from Eigen types
  Similarity3(const Matrix3& R, const Vector3& t, double s);

  /// Return the translation
  const Vector3 t() const;

  /// Return the rotation matrix
  const Matrix3 R() const;

  static Similarity3 identity();

  static Vector7 Logmap(const Similarity3& s, OptionalJacobian<7, 7> Hm = boost::none);

  static Similarity3 Expmap(const Vector7& v, OptionalJacobian<7, 7> Hm = boost::none);

  bool operator==(const Similarity3& other) const;

  /// Compare with tolerance
  bool equals(const Similarity3& sim, double tol) const;

  Point3 transform_from(const Point3& p) const;

  Matrix7 AdjointMap() const;

  /** syntactic sugar for transform_from */
  inline Point3 operator*(const Point3& p) const;

  Similarity3 inverse() const;

  Similarity3 operator*(const Similarity3& T) const;

  void print(const std::string& s) const;

  /// Dimensionality of tangent space = 7 DOF - used to autodetect sizes
  inline static size_t Dim() {
    return 7;
  };

  /// Dimensionality of tangent space = 7 DOF
  inline size_t dim() const {
    return 7;
  };

  /// Return the rotation matrix
  Rot3 rotation() const;

  /// Return the translation
  Point3 translation() const;

  /// Return the scale
  double scale() const;

  /// Update Similarity transform via 7-dim vector in tangent space
  struct ChartAtOrigin {
    static Similarity3 Retract(const Vector7& v,  ChartJacobian H = boost::none);

  /// 7-dimensional vector v in tangent space that makes other = this->retract(v)
  static Vector7 Local(const Similarity3& other,  ChartJacobian H = boost::none);
  };

  using LieGroup<Similarity3, 7>::inverse; // version with derivative
};

template<>
struct traits<Similarity3> : public internal::LieGroupTraits<Similarity3> {};
}

#endif /* GTSAM_UNSTABLE_GEOMETRY_SIMILARITY3_H_ */
