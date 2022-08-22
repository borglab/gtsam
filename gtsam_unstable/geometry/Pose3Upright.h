/**
 * @file Pose3Upright.h
 *
 * @brief Variation of a Pose3 in which the rotation is constained to purely yaw
 * This state is essentially a Pose2 with a z component, with conversions to
 * higher and lower dimensional states.
 *
 * @date Jan 24, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsam {

/**
 * A 3D Pose with fixed pitch and roll
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_UNSTABLE_EXPORT Pose3Upright {
public:
  static const size_t dimension = 4;

protected:

  Pose2 T_;
  double z_;

public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor initializes at origin
  Pose3Upright() : z_(0.0) {}

  /// Copy constructor
  Pose3Upright(const Pose3Upright& x) : T_(x.T_), z_(x.z_) {}
  Pose3Upright(const Rot2& bearing, const Point3& t);
  Pose3Upright(double x, double y, double z, double theta);
  Pose3Upright(const Pose2& pose, double z);

  /// Down-converts from a full Pose3
  Pose3Upright(const Pose3& fullpose);

  /// @}
  /// @name Testable
  /// @{

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** assert equality up to a tolerance */
  bool equals(const Pose3Upright& pose, double tol = 1e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  double x() const { return T_.x(); }
  double y() const { return T_.y(); }
  double z() const { return z_; }
  double theta() const { return T_.theta(); }

  Point2 translation2() const;
  Point3 translation() const;
  Rot2 rotation2() const;
  Rot3 rotation() const;
  Pose2 pose2() const;
  Pose3 pose() const;

  /// @}
  /// @name Manifold
  /// @{

  /// Dimensionality of tangent space = 4 DOF - used to autodetect sizes
  inline static size_t Dim() { return dimension; }

  /// Dimensionality of tangent space = 4 DOF
  inline size_t dim() const { return dimension; }

  /// Retraction from R^4 to Pose3Upright manifold neighborhood around current pose
  /// Tangent space parameterization is [x y z theta]
  Pose3Upright retract(const Vector& v) const;

  /// Local 3D coordinates of Pose3Upright manifold neighborhood around current pose
  Vector localCoordinates(const Pose3Upright& p2) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static Pose3Upright Identity() { return Pose3Upright(); }

  /// inverse transformation with derivatives
  Pose3Upright inverse(boost::optional<Matrix&> H1=boost::none) const;

  ///compose this transformation onto another (first *this and then p2)
  Pose3Upright compose(const Pose3Upright& p2,
      boost::optional<Matrix&> H1=boost::none,
      boost::optional<Matrix&> H2=boost::none) const;

  /// compose syntactic sugar
  inline Pose3Upright operator*(const Pose3Upright& T) const { return compose(T); }

  /**
   * Return relative pose between p1 and p2, in p1 coordinate frame
   * as well as optionally the derivatives
   */
  Pose3Upright between(const Pose3Upright& p2,
      boost::optional<Matrix&> H1=boost::none,
      boost::optional<Matrix&> H2=boost::none) const;

  /// @}
  /// @name Lie Group
  /// @{

  /// Exponential map at identity - create a rotation from canonical coordinates
  static Pose3Upright Expmap(const Vector& xi);

  /// Log map at identity - return the canonical coordinates of this rotation
  static Vector Logmap(const Pose3Upright& p);

  /// @}

private:

  // Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(T_);
    ar & BOOST_SERIALIZATION_NVP(z_);
  }

}; // \class Pose3Upright

template<>
struct traits<Pose3Upright> : public internal::Manifold<Pose3Upright> {};


} // \namespace gtsam
