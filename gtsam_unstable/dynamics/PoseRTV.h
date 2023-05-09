/**
 * @file PoseRTV.h
 * @brief Pose3 with translational velocity
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/ProductLieGroup.h>

namespace gtsam {

/// Syntactic sugar to clarify components
typedef Vector3 Velocity3;

/**
 * Robot state for use with IMU measurements
 * - contains translation, translational velocity and rotation
 * TODO(frank): Alex should deprecate/move to project
 */
class GTSAM_UNSTABLE_EXPORT PoseRTV : public ProductLieGroup<Pose3,Velocity3> {
protected:

  typedef ProductLieGroup<Pose3,Velocity3> Base;
  typedef OptionalJacobian<9, 9> ChartJacobian;

public:

  // constructors - with partial versions
  PoseRTV() {}
  PoseRTV(const Point3& t, const Rot3& rot, const Velocity3& vel)
  : Base(Pose3(rot, t), vel) {}
  PoseRTV(const Rot3& rot, const Point3& t, const Velocity3& vel)
  : Base(Pose3(rot, t), vel) {}
  explicit PoseRTV(const Point3& t)
  : Base(Pose3(Rot3(), t),Vector3::Zero()) {}
  PoseRTV(const Pose3& pose, const Velocity3& vel)
  : Base(pose, vel) {}
  explicit PoseRTV(const Pose3& pose)
  : Base(pose,Vector3::Zero()) {}

  // Construct from Base
  PoseRTV(const Base& base)
  : Base(base) {}

  /** build from components - useful for data files */
  PoseRTV(double roll, double pitch, double yaw, double x, double y, double z,
      double vx, double vy, double vz);

  /** build from single vector - useful for Matlab - in RtV format */
  explicit PoseRTV(const Vector& v);

  // access
  const Pose3& pose() const { return first; }
  const Velocity3& v() const { return second; }
  const Point3& t() const { return pose().translation(); }
  const Rot3& R() const { return pose().rotation(); }

  // longer function names
  const Point3& translation() const { return pose().translation(); }
  const Rot3& rotation() const { return pose().rotation(); }
  const Velocity3& velocity() const { return second; }

  // Access to vector for ease of use with Matlab
  // and avoidance of Point3
  Vector vector() const;
  Vector translationVec() const { return pose().translation(); }
  const Velocity3& velocityVec() const { return velocity(); }

  // testable
  bool equals(const PoseRTV& other, double tol=1e-6) const;
  void print(const std::string& s="") const;

  /// @name Manifold
  /// @{
  using Base::dimension;
  using Base::dim;
  using Base::Dim;
  using Base::retract;
  using Base::localCoordinates;
  using Base::LocalCoordinates;
  /// @}

  /// @name measurement functions
  /// @{

  /** range between translations */
  double range(const PoseRTV& other,
               OptionalJacobian<1,9> H1={},
               OptionalJacobian<1,9> H2={}) const;
  /// @}

  /// @name IMU-specific
  /// @{

  /// Dynamics integrator for ground robots
  /// Always move from time 1 to time 2
  PoseRTV planarDynamics(double vel_rate, double heading_rate, double max_accel, double dt) const;

  /// Simulates flying robot with simple flight model
  /// Integrates state x1 -> x2 given controls
  /// x1 = {p1, r1, v1}, x2 = {p2, r2, v2}, all in global coordinates
  /// @return x2
  PoseRTV flyingDynamics(double pitch_rate, double heading_rate, double lift_control, double dt) const;

  /// General Dynamics update - supply control inputs in body frame
  PoseRTV generalDynamics(const Vector& accel, const Vector& gyro, double dt) const;

  /// Dynamics predictor for both ground and flying robots, given states at 1 and 2
  /// Always move from time 1 to time 2
  /// @return imu measurement, as [accel, gyro]
  Vector6 imuPrediction(const PoseRTV& x2, double dt) const;

  /// predict measurement and where Point3 for x2 should be, as a way
  /// of enforcing a velocity constraint
  /// This version splits out the rotation and velocity for x2
  Point3 translationIntegration(const Rot3& r2, const Velocity3& v2, double dt) const;

  /// predict measurement and where Point3 for x2 should be, as a way
  /// of enforcing a velocity constraint
  /// This version takes a full PoseRTV, but ignores the existing translation for x2
  inline Point3 translationIntegration(const PoseRTV& x2, double dt) const {
    return translationIntegration(x2.rotation(), x2.velocity(), dt);
  }

  /// @return a vector for Matlab compatibility
  inline Vector translationIntegrationVec(const PoseRTV& x2, double dt) const {
    return translationIntegration(x2, dt);
  }

  /**
   * Apply transform to this pose, with optional derivatives
   * equivalent to:
   * local = trans.transformFrom(global, Dtrans, Dglobal)
   *
   * Note: the transform jacobian convention is flipped
   */
  PoseRTV transformed_from(const Pose3& trans,
      ChartJacobian Dglobal = {},
      OptionalJacobian<9, 6> Dtrans = {}) const;

  /// @}
  /// @name Utility functions
  /// @{

  /// RRTMbn - Function computes the rotation rate transformation matrix from
  /// body axis rates to euler angle (global) rates
  static Matrix RRTMbn(const Vector3& euler);
  static Matrix RRTMbn(const Rot3& att);

  /// RRTMnb - Function computes the rotation rate transformation matrix from
  /// euler angle rates to body axis rates
  static Matrix RRTMnb(const Vector3& euler);
  static Matrix RRTMnb(const Rot3& att);
  /// @}

private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(first);
    ar & BOOST_SERIALIZATION_NVP(second);
  }
#endif
};


template<>
struct traits<PoseRTV> : public internal::LieGroup<PoseRTV> {};

// Define Range functor specializations that are used in RangeFactor
template <typename A1, typename A2> struct Range;

template<>
struct Range<PoseRTV, PoseRTV> : HasRange<PoseRTV, PoseRTV, double> {};

} // \namespace gtsam
