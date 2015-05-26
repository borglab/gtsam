/**
 * @file PoseRTV.h
 * @brief Pose3 with translational velocity
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/base/dllexport.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

/// CRTP to construct the product Lie group of two other Lie groups, G and H
/// Assumes manifold structure from G and H, and binary constructor
template<class Derived, typename G, typename H>
class ProductLieGroup: public ProductManifold<Derived, G, H> {
  BOOST_CONCEPT_ASSERT((IsLieGroup<G>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<H>));
  typedef ProductManifold<Derived, G, H> Base;

public:
  enum {dimension = G::dimension + H::dimension};
  inline static size_t Dim() {return dimension;}
  inline size_t dim() const {return dimension;}

  typedef Eigen::Matrix<double, dimension, 1> TangentVector;

  /// Default constructor yields identity
  ProductLieGroup():Base(traits<G>::Identity(),traits<H>::Identity()) {}

  // Construct from two subgroup elements
  ProductLieGroup(const G& g, const H& h):Base(g,h) {}

  ProductLieGroup operator*(const Derived& other) const {
    return Derived(traits<G>::Compose(this->g(),other.g()), traits<H>::Compose(this->h(),other.h()));
  }
  ProductLieGroup inverse() const {
    return Derived(this->g().inverse(), this->h().inverse());
  }
};

// Define any direct product group to be a model of the multiplicative Group concept
template<class Derived, typename G, typename H>
struct traits<ProductLieGroup<Derived, G, H> > : internal::LieGroupTraits<
    ProductLieGroup<Derived, G, H> > {
};

/// Syntactic sugar to clarify components
typedef Point3 Velocity3;

/**
 * Robot state for use with IMU measurements
 * - contains translation, translational velocity and rotation
 */
class GTSAM_UNSTABLE_EXPORT PoseRTV : private ProductLieGroup<PoseRTV,Pose3,Velocity3> {
protected:

  typedef ProductLieGroup<PoseRTV,Pose3,Velocity3> Base;
  typedef OptionalJacobian<9, 9> ChartJacobian;

public:
  enum { dimension = 9 };

  // constructors - with partial versions
  PoseRTV() {}
  PoseRTV(const Point3& t, const Rot3& rot, const Velocity3& vel)
  : Base(Pose3(rot, t), vel) {}
  PoseRTV(const Rot3& rot, const Point3& t, const Velocity3& vel)
  : Base(Pose3(rot, t), vel) {}
  explicit PoseRTV(const Point3& t)
  : Base(Pose3(Rot3(), t),Velocity3()) {}
  PoseRTV(const Pose3& pose, const Velocity3& vel)
  : Base(pose, vel) {}
  explicit PoseRTV(const Pose3& pose)
  : Base(pose,Velocity3()) {}

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
  Vector translationVec() const { return pose().translation().vector(); }
  Vector velocityVec() const { return velocity().vector(); }

  // testable
  bool equals(const PoseRTV& other, double tol=1e-6) const;
  void print(const std::string& s="") const;

  // Manifold
  static size_t Dim() { return 9; }
  size_t dim() const { return Dim(); }

  /**
   * retract/unretract assume independence of components
   * Tangent space parameterization:
   *    - v(0-2): Rot3 (roll, pitch, yaw)
   *    - v(3-5): Point3
   *    - v(6-8): Translational velocity
   */
  PoseRTV retract(const Vector& v, ChartJacobian Horigin=boost::none, ChartJacobian Hv=boost::none) const;
  Vector localCoordinates(const PoseRTV& p, ChartJacobian Horigin=boost::none,ChartJacobian Hp=boost::none) const;

  // Lie TODO IS this a Lie group or just a Manifold????
  /**
   * expmap/logmap are poor approximations that assume independence of components
   * Currently implemented using the poor retract/unretract approximations
   */
  static PoseRTV Expmap(const Vector9& v, ChartJacobian H = boost::none);
  static Vector9 Logmap(const PoseRTV& p, ChartJacobian H = boost::none);

  static PoseRTV identity() { return PoseRTV(); }

  /** Derivatives calculated numerically */
  PoseRTV inverse(ChartJacobian H1=boost::none) const;

  /** Derivatives calculated numerically */
  PoseRTV compose(const PoseRTV& p,
      ChartJacobian H1=boost::none,
      ChartJacobian H2=boost::none) const;

  PoseRTV operator*(const PoseRTV& p) const { return compose(p); }

  /** Derivatives calculated numerically */
  PoseRTV between(const PoseRTV& p,
                  ChartJacobian H1=boost::none,
                  ChartJacobian H2=boost::none) const;

  // measurement functions
  /** Derivatives calculated numerically */
  double range(const PoseRTV& other,
               OptionalJacobian<1,9> H1=boost::none,
               OptionalJacobian<1,9> H2=boost::none) const;

  // IMU-specific

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
    return translationIntegration(x2, dt).vector();
  }

  /**
   * Apply transform to this pose, with optional derivatives
   * equivalent to:
   * local = trans.transform_from(global, Dtrans, Dglobal)
   *
   * Note: the transform jacobian convention is flipped
   */
  PoseRTV transformed_from(const Pose3& trans,
      ChartJacobian Dglobal = boost::none,
      OptionalJacobian<9, 6> Dtrans = boost::none) const;

  // Utility functions

  /// RRTMbn - Function computes the rotation rate transformation matrix from
  /// body axis rates to euler angle (global) rates
  static Matrix RRTMbn(const Vector& euler);

  static Matrix RRTMbn(const Rot3& att);

  /// RRTMnb - Function computes the rotation rate transformation matrix from
  /// euler angle rates to body axis rates
  static Matrix RRTMnb(const Vector& euler);

  static Matrix RRTMnb(const Rot3& att);

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(first);
    ar & BOOST_SERIALIZATION_NVP(second);
  }
};


template<>
struct traits<PoseRTV> : public internal::LieGroupTraits<PoseRTV> {};

} // \namespace gtsam
