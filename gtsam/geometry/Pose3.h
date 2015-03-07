/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *@file  Pose3.h
 *@brief 3D Pose
 */

// \callgraph

#pragma once

#ifndef POSE3_DEFAULT_COORDINATES_MODE
#define POSE3_DEFAULT_COORDINATES_MODE Pose3::FIRST_ORDER
#endif

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

namespace gtsam {

	class Pose2; // forward declare

  /**
   * A 3D pose (R,t) : (Rot3,Point3)
   * @addtogroup geometry
   * \nosubgrouping
   */
  class Pose3 : public DerivedValue<Pose3> {
  public:
    static const size_t dimension = 6;

    /** Pose Concept requirements */
    typedef Rot3 Rotation;
    typedef Point3 Translation;

  private:
    Rot3 R_;
    Point3 t_;

  public:

  	/// @name Standard Constructors
  	/// @{

    /** Default constructor is origin */
    Pose3() {}

    /** Copy constructor */
    Pose3(const Pose3& pose) : R_(pose.R_), t_(pose.t_) {}

    /** Construct from R,t */
    Pose3(const Rot3& R, const Point3& t) : R_(R), t_(t) {}

    /** Construct from Pose2 */
    explicit Pose3(const Pose2& pose2);

    /** Constructor from 4*4 matrix */
    Pose3(const Matrix &T) :
      R_(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0),
          T(2, 1), T(2, 2)), t_(T(0, 3), T(1, 3), T(2, 3)) {}

    /// @}
    /// @name Testable
    /// @{

    /// print with optional string
    void print(const std::string& s = "") const;

    /// assert equality up to a tolerance
    bool equals(const Pose3& pose, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity for group operation
    static Pose3 identity() { return Pose3(); }

    /// inverse transformation with derivatives
    Pose3 inverse(boost::optional<Matrix&> H1=boost::none) const;

    ///compose this transformation onto another (first *this and then p2)
    Pose3 compose(const Pose3& p2,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /// compose syntactic sugar
    Pose3 operator*(const Pose3& T) const {
      return Pose3(R_*T.R_, t_ + R_*T.t_);
    }

    /**
     * Return relative pose between p1 and p2, in p1 coordinate frame
     * as well as optionally the derivatives
     */
    Pose3 between(const Pose3& p2,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /// @}
    /// @name Manifold
    /// @{

    /** Enum to indicate which method should be used in Pose3::retract() and
     * Pose3::localCoordinates()
     */
    enum CoordinatesMode {
      EXPMAP, ///< The correct exponential map, computationally expensive.
      FIRST_ORDER ///< A fast first-order approximation to the exponential map.
    };

    /// Dimensionality of tangent space = 6 DOF - used to autodetect sizes
    static size_t Dim() { return dimension; }

    /// Dimensionality of the tangent space = 6 DOF
    size_t dim() const { return dimension; }

    /// Retraction with fast first-order approximation to the exponential map
    Pose3 retractFirstOrder(const Vector& d) const;

    /// Retraction from R^6 to Pose3 manifold neighborhood around current pose
    Pose3 retract(const Vector& d, Pose3::CoordinatesMode mode = POSE3_DEFAULT_COORDINATES_MODE) const;

    /// Local 6D coordinates of Pose3 manifold neighborhood around current pose
    Vector6 localCoordinates(const Pose3& T2, Pose3::CoordinatesMode mode = POSE3_DEFAULT_COORDINATES_MODE) const;

    /// @}
    /// @name Lie Group
    /// @{

    /// Exponential map at identity - create a rotation from canonical coordinates
    static Pose3 Expmap(const Vector& xi);

    /// Log map at identity - return the canonical coordinates of this rotation
    static Vector6 Logmap(const Pose3& p);

    /**
     * Calculate Adjoint map
     * Ad_pose is 6*6 matrix that when applied to twist xi, returns Ad_pose(xi)
     */
    Matrix6 adjointMap() const; /// FIXME Not tested - marked as incorrect
    Vector adjoint(const Vector& xi) const {return adjointMap()*xi; } /// FIXME Not tested - marked as incorrect

    /**
     * wedge for Pose3:
     * @param xi 6-dim twist (omega,v) where
     *  omega = (wx,wy,wz) 3D angular velocity
     *  v (vx,vy,vz) = 3D velocity
     * @return xihat, 4*4 element of Lie algebra that can be exponentiated
     */
    static Matrix wedge(double wx, double wy, double wz, double vx, double vy, double vz) {
      return Matrix_(4,4,
          0.,-wz,  wy,  vx,
          wz,  0.,-wx,  vy,
          -wy, wx,   0., vz,
          0.,  0.,  0.,  0.);
    }

  	/// @}
  	/// @name Group Action on Point3
  	/// @{

    /** receives the point in Pose coordinates and transforms it to world coordinates */
    Point3 transform_from(const Point3& p,
        boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /** syntactic sugar for transform_from */
    inline Point3 operator*(const Point3& p) const { return transform_from(p); }

    /** receives the point in world coordinates and transforms it to Pose coordinates */
    Point3 transform_to(const Point3& p,
        boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /// @}
  	/// @name Standard Interface
  	/// @{

    /// get rotation
    const Rot3& rotation() const { return R_; }

    /// get translation
    const Point3& translation() const { return t_; }

    /// get x
    double x() const { return t_.x(); }

    /// get y
    double y() const { return t_.y(); }

    /// get z
    double z() const { return t_.z(); }

    /** convert to 4*4 matrix */
    Matrix4 matrix() const;

    /** receives a pose in world coordinates and transforms it to local coordinates */
    Pose3 transform_to(const Pose3& pose) const;

    /**
     * Calculate range to a landmark
     * @param point 3D location of landmark
     * @return range (double)
     */
    double range(const Point3& point,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /**
     * Calculate range to another pose
     * @param pose Other SO(3) pose
     * @return range (double)
     */
    double range(const Pose3& pose,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /// @}
  	/// @name Advanced Interface
  	/// @{

    /**
     * Return the start and end indices (inclusive) of the translation component of the
     * exponential map parameterization
     * @return a pair of [start, end] indices into the tangent space vector
     */
  	inline static std::pair<size_t, size_t> translationInterval() { return std::make_pair(3, 5); }

  	/**
  	 * Return the start and end indices (inclusive) of the rotation component of the
  	 * exponential map parameterization
  	 * @return a pair of [start, end] indices into the tangent space vector
  	 */
  	static std::pair<size_t, size_t> rotationInterval() { return std::make_pair(0, 2); }

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
    	ar & boost::serialization::make_nvp("Pose3",
    	 			 boost::serialization::base_object<Value>(*this));
      ar & BOOST_SERIALIZATION_NVP(R_);
      ar & BOOST_SERIALIZATION_NVP(t_);
    }
  	/// @}

  }; // Pose3 class

  /**
   * wedge for Pose3:
   * @param xi 6-dim twist (omega,v) where
   *  omega = 3D angular velocity
   *  v = 3D velocity
   * @return xihat, 4*4 element of Lie algebra that can be exponentiated
   */
  template <>
  inline Matrix wedge<Pose3>(const Vector& xi) {
    return Pose3::wedge(xi(0),xi(1),xi(2),xi(3),xi(4),xi(5));
  }

} // namespace gtsam
