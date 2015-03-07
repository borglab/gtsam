/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3.h
 * @brief   3D rotation represented as a rotation matrix or quaternion
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */
// \callgraph

#pragma once

// You can override the default coordinate mode using this flag
#ifndef ROT3_DEFAULT_COORDINATES_MODE
#ifdef GTSAM_DEFAULT_QUATERNIONS
// Exponential map is very cheap for quaternions
#define ROT3_DEFAULT_COORDINATES_MODE Rot3::EXPMAP
#else
// For rotation matrices, the Cayley transform is a fast retract alternative
#define ROT3_DEFAULT_COORDINATES_MODE Rot3::CAYLEY
#endif
#endif

#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/3rdparty/Eigen/Eigen/Geometry>

namespace gtsam {

  /// Typedef to an Eigen Quaternion<double>, we disable alignment because
  /// geometry objects are stored in boost pool allocators, in Values
  /// containers, and and these pool allocators do not support alignment.
  typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

  /**
   * @brief A 3D rotation represented as a rotation matrix if the preprocessor
   * symbol GTSAM_DEFAULT_QUATERNIONS is not defined, or as a quaternion if it
   * is defined.
   * @addtogroup geometry
   * \nosubgrouping
   */
  class Rot3 : public DerivedValue<Rot3> {
  public:
    static const size_t dimension = 3;

  private:
#ifdef GTSAM_DEFAULT_QUATERNIONS
    /** Internal Eigen Quaternion */
    Quaternion quaternion_;
#else
    Matrix3 rot_;
#endif

  public:

    /// @name Constructors and named constructors
    /// @{

    /** default constructor, unit rotation */
    Rot3();

    /**
     * Constructor from columns
     * @param r1 X-axis of rotated frame
     * @param r2 Y-axis of rotated frame
     * @param r3 Z-axis of rotated frame
     */
    Rot3(const Point3& r1, const Point3& r2, const Point3& r3);

    /** constructor from a rotation matrix, as doubles in *row-major* order !!! */
    Rot3(double R11, double R12, double R13,
        double R21, double R22, double R23,
        double R31, double R32, double R33);

    /** constructor from a rotation matrix */
    Rot3(const Matrix& R);

//    /** constructor from a fixed size rotation matrix */
//    Rot3(const Matrix3& R);

    /** Constructor from a quaternion.  This can also be called using a plain
     * Vector, due to implicit conversion from Vector to Quaternion
     * @param q The quaternion
     */
    Rot3(const Quaternion& q);

    /** Virtual destructor */
    virtual ~Rot3() {}

    /* Static member function to generate some well known rotations */

    /// Rotation around X axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 Rx(double t);

    /// Rotation around Y axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 Ry(double t);

    /// Rotation around Z axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 Rz(double t);

    /// Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 RzRyRx(double x, double y, double z);

    /// Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    inline static Rot3 RzRyRx(const Vector& xyz) {
      assert(xyz.size() == 3);
      return RzRyRx(xyz(0), xyz(1), xyz(2));
    }

    /// Positive yaw is to right (as in aircraft heading). See ypr
    static Rot3 yaw  (double t) { return Rz(t); }

    /// Positive pitch is up (increasing aircraft altitude).See ypr
    static Rot3 pitch(double t) { return Ry(t); }

    //// Positive roll is to right (increasing yaw in aircraft).
    static Rot3 roll (double t) { return Rx(t); }

    /**
     * Returns rotation nRb from body to nav frame.
     * Positive yaw is to right (as in aircraft heading).
     * Positive pitch is up (increasing aircraft altitude).
     * Positive roll is to right (increasing yaw in aircraft).
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf.
     * Assumes vehicle coordinate frame X forward, Y right, Z down.
     */
    static Rot3 ypr  (double y, double p, double r) { return RzRyRx(r,p,y);}

    /** Create from Quaternion coefficients */
    static Rot3 quaternion(double w, double x, double y, double z) { Quaternion q(w, x, y, z); return Rot3(q); }

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param   w is the rotation axis, unit length
     * @param   theta rotation angle
     * @return incremental rotation matrix
     */
    static Rot3 rodriguez(const Vector& w, double theta);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param v a vector of incremental roll,pitch,yaw
     * @return incremental rotation matrix
     */
    static Rot3 rodriguez(const Vector& v);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param wx Incremental roll (about X)
     * @param wy Incremental pitch (about Y)
     * @param wz Incremental yaw (about Z)
     * @return incremental rotation matrix
     */
    static Rot3 rodriguez(double wx, double wy, double wz)
      { return rodriguez(Vector_(3,wx,wy,wz));}

    /// @}
    /// @name Testable
    /// @{

    /** print */
    void print(const std::string& s="R") const { gtsam::print((Matrix)matrix(), s);}

    /** equals with an tolerance */
    bool equals(const Rot3& p, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity rotation for group operation
    inline static Rot3 identity() {
      return Rot3();
    }

    /// derivative of inverse rotation R^T s.t. inverse(R)*R = identity
    Rot3 inverse(boost::optional<Matrix&> H1=boost::none) const;

    /// Compose two rotations i.e., R= (*this) * R2
    Rot3 compose(const Rot3& R2,
    boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /** compose two rotations */
    Rot3 operator*(const Rot3& R2) const;

    /**
     * Return relative rotation D s.t. R2=D*R1, i.e. D=R2*R1'
     */
    Rot3 between(const Rot3& R2,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    size_t dim() const { return dimension; }

    /**
     * The method retract() is used to map from the tangent space back to the manifold.
     * Its inverse, is localCoordinates(). For Lie groups, an obvious retraction is the
     * exponential map, but this can be expensive to compute. The following Enum is used
     * to indicate which method should be used.  The default
     * is determined by ROT3_DEFAULT_COORDINATES_MODE, which may be set at compile time,
     * and itself defaults to Rot3::CAYLEY, or if GTSAM_DEFAULT_QUATERNIONS is defined,
     * to Rot3::EXPMAP.
     */
    enum CoordinatesMode {
      EXPMAP, ///< Use the Lie group exponential map to retract
#ifndef GTSAM_DEFAULT_QUATERNIONS
      CAYLEY, ///< Retract and localCoordinates using the Cayley transform.
      SLOW_CAYLEY ///< Slow matrix implementation of Cayley transform (for tests only).
#endif
      };

#ifndef GTSAM_DEFAULT_QUATERNIONS
    /// Retraction from R^3 to Rot3 manifold using the Cayley transform
    Rot3 retractCayley(const Vector& omega) const;
#endif

    /// Retraction from R^3 to Rot3 manifold neighborhood around current rotation
    Rot3 retract(const Vector& omega, Rot3::CoordinatesMode mode = ROT3_DEFAULT_COORDINATES_MODE) const;

    /// Returns local retract coordinates in neighborhood around current rotation
    Vector3 localCoordinates(const Rot3& t2, Rot3::CoordinatesMode mode = ROT3_DEFAULT_COORDINATES_MODE) const;

    /// @}
    /// @name Lie Group
    /// @{

    /**
     * Exponential map at identity - create a rotation from canonical coordinates
     * using Rodriguez' formula
     */
    static Rot3 Expmap(const Vector& v)  {
      if(zero(v)) return Rot3();
      else return rodriguez(v);
    }

    /**
     * Log map at identity - return the canonical coordinates of this rotation
     */
    static Vector3 Logmap(const Rot3& R);

  	/// @}
  	/// @name Group Action on Point3
  	/// @{

    /**
		 * rotate point from rotated coordinate frame to world \f$ p^w = R_c^w p^c \f$
		 */
		Point3 rotate(const Point3& p, boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const;

		/// rotate point from rotated coordinate frame to world = R*p
		Point3 operator*(const Point3& p) const;

		/**
		 * rotate point from world to rotated frame \f$ p^c = (R_c^w)^T p^w \f$
		 */
		Point3 unrotate(const Point3& p, boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const;

    /// @}
  	/// @name Standard Interface
  	/// @{

    /** return 3*3 rotation matrix */
    Matrix3 matrix() const;

    /** return 3*3 transpose (inverse) rotation matrix   */
    Matrix3 transpose() const;

    /** returns column vector specified by index */
    Point3 column(int index) const;
    Point3 r1() const;
    Point3 r2() const;
    Point3 r3() const;

    /**
     * Use RQ to calculate xyz angle representation
     * @return a vector containing x,y,z s.t. R = Rot3::RzRyRx(x,y,z)
     */
    Vector3 xyz() const;

    /**
     * Use RQ to calculate yaw-pitch-roll angle representation
     * @return a vector containing ypr s.t. R = Rot3::ypr(y,p,r)
     */
    Vector3 ypr() const;

    /**
     * Use RQ to calculate roll-pitch-yaw angle representation
     * @return a vector containing ypr s.t. R = Rot3::ypr(y,p,r)
     */
    Vector3 rpy() const;

    /**
     * Accessor to get to component of angle representations
     * NOTE: these are not efficient to get to multiple separate parts,
     * you should instead use xyz() or ypr()
     * TODO: make this more efficient
     */
    inline double roll() const  { return ypr()(2); }

    /**
     * Accessor to get to component of angle representations
     * NOTE: these are not efficient to get to multiple separate parts,
     * you should instead use xyz() or ypr()
     * TODO: make this more efficient
     */
    inline double pitch() const { return ypr()(1); }

    /**
     * Accessor to get to component of angle representations
     * NOTE: these are not efficient to get to multiple separate parts,
     * you should instead use xyz() or ypr()
     * TODO: make this more efficient
     */
    inline double yaw() const   { return ypr()(0); }

  	/// @}
  	/// @name Advanced Interface
  	/// @{

    /** Compute the quaternion representation of this rotation.
     * @return The quaternion
     */
    Quaternion toQuaternion() const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version)
    {
    	 ar & boost::serialization::make_nvp("Rot3",
    			 boost::serialization::base_object<Value>(*this));
#ifndef GTSAM_DEFAULT_QUATERNIONS
    	 ar & boost::serialization::make_nvp("rot11", rot_(0,0));
       ar & boost::serialization::make_nvp("rot12", rot_(0,1));
       ar & boost::serialization::make_nvp("rot13", rot_(0,2));
       ar & boost::serialization::make_nvp("rot21", rot_(1,0));
       ar & boost::serialization::make_nvp("rot22", rot_(1,1));
       ar & boost::serialization::make_nvp("rot23", rot_(1,2));
       ar & boost::serialization::make_nvp("rot31", rot_(2,0));
       ar & boost::serialization::make_nvp("rot32", rot_(2,1));
       ar & boost::serialization::make_nvp("rot33", rot_(2,2));
#else
      ar & boost::serialization::make_nvp("w", quaternion_.w());
      ar & boost::serialization::make_nvp("x", quaternion_.x());
      ar & boost::serialization::make_nvp("y", quaternion_.y());
      ar & boost::serialization::make_nvp("z", quaternion_.z());
#endif
    }
  };

	/// @}

  /**
   * [RQ] receives a 3 by 3 matrix and returns an upper triangular matrix R
   * and 3 rotation angles corresponding to the rotation matrix Q=Qz'*Qy'*Qx'
   * such that A = R*Q = R*Qz'*Qy'*Qx'. When A is a rotation matrix, R will
   * be the identity and Q is a yaw-pitch-roll decomposition of A.
   * The implementation uses Givens rotations and is based on Hartley-Zisserman.
   * @param A 3 by 3 matrix A=RQ
   * @return an upper triangular matrix R
   * @return a vector [thetax, thetay, thetaz] in radians.
   */
  std::pair<Matrix3,Vector3> RQ(const Matrix3& A);
}
