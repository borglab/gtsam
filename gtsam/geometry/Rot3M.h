/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3M.h
 * @brief   3D Rotation represented as 3*3 matrix
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/3rdparty/Eigen/Eigen/Geometry>

namespace gtsam {

  /// Typedef to an Eigen Quaternion<double>, we disable alignment because
  /// geometry objects are stored in boost pool allocators, Values containers,
  /// and and these pool allocators do not support alignment.
  typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

  /**
   * @brief 3D Rotations represented as rotation matrices
   * @ingroup geometry
   * \nosubgrouping
   */
  class Rot3M {
  public:
	  static const size_t dimension = 3;

  private:
    /** we store columns ! */
    Point3 r1_, r2_, r3_;  

  public:

    /// @name Constructors and named constructors
    /// @{

    /** default constructor, unit rotation */
    Rot3M();

    /**
     * Constructor from columns
     * @param r1 X-axis of rotated frame
     * @param r2 Y-axis of rotated frame
     * @param r3 Z-axis of rotated frame
     */
    Rot3M(const Point3& r1, const Point3& r2, const Point3& r3);

    /** constructor from a rotation matrix, as doubles in *row-major* order !!! */
    Rot3M(double R11, double R12, double R13,
        double R21, double R22, double R23,
        double R31, double R32, double R33);

    /** constructor from a rotation matrix */
    Rot3M(const Matrix& R);

    /** Constructor from a quaternion.  This can also be called using a plain
     * Vector, due to implicit conversion from Vector to Quaternion
     * @param q The quaternion
     */
    Rot3M(const Quaternion& q);
    
    /** Constructor from a rotation matrix in a Rot3M */
    Rot3M(const Rot3M& r);

    /* Static member function to generate some well known rotations */

    /**
     * Rotations around axes as in http://en.wikipedia.org/wiki/Rotation_matrix
     * Counterclockwise when looking from unchanging axis.
     */
    static Rot3M Rx(double t);
    static Rot3M Ry(double t);
    static Rot3M Rz(double t);
    static Rot3M RzRyRx(double x, double y, double z);
    static Rot3M RzRyRx(const Vector& xyz) {
    	assert(xyz.size() == 3);
    	return RzRyRx(xyz(0), xyz(1), xyz(2));
    }

    /**
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf
     * Assumes vehicle coordinate frame X forward, Y right, Z down
     */
    static Rot3M yaw  (double t) { return Rz(t); } // positive yaw is to right (as in aircraft heading)
    static Rot3M pitch(double t) { return Ry(t); } // positive pitch is up (increasing aircraft altitude)
    static Rot3M roll (double t) { return Rx(t); } // positive roll is to right (increasing yaw in aircraft)

    /// Returns rotation matrix nRb from body to nav frame
    static Rot3M ypr  (double y, double p, double r) { return RzRyRx(r,p,y);}

    /** Create from Quaternion coefficients */
    static Rot3M quaternion(double w, double x, double y, double z) { Quaternion q(w, x, y, z); return Rot3M(q); }

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param   w is the rotation axis, unit length
     * @param   theta rotation angle
     * @return incremental rotation matrix
     */
    static Rot3M rodriguez(const Vector& w, double theta);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param v a vector of incremental roll,pitch,yaw
     * @return incremental rotation matrix
     */
    static Rot3M rodriguez(const Vector& v);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param wx Incremental roll (about X)
     * @param wy Incremental pitch (about Y)
     * @param wz Incremental yaw (about Z)
     * @return incremental rotation matrix
     */
    static Rot3M rodriguez(double wx, double wy, double wz)
  		{ return rodriguez(Vector_(3,wx,wy,wz));}

    /// @}
    /// @name Testable
    /// @{

    /** print */
    void print(const std::string& s="R") const { gtsam::print(matrix(), s);}

    /** equals with an tolerance */
    bool equals(const Rot3M& p, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity for group operation
    inline static Rot3M identity() {
      return Rot3M();
    }

    /// Compose two rotations i.e., R= (*this) * R2
    Rot3M compose(const Rot3M& R2,
  	boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /// rotate point from rotated coordinate frame to world = R*p
    Point3 operator*(const Point3& p) const;

    /// derivative of inverse rotation R^T s.t. inverse(R)*R = identity
    Rot3M inverse(boost::optional<Matrix&> H1=boost::none) const;

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    size_t dim() const { return dimension; }

    /// Retraction from R^3 to Pose2 manifold neighborhood around current pose
    Rot3M retract(const Vector& v) const { return compose(Expmap(v)); }

    /// Returns inverse retraction
    Vector localCoordinates(const Rot3M& t2) const { return Logmap(between(t2)); }

    /// @}
    /// @name Lie Group
    /// @{

    /**
     * Exponential map at identity - create a rotation from canonical coordinates
     * using Rodriguez' formula
     */
		static Rot3M Expmap(const Vector& v)  {
    	if(zero(v)) return Rot3M();
    	else return rodriguez(v);
    }

    /**
     * Log map at identity - return the canonical coordinates of this rotation
     */
    static Vector Logmap(const Rot3M& R);

    /// @}

    /** return 3*3 rotation matrix */
    Matrix matrix() const;

    /** return 3*3 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** returns column vector specified by index */
    Point3 column(int index) const;
    Point3 r1() const;
    Point3 r2() const;
    Point3 r3() const;

    /**
     * Use RQ to calculate xyz angle representation
     * @return a vector containing x,y,z s.t. R = Rot3M::RzRyRx(x,y,z)
     */
    Vector xyz() const;

    /**
     * Use RQ to calculate yaw-pitch-roll angle representation
     * @return a vector containing ypr s.t. R = Rot3M::ypr(y,p,r)
     */
    Vector ypr() const;

    /**
     * Use RQ to calculate roll-pitch-yaw angle representation
     * @return a vector containing rpy s.t. R = Rot3M::ypr(y,p,r)
     */
    Vector rpy() const;

    /**
     * Accessors to get to components of angle representations
     * NOTE: these are not efficient to get to multiple separate parts,
     * you should instead use xyz() or ypr()
     * TODO: make this more efficient
     */
    inline double roll() const  { return ypr()(2); }
    inline double pitch() const { return ypr()(1); }
    inline double yaw() const   { return ypr()(0); }

    /** Compute the quaternion representation of this rotation.
     * @return The quaternion
     */
    Quaternion toQuaternion() const;

    /**
     * Return relative rotation D s.t. R2=D*R1, i.e. D=R2*R1'
     */
    Rot3M between(const Rot3M& R2,
    		boost::optional<Matrix&> H1=boost::none,
    		boost::optional<Matrix&> H2=boost::none) const;

    /** compose two rotations */
    Rot3M operator*(const Rot3M& R2) const;

    /**
     * rotate point from rotated coordinate frame to
     * world = R*p
     */
    Point3 rotate(const Point3& p,
  	boost::optional<Matrix&> H1=boost::none,  boost::optional<Matrix&> H2=boost::none) const;

    /**
     * rotate point from world to rotated
     * frame = R'*p
     */
    Point3 unrotate(const Point3& p,
    	boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(r1_);
      ar & BOOST_SERIALIZATION_NVP(r2_);
      ar & BOOST_SERIALIZATION_NVP(r3_);
    }
  };

  /**
   * [RQ] receives a 3 by 3 matrix and returns an upper triangular matrix R
   * and 3 rotation angles corresponding to the rotation matrix Q=Qz'*Qy'*Qx'
   * such that A = R*Q = R*Qz'*Qy'*Qx'. When A is a rotation matrix, R will
   * be the identity and Q is a yaw-pitch-roll decomposition of A.
   * The implementation uses Givens rotations and is based on Hartley-Zisserman.
   * @param a 3 by 3 matrix A=RQ
   * @return an upper triangular matrix R
   * @return a vector [thetax, thetay, thetaz] in radians.
   */
  std::pair<Matrix,Vector> RQ(const Matrix& A);

}
