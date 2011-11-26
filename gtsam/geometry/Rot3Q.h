/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3Q.h
 * @brief   3D Rotation represented as 3*3 matrix
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3M.h>
#include <gtsam/3rdparty/Eigen/Eigen/Geometry>

namespace gtsam {

  typedef Eigen::Quaternion<double,Eigen::DontAlign> Quaternion; ///< Typedef to an Eigen Quaternion<double>

  /**
   * @brief 3D Rotations represented as rotation matrices
   * @ingroup geometry
   */
  class Rot3Q {
  public:
	  static const size_t dimension = 3;

  private:
    /** Internal Eigen Quaternion */
    Quaternion quaternion_;

  public:

    /** default constructor, unit rotation */
    Rot3Q() : quaternion_(Quaternion::Identity()) {}

    /** constructor from columns */
    Rot3Q(const Point3& r1, const Point3& r2, const Point3& r3) :
      quaternion_((Eigen::Matrix3d() <<
          r1.x(), r2.x(), r3.x(),
          r1.y(), r2.y(), r3.y(),
          r1.z(), r2.z(), r3.z()).finished()) {}

    /** constructor from doubles in *row* order !!! */
    Rot3Q(double R11, double R12, double R13,
        double R21, double R22, double R23,
        double R31, double R32, double R33) :
          quaternion_((Eigen::Matrix3d() <<
              R11, R12, R13,
              R21, R22, R23,
              R31, R32, R33).finished()) {}

    /** constructor from matrix */
    Rot3Q(const Matrix& R) : quaternion_(Eigen::Matrix3d(R)) {}

    /** Constructor from a quaternion.  This can also be called using a plain
     * Vector, due to implicit conversion from Vector to Quaternion
     * @param q The quaternion
     */
    Rot3Q(const Quaternion& q) : quaternion_(q) {}

    /** Constructor from a rotation matrix rotation Rot3M */
    Rot3Q(const Rot3M& r) : quaternion_(Eigen::Matrix3d(r.matrix())) {}

    /* Static member function to generate some well known rotations */

    /**
     * Rotations around axes as in http://en.wikipedia.org/wiki/Rotation_matrix
     * Counterclockwise when looking from unchanging axis.
     */
    static Rot3Q Rx(double t) { return Quaternion(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitX())); }
    static Rot3Q Ry(double t) { return Quaternion(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitY())); }
    static Rot3Q Rz(double t) { return Quaternion(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ())); }
    static Rot3Q RzRyRx(double x, double y, double z);
    inline static Rot3Q RzRyRx(const Vector& xyz) {
    	assert(xyz.size() == 3);
    	return RzRyRx(xyz(0), xyz(1), xyz(2));
    }

    /**
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf
     * Assumes vehicle coordinate frame X forward, Y right, Z down
     */
    static Rot3Q yaw  (double t) { return Rz(t); } // positive yaw is to right (as in aircraft heading)
    static Rot3Q pitch(double t) { return Ry(t); } // positive pitch is up (increasing aircraft altitude)
    static Rot3Q roll (double t) { return Rx(t); } // positive roll is to right (increasing yaw in aircraft)
    static Rot3Q ypr  (double y, double p, double r) { return RzRyRx(r,p,y);}

    /** Create from Quaternion parameters */
    static Rot3Q quaternion(double w, double x, double y, double z) { Quaternion q(w, x, y, z); return Rot3Q(q); }

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param   w is the rotation axis, unit length
     * @param   theta rotation angle
     * @return incremental rotation matrix
     */
    static Rot3Q rodriguez(const Vector& w, double theta);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param v a vector of incremental roll,pitch,yaw
     * @return incremental rotation matrix
     */
    static Rot3Q rodriguez(const Vector& v);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param wx Incremental roll (about X)
     * @param wy Incremental pitch (about Y)
     * @param wz Incremental yaw (about Z)
     * @return incremental rotation matrix
     */
    static Rot3Q rodriguez(double wx, double wy, double wz)
  		{ return rodriguez(Vector_(3,wx,wy,wz));}

    /// @name Testable
    /// @{

    /** print */
    void print(const std::string& s="R") const { gtsam::print(matrix(), s);}

    /** equals with an tolerance */
    bool equals(const Rot3Q& p, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity for group operation
    inline static Rot3Q identity() {
      return Rot3Q();
    }

    /// derivative of inverse rotation R^T s.t. inverse(R)*R = Rot3Q()
    Rot3Q inverse(boost::optional<Matrix&> H1=boost::none) const {
    	if (H1) *H1 = -matrix();
    	return Rot3Q(quaternion_.inverse());
    }

    /// Compose two rotations i.e., R= (*this) * R2
    Rot3Q compose(const Rot3Q& R2,
  	boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /// rotate point from rotated coordinate frame to world = R*p
    inline Point3 operator*(const Point3& p) const {
      Eigen::Vector3d r = quaternion_ * Eigen::Vector3d(p.x(), p.y(), p.z());
      return Point3(r(0), r(1), r(2));
    }

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    size_t dim() const { return dimension; }

    /// Retraction from R^3 to Pose2 manifold neighborhood around current pose
    Rot3Q retract(const Vector& v) const { return compose(Expmap(v)); }

    /// Returns inverse retraction
    Vector localCoordinates(const Rot3Q& t2) const { return Logmap(between(t2)); }

    /// @}
    /// @name Lie Group
    /// @{

    /**
     * Exponential map at identity - create a rotation from canonical coordinates
     * using Rodriguez' formula
     */
		static Rot3Q Expmap(const Vector& v)  {
    	if(zero(v)) return Rot3Q();
    	else return rodriguez(v);
    }

    /**
     * Log map at identity - return the canonical coordinates of this rotation
     */
    static Vector Logmap(const Rot3Q& R);

    /// @}

    /** return 3*3 rotation matrix */
    Matrix matrix() const;

    /** return 3*3 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** returns column vector specified by index */
    Point3 r1() const { return Point3(quaternion_.toRotationMatrix().col(0)); }
    Point3 r2() const { return Point3(quaternion_.toRotationMatrix().col(1)); }
    Point3 r3() const { return Point3(quaternion_.toRotationMatrix().col(2)); }

    /**
     * Use RQ to calculate xyz angle representation
     * @return a vector containing x,y,z s.t. R = Rot3Q::RzRyRx(x,y,z)
     */
    Vector xyz() const;

    /**
     * Use RQ to calculate yaw-pitch-roll angle representation
     * @return a vector containing ypr s.t. R = Rot3Q::ypr(y,p,r)
     */
    Vector ypr() const;

    /**
     * Use RQ to calculate roll-pitch-yaw angle representation
     * @return a vector containing ypr s.t. R = Rot3Q::ypr(y,p,r)
     */
    Vector rpy() const;

    /** Compute the quaternion representation of this rotation.
     * @return The quaternion
     */
    Quaternion toQuaternion() const { return quaternion_; }

    /**
     * Return relative rotation D s.t. R2=D*R1, i.e. D=R2*R1'
     */
    Rot3Q between(const Rot3Q& R2,
    		boost::optional<Matrix&> H1=boost::none,
    		boost::optional<Matrix&> H2=boost::none) const;

    /** compose two rotations */
    Rot3Q operator*(const Rot3Q& R2) const { return Rot3Q(quaternion_ * R2.quaternion_); }

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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(quaternion_);
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
