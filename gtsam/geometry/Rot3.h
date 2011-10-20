/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3.h
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

typedef Eigen::Quaterniond Quaternion;

  /**
   * @brief 3D Rotations represented as rotation matrices
   * @ingroup geometry
   */
  class Rot3: public Lie<Rot3> {
  public:
	  static const size_t dimension = 3;

  private:
    /** we store columns ! */
    Point3 r1_, r2_, r3_;  

  public:

    /** default constructor, unit rotation */
    Rot3() :
      r1_(Point3(1.0,0.0,0.0)),
      r2_(Point3(0.0,1.0,0.0)),
      r3_(Point3(0.0,0.0,1.0)) {}

    /** constructor from columns */
    Rot3(const Point3& r1, const Point3& r2, const Point3& r3) :
      r1_(r1), r2_(r2), r3_(r3) {}

    /** constructor from doubles in *row* order !!! */
    Rot3(double R11, double R12, double R13,
        double R21, double R22, double R23,
        double R31, double R32, double R33) :
          r1_(Point3(R11, R21, R31)),
          r2_(Point3(R12, R22, R32)),
          r3_(Point3(R13, R23, R33)) {}

    /** constructor from matrix */
    Rot3(const Matrix& R):
      r1_(Point3(R(0,0), R(1,0), R(2,0))),
      r2_(Point3(R(0,1), R(1,1), R(2,1))),
      r3_(Point3(R(0,2), R(1,2), R(2,2))) {}

    /** Constructor from a quaternion.  This can also be called using a plain
     * Vector, due to implicit conversion from Vector to Quaternion
     * @param q The quaternion
     */
    Rot3(const Quaternion& q) {
      Eigen::Matrix3d R = q.toRotationMatrix();
      r1_ = Point3(R.col(0));
      r2_ = Point3(R.col(1));
      r3_ = Point3(R.col(2));
    }

    /** Static member function to generate some well known rotations */

    /**
     * Rotations around axes as in http://en.wikipedia.org/wiki/Rotation_matrix
     * Counterclockwise when looking from unchanging axis.
     */
    static Rot3 Rx(double t);
    static Rot3 Ry(double t);
    static Rot3 Rz(double t);
    static Rot3 RzRyRx(double x, double y, double z);
    inline static Rot3 RzRyRx(const Vector& xyz) {
    	assert(xyz.size() == 3);
    	return RzRyRx(xyz(0), xyz(1), xyz(2));
    }

    /**
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf
     * Assumes vehicle coordinate frame X forward, Y right, Z down
     */
    static Rot3 yaw  (double t) { return Rz(t);} // positive yaw is to right (as in aircraft heading)
    static Rot3 pitch(double t) { return Ry(t);} // positive pitch is up (increasing aircraft altitude)
    static Rot3 roll (double t) { return Rx(t);} // positive roll is to right (increasing yaw in aircraft)
    static Rot3 ypr  (double y, double p, double r) { return RzRyRx(r,p,y);}

    /** Create from Quaternion parameters */
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
     * @param wx
     * @param wy
     * @param wz
     * @return incremental rotation matrix
     */
    static inline Rot3 rodriguez(double wx, double wy, double wz)
  		{ return rodriguez(Vector_(3,wx,wy,wz));}

    /** print */
    void print(const std::string& s="R") const { gtsam::print(matrix(), s);}

    /** equals with an tolerance */
    bool equals(const Rot3& p, double tol = 1e-9) const;

    /** return 3*3 rotation matrix */
    Matrix matrix() const;

    /** return 3*3 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** returns column vector specified by index */
    Point3 column(int index) const;
    Point3 r1() const { return r1_; }
    Point3 r2() const { return r2_; }
    Point3 r3() const { return r3_; }

    /**
     * Use RQ to calculate xyz angle representation
     * @return a vector containing x,y,z s.t. R = Rot3::RzRyRx(x,y,z)
     */
    Vector xyz() const;

    /**
     * Use RQ to calculate yaw-pitch-roll angle representation
     * @return a vector containing ypr s.t. R = Rot3::ypr(y,p,r)
     */
    Vector ypr() const;

    /**
     * Use RQ to calculate roll-pitch-yaw angle representation
     * @return a vector containing ypr s.t. R = Rot3::ypr(y,p,r)
     */
    Vector rpy() const;

    /** Compute the quaternion representation of this rotation.
     * @return The quaternion
     */
    Quaternion toQuaternion() const {
      return Quaternion((Eigen::Matrix3d() <<
          r1_.x(), r2_.x(), r3_.x(),
          r1_.y(), r2_.y(), r3_.y(),
          r1_.z(), r2_.z(), r3_.z()).finished());
    }

    /** dimension of the variable - used to autodetect sizes */
    inline static size_t Dim() { return dimension; }

    /** Lie requirements */

    /** return DOF, dimensionality of tangent space */
    inline size_t dim() const { return dimension; }

    /** Compose two rotations i.e., R= (*this) * R2
     */
    Rot3 compose(const Rot3& R2,
  	boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /** Exponential map at identity - create a rotation from canonical coordinates
     * using Rodriguez' formula
     */
		static Rot3 Expmap(const Vector& v)  {
    	if(zero(v)) return Rot3();
    	else return rodriguez(v);
    }

    // Log map at identity - return the canonical coordinates of this rotation
    static Vector Logmap(const Rot3& R);

    /** default implementations of binary functions */
    inline Rot3 expmap(const Vector& v) const { return gtsam::expmap_default(*this, v); }
    inline Vector logmap(const Rot3& p2) const { return gtsam::logmap_default(*this, p2);}

    // derivative of inverse rotation R^T s.t. inverse(R)*R = Rot3()
    inline Rot3 inverse(boost::optional<Matrix&> H1=boost::none) const {
    	if (H1) *H1 = -matrix();
    	return Rot3(
    			r1_.x(), r1_.y(), r1_.z(),
    			r2_.x(), r2_.y(), r2_.z(),
    			r3_.x(), r3_.y(), r3_.z());
    }

    /**
     * Return relative rotation D s.t. R2=D*R1, i.e. D=R2*R1'
     */
    Rot3 between(const Rot3& R2,
    		boost::optional<Matrix&> H1=boost::none,
    		boost::optional<Matrix&> H2=boost::none) const;

    /** compose two rotations */
    Rot3 operator*(const Rot3& R2) const {
      return Rot3(rotate(R2.r1_), rotate(R2.r2_), rotate(R2.r3_));
    }

    /**
     * rotate point from rotated coordinate frame to
     * world = R*p
     */
    inline Point3 operator*(const Point3& p) const { return rotate(p);}

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
