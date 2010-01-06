/**
 * @file    Rot3.h
 * @brief   Rotation
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Point3.h"
#include "Testable.h"

namespace gtsam {

  /* Rotation matrix */
  class Rot3: Testable<Rot3> {
  private:
    /** we store columns ! */
    Point3 r1_, r2_, r3_;  
		
  public:
	 
		/** default constructor, unit rotation */
		Rot3() : r1_(Point3(1.0,0.0,0.0)),
				r2_(Point3(0.0,1.0,0.0)),
				r3_(Point3(0.0,0.0,1.0)) {
			}

		/** constructor from columns */
		Rot3(const Point3& r1, const Point3& r2, const Point3& r3) :
				r1_(r1), r2_(r2), r3_(r3) {
			}
			
		/**  constructor from vector */
		Rot3(const Vector &v) :
			r1_(Point3(v(0),v(1),v(2))),
				r2_(Point3(v(3),v(4),v(5))),
				r3_(Point3(v(6),v(7),v(8)))
				{  }

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

		/** print */
		void print(const std::string& s="R") const { gtsam::print(matrix(), s);}

		/** equals with an tolerance */
		bool equals(const Rot3& p, double tol = 1e-9) const;

    /** return DOF, dimensionality of tangent space */
    size_t dim() const { return 3;}
		
    /**
     * @param a 3-dim tangent vector d (canonical coordinates of between(R,S))
     * @return new rotation S=exp(d)*R
     */
    Rot3 exmap(const Vector& d) const;
		
    /**
     * @return log(R), i.e. canonical coordinates of R
     */
    Vector log() const;

    /** return vectorized form (column-wise)*/
    Vector vector() const;

    /** return 3*3 rotation matrix */
    Matrix matrix() const;

    /** return 3*3 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** returns column vector specified by index */
    Point3 column(int index) const;

    /** inverse transformation  */
    Rot3 inverse() const;
		
    /** composition */
    inline Rot3 operator*(const Rot3& B) const { return Rot3(matrix()*B.matrix());}

    /**  rotate from rotated to world, syntactic sugar = R*p  */
    inline Point3 operator*(const Point3& p) const {
    	return r1_ * p.x() + r2_ * p.y() + r3_ * p.z();
    }

    /** rotate from world to rotated = R'*p */
    Point3 unrotate(const Point3& p) const;

    /** use RQ to calculate yaw-pitch-roll angle representation */
    Vector ypr() const;

    /** friends */
    friend Matrix Dunrotate1(const Rot3& R, const Point3& p);

  private:
  	/** Serialization function */
  	friend class boost::serialization::access;
  	template<class Archive>
  	void serialize(Archive & ar, const unsigned int version)
  	{
  		ar & BOOST_SERIALIZATION_NVP(r1_);
  		ar & BOOST_SERIALIZATION_NVP(r2_);
  		ar & BOOST_SERIALIZATION_NVP(r3_);
  	}
  };

  /**
   * Rodriguez' formula to compute an incremental rotation matrix
   * @param   w is the rotation axis, unit length
   * @param   theta rotation angle
   * @return incremental rotation matrix
   */
  Rot3 rodriguez(const Vector& w, double theta);

  /**
   * Rodriguez' formula to compute an incremental rotation matrix
   * @param v a vector of incremental roll,pitch,yaw
   * @return incremental rotation matrix
   */
  Rot3 rodriguez(const Vector& v);

  /**
   * Rodriguez' formula to compute an incremental rotation matrix
   * @param wx
   * @param wy
   * @param wz
   * @return incremental rotation matrix
   */
  inline Rot3 rodriguez(double wx, double wy, double wz) { return rodriguez(Vector_(3,wx,wy,wz));}

  /**
   * Update Rotation with incremental rotation
   * @param v a vector of incremental roll,pitch,yaw
   * @param R a rotated frame
   * @return incremental rotation matrix
   */
  Rot3 exmap(const Rot3& R, const Vector& v);

  /**
   * @param a rotation R
   * @param a rotation S
   * @return log(S*R'), i.e. canonical coordinates of between(R,S)
   */
  Vector log(const Rot3& R, const Rot3& S);

  /**
   * rotate point from rotated coordinate frame to 
   * world = R*p
   */
  Point3  rotate (const Rot3& R, const Point3& p);
  Matrix Drotate1(const Rot3& R, const Point3& p);
  Matrix Drotate2(const Rot3& R); // does not depend on p !

  /**
   * rotate point from world to rotated 
   * frame = R'*p
   */
  Point3  unrotate (const Rot3& R, const Point3& p);
  Matrix Dunrotate1(const Rot3& R, const Point3& p);
  Matrix Dunrotate2(const Rot3& R); // does not depend on p !

	/**
	 * Return relative rotation D s.t. R2=D*R1, i.e. D=R2*R1'
	 */
	Rot3 between(const Rot3& R1, const Rot3& R2);

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
