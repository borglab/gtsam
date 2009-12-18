/*
 * Rot2.h
 *
 *  Created on: Dec 9, 2009
 *      Author: Frank Dellaert
 */

#ifndef ROT2_H_
#define ROT2_H_

#include "Testable.h"
#include "Point2.h"
#include "Matrix.h"

namespace gtsam {

  /* Rotation matrix */
  class Rot2: Testable<Rot2> {
  private:
    /** we store cos(theta) and sin(theta) */
    double c_, s_;

    /** private constructor from cos/sin */
    Rot2(double c, double s) :
      c_(c), s_(s) {
    }

  public:

    /** default constructor, zero rotation */
    Rot2() : c_(1.0), s_(0.0) {}

    /** constructor from angle == exponential map at identity */
    Rot2(double theta) : c_(cos(theta)), s_(sin(theta)) {}

    /** return angle */
    double theta() const { return atan2(s_,c_); }

    /** return cos */
    double c() const { return c_; }

    /** return sin */
    double s() const { return s_; }

    /** print */
    void print(const std::string& s = "theta") const;

    /** equals with an tolerance */
    bool equals(const Rot2& R, double tol = 1e-9) const;

    /** return DOF, dimensionality of tangent space */
    inline size_t dim() const { return 1;}

    /** Given 1-dim tangent vector, create new rotation */
    Rot2 exmap(const Vector& d) const;

    /** Return the 1-dim tangent vector of R about this rotation */
    Vector log(const Rot2& R) const { return Vector_(1, R.theta() - theta()); }

    /** return vectorized form (column-wise)*/
    inline Vector vector() const { return Vector_(2,c_,s_);}

    /** return 2*2 rotation matrix */
    Matrix matrix() const;

    /** return 2*2 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** return 2*2 negative transpose */
    Matrix negtranspose() const;

    /** inverse transformation  */
    Rot2 inverse() const;

    /** compose with the inverse of this rotation */
    Rot2 invcompose(const Rot2& R) const;

    /** composition via sum and difference formulas */
    Rot2 operator*(const Rot2& R) const;

    /**  rotate from rotated to world, syntactic sugar = R*p  */
    Point2 operator*(const Point2& p) const;

    /** rotate from world to rotated = R'*p */
    Point2 unrotate(const Point2& p) const;

    /** friends */
    friend Matrix Dunrotate1(const Rot2& R, const Point2& p);

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(c_);
      ar & BOOST_SERIALIZATION_NVP(s_);
    }
  };

  /**
   * Update Rotation with incremental rotation
   * @param v a vector of incremental angle
   * @param R a 2D rotation
   * @return incremental rotation matrix
   */
  Rot2 exmap(const Rot2& R, const Vector& v);

  /**
   * rotate point from rotated coordinate frame to
   * world = R*p
   */
  Point2 rotate(const Rot2& R, const Point2& p);
  Matrix Drotate1(const Rot2& R, const Point2& p);
  Matrix Drotate2(const Rot2& R); // does not depend on p !

  /**
   * rotate point from world to rotated
   * frame = R'*p
   */
  Point2 unrotate(const Rot2& R, const Point2& p);
  Matrix Dunrotate1(const Rot2& R, const Point2& p);
  Matrix Dunrotate2(const Rot2& R); // does not depend on p !

} // gtsam

#endif /* ROT2_H_ */
