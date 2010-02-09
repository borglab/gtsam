/*
 * Rot2.h
 *
 *  Created on: Dec 9, 2009
 *      Author: Frank Dellaert
 */

#ifndef ROT2_H_
#define ROT2_H_

#include <boost/optional.hpp>
#include "Testable.h"
#include "Point2.h"
#include "Matrix.h"
#include "Lie.h"

namespace gtsam {

  /* Rotation matrix */
  class Rot2: Testable<Rot2>, public Lie<Rot2> {
  private:
    /** we store cos(theta) and sin(theta) */
    double c_, s_;

  public:

    /** constructor from cos/sin */
    Rot2(double c, double s) :
      c_(c), s_(s) {
    	// rtodo: Could do this scale correction only when creating from compose
    	// Don't let scale drift
    	double scale = c*c + s*s;
    	if(scale != 1.0) {
    		scale = pow(scale, -0.5);
    		c_ *= scale;
    		s_ *= scale;
    	}
    }

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

    /** return 2*2 rotation matrix */
    Matrix matrix() const;

    /** return 2*2 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** return 2*2 negative transpose */
    Matrix negtranspose() const;

    /** rotate from world to rotated = R*p */
    Point2 rotate(const Point2& p) const;

    /** rotate from world to rotated = R'*p */
    Point2 unrotate(const Point2& p) const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(c_);
      ar & BOOST_SERIALIZATION_NVP(s_);
    }
  };


  // Lie group functions

  /** Global print calls member function */
  inline void print(const Rot2& r, const std::string& s = "") { r.print(s); }

  /** Dimensionality of the tangent space */
  inline size_t dim(const Rot2&) { return 1; }

  /** Expmap around identity - create a rotation from an angle */
  template<> inline Rot2 expmap(const Vector& v) {
    if (zero(v)) return (Rot2());
    else return Rot2(v(0));
  }

  /** Logmap around identity - return the angle of the rotation */
  inline Vector logmap(const Rot2& r) {
    return Vector_(1, r.theta());
  }

  /** Compose - make a new rotation by adding angles */
  inline Rot2 compose(const Rot2& r0, const Rot2& r1) {
    return Rot2(
        r0.c() * r1.c() - r0.s() * r1.s(),
        r0.s() * r1.c() + r0.c() * r1.s());
  }

  /** Syntactic sugar R1*R2 = compose(R1,R2) */
  inline Rot2 operator*(const Rot2& r0, const Rot2& r1) {
    return compose(r0, r1);
  }

  /** The inverse rotation - negative angle */
  inline Rot2 inverse(const Rot2& r) { return Rot2(r.c(), -r.s());}

  /** Shortcut to compose the inverse: invcompose(R0,R1) = inverse(R0)*R1 */
  inline Rot2 invcompose(const Rot2& r0, const Rot2& r1) {
    return Rot2(
         r0.c() * r1.c() + r0.s() * r1.s(),
        -r0.s() * r1.c() + r0.c() * r1.s());
  }


  /**
   * rotate point from rotated coordinate frame to
   * world = R*p
   */
  inline Point2 operator*(const Rot2& R, const Point2& p) {return R.rotate(p);}
	Point2 rotate(const Rot2 & R, const Point2& p, boost::optional<Matrix&> H1 =
			boost::none, boost::optional<Matrix&> H2 = boost::none);

  /**
   * rotate point from world to rotated
   * frame = R'*p
   */
	Point2 unrotate(const Rot2 & R, const Point2& p, boost::optional<Matrix&> H1 =
			boost::none, boost::optional<Matrix&> H2 = boost::none);

	/**
	 * Calculate relative bearing to a landmark in local coordinate frame
	 * @param point 2D location of landmark
	 * @param H optional reference for Jacobian
	 * @return 2D rotation \in SO(2)
	 */
	Rot2 relativeBearing(const Point2& d);

	/**
	 * Calculate relative bearing and optional derivative
	 */
	Rot2 relativeBearing(const Point2& d, boost::optional<Matrix&> H);

} // gtsam

#endif /* ROT2_H_ */
