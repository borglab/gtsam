/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Rot2.h
 *
 *  Created on: Dec 9, 2009
 *      Author: Frank Dellaert
 */

#ifndef ROT2_H_
#define ROT2_H_

#include <boost/optional.hpp>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

  /** Rotation matrix
   * NOTE: the angle theta is in radians unless explicitly stated
   */
  class Rot2: Testable<Rot2>, public Lie<Rot2> {

  public:
	  /** get the dimension by the type */
	  static const size_t dimension =  1;

  private:

    /** we store cos(theta) and sin(theta) */
    double c_, s_;

    /** private constructor from cos/sin */
    inline Rot2(double c, double s) : c_(c), s_(s) {}

    /** normalize to make sure cos and sin form unit vector */
    Rot2& normalize();

  public:

    /** default constructor, zero rotation */
    Rot2() : c_(1.0), s_(0.0) {}

    /** "named constructors" */

    /** Named constructor from angle == exponential map at identity  - theta is in radians*/
    static Rot2 fromAngle(double theta) {
		return Rot2(cos(theta), sin(theta));
	}

    /** Named constructor from angle in degrees */
    static Rot2 fromDegrees(double theta) {
    	const double degree = M_PI / 180;
    	return fromAngle(theta * degree);
    }

    /** Named constructor from cos(theta),sin(theta) pair, will *not* normalize! */
    static Rot2 fromCosSin(double c, double s);

	/**
	 * Named constructor with derivative
	 * Calculate relative bearing to a landmark in local coordinate frame
	 * @param point 2D location of landmark
	 * @param H optional reference for Jacobian
	 * @return 2D rotation \in SO(2)
	 */
	static Rot2 relativeBearing(const Point2& d, boost::optional<Matrix&> H=boost::none);

	/** Named constructor that behaves as atan2, i.e., y,x order (!) and normalizes */
  	static Rot2 atan2(double y, double x);

  	/** return angle (RADIANS) */
    double theta() const { return ::atan2(s_,c_); }

    /** return angle (DEGREES) */
    double degrees() const {
    	const double degree = M_PI / 180;
    	return theta() / degree;
    }

    /** return cos */
    inline double c() const { return c_; }

    /** return sin */
    inline double s() const { return s_; }

    /** print */
    void print(const std::string& s = "theta") const;

    /** equals with an tolerance */
    bool equals(const Rot2& R, double tol = 1e-9) const;

    /** dimension of the variable - used to autodetect sizes */
    inline static size_t Dim() { return dimension; }

    /** Lie requirements */

    /** Dimensionality of the tangent space */
    inline size_t dim() const { return dimension; }

    /** Compose - make a new rotation by adding angles */
    inline Rot2 compose(const Rot2& R1) const { return *this * R1;}

    /** Expmap around identity - create a rotation from an angle */
    static Rot2 Expmap(const Vector& v) {
    	if (zero(v)) return (Rot2());
    	else return Rot2::fromAngle(v(0));
    }

    /** Logmap around identity - return the angle of the rotation */
    static inline Vector Logmap(const Rot2& r) {
      return Vector_(1, r.theta());
    }

    /** Binary expmap  */
    inline Rot2 expmap(const Vector& v) const { return *this * Expmap(v); }

    /** Binary Logmap  */
    inline Vector logmap(const Rot2& p2) const { return Logmap(between(p2));}

    /** Between using the default implementation */
    inline Rot2 between(const Rot2& p2) const { return between_default(*this, p2); }

    /** return 2*2 rotation matrix */
    Matrix matrix() const;

    /** return 2*2 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** The inverse rotation - negative angle */
    Rot2 inverse() const { return Rot2(c_, -s_);}

    /** Compose - make a new rotation by adding angles */
    Rot2 operator*(const Rot2& R) const {
			return fromCosSin(c_ * R.c_ - s_ * R.s_, s_ * R.c_ + c_ * R.s_);
		}

    /**
     * rotate point from rotated coordinate frame to
     * world = R*p
     */
  	Point2 rotate(const Point2& p, boost::optional<Matrix&> H1 =
  			boost::none, boost::optional<Matrix&> H2 = boost::none) const;

    /** syntactic sugar for rotate */
    inline Point2 operator*(const Point2& p) const {return rotate(p);}

    /**
     * rotate point from world to rotated
     * frame = R'*p
     */
    Point2 unrotate(const Point2& p, boost::optional<Matrix&> H1 =
  			boost::none, boost::optional<Matrix&> H2 = boost::none) const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(c_);
      ar & BOOST_SERIALIZATION_NVP(s_);
    }

  }; // Rot2

} // gtsam

#endif /* ROT2_H_ */
