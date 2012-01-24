/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Point2.h
 * @brief   2D Point
 * @author  Frank Dellaert
 */

#pragma once

#include <boost/serialization/nvp.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

/**
 * A 2D point
 * Complies with the Testable Concept
 * Functional, so no set functions: once created, a point is constant.
 * @ingroup geometry
 * \nosubgrouping
 */
class Point2 {
public:
	/// dimension of the variable - used to autodetect sizes
	static const size_t dimension = 2;
private:
	double x_, y_;

public:

	/// @name Standard Constructors
	/// @{

	/// default constructor
	Point2(): x_(0), y_(0) {}

	/// copy constructor
	Point2(const Point2 &p) : x_(p.x_), y_(p.y_) {}

	/// construct from doubles
	Point2(double x, double y): x_(x), y_(y) {}

	/// @}
	/// @name Advanced Constructors
	/// @{

	/// construct from 2D vector
	Point2(const Vector& v) : x_(v(0)), y_(v(1)) { assert(v.size() == 2); }

  /// @}
  /// @name Testable
  /// @{

	/// print with optional string
	void print(const std::string& s = "") const;

	/// equals with an tolerance, prints out message if unequal
	bool equals(const Point2& q, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

	/// identity
	inline static Point2 identity() {
		return Point2();
	}

	/// "Inverse" - negates each coordinate such that compose(p,inverse(p))=Point2()
	inline Point2 inverse() const { return Point2(-x_, -y_); }

	/// syntactic sugar for inverse
	inline Point2 operator- () const {return Point2(-x_,-y_);}

	/// "Compose", just adds the coordinates of two points. With optional derivatives
	inline Point2 compose(const Point2& p2,
			boost::optional<Matrix&> H1=boost::none,
			boost::optional<Matrix&> H2=boost::none) const {
		if(H1) *H1 = eye(2);
		if(H2) *H2 = eye(2);
		return *this + p2;
	}

	/// syntactic sugar for adding two points (compose())
	inline Point2 operator + (const Point2& q) const {return Point2(x_+q.x_,y_+q.y_);}

	/// "Between", subtracts point coordinates. between(p,q) = compose(inverse(p),q)
	inline Point2 between(const Point2& p2,
			boost::optional<Matrix&> H1=boost::none,
			boost::optional<Matrix&> H2=boost::none) const {
		if(H1) *H1 = -eye(2);
		if(H2) *H2 = eye(2);
		return p2 - (*this);
	}

	/// syntactic sugar for vector subtraction, i.e., between()
	inline Point2 operator - (const Point2& q) const {return Point2(x_-q.x_,y_-q.y_);}

  /// @}
  /// @name Manifold
  /// @{

	/// dimension of the variable - used to autodetect sizes
	inline static size_t Dim() { return dimension; }

  /// Dimensionality of tangent space = 2 DOF
	inline size_t dim() const { return dimension; }

	/// Updates a with tangent space delta
	inline Point2 retract(const Vector& v) const { return *this + Point2(v); }

	/// Local coordinates of manifold neighborhood around current value
	inline Vector localCoordinates(const Point2& t2) const { return Logmap(between(t2)); }

  /// @}
  /// @name Lie Group
  /// @{

	/// Exponential map around identity - just create a Point2 from a vector
	static inline Point2 Expmap(const Vector& v) { return Point2(v); }

	/// Log map around identity - just return the Point2 as a vector
	static inline Vector Logmap(const Point2& dp) { return Vector_(2, dp.x(), dp.y()); }

  /// @}
  /// @name Vector Space
  /// @{

	/** norm of point */
	double norm() const;

	/** creates a unit vector */
	Point2 unit() const { return *this/norm(); }

	/** distance between two points */
	inline double dist(const Point2& p2) const {
		return (p2 - *this).norm();
	}

	/// multiply with a scalar
	inline Point2 operator * (double s) const {return Point2(x_*s,y_*s);}

	/// divide by a scalar
	inline Point2 operator / (double q) const {return Point2(x_/q,y_/q);}

  /// @}
	/// @name Standard Interface
	/// @{

	/// equality
	inline bool operator ==(const Point2& q) const {return x_==q.x_ && q.y_==q.y_;}

	/// get x
	double x() const {return x_;}

	/// get y
	double y() const {return y_;}

	/// return vectorized form (column-wise). TODO: why does this function exist?
	Vector vector() const { return Vector_(2, x_, y_); }

	/// @}
	/// @name Deprecated (non-const, non-functional style. Do not use).
	/// @{
	inline void operator += (const Point2& q) {x_+=q.x_;y_+=q.y_;}
	inline void operator *= (double s) {x_*=s;y_*=s;}
	/// @}

private:

	/// @name Advanced Interface
	/// @{

	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(x_);
		ar & BOOST_SERIALIZATION_NVP(y_);
	}

	/// @}

};

/// multiply with scalar
inline Point2 operator*(double s, const Point2& p) {return p*s;}

}

