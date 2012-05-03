/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FixedVector.h
 * @brief Extension of boost's bounded_vector to allow for fixed size operation
 * @author Alex Cunningham
 */

#pragma once

#include <stdarg.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

/**
 * Fixed size vectors - compatible with boost vectors, but with compile-type
 * size checking.
 */
template<size_t N>
class FixedVector : public Eigen::Matrix<double, N, 1> {
public:
	typedef Eigen::Matrix<double, N, 1> Base;

	/** default constructor */
	FixedVector() {}

	/** copy constructors */
	FixedVector(const FixedVector& v) : Base(v) {}

	/** Convert from a variable-size vector to a fixed size vector */
	FixedVector(const Vector& v) : Base(v) {}

	/** Initialize with a C-style array */
	FixedVector(const double* values) {
		std::copy(values, values+N, this->data());
	}

	/**
	 *  nice constructor, dangerous as number of arguments must be exactly right
	 *  and you have to pass doubles !!! always use 0.0 never 0
	 *
	 *  NOTE: this will throw warnings/explode if there is no argument
	 *  before the variadic section, so there is a meaningless size argument.
	 */
	FixedVector(size_t n, ...) {
	    va_list ap;
	    va_start(ap, n);
	    for(size_t i = 0 ; i < N ; i++) {
	      double value = va_arg(ap, double);
	      (*this)(i) = value;
	    }
	    va_end(ap);
	}

	/**
	 * Create vector initialized to a constant value
	 * @param constant value
	 */
	inline static FixedVector repeat(double value) {
		return FixedVector(Base::Constant(value));
	}

	/**
	 * Create basis vector of
	 * with a constant in spot i
	 * @param index of the one
	 * @param value is the value to insert into the vector
	 * @return delta vector
	 */
	inline static FixedVector delta(size_t i, double value) {
		return FixedVector(Base::Unit(i) * value);
	}

	/**
	 * Create basis vector,
	 * with one in spot i
	 * @param index of the one
	 * @return basis vector
	 */
	inline static FixedVector basis(size_t i) { return FixedVector(Base::Unit(i)); }

	/**
	 * Create zero vector
	 */
	inline static FixedVector zero() { return FixedVector(Base::Zero());}

	/**
	 * Create vector initialized to ones
	 */
	inline static FixedVector ones() { return FixedVector(FixedVector::Ones());}

	static size_t dim() { return Base::max_size; }

	void print(const std::string& name="") const { gtsam::print(Vector(*this), name); }

	template<size_t M>
	bool equals(const FixedVector<M>& other, double tol=1e-9) const {
		return false;
	}

	bool equals(const FixedVector& other, double tol=1e-9) const {
		return equal_with_abs_tol(*this,other,tol);
	}

};


} // \namespace
