/**
 * @file FixedVector.h
 * @brief Extension of boost's bounded_vector to allow for fixed size operation
 * @author Alex Cunningham
 */

#pragma once

#include <Testable.h>
#include <stdarg.h>
#include <Vector.h>

namespace gtsam {

/**
 * Fixed size vectors - compatible with boost vectors, but with compile-type
 * size checking.
 */
template<size_t N>
class FixedVector : public boost::numeric::ublas::bounded_vector<double, N> ,
					public Testable<FixedVector<N> > {
public:
	typedef boost::numeric::ublas::bounded_vector<double, N> Base;

	/** default constructor */
	FixedVector() {}

	/** copy constructors */
	FixedVector(const FixedVector& v) : Base(v) {}

	/** Convert from a variable-size vector to a fixed size vector */
	FixedVector(const Vector& v) : Base(v) {}

	/** Initialize with a C-style array */
	FixedVector(const double* values) {
		std::copy(values, values+N, this->data().begin());
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
		FixedVector v;
		for (size_t i=0; i<N; ++i)
			v(i) = value;
		return v;
	}

	/**
	 * Create basis vector of
	 * with a constant in spot i
	 * @param index of the one
	 * @param value is the value to insert into the vector
	 * @return delta vector
	 */
	inline static FixedVector delta(size_t i, double value) {
		FixedVector v = zero();
		v(i) = value;
		return v;
	}

	/**
	 * Create basis vector,
	 * with one in spot i
	 * @param index of the one
	 * @return basis vector
	 */
	inline static FixedVector basis(size_t i) { return delta(i, 1.0); }

	/**
	 * Create zero vector
	 */
	inline static FixedVector zero() { return repeat(0.0);}

	/**
	 * Create vector initialized to ones
	 */
	inline static FixedVector ones() { return repeat(1.0);}

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
