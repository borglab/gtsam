/**
 * @file    numericalDerivative.h
 * @brief   Some functions to compute numerical derivatives
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>

//#define LINEARIZE_AT_IDENTITY

namespace gtsam {

	/*
	 * Note that all of these functions have two versions, a boost.function version and a
	 * standard C++ function pointer version.  This allows reformulating the arguments of
	 * a function to fit the correct structure, which is useful for situations like
	 * member functions and functions with arguments not involved in the derivative:
	 *
	 * Usage of the boost bind version to rearrange arguments:
	 *   for a function with one relevant param and an optional derivative:
	 *   	Foo bar(const Obj& a, boost::optional<Matrix&> H1)
	 *   Use boost.bind to restructure:
	 *   	boost::bind(bar, _1, boost::none)
	 *   This syntax will fix the optional argument to boost::none, while using the first argument provided
	 *
	 * For member functions, such as below, with an instantiated copy instanceOfSomeClass
	 * 		Foo SomeClass::bar(const Obj& a)
	 * Use boost bind as follows to create a function pointer that uses the member function:
	 * 	    boost::bind(&SomeClass::bar, ref(instanceOfSomeClass), _1)
	 *
	 * For additional details, see the documentation:
	 * 		http://www.boost.org/doc/libs/1_43_0/libs/bind/bind.html
	 */

	/**
	 * Numerically compute gradient of scalar function
	 * Class X is the input argument
	 * The class X needs to have dim, expmap, logmap
	 */
	template<class X>
	Vector numericalGradient(boost::function<double(const X&)> h, const X& x, double delta=1e-5) {
		double factor = 1.0/(2.0*delta);
		const size_t n = x.dim();
		Vector d(n,0.0), g(n,0.0);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; double hxplus = h(expmap(x,d));
			d(j) -= 2*delta; double hxmin  = h(expmap(x,d));
			d(j) +=   delta; g(j) = (hxplus-hxmin)*factor;
		}
		return g;
	}

	template<class X>
	Vector numericalGradient(double (*h)(const X&), const X& x, double delta=1e-5) {
		return numericalGradient<X>(boost::bind(h, _1), x, delta);
	}

	/**
	 * Compute numerical derivative in argument 1 of unary function
	 * @param h unary function yielding m-vector
	 * @param x n-dimensional value at which to evaluate h
	 * @param delta increment for numerical derivative
	 * Class Y is the output argument
	 * Class X is the input argument
	 * @return m*n Jacobian computed via central differencing
	 * Both classes X,Y need dim, expmap, logmap
	 */
	template<class Y, class X>
	Matrix numericalDerivative11(boost::function<Y(const X&)> h, const X& x, double delta=1e-5) {
		Y hx = h(x);
		double factor = 1.0/(2.0*delta);
		const size_t m = dim(hx), n = dim(x);
		Vector d(n,0.0);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = logmap(hx, h(expmap(x,d)));
			d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(expmap(x,d)));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}

	template<class Y, class X>
	Matrix numericalDerivative11(Y (*h)(const X&), const X& x, double delta=1e-5) {
		return numericalDerivative11<Y,X>(boost::bind(h, _1), x, delta);
	}

	/**
	 * Compute numerical derivative in argument 1 of binary function
	 * @param h binary function yielding m-vector
	 * @param x1 n-dimensional first argument value
	 * @param x2 second argument value
	 * @param delta increment for numerical derivative
	 * @return m*n Jacobian computed via central differencing
	 * All classes Y,X1,X2 need dim, expmap, logmap
	 */
	template<class Y, class X1, class X2>
	Matrix numericalDerivative21(boost::function<Y(const X1&, const X2&)> h,
			const X1& x1, const X2& x2, double delta=1e-5) {
		Y hx = h(x1,x2);
		double factor = 1.0/(2.0*delta);
		const size_t m = dim(hx), n = dim(x1);
		Vector d(n,0.0);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = logmap(hx, h(expmap(x1,d),x2));
			d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(expmap(x1,d),x2));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}

	template<class Y, class X1, class X2>
	Matrix numericalDerivative21(Y (*h)(const X1&, const X2&),
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative21<Y,X1,X2>(boost::bind(h, _1, _2), x1, x2, delta);
	}

	/**
	 * Compute numerical derivative in argument 2 of binary function
	 * @param h binary function yielding m-vector
	 * @param x1 first argument value
	 * @param x2 n-dimensional second argument value
	 * @param delta increment for numerical derivative
	 * @return m*n Jacobian computed via central differencing
	 * All classes Y,X1,X2 need dim, expmap, logmap
	 */
	template<class Y, class X1, class X2>
	Matrix numericalDerivative22
	(boost::function<Y(const X1&, const X2&)> h,
			const X1& x1, const X2& x2, double delta=1e-5)
	{
		Y hx = h(x1,x2);
		double factor = 1.0/(2.0*delta);
		const size_t m = dim(hx), n = dim(x2);
		Vector d(n,0.0);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = logmap(hx, h(x1,expmap(x2,d)));
			d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(x1,expmap(x2,d)));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}
	template<class Y, class X1, class X2>
	Matrix numericalDerivative22
	(Y (*h)(const X1&, const X2&),
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative22<Y,X1,X2>(boost::bind(h, _1, _2), x1, x2, delta);
	}

	/**
	 * Compute numerical derivative in argument 1 of ternary function
	 * @param h ternary function yielding m-vector
	 * @param x1 n-dimensional first argument value
	 * @param x2 second argument value
	 * @param x3 third argument value
	 * @param delta increment for numerical derivative
	 * @return m*n Jacobian computed via central differencing
	 * All classes Y,X1,X2,X3 need dim, expmap, logmap
	 */
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative31
	(boost::function<Y(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5)
	{
		Y hx = h(x1,x2,x3);
		double factor = 1.0/(2.0*delta);
		const size_t m = dim(hx), n = dim(x1);
		Vector d(n,0.0);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = logmap(hx, h(expmap(x1,d),x2,x3));
			d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(expmap(x1,d),x2,x3));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative31
	(Y (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative31<Y,X1,X2, X3>(boost::bind(h, _1, _2, _3), x1, x2, x3, delta);
	}

	// arg 2
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative32
	(boost::function<Y(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5)
	{
		Y hx = h(x1,x2,x3);
		double factor = 1.0/(2.0*delta);
		const size_t m = dim(hx), n = dim(x2);
		Vector d(n,0.0);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = logmap(hx, h(x1, expmap(x2,d),x3));
			d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(x1, expmap(x2,d),x3));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative32
	(Y (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative32<Y,X1,X2, X3>(boost::bind(h, _1, _2, _3), x1, x2, x3, delta);
	}

	// arg 3
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative33
	(boost::function<Y(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5)
	{
		Y hx = h(x1,x2,x3);
		double factor = 1.0/(2.0*delta);
		const size_t m = dim(hx), n = dim(x3);
		Vector d(n,0.0);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = logmap(hx, h(x1, x2, expmap(x3,d)));
			d(j) -= 2*delta; Vector hxmin  = logmap(hx, h(x1, x2, expmap(x3,d)));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative33
	(Y (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative33<Y,X1,X2, X3>(boost::bind(h, _1, _2, _3), x1, x2, x3, delta);
	}
}
