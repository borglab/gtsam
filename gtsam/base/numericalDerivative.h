/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    numericalDerivative.h
 * @brief   Some functions to compute numerical derivatives
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <gtsam/base/LieVector.h>
#include <gtsam/base/Matrix.h>



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
	 * 		http://www.boost.org/doc/libs/release/libs/bind/bind.html
	 */


	/** global functions for converting to a LieVector for use with numericalDerivative */
  inline LieVector makeLieVector(const Vector& v) { return LieVector(v); }
	inline LieVector makeLieVectorD(double d) { return LieVector(Vector_(1, d)); }

	/**
	 * Numerically compute gradient of scalar function
	 * Class X is the input argument
	 * The class X needs to have dim, expmap, logmap
	 */
	template<class X>
	Vector numericalGradient(boost::function<double(const X&)> h, const X& x, double delta=1e-5) {
		double factor = 1.0/(2.0*delta);
		const size_t n = x.dim();
		Vector d = zero(n), g = zero(n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; double hxplus = h(x.retract(d));
			d(j) -= 2*delta; double hxmin  = h(x.retract(d));
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
		const size_t m = hx.dim(), n = x.dim();
		Vector d = zero(n);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = hx.localCoordinates(h(x.retract(d)));
			d(j) -= 2*delta; Vector hxmin  = hx.localCoordinates(h(x.retract(d)));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}

	/** use a raw C++ function pointer */
	template<class Y, class X>
	Matrix numericalDerivative11(Y (*h)(const X&), const X& x, double delta=1e-5) {
		return numericalDerivative11<Y,X>(boost::bind(h, _1), x, delta);
	}

	/** remapping for double valued functions */
	template<class X>
	Matrix numericalDerivative11(boost::function<double(const X&)> h, const X& x, double delta=1e-5) {
		return numericalDerivative11<LieVector, X>(boost::bind(makeLieVectorD, boost::bind(h, _1)), x, delta);
	}

	template<class X>
	Matrix numericalDerivative11(double (*h)(const X&), const X& x, double delta=1e-5) {
		return numericalDerivative11<LieVector, X>(boost::bind(makeLieVectorD, boost::bind(h, _1)), x, delta);
	}

	/** remapping for vector valued functions */
	template<class X>
	Matrix numericalDerivative11(boost::function<Vector(const X&)> h, const X& x, double delta=1e-5) {
		return numericalDerivative11<LieVector, X>(boost::bind(makeLieVector, boost::bind(h, _1)), x, delta);
	}

	template<class X>
	Matrix numericalDerivative11(Vector (*h)(const X&), const X& x, double delta=1e-5) {
		return numericalDerivative11<LieVector, X>(boost::bind(makeLieVector, boost::bind(h, _1)), x, delta);
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
		const size_t m = hx.dim(), n = x1.dim();
		Vector d = zero(n);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = hx.localCoordinates(h(x1.retract(d),x2));
			d(j) -= 2*delta; Vector hxmin  = hx.localCoordinates(h(x1.retract(d),x2));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}

	/** use a raw C++ function pointer */
	template<class Y, class X1, class X2>
	inline Matrix numericalDerivative21(Y (*h)(const X1&, const X2&),
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative21<Y,X1,X2>(boost::bind(h, _1, _2), x1, x2, delta);
	}

	/** pseudo-partial template specialization for double return values */
	template<class X1, class X2>
	Matrix numericalDerivative21(boost::function<double(const X1&, const X2&)> h,
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative21<LieVector,X1,X2>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2)), x1, x2, delta);
	}

	template<class X1, class X2>
	Matrix numericalDerivative21(double (*h)(const X1&, const X2&),
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative21<LieVector,X1,X2>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2)), x1, x2, delta);
	}

	/** pseudo-partial template specialization for vector return values */
	template<class X1, class X2>
	Matrix numericalDerivative21(boost::function<Vector(const X1&, const X2&)> h,
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative21<LieVector,X1,X2>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2)), x1, x2, delta);
	}

	template<class X1, class X2>
	inline Matrix numericalDerivative21(Vector (*h)(const X1&, const X2&),
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative21<LieVector,X1,X2>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2)), x1, x2, delta);
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
			const X1& x1, const X2& x2, double delta=1e-5) {
		Y hx = h(x1,x2);
		double factor = 1.0/(2.0*delta);
		const size_t m = hx.dim(), n = x2.dim();
		Vector d = zero(n);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = hx.localCoordinates(h(x1,x2.retract(d)));
			d(j) -= 2*delta; Vector hxmin  = hx.localCoordinates(h(x1,x2.retract(d)));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}

	/** use a raw C++ function pointer */
	template<class Y, class X1, class X2>
	inline Matrix numericalDerivative22
	(Y (*h)(const X1&, const X2&), const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative22<Y,X1,X2>(boost::bind(h, _1, _2), x1, x2, delta);
	}

	/** pseudo-partial template specialization for double return values */
	template<class X1, class X2>
	Matrix numericalDerivative22(boost::function<double(const X1&, const X2&)> h,
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative22<LieVector,X1,X2>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2)), x1, x2, delta);
	}

	template<class X1, class X2>
	inline Matrix numericalDerivative22(double (*h)(const X1&, const X2&),
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative22<LieVector,X1,X2>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2)), x1, x2, delta);
	}

	/** pseudo-partial template specialization for vector return values */
	template<class X1, class X2>
	Matrix numericalDerivative22(boost::function<Vector(const X1&, const X2&)> h,
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative22<LieVector,X1,X2>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2)), x1, x2, delta);
	}

	template<class X1, class X2>
	inline Matrix numericalDerivative22(Vector (*h)(const X1&, const X2&),
			const X1& x1, const X2& x2, double delta=1e-5) {
		return numericalDerivative22<LieVector,X1,X2>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2)), x1, x2, delta);
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
		const size_t m = hx.dim(), n = x1.dim();
		Vector d = zero(n);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = hx.localCoordinates(h(x1.retract(d),x2,x3));
			d(j) -= 2*delta; Vector hxmin  = hx.localCoordinates(h(x1.retract(d),x2,x3));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}
	template<class Y, class X1, class X2, class X3>
	inline Matrix numericalDerivative31
	(Y (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative31<Y,X1,X2, X3>(boost::bind(h, _1, _2, _3), x1, x2, x3, delta);
	}

	/** pseudo-partial template specialization for double return values */
	template<class X1, class X2, class X3>
	Matrix numericalDerivative31(boost::function<double(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative31<LieVector,X1,X2,X3>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	template<class X1, class X2, class X3>
	inline Matrix numericalDerivative31(double (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative31<LieVector,X1,X2,X3>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	/** pseudo-partial template specialization for vector return values */
	template<class X1, class X2, class X3>
	Matrix numericalDerivative31(boost::function<Vector(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative31<LieVector,X1,X2,X3>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	template<class X1, class X2, class X3>
	inline Matrix numericalDerivative31(Vector (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative31<LieVector,X1,X2,X3>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	/**
	 * Compute numerical derivative in argument 2 of ternary function
	 * @param h ternary function yielding m-vector
	 * @param x1 n-dimensional first argument value
	 * @param x2 second argument value
	 * @param x3 third argument value
	 * @param delta increment for numerical derivative
	 * @return m*n Jacobian computed via central differencing
	 * All classes Y,X1,X2,X3 need dim, expmap, logmap
	 */
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative32
	(boost::function<Y(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5)
	{
		Y hx = h(x1,x2,x3);
		double factor = 1.0/(2.0*delta);
		const size_t m = hx.dim(), n = x2.dim();
		Vector d = zero(n);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = hx.localCoordinates(h(x1, x2.retract(d),x3));
			d(j) -= 2*delta; Vector hxmin  = hx.localCoordinates(h(x1, x2.retract(d),x3));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}
	template<class Y, class X1, class X2, class X3>
	inline Matrix numericalDerivative32
	(Y (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative32<Y,X1,X2, X3>(boost::bind(h, _1, _2, _3), x1, x2, x3, delta);
	}

	/** pseudo-partial template specialization for double return values */
	template<class X1, class X2, class X3>
	Matrix numericalDerivative32(boost::function<double(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative32<LieVector,X1,X2,X3>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	template<class X1, class X2, class X3>
	inline Matrix numericalDerivative32(double (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative32<LieVector,X1,X2,X3>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	/** pseudo-partial template specialization for vector return values */
	template<class X1, class X2, class X3>
	Matrix numericalDerivative32(boost::function<Vector(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative32<LieVector,X1,X2,X3>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	template<class X1, class X2, class X3>
	inline Matrix numericalDerivative32(Vector (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative32<LieVector,X1,X2,X3>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	/**
	 * Compute numerical derivative in argument 3 of ternary function
	 * @param h ternary function yielding m-vector
	 * @param x1 n-dimensional first argument value
	 * @param x2 second argument value
	 * @param x3 third argument value
	 * @param delta increment for numerical derivative
	 * @return m*n Jacobian computed via central differencing
	 * All classes Y,X1,X2,X3 need dim, expmap, logmap
	 */
	template<class Y, class X1, class X2, class X3>
	Matrix numericalDerivative33
	(boost::function<Y(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5)
	{
		Y hx = h(x1,x2,x3);
		double factor = 1.0/(2.0*delta);
		const size_t m = hx.dim(), n = x3.dim();
		Vector d = zero(n);
		Matrix H = zeros(m,n);
		for (size_t j=0;j<n;j++) {
			d(j) +=   delta; Vector hxplus = hx.localCoordinates(h(x1, x2, x3.retract(d)));
			d(j) -= 2*delta; Vector hxmin  = hx.localCoordinates(h(x1, x2, x3.retract(d)));
			d(j) +=   delta; Vector dh = (hxplus-hxmin)*factor;
			for (size_t i=0;i<m;i++) H(i,j) = dh(i);
		}
		return H;
	}
	template<class Y, class X1, class X2, class X3>
	inline Matrix numericalDerivative33
	(Y (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative33<Y,X1,X2, X3>(boost::bind(h, _1, _2, _3), x1, x2, x3, delta);
	}

	/** pseudo-partial template specialization for double return values */
	template<class X1, class X2, class X3>
	Matrix numericalDerivative33(boost::function<double(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative33<LieVector,X1,X2,X3>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	template<class X1, class X2, class X3>
	inline Matrix numericalDerivative33(double (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative33<LieVector,X1,X2,X3>(
				boost::bind(makeLieVectorD, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	/** pseudo-partial template specialization for vector return values */
	template<class X1, class X2, class X3>
	Matrix numericalDerivative33(boost::function<Vector(const X1&, const X2&, const X3&)> h,
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative33<LieVector,X1,X2,X3>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

	template<class X1, class X2, class X3>
	inline Matrix numericalDerivative33(Vector (*h)(const X1&, const X2&, const X3&),
			const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
		return numericalDerivative33<LieVector,X1,X2,X3>(
				boost::bind(makeLieVector, boost::bind(h, _1, _2, _3)), x1, x2, x3, delta);
	}

  /**
   * Compute numerical Hessian matrix.  Requires a single-argument Lie->scalar
   * function.  This is implemented simply as the derivative of the gradient.
   * @param f A function taking a Lie object as input and returning a scalar
   * @param x The center point for computing the Hessian
   * @param delta The numerical derivative step size
   * @return n*n Hessian matrix computed via central differencing
   */
	template<class X>
	inline Matrix numericalHessian(boost::function<double(const X&)> f, const X& x, double delta=1e-5) {
	  return numericalDerivative11<X>(boost::function<Vector(const X&)>(boost::bind(
	      static_cast<Vector (*)(boost::function<double(const X&)>,const X&, double)>(&numericalGradient<X>),
	      f, _1, delta)), x, delta);
	}

	template<class X>
	inline Matrix numericalHessian(double (*f)(const X&), const X& x, double delta=1e-5) {
	  return numericalHessian(boost::function<double(const X&)>(f), x, delta);
	}


  /** Helper class that computes the derivative of f w.r.t. x1, centered about
   * x1_, as a function of x2
   */
  template<class X1, class X2>
  class G_x1 {
    const boost::function<double(const X1&, const X2&)>& f_;
    const X1& x1_;
    double delta_;
  public:
    G_x1(const boost::function<double(const X1&, const X2&)>& f, const X1& x1, double delta) : f_(f), x1_(x1), delta_(delta) {}
    Vector operator()(const X2& x2) {
      return numericalGradient<X1>(boost::function<double (const X1&)>(boost::bind(f_, _1, x2)), x1_, delta_);
    }
  };

  template<class X1, class X2>
  inline Matrix numericalHessian212(boost::function<double(const X1&, const X2&)> f, const X1& x1, const X2& x2, double delta=1e-5) {
    G_x1<X1,X2> g_x1(f, x1, delta);
    return numericalDerivative11<X2>(boost::function<Vector (const X2&)>(boost::bind<Vector>(boost::ref(g_x1), _1)), x2, delta);
  }


  template<class X1, class X2>
  inline Matrix numericalHessian212(double (*f)(const X1&, const X2&), const X1& x1, const X2& x2, double delta=1e-5) {
    return numericalHessian212(boost::function<double (const X1&, const X2&)>(f), x1, x2, delta);
  }


  template<class X1, class X2>
  inline Matrix numericalHessian211(boost::function<double(const X1&, const X2&)> f, const X1& x1, const X2& x2, double delta=1e-5) {

    Vector (*numGrad)(boost::function<double(const X1&)>, const X1&, double) = &numericalGradient<X1>;
    boost::function<double (const X1&)> f2(boost::bind(f, _1, x2));

    return numericalDerivative11<X1>(boost::function<Vector (const X1&)>(boost::bind(numGrad, f2, _1, delta)), x1, delta);
  }


  template<class X1, class X2>
  inline Matrix numericalHessian211(double (*f)(const X1&, const X2&), const X1& x1, const X2& x2, double delta=1e-5) {
    return numericalHessian211(boost::function<double (const X1&, const X2&)>(f), x1, x2, delta);
  }


  template<class X1, class X2>
  inline Matrix numericalHessian222(boost::function<double(const X1&, const X2&)> f, const X1& x1, const X2& x2, double delta=1e-5) {

    Vector (*numGrad)(boost::function<double(const X2&)>, const X2&, double) = &numericalGradient<X2>;
    boost::function<double (const X2&)> f2(boost::bind(f, x1, _1));

    return numericalDerivative11<X2>(boost::function<Vector (const X2&)>(boost::bind(numGrad, f2, _1, delta)), x2, delta);
  }


  template<class X1, class X2>
  inline Matrix numericalHessian222(double (*f)(const X1&, const X2&), const X1& x1, const X2& x2, double delta=1e-5) {
    return numericalHessian222(boost::function<double (const X1&, const X2&)>(f), x1, x2, delta);
  }

  /**
   * Numerical Hessian for tenary functions
   */
  /* **************************************************************** */
  template<class X1, class X2, class X3>
  inline Matrix numericalHessian311(boost::function<double(const X1&, const X2&, const X3&)> f,
      const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {

    Vector (*numGrad)(boost::function<double(const X1&)>, const X1&, double) = &numericalGradient<X1>;
    boost::function<double (const X1&)> f2(boost::bind(f, _1, x2, x3));

    return numericalDerivative11<X1>(boost::function<Vector (const X1&)>(boost::bind(numGrad, f2, _1, delta)), x1, delta);
  }

  template<class X1, class X2, class X3>
  inline Matrix numericalHessian311(double (*f)(const X1&, const X2&, const X3&), const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian311(boost::function<double (const X1&, const X2&, const X3&)>(f), x1, x2, x3, delta);
  }

  /* **************************************************************** */
  template<class X1, class X2, class X3>
  inline Matrix numericalHessian322(boost::function<double(const X1&, const X2&, const X3&)> f,
      const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {

    Vector (*numGrad)(boost::function<double(const X2&)>, const X2&, double) = &numericalGradient<X2>;
    boost::function<double (const X2&)> f2(boost::bind(f, x1, _1, x3));

    return numericalDerivative11<X2>(boost::function<Vector (const X2&)>(boost::bind(numGrad, f2, _1, delta)), x2, delta);
  }

  template<class X1, class X2, class X3>
  inline Matrix numericalHessian322(double (*f)(const X1&, const X2&, const X3&), const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian322(boost::function<double (const X1&, const X2&, const X3&)>(f), x1, x2, x3, delta);
  }

  /* **************************************************************** */
  template<class X1, class X2, class X3>
  inline Matrix numericalHessian333(boost::function<double(const X1&, const X2&, const X3&)> f,
      const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {

    Vector (*numGrad)(boost::function<double(const X3&)>, const X3&, double) = &numericalGradient<X3>;
    boost::function<double (const X3&)> f2(boost::bind(f, x1, x2, _1));

    return numericalDerivative11<X3>(boost::function<Vector (const X3&)>(boost::bind(numGrad, f2, _1, delta)), x3, delta);
  }

  template<class X1, class X2, class X3>
  inline Matrix numericalHessian333(double (*f)(const X1&, const X2&, const X3&), const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian333(boost::function<double (const X1&, const X2&, const X3&)>(f), x1, x2, x3, delta);
  }

  /* **************************************************************** */
  template<class X1, class X2, class X3>
  inline Matrix numericalHessian312(boost::function<double(const X1&, const X2&, const X3&)> f, const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian212<X1,X2>(boost::function<double (const X1&, const X2&)>(boost::bind(f, _1, _2, x3)), x1, x2, delta );
  }

  template<class X1, class X2, class X3>
  inline Matrix numericalHessian313(boost::function<double(const X1&, const X2&, const X3&)> f, const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian212<X1,X3>(boost::function<double (const X1&, const X3&)>(boost::bind(f, _1, x2, _2)), x1, x3, delta );
  }

  template<class X1, class X2, class X3>
  inline Matrix numericalHessian323(boost::function<double(const X1&, const X2&, const X3&)> f, const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian212<X2,X3>(boost::function<double (const X2&, const X3&)>(boost::bind(f, x1, _1, _2)), x2, x3, delta );
  }

  /* **************************************************************** */
  template<class X1, class X2, class X3>
  inline Matrix numericalHessian312(double (*f)(const X1&, const X2&, const X3&), const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian312(boost::function<double (const X1&, const X2&, const X3&)>(f), x1, x2, x3, delta);
  }

  template<class X1, class X2, class X3>
  inline Matrix numericalHessian313(double (*f)(const X1&, const X2&, const X3&), const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian313(boost::function<double (const X1&, const X2&, const X3&)>(f), x1, x2, x3, delta);
  }

  template<class X1, class X2, class X3>
  inline Matrix numericalHessian323(double (*f)(const X1&, const X2&, const X3&), const X1& x1, const X2& x2, const X3& x3, double delta=1e-5) {
    return numericalHessian323(boost::function<double (const X1&, const X2&, const X3&)>(f), x1, x2, x3, delta);
  }
}
