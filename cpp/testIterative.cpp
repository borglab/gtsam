/**
 *  @file   testIterative.cpp
 *  @brief  Unit tests for iterative methods
 *  @author Frank Dellaert
 **/

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "smallExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
VectorConfig gradient(const GaussianFactorGraph& Ab, const VectorConfig& x) {
	return Ab.gradient(x);
}

/* ************************************************************************* */
typedef pair<Matrix,Vector> System;

/**
 * gradient of objective function 0.5*|Ax-b|^2 at x = A'*(Ax-b)
 */
Vector gradient(const System& Ab, const Vector& x) {
	const Matrix& A = Ab.first;
	const Vector& b = Ab.second;
	return A ^ (A * x - b);
}

/**
 * Apply operator A
 */
Vector operator*(const System& Ab, const Vector& x) {
	const Matrix& A = Ab.first;
	return A * x;
}

/**
 * Apply operator A^T
 */
Vector operator^(const System& Ab, const Vector& x) {
	const Matrix& A = Ab.first;
	return A ^ x;
}

/* ************************************************************************* */
// Method of conjugate gradients (CG)
// "System" class S needs gradient(S,v), e=S*v, v=S^e
// "Vector" class V needs dot(v,v), -v, v+v, s*v
// "Vector" class E needs dot(v,v)
template <class S, class V, class E>
V CGD(const S& Ab, V x, double threshold = 1e-9) {

	// Start with g0 = A'*(A*x0-b), d0 = - g0
	// i.e., first step is in direction of negative gradient
	V g = gradient(Ab, x);
	V d = -g;
	double prev_dotg = dot(g, g);

	// loop max n times
	size_t n = x.size();
	for (int k = 1; k <= n; k++) {

		// calculate optimal step-size
		E Ad = Ab * d;
		double alpha = -dot(d, g) / dot(Ad, Ad);

		// do step in new search direction
		x = x + alpha * d;
		if (k == n) break;

		// update gradient
		g = g + alpha * (Ab ^ Ad);

		// check for convergence
		double dotg = dot(g, g);
		if (dotg < threshold) break;

		// calculate new search direction
		double beta = dotg / prev_dotg;
		prev_dotg = dotg;
		d = -g + beta * d;
	}
	return x;
}

/* ************************************************************************* */
// Method of conjugate gradients (CG), Matrix version
Vector conjugateGradientDescent(const Matrix& A, const Vector& b,
		const Vector& x, double threshold = 1e-9) {
	System Ab = make_pair(A, b);
	return CGD<System, Vector, Vector> (Ab, x);
}

/* ************************************************************************* */
// Method of conjugate gradients (CG), Gaussian Factor Graph version
VectorConfig conjugateGradientDescent(const GaussianFactorGraph& Ab,
		const VectorConfig& x, double threshold = 1e-9) {
	return CGD<GaussianFactorGraph, VectorConfig, Errors> (Ab, x);
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, gradientDescent )
{
	// Expected solution
	Ordering ord;
  ord += "l1","x1","x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
  VectorConfig expected = fg.optimize(ord); // destructive

  // Do gradient descent
  GaussianFactorGraph fg2 = createGaussianFactorGraph();
  VectorConfig zero = createZeroDelta();
	VectorConfig actual = fg2.gradientDescent(zero);
	CHECK(assert_equal(expected,actual,1e-2));

  // Do conjugate gradient descent
	//VectorConfig actual2 = fg2.conjugateGradientDescent(zero);
	VectorConfig actual2 = conjugateGradientDescent(fg2,zero);
	CHECK(assert_equal(expected,actual2,1e-2));

	// Do conjugate gradient descent, Matrix version
	Matrix A;Vector b;
	boost::tie(A,b) = fg2.matrix(ord);
//	print(A,"A");
//	print(b,"b");
	Vector x0 = gtsam::zero(6);
	Vector actualX = conjugateGradientDescent(A,b,x0);
	Vector expectedX = Vector_(6, -0.1, 0.1, -0.1, -0.1, 0.1, -0.2);
	CHECK(assert_equal(expectedX,actualX,1e-9));

	// Do conjugate gradient descent, System version
	System Ab = make_pair(A,b);
	Vector actualX2 = CGD<System,Vector,Vector>(Ab,x0);
	CHECK(assert_equal(expectedX,actualX2,1e-9));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
