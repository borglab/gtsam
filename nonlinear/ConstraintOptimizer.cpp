/**
 * @file ConstraintOptimizer.cpp
 * @author Alex Cunningham
 */

/** IMPORTANT NOTE: this file is only compiled when LDL is available */

#include <gtsam/nonlinear/ConstraintOptimizer.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
void gtsam::BFGSEstimator::update(const Vector& dfx, const boost::optional<Vector&> step) {
	if (step) {
		Vector Bis = B_ * *step,
				y = dfx - prev_dfx_;
		B_ = B_ + outer_prod(y, y) / inner_prod(y, *step)
                - outer_prod(Bis, Bis) / inner_prod(*step, Bis);
	}
	prev_dfx_ = dfx;
}

/* ************************************************************************* */
pair<Vector, Vector> gtsam::solveCQP(const Matrix& B, const Matrix& A,
								     const Vector& g, const Vector& h) {
	// find the dimensions
	size_t n = B.size1(), p = A.size2();

	// verify matrices
	if (n != B.size2())
		throw invalid_argument("solveCQP: B matrix is not square!");
	if (A.size1() != n)
		throw invalid_argument("solveCQP: A matrix needs m = B.size1()");

	// construct G matrix
	Matrix G = zeros(n+p, n+p);
	insertSub(G, B, 0, 0);
	insertSub(G, A, 0, n);
	insertSub(G, trans(A), n, 0);

	Vector rhs = zero(n+p);
	subInsert(rhs, -1.0*g, 0);
	subInsert(rhs, -1.0*h, n);

	// solve the system with the LDL solver
	Vector fullResult = solve_ldl(G, rhs);

	return make_pair(sub(fullResult, 0, n), sub(fullResult, n, n+p));
}

/* ************************************************************************* */
Vector gtsam::linesearch(const Vector& x0, const Vector& delta,
		double (*penalty)(const Vector&), size_t maxIt) {

	// calculate the initial error
	double init_error = penalty(x0);
	Vector step = delta;
	for (size_t i=0; i<maxIt; ++i) {
		Vector x = x0 + step;
		double cur_error = penalty(x);
		if (cur_error < init_error) // if we have improved, return the step
			return step;
		else { // otherwise, make a smaller step
			step = 0.5 * step;
		}
	}

	// TODO: should do something clever here
	return step;
}

