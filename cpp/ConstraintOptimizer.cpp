/**
 * @file ConstraintOptimizer.cpp
 * @author Alex Cunningham
 */

#include <ConstraintOptimizer.h>

using namespace std;
using namespace gtsam;


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

