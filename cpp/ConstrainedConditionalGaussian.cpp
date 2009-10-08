/**
 * @file ConstrainedConditionalGaussian.cpp
 * @brief Implements the constrained version of the conditional gaussian class,
 * primarily handling the possible solutions
 * @author Alex Cunningham
 */

#include <boost/numeric/ublas/lu.hpp>
#include "ConstrainedConditionalGaussian.h"
#include "Matrix.h"

using namespace gtsam;
using namespace std;

ConstrainedConditionalGaussian::ConstrainedConditionalGaussian() {

}

ConstrainedConditionalGaussian::ConstrainedConditionalGaussian(const Vector& v) :
	ConditionalGaussian(v, eye(v.size())) {
}

ConstrainedConditionalGaussian::ConstrainedConditionalGaussian(const Vector& b,
		const Matrix& A) :
	ConditionalGaussian(b, A) {
}

ConstrainedConditionalGaussian::ConstrainedConditionalGaussian(const Vector& b,
		const Matrix& A1, const std::string& parent, const Matrix& A2) :
	ConditionalGaussian(b, A1, parent, A2) {
}

ConstrainedConditionalGaussian::ConstrainedConditionalGaussian(const Vector& b,
		const Matrix& A1, const std::string& parentY, const Matrix& A2,
		const std::string& parentZ, const Matrix& A3)
: ConditionalGaussian(b, A1, parentY, A2, parentZ, A3)
{
}

ConstrainedConditionalGaussian::ConstrainedConditionalGaussian(const Matrix& A1,
			const std::map<std::string, Matrix>& parents, const Vector& b)
: ConditionalGaussian(b, A1, parents)
{
}


ConstrainedConditionalGaussian::ConstrainedConditionalGaussian(
		const ConstrainedConditionalGaussian& df) {
}

Vector ConstrainedConditionalGaussian::solve(const FGConfig& x) const {
	// sum the RHS
	Vector rhs = d_;
	for (map<string, Matrix>::const_iterator it = parents_.begin(); it
			!= parents_.end(); it++) {
		const string& j = it->first;
		const Matrix& Aj = it->second;
		rhs -= Aj * x[j];
	}

	// verify invertibility of A matrix
	Matrix A = R_;
	Matrix b = Matrix_(rhs.size(), 1, rhs);
	if (A.size1() != A.size2())
		throw invalid_argument("Matrix A is not invertible - non-square matrix");
	using namespace boost::numeric::ublas;
	if (lu_factorize(A))
		throw invalid_argument("Matrix A is singular");

	// get the actual solution
	//FIXME: This is just the Matrix::solve() function, but due to name conflicts
	// the compiler won't find the real version in Matrix.h
	lu_substitute<const Matrix, Matrix> (A, b);
	return Vector_(b);

	//TODO: Handle overdetermined case

	//TODO: Handle underdetermined case

}
