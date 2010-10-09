/*
 * BayesNetPreconditioner.cpp
 * Created on: Dec 31, 2009
 * @Author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <gtsam/linear/BayesNetPreconditioner.h>

namespace gtsam {

	/* ************************************************************************* */
	BayesNetPreconditioner::BayesNetPreconditioner(const GaussianFactorGraph& Ab,
			const GaussianBayesNet& Rd) :
		Ab_(Ab), Rd_(Rd) {
	}

	/* ************************************************************************* */
	// R*x = y by solving x=inv(R)*y
	VectorValues BayesNetPreconditioner::backSubstitute(const VectorValues& y) const {
		return gtsam::backSubstitute(Rd_, y);
	}

	/* ************************************************************************* */
	// gy=inv(L)*gx by solving L*gy=gx.
	VectorValues BayesNetPreconditioner::backSubstituteTranspose(
			const VectorValues& gx) const {
		return gtsam::backSubstituteTranspose(Rd_, gx);
	}

	/* ************************************************************************* */
	double BayesNetPreconditioner::error(const VectorValues& y) const {
		return Ab_.error(x(y));
	}

	/* ************************************************************************* */
	// gradient is inv(R')*A'*(A*inv(R)*y-b),
	VectorValues BayesNetPreconditioner::gradient(const VectorValues& y) const {
		VectorValues gx = VectorValues::zero(y);
		Errors e = Ab_.errors(x(y));
		Ab_.transposeMultiplyAdd(1.0,e,gx);
		return gtsam::backSubstituteTranspose(Rd_, gx);
	}

	/* ************************************************************************* */
	// Apply operator *
	Errors BayesNetPreconditioner::operator*(const VectorValues& y) const {
		return Ab_ * x(y);
	}

	/* ************************************************************************* */
	// In-place version that overwrites e
	// TODO: version that takes scratch space for x
	void BayesNetPreconditioner::multiplyInPlace(const VectorValues& y, Errors& e) const {
		VectorValues x = y;
		backSubstituteInPlace(Rd_,x);
		Ab_.multiplyInPlace(x,e);
	}

	/* ************************************************************************* */
	// Apply operator inv(R')*A'*e
	VectorValues BayesNetPreconditioner::operator^(const Errors& e) const {
		VectorValues x = Ab_ ^ e; // x = A'*e2
		return gtsam::backSubstituteTranspose(Rd_, x);
	}

	/* ************************************************************************* */
	// y += alpha*inv(R')*A'*e
	void BayesNetPreconditioner::transposeMultiplyAdd(double alpha,
			const Errors& e, VectorValues& y) const {
		VectorValues x = VectorValues::zero(y);
		Ab_.transposeMultiplyAdd(1.0,e,x); // x += A'*e
		axpy(alpha, gtsam::backSubstituteTranspose(Rd_, x), y); // y += alpha*inv(R')*x
	}

	/* ************************************************************************* */

} // namespace gtsam


