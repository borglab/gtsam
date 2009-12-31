/*
 * BayesNetPreconditioner.cpp
 * Created on: Dec 31, 2009
 * @Author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "BayesNetPreconditioner.h"

namespace gtsam {

	/* ************************************************************************* */
	BayesNetPreconditioner::BayesNetPreconditioner(const GaussianFactorGraph& Ab,
			const GaussianBayesNet& Rd) :
		Ab_(Ab), Rd_(Rd) {
	}

	/* ************************************************************************* */
	// R*x = y by solving x=inv(R)*y
	VectorConfig BayesNetPreconditioner::backSubstitute(const VectorConfig& y) const {
		return gtsam::backSubstitute(Rd_, y);
	}

	/* ************************************************************************* */
	// gy=inv(L)*gx by solving L*gy=gx.
	VectorConfig BayesNetPreconditioner::backSubstituteTranspose(
			const VectorConfig& gx) const {
		return gtsam::backSubstituteTranspose(Rd_, gx);
	}

	/* ************************************************************************* */
	double BayesNetPreconditioner::error(const VectorConfig& y) const {
		return Ab_.error(x(y));
	}

	/* ************************************************************************* */
	// gradient is inv(R')*A'*(A*inv(R)*y-b),
	VectorConfig BayesNetPreconditioner::gradient(const VectorConfig& y) const {
		VectorConfig gx = Ab_ ^ Ab_.errors(x(y));
		return gtsam::backSubstituteTranspose(Rd_, gx);
	}

	/* ************************************************************************* */
	// Apply operator *
	Errors BayesNetPreconditioner::operator*(const VectorConfig& y) const {
		return Ab_ * x(y);
	}

	/* ************************************************************************* */
	// Apply operator inv(R')*A'*e
	VectorConfig BayesNetPreconditioner::operator^(const Errors& e) const {
		VectorConfig x = Ab_ ^ e; // x = A'*e2
		return gtsam::backSubstituteTranspose(Rd_, x);
	}

	/* ************************************************************************* */

} // namespace gtsam


