/*
 * BayesNetPreconditioner.h
 * Created on: Dec 31, 2009
 * @Author: Frank Dellaert
 */

#ifndef BAYESNETPRECONDITIONER_H_
#define BAYESNETPRECONDITIONER_H_

#include "GaussianFactorGraph.h"
#include "GaussianBayesNet.h"

namespace gtsam {

	/**
	 * Upper-triangular preconditioner R for the system |A*x-b|^2
	 * The new system will be |A*inv(R)*y-b|^2, i.e., R*x=y
	 * This class can solve for x=inv(R)*y by back-substituting R*x=y
	 * and also apply the chain rule gy=inv(R')*gx by solving R'*gy=gx.
	 * This is not used currently, just to debug operators below
	 */
	class BayesNetPreconditioner {

		// The original system
		const GaussianFactorGraph& Ab_;

		// The preconditioner
		const GaussianBayesNet& Rd_;

	public:

		/** Constructor */
		BayesNetPreconditioner(const GaussianFactorGraph& Ab,
				const GaussianBayesNet& Rd);

		// R*x = y by solving x=inv(R)*y
		VectorConfig backSubstitute(const VectorConfig& y) const;

		// gy=inv(L)*gx by solving L*gy=gx.
		VectorConfig backSubstituteTranspose(const VectorConfig& gx) const;

		/* x = inv(R)*y */
		inline VectorConfig x(const VectorConfig& y) const {
			return backSubstitute(y);
		}

		/* error, given y */
		double error(const VectorConfig& y) const;

		/** gradient */
		VectorConfig gradient(const VectorConfig& y) const;

		/** Apply operator A */
		Errors operator*(const VectorConfig& y) const;

		/** Apply operator A' */
		VectorConfig operator^(const Errors& e) const;
	};

} // namespace gtsam

#endif /* BAYESNETPRECONDITIONER_H_ */
