/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * BayesNetPreconditioner.h
 * Created on: Dec 31, 2009
 * @Author: Frank Dellaert
 */

#ifndef BAYESNETPRECONDITIONER_H_
#define BAYESNETPRECONDITIONER_H_

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>

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
		VectorValues backSubstitute(const VectorValues& y) const;

		// gy=inv(L)*gx by solving L*gy=gx.
		VectorValues backSubstituteTranspose(const VectorValues& gx) const;

		/* x = inv(R)*y */
		inline VectorValues x(const VectorValues& y) const {
			return backSubstitute(y);
		}

		/* error, given y */
		double error(const VectorValues& y) const;

		/** gradient */
		VectorValues gradient(const VectorValues& y) const;

		/** Apply operator A */
		Errors operator*(const VectorValues& y) const;

		/** In-place version that overwrites e */
		void multiplyInPlace(const VectorValues& y, Errors& e) const;

		/** Apply operator A' */
		VectorValues operator^(const Errors& e) const;

		/** BLAS level 2 equivalent y += alpha*inv(R')*A'*e */
		void transposeMultiplyAdd(double alpha, const Errors& e, VectorValues& y) const;
	};

} // namespace gtsam

#endif /* BAYESNETPRECONDITIONER_H_ */
