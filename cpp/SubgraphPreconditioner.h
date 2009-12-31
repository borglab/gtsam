/*
 * SubgraphPreconditioner.h
 * Created on: Dec 31, 2009
 * @author: Frank Dellaert
 */

#ifndef SUBGRAPHPRECONDITIONER_H_
#define SUBGRAPHPRECONDITIONER_H_

#include "GaussianFactorGraph.h"
#include "GaussianBayesNet.h"

namespace gtsam {

	/**
	 * Subgraph conditioner class, as explained in the RSS 2010 submission.
	 * Starting with a graph A*x=b, we split it in two systems A1*x=b1 and A2*x=b2
	 * We solve R1*x=c1, and make the substitution y=R1*x-c1.
	 * To use the class, give the Bayes Net R1*x=c1 and Graph A2*x=b2.
	 * Then solve for yhat using CG, and solve for xhat = system.x(yhat).
	 */
	class SubgraphPreconditioner {

	private:

		const GaussianBayesNet& Rc1_;

		const GaussianFactorGraph& Ab2_;
		const VectorConfig& xbar_;
		const Errors b2bar_; /** b2 - A2*xbar */

	public:

		/**
		 * Constructor
		 * @param Rc1: the Bayes Net R1*x=c1
		 * @param Ab2: the Graph A2*x=b2
		 * @param xbar: the solution to R1*x=c1
		 */
		SubgraphPreconditioner(const GaussianBayesNet& Rc1,
				const GaussianFactorGraph& Ab2, const VectorConfig& xbar);

		/* x = xbar + inv(R1)*y */
		VectorConfig x(const VectorConfig& y) const;

		/* error, given y */
		double error(const VectorConfig& y) const;

		/** gradient */
		VectorConfig gradient(const VectorConfig& y) const;

		/** Apply operator A */
		Errors operator*(const VectorConfig& y) const;

		/** Apply operator A' */
		VectorConfig operator^(const Errors& e) const;
	};

} // nsamespace gtsam

#endif /* SUBGRAPHPRECONDITIONER_H_ */
