/*
 * SubgraphPreconditioner.h
 * Created on: Dec 31, 2009
 * @author: Frank Dellaert
 */

#ifndef SUBGRAPHPRECONDITIONER_H_
#define SUBGRAPHPRECONDITIONER_H_

#include "GaussianFactorGraph.h"
#include "GaussianBayesNet.h"
#include "Ordering.h"

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

		/** gradient = y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar) */
		VectorConfig gradient(const VectorConfig& y) const;

		/** Apply operator A */
		Errors operator*(const VectorConfig& y) const;

		/** Apply operator A' */
		VectorConfig operator^(const Errors& e) const;

		/** print the object */
		void print(const std::string& s = "SubgraphPreconditioner") const;
	};

  /**
   * A linear system solver using subgraph preconditioning conjugate gradient
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> VectorConfig
   */
	template <class NonlinearGraph, class Config>
	class SubgraphPCG {

	private:
		typedef typename Config::Key Key;
		typedef typename NonlinearGraph::Constraint Constraint;
		typedef typename NonlinearGraph::Pose Pose;

		const size_t maxIterations_;
		const bool verbose_;
		const double epsilon_, epsilon_abs_;

		/* the ordering derived from the spanning tree */
		boost::shared_ptr<Ordering> ordering_;

		/* the solution computed from the first subgraph */
		boost::shared_ptr<Config> theta_bar_;

		NonlinearGraph T_, C_;

	public:
		// kai: this constructor is for compatible with Factorization
		SubgraphPCG() { throw std::runtime_error("SubgraphPCG: this constructor is only for compatibility!");}

		SubgraphPCG(const NonlinearGraph& G, const Config& config);

		boost::shared_ptr<Ordering> ordering() const { return ordering_; }

		boost::shared_ptr<Config> theta_bar() const { return theta_bar_; }

  	/**
  	 * solve for the optimal displacement in the tangent space, and then solve
  	 * the resulted linear system
  	 */
  	VectorConfig optimize(GaussianFactorGraph& fg, const Ordering& ordering) const {
  		throw std::runtime_error("SubgraphPCG:: optimize is not supported!");
  	}

		/**
		 * linearize the non-linear graph around the current config,
		 */
  	VectorConfig linearizeAndOptimize(const NonlinearGraph& g, const Config& config,
  			const Ordering& ordering) const;
	};
} // nsamespace gtsam

#endif /* SUBGRAPHPRECONDITIONER_H_ */
