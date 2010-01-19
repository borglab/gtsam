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

	public:
		typedef boost::shared_ptr<const GaussianBayesNet> sharedBayesNet;
		typedef boost::shared_ptr<const GaussianFactorGraph> sharedFG;
		typedef boost::shared_ptr<const VectorConfig> sharedConfig;
		typedef boost::shared_ptr<const Errors> sharedErrors;

	private:
		sharedBayesNet Rc1_;
		sharedFG Ab2_;
		sharedConfig xbar_;
		sharedErrors b2bar_; /** b2 - A2*xbar */

	public:

		/**
		 * Constructor
		 * @param Rc1: the Bayes Net R1*x=c1
		 * @param Ab2: the Graph A2*x=b2
		 * @param xbar: the solution to R1*x=c1
		 */
		SubgraphPreconditioner(sharedBayesNet& Rc1,	sharedFG& Ab2, sharedConfig& xbar);

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
   * A nonlinear system solver using subgraph preconditioning conjugate gradient
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> VectorConfig
   */
	template<class G, class T>
	class SubgraphPCG {

	private:
		typedef typename T::Key Key;
		typedef typename G::Constraint Constraint;
		typedef typename G::Pose Pose;

		const size_t maxIterations_;
		const bool verbose_;
		const double epsilon_, epsilon_abs_;

		/* the ordering derived from the spanning tree */
		boost::shared_ptr<Ordering> ordering_;

		/* the solution computed from the first subgraph */
		boost::shared_ptr<T> theta_bar_;

		G T_, C_;

	public:
		// kai: this constructor is for compatible with Factorization
		SubgraphPCG() { throw std::runtime_error("SubgraphPCG: this constructor is only for compatibility!");}

		SubgraphPCG(const G& g, const T& config);

		boost::shared_ptr<Ordering> ordering() const { return ordering_; }

		boost::shared_ptr<T> theta_bar() const { return theta_bar_; }

		/**
		 * linearize the non-linear graph around the current config and build the subgraph preconditioner systme
		 */
		SubgraphPreconditioner linearize(const G& g, const T& theta_bar) const;

  	/**
  	 * solve for the optimal displacement in the tangent space, and then solve
  	 * the resulted linear system
  	 */
  	VectorConfig optimize(SubgraphPreconditioner& system, const Ordering& ordering) const;
	};
} // nsamespace gtsam

#endif /* SUBGRAPHPRECONDITIONER_H_ */
