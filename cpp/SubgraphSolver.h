/*
 * SubgraphSolver.h
 * Created on: Dec 31, 2009
 * @author: Frank Dellaert
 */

#pragma once

#include "GaussianFactorGraph.h"
#include "GaussianBayesNet.h"
#include "Ordering.h"
#include "SubgraphPreconditioner.h"

namespace gtsam {

  /**
   * A nonlinear system solver using subgraph preconditioning conjugate gradient
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> VectorConfig
   */
	template<class G, class T>
	class SubgraphSolver {

	private:
		typedef typename T::Key Key;
		typedef typename G::Constraint Constraint;
		typedef typename G::Pose Pose;

		// TODO not hardcode
		static const size_t maxIterations_=100;
		static const bool verbose_=false;
		static const double epsilon_=1e-4, epsilon_abs_=1e-5;

		/* the ordering derived from the spanning tree */
		boost::shared_ptr<Ordering> ordering_;

		/* the solution computed from the first subgraph */
		boost::shared_ptr<T> theta_bar_;

		G T_, C_;

	public:
		SubgraphSolver() {}

		SubgraphSolver(const G& g, const T& theta0);

		void initialize(const G& g, const T& theta0);

		boost::shared_ptr<Ordering> ordering() const { return ordering_; }

		boost::shared_ptr<T> theta_bar() const { return theta_bar_; }

		/**
		 * linearize the non-linear graph around the current config and build the subgraph preconditioner systme
		 */
		boost::shared_ptr<SubgraphPreconditioner> linearize(const G& g, const T& theta_bar) const;

  	/**
  	 * solve for the optimal displacement in the tangent space, and then solve
  	 * the resulted linear system
  	 */
  	VectorConfig optimize(SubgraphPreconditioner& system) const;
	};

	template<class G, class T> const size_t SubgraphSolver<G,T>::maxIterations_;
	template<class G, class T> const bool SubgraphSolver<G,T>::verbose_;
	template<class G, class T> const double SubgraphSolver<G,T>::epsilon_;
	template<class G, class T> const double	SubgraphSolver<G,T>::epsilon_abs_;

} // nsamespace gtsam
