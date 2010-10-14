/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * SubgraphSolver.h
 * Created on: Dec 31, 2009
 * @author: Frank Dellaert
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/SubgraphPreconditioner.h>

namespace gtsam {

  /**
   * A nonlinear system solver using subgraph preconditioning conjugate gradient
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> VectorValues
   */
	template<class Graph, class Values>
	class SubgraphSolver {

	private:
		typedef typename Values::Key Key;
		typedef typename Graph::Constraint Constraint;
		typedef typename Graph::Pose Pose;

		// TODO not hardcode
		static const size_t maxIterations_=100;
		static const bool verbose_=false;
		static const double epsilon_=1e-4, epsilon_abs_=1e-5;

		/* the ordering derived from the spanning tree */
		boost::shared_ptr<Ordering> ordering_;

		/* the solution computed from the first subgraph */
		boost::shared_ptr<Values> theta_bar_;

		Graph T_, C_;

	public:
		SubgraphSolver() {}

		SubgraphSolver(const Graph& G, const Values& theta0);

		void initialize(const Graph& G, const Values& theta0);

		boost::shared_ptr<Ordering> ordering() const { return ordering_; }

		boost::shared_ptr<Values> theta_bar() const { return theta_bar_; }

		/**
		 * linearize the non-linear graph around the current config and build the subgraph preconditioner systme
		 */
		boost::shared_ptr<SubgraphPreconditioner> linearize(const Graph& G, const Values& theta_bar) const;

  	/**
  	 * solve for the optimal displacement in the tangent space, and then solve
  	 * the resulted linear system
  	 */
  	VectorValues optimize(SubgraphPreconditioner& system) const;

  	boost::shared_ptr<SubgraphSolver> prepareLinear(const SubgraphPreconditioner& fg) const {
  		return boost::shared_ptr<SubgraphSolver>(new SubgraphSolver(*this));
  	}
	};

	template<class Graph, class Values> const size_t SubgraphSolver<Graph,Values>::maxIterations_;
	template<class Graph, class Values> const bool SubgraphSolver<Graph,Values>::verbose_;
	template<class Graph, class Values> const double SubgraphSolver<Graph,Values>::epsilon_;
	template<class Graph, class Values> const double	SubgraphSolver<Graph,Values>::epsilon_abs_;

} // nsamespace gtsam
