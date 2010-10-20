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
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

  /**
   * A nonlinear system solver using subgraph preconditioning conjugate gradient
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> VectorValues
   */
	template<class GRAPH, class VALUES>
	class SubgraphSolver {

	private:
		typedef typename VALUES::Key Key;
		typedef typename GRAPH::Constraint Constraint;
		typedef typename GRAPH::Pose Pose;

		// TODO not hardcode
		static const size_t maxIterations_=100;
		static const double epsilon_=1e-4, epsilon_abs_=1e-5;
		static const bool verbose_=true;

		/* the ordering derived from the spanning tree */
		boost::shared_ptr<Ordering> ordering_;

		/* the solution computed from the first subgraph */
		boost::shared_ptr<VALUES> theta_bar_;

		GRAPH T_, C_;

	public:
		SubgraphSolver() {}

		SubgraphSolver(const GRAPH& G, const VALUES& theta0);

		void initialize(const GRAPH& G, const VALUES& theta0);

		boost::shared_ptr<Ordering> ordering() const { return ordering_; }

		boost::shared_ptr<VALUES> theta_bar() const { return theta_bar_; }

		/**
		 * linearize the non-linear graph around the current config and build the subgraph preconditioner systme
		 */
		boost::shared_ptr<SubgraphPreconditioner> linearize(const GRAPH& G, const VALUES& theta_bar) const;


		/**
		 * solve for the optimal displacement in the tangent space, and then solve
		 * the resulted linear system
		 */
		VectorValues optimize(SubgraphPreconditioner& system) const;

		boost::shared_ptr<SubgraphSolver> prepareLinear(const SubgraphPreconditioner& fg) const {
			return boost::shared_ptr<SubgraphSolver>(new SubgraphSolver(*this));
		}


	  	/** expmap the Values given the stored Ordering */
	  	VALUES expmap(const VALUES& config, const VectorValues& delta) const {
	  	  return config.expmap(delta, *ordering_);
	  	}
	};

	template<class GRAPH, class VALUES> const size_t SubgraphSolver<GRAPH,VALUES>::maxIterations_;
	template<class GRAPH, class VALUES> const bool SubgraphSolver<GRAPH,VALUES>::verbose_;
	template<class GRAPH, class VALUES> const double SubgraphSolver<GRAPH,VALUES>::epsilon_;
	template<class GRAPH, class VALUES> const double SubgraphSolver<GRAPH,VALUES>::epsilon_abs_;

} // nsamespace gtsam
