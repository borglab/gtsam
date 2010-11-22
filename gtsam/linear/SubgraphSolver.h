/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once


#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

  /**
   * A nonlinear system solver using subgraph preconditioning conjugate gradient
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> VectorValues
   */
	template<class GRAPH, class LINEAR, class VALUES>
	class SubgraphSolver : public IterativeSolver {

	private:
		typedef typename VALUES::Key Key;
		typedef typename GRAPH::Pose Pose;
		typedef typename GRAPH::Constraint Constraint;

		typedef boost::shared_ptr<const SubgraphSolver> shared_ptr ;
		typedef boost::shared_ptr<Ordering> shared_ordering ;
		typedef boost::shared_ptr<GRAPH> shared_graph ;
		typedef boost::shared_ptr<LINEAR> shared_linear ;
		typedef boost::shared_ptr<VALUES> shared_values ;
		typedef boost::shared_ptr<SubgraphPreconditioner> shared_preconditioner ;
		typedef std::map<Index,Index> mapPairIndex ;

		/* the ordering derived from the spanning tree */
		shared_ordering ordering_;

		/* the indice of two vertices in the gaussian factor graph */
		mapPairIndex pairs_;

		/* preconditioner */
		shared_preconditioner pc_;

	public:

		SubgraphSolver(const GRAPH& G, const VALUES& theta0, const Parameters &parameters = Parameters()):
			IterativeSolver(parameters){ initialize(G,theta0); }

		SubgraphSolver(const LINEAR& GFG) {
			std::cout << "[SubgraphSolver] Unexpected usage.." << std::endl;
			throw std::runtime_error("SubgraphSolver: gaussian factor graph initialization not supported");
		}

		SubgraphSolver(const SubgraphSolver& solver) :
			IterativeSolver(solver), ordering_(solver.ordering_), pairs_(solver.pairs_), pc_(solver.pc_){}

		SubgraphSolver(shared_ordering ordering,
				mapPairIndex pairs,
				shared_preconditioner pc,
				sharedParameters parameters = boost::make_shared<Parameters>()) :
					IterativeSolver(parameters), ordering_(ordering), pairs_(pairs), pc_(pc) {}

		void replaceFactors(const typename LINEAR::shared_ptr &graph);
		VectorValues::shared_ptr optimize() const ;
		shared_ordering ordering() const { return ordering_; }

	protected:
		void initialize(const GRAPH& G, const VALUES& theta0);

	private:
	  	SubgraphSolver():IterativeSolver(){}
	};

} // nsamespace gtsam
