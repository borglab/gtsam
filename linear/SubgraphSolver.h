/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/inference/EliminationTree-inl.h>
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

  /**
   * A nonlinear system solver using subgraph preconditioning conjugate gradient
   * Concept NonLinearSolver<G,T,L> implements
   *   linearize: G * T -> L
   *   solve : L -> VectorValues
   */
	template<class GRAPH, class LINEAR, class VALUES>
	class SubgraphSolver {

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

		// TODO not hardcode
		static const size_t maxIterations_=100;
		static const double epsilon_=1e-4, epsilon_abs_=1e-5;
		static const bool verbose_=true;

		/* the ordering derived from the spanning tree */
		shared_ordering ordering_;

		/* the indice of two vertices in the gaussian factor graph */
		mapPairIndex pairs_;

		/* preconditioner */
		shared_preconditioner pc_;

	public:
	  	SubgraphSolver(){}

	  	SubgraphSolver(const LINEAR &GFG) {
			throw std::runtime_error("SubgraphSolver: gaussian factor graph initialization not supported");
	  	}

	  	SubgraphSolver(const SubgraphSolver& solver) :
	  		ordering_(solver.ordering_), pairs_(solver.pairs_),	pc_(solver.pc_){}

	  	SubgraphSolver(shared_ordering ordering,
	  				   mapPairIndex pairs,
	  				   shared_preconditioner pc) :
	  		ordering_(ordering), pairs_(pairs), pc_(pc) {}


		SubgraphSolver(const GRAPH& G, const VALUES& theta0) { initialize(G,theta0); }

		shared_ptr update(const LINEAR &graph) const {

			shared_linear Ab1 = boost::make_shared<LINEAR>(),
						  Ab2 = boost::make_shared<LINEAR>();

			if (verbose_) cout << "split the graph ...";
			graph.split(pairs_, *Ab1, *Ab2) ;
			if (verbose_) cout << ",with " << Ab1->size() << " and " << Ab2->size() << " factors" << endl;

			//		// Add a HardConstra	int to the root, otherwise the root will be singular
			//		Key root = keys.back();
			//		T_.addHardConstraint(root, theta0[root]);
			//
			//		// compose the approximate solution
			//		theta_bar_ = composePoses<GRAPH, Constraint, Pose, Values> (T_, tree, theta0[root]);

			LINEAR sacrificialAb1 = *Ab1; // duplicate !!!!!
			SubgraphPreconditioner::sharedBayesNet Rc1 = EliminationTree<GaussianFactor>::Create(sacrificialAb1)->eliminate();
			SubgraphPreconditioner::sharedValues xbar = gtsam::optimize_(*Rc1);

			shared_preconditioner pc = boost::make_shared<SubgraphPreconditioner>(Ab1,Ab2,Rc1,xbar);
			return boost::make_shared<SubgraphSolver>(ordering_, pairs_, pc) ;
		}

		 VectorValues::shared_ptr optimize() const {

			// preconditioned conjugate gradient
			VectorValues zeros = pc_->zero();
			VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues,
					Errors> (*pc_, zeros, verbose_, epsilon_, epsilon_abs_, maxIterations_);

			boost::shared_ptr<VectorValues> xbar = boost::make_shared<VectorValues>() ;
			*xbar = pc_->x(ybar);
			return xbar;
		}

		shared_ordering ordering() const { return ordering_; }

	protected:

		void initialize(const GRAPH& G, const VALUES& theta0) {
			// generate spanning tree
			PredecessorMap<Key> tree_ = gtsam::findMinimumSpanningTree<GRAPH, Key, Constraint>(G);

			// make the ordering
			list<Key> keys = predecessorMap2Keys(tree_);
			ordering_ = boost::make_shared<Ordering>(list<Symbol>(keys.begin(), keys.end()));

			// build factor pairs
			pairs_.clear();
			typedef pair<Key,Key> EG ;
			BOOST_FOREACH( const EG &eg, tree_ ) {
				Symbol key1 = Symbol(eg.first),
					   key2 = Symbol(eg.second) ;
				pairs_.insert(pair<Index, Index>((*ordering_)[key1], (*ordering_)[key2])) ;
			}
		}

	};

	template<class GRAPH, class LINEAR, class VALUES> const size_t SubgraphSolver<GRAPH, LINEAR, VALUES>::maxIterations_;
	template<class GRAPH, class LINEAR, class VALUES> const bool SubgraphSolver<GRAPH, LINEAR, VALUES>::verbose_;
	template<class GRAPH, class LINEAR, class VALUES> const double SubgraphSolver<GRAPH, LINEAR, VALUES>::epsilon_;
	template<class GRAPH, class LINEAR, class VALUES> const double SubgraphSolver<GRAPH, LINEAR, VALUES>::epsilon_abs_;

} // nsamespace gtsam
