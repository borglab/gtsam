/*
 * JunctionTree.h
 *
 *   Created on: Feb 4, 2010
 *       Author: nikai
 *  Description: The junction tree
 */

#pragma once

#include <set>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include "GaussianConditional.h"
#include "GaussianFactorGraph.h"
#include "BayesTree.h"

namespace gtsam {

	/* ************************************************************************* */
	template <class Conditional, class FG>
	class JunctionTree/*: public BayesTree<Conditional>*/ {
	public:
		typedef typename BayesTree<Conditional>::sharedClique sharedClique;

		// the threshold for the sizes of submaps. Smaller ones will be absorbed into the separator
		static const int const_minNodesPerMap_default = 10;
		static const int const_minNodesPerMap_ultra = 1;

		// when to stop partitioning
		static const int const_numNodeStopPartition_default = 50;
		static const int const_numNodeStopPartition_ultra = 3;     // so that A,B,C all have one variable


		// the class for subgraphs that also include the pointers to the parents and two children
		class SubFG : public FG {
		public:
			typedef typename boost::shared_ptr<SubFG> shared_ptr;
			shared_ptr parent_;                  // the parent subgraph node
			Ordering frontal_;                   // the frontal varaibles
			Unordered separator_;         // the separator variables

			friend class JunctionTree<Conditional, FG>;

		public:
			std::vector<shared_ptr> children_;         // the child cliques

			// empty constructor
			SubFG() {}

			// constructor with all the information
			SubFG(const FG& fgLocal, const Ordering& frontal, const Unordered& separator,
					 const shared_ptr& parent)
				: frontal_(frontal), separator_(separator), FG(fgLocal), parent_(parent) {}

			// constructor for an empty graph
			SubFG(const Ordering& frontal, const Unordered& separator, const shared_ptr& parent)
				: frontal_(frontal), separator_(separator), parent_(parent) {}

			const Ordering& frontal() const            { return frontal_;}
			const Unordered& separator() const         { return separator_;}
			std::vector<shared_ptr>& children()        { return children_; } // TODO:: add const

			// add a child node
			void addChild(const shared_ptr& child) { children_.push_back(child); }

			void printTree(const std::string& indent) const;
		};

		// typedef for shared pointers to cliques
		typedef typename SubFG::shared_ptr sharedSubFG;
		typedef boost::function<void (sharedSubFG)> VisitorSubFG;

	protected:
		// Root clique
		sharedSubFG rootFG_;

	private:

		// utility function called by eliminateBottomUp
		std::pair<FG, sharedClique> eliminateOneClique(sharedSubFG fg_, BayesTree<Conditional>& bayesTree);

	public:

		JunctionTree() : verboseLevel(0) {}

		// return the root graph
		sharedSubFG rootFG() const { return rootFG_; }

		// eliminate the factors in the subgraphs
		BayesTree<Conditional> eliminate();

		// print the object
		void print(const std::string& str) const {
			if (rootFG_.get()) rootFG_->printTree("");
		}

		// iterate over all the subgraphs from root to leaves in the DFS order, recursive
		void iterSubGraphsDFS(VisitorSubFG visitor, sharedSubFG current = sharedSubFG());

		// iterate over all the subgraphs from root to leaves in the BFS order, non-recursive
		void iterSubGraphsBFS(VisitorSubFG visitor);

		// the output level
		int verboseLevel;
	}; // JunctionTree

	/* ************************************************************************* */
	/**
	 * Linear JunctionTree which can do optimization
	 */
	template <class Conditional, class FG>
	class LinearJunctionTree: public JunctionTree<Conditional, FG> {
	public:
		typedef JunctionTree<Conditional, FG> Base;
		typedef typename BayesTree<Conditional>::sharedClique sharedClique;
		typedef typename JunctionTree<Conditional, FG>::sharedSubFG sharedSubFG;

	protected:
		// back-substitute in topological sort order (parents first)
		void btreeBackSubstitue(typename BayesTree<Conditional>::sharedClique current, VectorConfig& config);

	public :

		LinearJunctionTree() : Base() {}

		// constructor
		LinearJunctionTree(const FG& fg, const Ordering& ordering, int numNodeStopPartition = Base::const_numNodeStopPartition_default,
				int minNodesPerMap = Base::const_minNodesPerMap_default) :
			Base(fg, ordering, numNodeStopPartition, minNodesPerMap) {}

		// optimize the linear graph
		VectorConfig optimize();
	}; // Linear JunctionTree

	class SymbolicConditional;
	class SymbolicFactorGraph;

	/**
	 *  recursive partitioning
	 */
	typedef JunctionTree<SymbolicConditional, SymbolicFactorGraph> SymbolicTSAM;
	typedef JunctionTree<GaussianConditional, GaussianFactorGraph> GaussianTSAM;

} // namespace gtsam
