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
#include "BayesTree.h"
#include "SymbolicConditional.h"

namespace gtsam {

	/* ************************************************************************* */
	template <class FG>
	class JunctionTree : public Testable<JunctionTree<FG> > {
	public:
		// the class for subgraphs that also include the pointers to the parents and two children
		class Clique : public FG {
		private:
			typedef typename boost::shared_ptr<Clique> shared_ptr;
			shared_ptr parent_;                  // the parent subgraph node
			std::vector<shared_ptr> children_;   // the child cliques
			Ordering frontal_;                   // the frontal varaibles
			Unordered separator_;                // the separator variables

			friend class JunctionTree<FG>;

		public:

			// empty constructor
			Clique() {}

			// constructor with all the information
			Clique(const FG& fgLocal, const Ordering& frontal, const Unordered& separator,
					 const shared_ptr& parent)
				: frontal_(frontal), separator_(separator), FG(fgLocal), parent_(parent) {}

			// constructor for an empty graph
			Clique(const Ordering& frontal, const Unordered& separator, const shared_ptr& parent)
				: frontal_(frontal), separator_(separator), parent_(parent) {}

			// return the members
			const Ordering& frontal() const            { return frontal_;}
			const Unordered& separator() const         { return separator_;}
			const std::vector<shared_ptr>& children()  { return children_; }

			// add a child node
			void addChild(const shared_ptr& child) { children_.push_back(child); }

			// print the object
			void print(const std::string& indent) const;
			void printTree(const std::string& indent) const;

			// check equality
			bool equals(const Clique& other) const;
		};

		// typedef for shared pointers to cliques
		typedef typename Clique::shared_ptr sharedClique;

	protected:
		// Root clique
		sharedClique root_;

	private:
		// distribute the factors along the Bayes tree
		sharedClique distributeFactors(FG& fg, const BayesTree<SymbolicConditional>::sharedClique clique);

		// utility function called by eliminate
		template <class Conditional>
		std::pair<FG, BayesTree<Conditional> > eliminateOneClique(sharedClique fg_);

	public:
		// constructor
		JunctionTree() {}

		// constructor given a factor graph and the elimination ordering
		JunctionTree(FG& fg, const Ordering& ordering);

		// return the root clique
		sharedClique root() const { return root_; }

		// eliminate the factors in the subgraphs
		template <class Conditional>
		BayesTree<Conditional> eliminate();

		// print the object
		void print(const std::string& str) const {
			cout << str << endl;
			if (root_.get()) root_->printTree("");
		}

		/** check equality */
		bool equals(const JunctionTree<FG>& other, double tol = 1e-9) const;

	}; // JunctionTree

} // namespace gtsam
