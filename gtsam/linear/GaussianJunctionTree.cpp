/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * GaussianJunctionTree.cpp
 * Created on: Jul 12, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: the Gaussian junction tree
 */

#include <gtsam/inference/ClusterTree-inl.h>
#include <gtsam/inference/JunctionTree-inl.h>
#include <gtsam/linear/GaussianJunctionTree.h>

#include <vector>

#include <boost/foreach.hpp>

namespace gtsam {

  // explicit template instantiation
  template class JunctionTree<GaussianFactorGraph>;
  template class ClusterTree<GaussianFactorGraph>;

	using namespace std;

	/* ************************************************************************* */
	void GaussianJunctionTree::btreeBackSubstitute(const boost::shared_ptr<const BayesTree::Clique>& current, VectorValues& config) const {
		// solve the bayes net in the current node
	  current->conditional()->solveInPlace(config);

	  //		GaussianBayesNet::const_reverse_iterator it = current->rbegin();
//		for (; it!=current->rend(); ++it) {
//			(*it)->solveInPlace(config); // solve and store result
//
////			Vector x = (*it)->solve(config); // Solve for that variable
////			config[(*it)->key()] = x;   // store result in partial solution
//		}

		// solve the bayes nets in the child nodes
		BOOST_FOREACH(const BayesTree::sharedClique& child, current->children()) {
			btreeBackSubstitute(child, config);
		}
	}

	/* ************************************************************************* */
	void GaussianJunctionTree::btreeRHS(const boost::shared_ptr<const BayesTree::Clique>& current, VectorValues& config) const {
		current->conditional()->rhs(config);
		BOOST_FOREACH(const BayesTree::sharedClique& child, current->children())
			btreeRHS(child, config);
	}

	/* ************************************************************************* */
	VectorValues GaussianJunctionTree::optimize(Eliminate function) const {
	  tic(1, "GJT eliminate");
		// eliminate from leaves to the root
		boost::shared_ptr<const BayesTree::Clique> rootClique(this->eliminate(function));
    toc(1, "GJT eliminate");

		// Allocate solution vector and copy RHS
    tic(2, "allocate VectorValues");
    vector<size_t> dims(rootClique->conditional()->back()+1, 0);
		countDims(rootClique, dims);
		VectorValues result(dims);
		btreeRHS(rootClique, result);
    toc(2, "allocate VectorValues");

		// back-substitution
    tic(3, "back-substitute");
		btreeBackSubstitute(rootClique, result);
    toc(3, "back-substitute");
		return result;
	}

	/* ************************************************************************* */
} //namespace gtsam
