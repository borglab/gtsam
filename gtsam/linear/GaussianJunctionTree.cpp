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
	/**
	 * GaussianJunctionTree
	 */
	void GaussianJunctionTree::btreeBackSubstitute(const boost::shared_ptr<const BayesTree::Clique>& current, VectorValues& config) const {
		// solve the bayes net in the current node
		GaussianBayesNet::const_reverse_iterator it = current->rbegin();
		for (; it!=current->rend(); ++it) {
			Vector x = (*it)->solve(config); // Solve for that variable
			config[(*it)->key()] = x;   // store result in partial solution
		}

		// solve the bayes nets in the child nodes
		BOOST_FOREACH(const BayesTree::sharedClique& child, current->children()) {
			btreeBackSubstitute(child, config);
		}
	}

  /* ************************************************************************* */
	void countDims(const boost::shared_ptr<const BayesTree<GaussianConditional>::Clique>& clique, vector<size_t>& dims) {
	  BOOST_FOREACH(const boost::shared_ptr<const GaussianConditional>& cond, *clique) {
	    // There should be no two conditionals on the same variable
	    assert(dims[cond->key()] == 0);
	    dims[cond->key()] = cond->dim();
	  }
	  BOOST_FOREACH(const boost::shared_ptr<const BayesTree<GaussianConditional>::Clique>& child, clique->children()) {
	    countDims(child, dims);
	  }
	}

	/* ************************************************************************* */
	VectorValues GaussianJunctionTree::optimize() const {
	  tic("GJT optimize 1: eliminate");
		// eliminate from leaves to the root
		boost::shared_ptr<const BayesTree::Clique> rootClique(this->eliminate());
    toc("GJT optimize 1: eliminate");

		// Allocate solution vector
    tic("GJT optimize 2: allocate VectorValues");
		vector<size_t> dims(rootClique->back()->key() + 1, 0);
		countDims(rootClique, dims);
		VectorValues result(dims);
    toc("GJT optimize 2: allocate VectorValues");

		// back-substitution
    tic("GJT optimize 3: back-substitute");
		btreeBackSubstitute(rootClique, result);
    toc("GJT optimize 3: back-substitute");
		return result;
	}

} //namespace gtsam
