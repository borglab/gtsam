/*
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

	using namespace std;

	/* ************************************************************************* */
	/**
	 * GaussianJunctionTree
	 */
	void GaussianJunctionTree::btreeBackSubstitue(const boost::shared_ptr<const BayesTree::Clique>& current, VectorConfig& config) const {
		// solve the bayes net in the current node
		GaussianBayesNet::const_reverse_iterator it = current->rbegin();
		for (; it!=current->rend(); ++it) {
			Vector x = (*it)->solve(config); // Solve for that variable
			config[(*it)->key()] = x;   // store result in partial solution
		}

		// solve the bayes nets in the child nodes
		BOOST_FOREACH(const typename BayesTree::sharedClique& child, current->children()) {
			btreeBackSubstitue(child, config);
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
	VectorConfig GaussianJunctionTree::optimize() const {
	  tic("GJT optimize 1: eliminate");
		// eliminate from leaves to the root
		boost::shared_ptr<const BayesTree::Clique> rootClique(this->eliminate());
    toc("GJT optimize 1: eliminate");

		// Allocate solution vector
    tic("GJT optimize 2: allocate VectorConfig");
		vector<size_t> dims(rootClique->back()->key() + 1, 0);
		countDims(rootClique, dims);
		VectorConfig result(dims);
    toc("GJT optimize 2: allocate VectorConfig");

		// back-substitution
    tic("GJT optimize 3: back-substitute");
		btreeBackSubstitue(rootClique, result);
    toc("GJT optimize 3: back-substitute");
		return result;
	}

} //namespace gtsam
