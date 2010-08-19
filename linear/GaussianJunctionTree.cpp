/*
 * GaussianJunctionTree.cpp
 * Created on: Jul 12, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: the Gaussian junction tree
 */

#include <boost/foreach.hpp>

#include <gtsam/inference/ClusterTree-inl.h>
#include <gtsam/inference/JunctionTree-inl.h>
#include <gtsam/linear/GaussianJunctionTree.h>

namespace gtsam {

	// explicit template instantiation
	template class JunctionTree<GaussianFactorGraph>;

	using namespace std;

	/* ************************************************************************* */
	/**
	 * GaussianJunctionTree
	 */
	void GaussianJunctionTree::btreeBackSubstitue(
			BayesTree<GaussianConditional>::sharedClique current,
			VectorConfig& config) {
		// solve the bayes net in the current node
		BayesNet<GaussianConditional>::const_reverse_iterator it = current->rbegin();
		for (; it!=current->rend(); it++) {
			Vector x = (*it)->solve(config); // Solve for that variable
			config.insert((*it)->key(),x);   // store result in partial solution
		}

		// solve the bayes nets in the child nodes
		typedef BayesTree<GaussianConditional>::sharedClique sharedBayesClique;
		BOOST_FOREACH(sharedBayesClique child, current->children_) {
			btreeBackSubstitue(child, config);
		}
	}

	/* ************************************************************************* */
	VectorConfig GaussianJunctionTree::optimize() {
		// eliminate from leaves to the root
		BayesTree<GaussianConditional>::sharedClique rootClique;
		rootClique = this->eliminate<GaussianConditional>();

		// back-substitution
		VectorConfig result;
		btreeBackSubstitue(rootClique, result);
		return result;
	}


} //namespace gtsam
