/**
 * @file    GaussianISAM2
 * @brief   Full non-linear ISAM
 * @author  Michael Kaess
 */

#include <gtsam/slam/GaussianISAM2.h>

using namespace std;
using namespace gtsam;

// Explicitly instantiate so we don't have to include everywhere
#include <gtsam/inference/ISAM2-inl.h>

template class ISAM2<GaussianConditional, simulated2D::Config>;
template class ISAM2<GaussianConditional, planarSLAM::Config>;

namespace gtsam {

/* ************************************************************************* */
void optimize2(const GaussianISAM2::sharedClique& clique, double threshold, set<Symbol>& changed, VectorConfig& result) {
	// if none of the variables in this clique (frontal and separator!) changed
	// significantly, then by the running intersection property, none of the
	// cliques in the children need to be processed
	bool process_children = false;

	// parents are assumed to already be solved and available in result
	GaussianISAM2::Clique::const_reverse_iterator it;
	for (it = clique->rbegin(); it!=clique->rend(); it++) {
		GaussianConditional::shared_ptr cg = *it;

		// only solve if at least one of the separator variables changed
		// significantly, ie. is in the set "changed"
		bool found = true;
		if (cg->nrParents()>0) {
			found = false;
			BOOST_FOREACH(const Symbol& key, cg->parents()) {
				if (changed.find(key)!=changed.end()) {
					found = true;
				}
			}
		}
		if (found) {
			// Solve for that variable
			Vector x = cg->solve(result);
			process_children = true;

			// store result in partial solution
			result.insert(cg->key(), x);

			// if change is above threshold, add to set of changed variables
			if (max(abs(x)) >= threshold) {
				changed.insert(cg->key());
				process_children = true;
			}
		}
	}
	if (process_children) {
		BOOST_FOREACH(const GaussianISAM2::sharedClique& child, clique->children_) {
			optimize2(child, threshold, changed, result);
		}
	}
}

/* ************************************************************************* */
// fast full version without threshold
void optimize2(const GaussianISAM2::sharedClique& clique, VectorConfig& result) {
	// parents are assumed to already be solved and available in result
	GaussianISAM2::Clique::const_reverse_iterator it;
	for (it = clique->rbegin(); it!=clique->rend(); it++) {
		GaussianConditional::shared_ptr cg = *it;
		Vector x = cg->solve(result);
		// store result in partial solution
		result.insert(cg->key(), x);
	}
	BOOST_FOREACH(const GaussianISAM2::sharedClique& child, clique->children_) {
		optimize2(child, result);
	}
}

/* ************************************************************************* */
VectorConfig optimize2(const GaussianISAM2& bayesTree, double threshold) {
	VectorConfig result;
	set<Symbol> changed;
	// starting from the root, call optimize on each conditional
	if (threshold<=0.) {
		optimize2(bayesTree.root(), result);
	} else {
		optimize2(bayesTree.root(), threshold, changed, result);
	}
	return result;
}

/* ************************************************************************* */
VectorConfig optimize2(const GaussianISAM2_P& bayesTree, double threshold) {
	VectorConfig result;
	set<Symbol> changed;
	// starting from the root, call optimize on each conditional
	if (threshold<=0.) {
		optimize2(bayesTree.root(), result);
	} else {
		optimize2(bayesTree.root(), threshold, changed, result);
	}
	return result;
}

/* ************************************************************************* */
void nnz_internal(const GaussianISAM2::sharedClique& clique, int& result) {
	// go through the conditionals of this clique
	GaussianISAM2::Clique::const_reverse_iterator it;
	for (it = clique->rbegin(); it!=clique->rend(); it++) {
		GaussianConditional::shared_ptr cg = *it;
		int dimSep = 0;
		for (GaussianConditional::const_iterator matrix_it = cg->parentsBegin(); matrix_it != cg->parentsEnd(); matrix_it++) {
			dimSep += matrix_it->second.size2();
		}
		int dimR = cg->dim();
		result += ((dimR+1)*dimR)/2 + dimSep*dimR;
	}
	// traverse the children
	BOOST_FOREACH(const GaussianISAM2::sharedClique& child, clique->children_) {
		nnz_internal(child, result);
	}
}

/* ************************************************************************* */
int calculate_nnz(const GaussianISAM2::sharedClique& clique) {
	int result = 0;
	// starting from the root, add up entries of frontal and conditional matrices of each conditional
	nnz_internal(clique, result);
	return result;
}

} /// namespace gtsam
