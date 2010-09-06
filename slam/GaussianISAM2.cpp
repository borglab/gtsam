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
void optimize2(const GaussianISAM2::sharedClique& clique, double threshold, VectorConfig& result) {
	bool process_children = false;
	// parents are assumed to already be solved and available in result
	GaussianISAM2::Clique::const_reverse_iterator it;
	for (it = clique->rbegin(); it!=clique->rend(); it++) {
		GaussianConditional::shared_ptr cg = *it;
    Vector x = cg->solve(result); // Solve for that variable
    if (max(abs(x)) >= threshold) {
    	process_children = true;
    }
    result.insert(cg->key(), x);   // store result in partial solution
  }
	if (process_children) {
		BOOST_FOREACH(const GaussianISAM2::sharedClique& child, clique->children_) {
			optimize2(child, threshold, result);
		}
	}
}

/* ************************************************************************* */
VectorConfig optimize2(const GaussianISAM2& bayesTree, double threshold) {
	VectorConfig result;
	// starting from the root, call optimize on each conditional
	optimize2(bayesTree.root(), threshold, result);
	return result;
}

/* ************************************************************************* */
VectorConfig optimize2(const GaussianISAM2_P& bayesTree, double threshold) {
	VectorConfig result;
	// starting from the root, call optimize on each conditional
	optimize2(bayesTree.root(), threshold, result);
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
    result += (dimR+1)*dimR/2 + dimSep*dimR;
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
