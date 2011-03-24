/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexFactor.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 17, 2010
 */

#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/IndexConditional.h>

using namespace std;

namespace gtsam {

	template class Factor<Index> ;

	/* ************************************************************************* */
	void IndexFactor::assertInvariants() const {
		Base::assertInvariants();
//#ifndef NDEBUG
//#ifndef GTSAM_NO_ENFORCE_ORDERING
//		std::set<Index> uniqueSorted(keys_.begin(), keys_.end());
//		assert(uniqueSorted.size() == keys_.size() && std::equal(uniqueSorted.begin(), uniqueSorted.end(), keys_.begin()));
//#endif
//#endif
	}

	/* ************************************************************************* */
	IndexFactor::IndexFactor(const IndexConditional& c) :
		Base(c) {
		assertInvariants();
	}

	/* ************************************************************************* */
#ifdef TRACK_ELIMINATE
	boost::shared_ptr<IndexConditional> IndexFactor::eliminateFirst() {
		boost::shared_ptr<IndexConditional> result(Base::eliminateFirst<
				IndexConditional>());
		assertInvariants();
		return result;
	}

	/* ************************************************************************* */
	boost::shared_ptr<BayesNet<IndexConditional> > IndexFactor::eliminate(
			size_t nrFrontals) {
		boost::shared_ptr<BayesNet<IndexConditional> > result(Base::eliminate<
				IndexConditional>(nrFrontals));
		assertInvariants();
		return result;
	}
#endif

	/* ************************************************************************* */
	void IndexFactor::permuteWithInverse(const Permutation& inversePermutation) {
		BOOST_FOREACH(Index& key, keys_)
						key = inversePermutation[key];
		assertInvariants();
	}
	/* ************************************************************************* */
} // gtsam
