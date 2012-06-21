/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexFactor.cpp
 * @author  Richard Roberts
 * @date    Oct 17, 2010
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
		BOOST_FOREACH(Index& key, keys())
			key = inversePermutation[key];
		assertInvariants();
	}
	/* ************************************************************************* */
} // gtsam
