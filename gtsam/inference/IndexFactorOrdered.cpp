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

#include <gtsam/inference/IndexFactorOrdered.h>
#include <gtsam/inference/FactorOrdered.h>
#include <gtsam/inference/IndexConditionalOrdered.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  void IndexFactorOrdered::assertInvariants() const {
    Base::assertInvariants();
  }

  /* ************************************************************************* */
  IndexFactorOrdered::IndexFactorOrdered(const IndexConditionalOrdered& c) :
    Base(c) {
    assertInvariants();
  }

  /* ************************************************************************* */
#ifdef TRACK_ELIMINATE
  boost::shared_ptr<IndexConditionalOrdered> IndexFactorOrdered::eliminateFirst() {
    boost::shared_ptr<IndexConditionalOrdered> result(Base::eliminateFirst<
        IndexConditionalOrdered>());
    assertInvariants();
    return result;
  }

  /* ************************************************************************* */
  boost::shared_ptr<BayesNetOrdered<IndexConditionalOrdered> > IndexFactorOrdered::eliminate(
      size_t nrFrontals) {
    boost::shared_ptr<BayesNetOrdered<IndexConditionalOrdered> > result(Base::eliminate<
        IndexConditionalOrdered>(nrFrontals));
    assertInvariants();
    return result;
  }
#endif

  /* ************************************************************************* */
  void IndexFactorOrdered::permuteWithInverse(const Permutation& inversePermutation) {
    BOOST_FOREACH(Index& key, keys())
      key = inversePermutation[key];
    assertInvariants();
  }

  /* ************************************************************************* */
  void IndexFactorOrdered::reduceWithInverse(const internal::Reduction& inverseReduction) {
    BOOST_FOREACH(Index& key, keys())
      key = inverseReduction[key];
    assertInvariants();
  }

  /* ************************************************************************* */
} // gtsam
