/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexConditional.cpp
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#include <gtsam/base/FastSet.h>
#include <gtsam/inference/IndexConditional.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <boost/lambda/lambda.hpp>
#pragma GCC diagnostic pop

namespace gtsam {

  using namespace std;
  using namespace boost::lambda;

  template class Conditional<Index>;

  /* ************************************************************************* */
  void IndexConditional::assertInvariants() const {
    // Checks for uniqueness of keys
    Base::assertInvariants();
  }

  /* ************************************************************************* */
  bool IndexConditional::reduceSeparatorWithInverse(const internal::Reduction& inverseReduction) {
#ifndef NDEBUG
    BOOST_FOREACH(KeyType key, frontals()) { assert(inverseReduction.find(key) == inverseReduction.end()); }
#endif
    bool parentChanged = false;
    BOOST_FOREACH(KeyType& parent, parents()) {
      internal::Reduction::const_iterator it = inverseReduction.find(parent);
      if(it != inverseReduction.end()) {
        parentChanged = true;
        parent = it->second;
      }
    }
    assertInvariants();
    return parentChanged;
  }

  /* ************************************************************************* */
  void IndexConditional::permuteWithInverse(const Permutation& inversePermutation) {
    BOOST_FOREACH(Index& key, keys())
            key = inversePermutation[key];
    assertInvariants();
  }
  /* ************************************************************************* */

}
