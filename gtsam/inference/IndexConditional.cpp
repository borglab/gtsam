/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexConditional.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 17, 2010
 */

#include <gtsam/base/FastSet.h>
#include <gtsam/inference/IndexConditional.h>
#include <boost/lambda/lambda.hpp>

namespace gtsam {

  using namespace std;
  using namespace boost::lambda;

  template class Conditional<Index>;

  /* ************************************************************************* */
  void IndexConditional::assertInvariants() const {
    // Checks for uniqueness of keys
    Base::assertInvariants();
#ifndef NDEBUG
    // Check that separator keys are sorted
    FastSet<Index> uniquesorted(beginFrontals(), endFrontals());
    assert(uniquesorted.size() == nrFrontals() && std::equal(uniquesorted.begin(), uniquesorted.end(), beginFrontals()));
    // Check that separator keys are less than parent keys
    BOOST_FOREACH(Index j, frontals()) {
      assert(find_if(beginParents(), endParents(), _1 < j) == endParents()); }
#endif
  }

}
