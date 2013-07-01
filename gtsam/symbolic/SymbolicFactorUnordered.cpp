/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicFactor.cpp
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/inference/OrderingUnordered.h>
#include <gtsam/symbolic/SymbolicFactorUnordered.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>
#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  std::pair<boost::shared_ptr<SymbolicConditionalUnordered>, boost::shared_ptr<SymbolicFactorUnordered> >
    EliminateSymbolicUnordered(const SymbolicFactorGraphUnordered& factors, const OrderingUnordered& keys)
  {
    // Gather all keys
    FastSet<Key> allKeys;
    BOOST_FOREACH(const SymbolicFactorUnordered::shared_ptr& factor, factors) {
      allKeys.insert(factor->begin(), factor->end()); }

    // Check keys
    BOOST_FOREACH(Key key, keys) {
      if(allKeys.find(key) == allKeys.end())
        throw std::runtime_error("Requested to eliminate a key that is not in the factors");
    }

    // Sort frontal keys
    FastSet<Key> frontals(keys);
    const size_t nFrontals = keys.size();

    // Build a key vector with the frontals followed by the separator
    vector<Key> orderedKeys(allKeys.size());
    std::copy(keys.begin(), keys.end(), orderedKeys.begin());
    std::set_difference(allKeys.begin(), allKeys.end(), frontals.begin(), frontals.end(), orderedKeys.begin() + nFrontals);

    // Return resulting conditional and factor
    return make_pair(
      boost::make_shared<SymbolicConditionalUnordered>(
      SymbolicConditionalUnordered::FromKeys(orderedKeys, nFrontals)),
      boost::make_shared<SymbolicFactorUnordered>(
      SymbolicFactorUnordered::FromIterator(orderedKeys.begin() + nFrontals, orderedKeys.end())));
  }

} // gtsam
