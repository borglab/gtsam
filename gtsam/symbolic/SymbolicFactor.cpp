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

#include <gtsam/base/FastVector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  std::pair<boost::shared_ptr<SymbolicConditional>, boost::shared_ptr<SymbolicFactor> >
    EliminateSymbolic(const SymbolicFactorGraph& factors, const Ordering& keys)
  {
    // Gather all keys
    FastSet<Key> allKeys;
    BOOST_FOREACH(const SymbolicFactor::shared_ptr& factor, factors) {
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
    FastVector<Key> orderedKeys(allKeys.size());
    std::copy(keys.begin(), keys.end(), orderedKeys.begin());
    std::set_difference(allKeys.begin(), allKeys.end(), frontals.begin(), frontals.end(), orderedKeys.begin() + nFrontals);

    // Return resulting conditional and factor
    return make_pair(
      boost::make_shared<SymbolicConditional>(
      SymbolicConditional::FromKeys(orderedKeys, nFrontals)),
      boost::make_shared<SymbolicFactor>(
      SymbolicFactor::FromIterators(orderedKeys.begin() + nFrontals, orderedKeys.end())));
  }

  /* ************************************************************************* */
  bool SymbolicFactor::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

  /* ************************************************************************* */
  std::pair<boost::shared_ptr<SymbolicConditional>, boost::shared_ptr<SymbolicFactor> >
    SymbolicFactor::eliminate(const Ordering& keys) const
  {
    SymbolicFactorGraph graph;
    graph += *this; // TODO: Is there a way to avoid copying this factor?
    return EliminateSymbolic(graph, keys);
  }

} // gtsam
