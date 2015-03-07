/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicFactor-inst.h
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicConditional.h>

namespace gtsam
{
  namespace internal
  {
    /** Implementation of dense elimination function for symbolic factors.  This is a templated
     *  version for internally doing symbolic elimination on any factor. */
    template<class FACTOR>
    std::pair<boost::shared_ptr<SymbolicConditional>, boost::shared_ptr<SymbolicFactor> >
      EliminateSymbolic(const FactorGraph<FACTOR>& factors, const Ordering& keys)
    {
      // Gather all keys
      FastSet<Key> allKeys;
      BOOST_FOREACH(const boost::shared_ptr<FACTOR>& factor, factors) {
        allKeys.insert(factor->begin(), factor->end());
      }

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
      return std::make_pair(
        SymbolicConditional::FromKeysShared(orderedKeys, nFrontals),
        SymbolicFactor::FromIteratorsShared(orderedKeys.begin() + nFrontals, orderedKeys.end()));
    }
  }
}
