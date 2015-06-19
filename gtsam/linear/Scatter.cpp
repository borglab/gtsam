/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Scatter.cpp
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @date    June 2015
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/Scatter.h>
#include <gtsam/inference/Ordering.h>

#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
string SlotEntry::toString() const {
  ostringstream oss;
  oss << "SlotEntry: slot=" << slot << ", dim=" << dimension;
  return oss.str();
}

/* ************************************************************************* */
Scatter::Scatter(const GaussianFactorGraph& gfg,
                 boost::optional<const Ordering&> ordering) {
  gttic(Scatter_Constructor);
  static const DenseIndex none = std::numeric_limits<size_t>::max();

  // First do the set union.
  BOOST_FOREACH (const GaussianFactor::shared_ptr& factor, gfg) {
    if (factor) {
      for (GaussianFactor::const_iterator variable = factor->begin();
           variable != factor->end(); ++variable) {
        // TODO: Fix this hack to cope with zero-row Jacobians that come from
        // BayesTreeOrphanWrappers
        const JacobianFactor* asJacobian =
            dynamic_cast<const JacobianFactor*>(factor.get());
        if (!asJacobian || asJacobian->cols() > 1)
          insert(
              make_pair(*variable, SlotEntry(none, factor->getDim(variable))));
      }
    }
  }

  // If we have an ordering, pre-fill the ordered variables first
  size_t slot = 0;
  if (ordering) {
    BOOST_FOREACH (Key key, *ordering) {
      const_iterator entry = find(key);
      if (entry == end())
        throw std::invalid_argument(
            "The ordering provided to the HessianFactor Scatter constructor\n"
            "contained extra variables that did not appear in the factors to "
            "combine.");
      at(key).slot = (slot++);
    }
  }

  // Next fill in the slot indices (we can only get these after doing the set
  // union.
  BOOST_FOREACH (value_type& var_slot, *this) {
    if (var_slot.second.slot == none) var_slot.second.slot = (slot++);
  }
}

/* ************************************************************************* */

}  // gtsam
