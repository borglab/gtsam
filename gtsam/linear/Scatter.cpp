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

#include <algorithm>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
string SlotEntry::toString() const {
  ostringstream oss;
  oss << "SlotEntry: key=" << key << ", dim=" << dimension;
  return oss.str();
}

Scatter::Scatter(const GaussianFactorGraph& gfg) : Scatter(gfg, Ordering()) {}

/* ************************************************************************* */
Scatter::Scatter(const GaussianFactorGraph& gfg,
    const Ordering& ordering) {
  gttic(Scatter_Constructor);

  // If we have an ordering, pre-fill the ordered variables first
  for (Key key : ordering) {
    add(key, 0);
  }

  // Now, find dimensions of variables and/or extend
  for (const auto& factor : gfg) {
    if (!factor)
      continue;

    // TODO: Fix this hack to cope with zero-row Jacobians that come from BayesTreeOrphanWrappers
    const JacobianFactor* asJacobian = dynamic_cast<const JacobianFactor*>(factor.get());
    if (asJacobian && asJacobian->cols() <= 1) continue;

    // loop over variables
    for (GaussianFactor::const_iterator variable = factor->begin();
         variable != factor->end(); ++variable) {
      const Key key = *variable;
      iterator it = find(key); // theoretically expensive, yet cache friendly
      if (it!=end())
        it->dimension = factor->getDim(variable);
      else
        add(key, factor->getDim(variable));
    }
  }

  // To keep the same behavior as before, sort the keys after the ordering
  iterator first = begin();
  first += ordering.size();
  if (first != end()) std::sort(first, end());
}

/* ************************************************************************* */
void Scatter::add(Key key, size_t dim) {
  emplace_back(SlotEntry(key, dim));
}

/* ************************************************************************* */
FastVector<SlotEntry>::iterator Scatter::find(Key key) {
  iterator it = begin();
  while(it != end()) {
    if (it->key == key)
      return it;
    ++it;
  }
  return it; // end()
}

/* ************************************************************************* */

} // gtsam
