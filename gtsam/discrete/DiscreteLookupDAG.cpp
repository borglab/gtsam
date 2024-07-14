/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteLookupDAG.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteLookupDAG.h>
#include <gtsam/discrete/DiscreteValues.h>

#include <iterator>
#include <string>
#include <utility>

using std::pair;
using std::vector;

namespace gtsam {

/* ************************************************************************** */
DiscreteLookupDAG DiscreteLookupDAG::FromBayesNet(
    const DiscreteBayesNet& bayesNet) {
  DiscreteLookupDAG dag;
  for (auto&& conditional : bayesNet) {
    dag.push_back(conditional);
  }
  return dag;
}

DiscreteValues DiscreteLookupDAG::argmax(DiscreteValues result) const {
  // Argmax each node in turn in topological sort order (parents first).
  for (auto it = std::make_reverse_iterator(end());
       it != std::make_reverse_iterator(begin()); ++it) {
    // dereference to get the sharedFactor to the lookup table
    (*it)->argmaxInPlace(&result);
  }
  return result;
}
/* ************************************************************************** */

}  // namespace gtsam
