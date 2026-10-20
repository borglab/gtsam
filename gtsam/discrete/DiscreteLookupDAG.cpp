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
#include <vector>
#include <cassert>

using std::pair;
using std::vector;

namespace gtsam {

/* ************************************************************************** */
// TODO(dellaert): copy/paste from DiscreteConditional.cpp :-(
void DiscreteLookupTable::print(const std::string& s,
                                const KeyFormatter& formatter) const {
  using std::cout;
  using std::endl;

  cout << s << " g( ";
  for (const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
    cout << formatter(*it) << " ";
  }
  if (nrParents()) {
    cout << "; ";
    for (const_iterator it = beginParents(); it != endParents(); ++it) {
      cout << formatter(*it) << " ";
    }
  }
  cout << "):\n";
  printValues(formatter);
  cout << endl;
}

/* ************************************************************************** */
void DiscreteLookupTable::argmaxInPlace(DiscreteValues* values) const {
  checkParents(*values);

  // Enumerate the frontal assignments in place in `values`, evaluating the
  // stored factor directly, allocating no factor.
  // Each entry tracks a frontal value, its cardinality, and its value
  // in the best assignment so far.
  struct Frontal {
    DiscreteValues::iterator entry;
    size_t cardinality;
    size_t best;
  };

  std::vector<Frontal> frontals;
  frontals.reserve(nrFrontals());
  for (Key j : this->frontals()) {
    auto entry = values->emplace(j, 0).first;
    entry->second = 0;
    frontals.push_back({entry, cardinality(j), 0});
  }

  // Advance to the next assignment, with the last frontal varying fastest as
  // in frontalAssignments(), so that ties resolve in the same order.
  auto advance = [&frontals]() {
    for (size_t i = frontals.size(); i-- > 0;) {
      if (++frontals[i].entry->second < frontals[i].cardinality) return true;
      frontals[i].entry->second = 0;
    }
    return false;
  };

  double maxP = 0;
  do {
    const double pValueS = (*this)(*values);  // P(F=value|S=parentsValues)
    // Update maximum solution if better
    if (pValueS > maxP) {
      maxP = pValueS;
      for (auto& frontal : frontals) frontal.best = frontal.entry->second;
    }
  } while (advance());

  // set values (inPlace) to maximum
  for (auto& frontal : frontals) frontal.entry->second = frontal.best;
}

/* ************************************************************************** */
DiscreteLookupDAG DiscreteLookupDAG::FromBayesNet(
    const DiscreteBayesNet& bayesNet) {
  DiscreteLookupDAG dag;
  for (auto&& conditional : bayesNet) {
    if (auto lookupTable =
            std::dynamic_pointer_cast<DiscreteLookupTable>(conditional)) {
      dag.push_back(lookupTable);
    } else {
      throw std::runtime_error(
          "DiscreteFactorGraph::maxProduct: Expected look up table.");
    }
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
