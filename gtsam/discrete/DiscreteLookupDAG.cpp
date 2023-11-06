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
  ADT::print("", formatter);
  cout << endl;
}

/* ************************************************************************** */
void DiscreteLookupTable::argmaxInPlace(DiscreteValues* values) const {
  ADT pFS = choose(*values, true);  // P(F|S=parentsValues)

  // Initialize
  DiscreteValues mpe;
  double maxP = 0;

  // Get all Possible Configurations
  const auto allPosbValues = frontalAssignments();

  // Find the maximum
  for (const auto& frontalVals : allPosbValues) {
    double pValueS = pFS(frontalVals);  // P(F=value|S=parentsValues)
    // Update maximum solution if better
    if (pValueS > maxP) {
      maxP = pValueS;
      mpe = frontalVals;
    }
  }

  // set values (inPlace) to maximum
  for (Key j : frontals()) {
    (*values)[j] = mpe[j];
  }
}

/* ************************************************************************** */
size_t DiscreteLookupTable::argmax(const DiscreteValues& parentsValues) const {
  ADT pFS = choose(parentsValues, true);  // P(F|S=parentsValues)

  // Then, find the max over all remaining
  // TODO(Duy): only works for one key now, seems horribly slow this way
  size_t mpe = 0;
  double maxP = 0;
  DiscreteValues frontals;
  assert(nrFrontals() == 1);
  Key j = (firstFrontalKey());
  for (size_t value = 0; value < cardinality(j); value++) {
    frontals[j] = value;
    double pValueS = pFS(frontals);  // P(F=value|S=parentsValues)
    // Update MPE solution if better
    if (pValueS > maxP) {
      maxP = pValueS;
      mpe = value;
    }
  }
  return mpe;
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
  for (auto it = std::make_reverse_iterator(end()); it != std::make_reverse_iterator(begin()); ++it) {
    // dereference to get the sharedFactor to the lookup table
    (*it)->argmaxInPlace(&result);
  }
  return result;
}
/* ************************************************************************** */

}  // namespace gtsam
