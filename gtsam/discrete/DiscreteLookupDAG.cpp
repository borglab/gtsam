/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteLookupTable.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteLookupDAG.h>
#include <gtsam/discrete/DiscreteValues.h>

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

/* ************************************************************************* */
// TODO(dellaert): copy/paste from DiscreteConditional.cpp :-(
vector<DiscreteValues> DiscreteLookupTable::frontalAssignments() const {
  vector<pair<Key, size_t>> pairs;
  for (Key key : frontals()) pairs.emplace_back(key, cardinalities_.at(key));
  vector<pair<Key, size_t>> rpairs(pairs.rbegin(), pairs.rend());
  return DiscreteValues::CartesianProduct(rpairs);
}

/* ************************************************************************** */
// TODO(dellaert): copy/paste from DiscreteConditional.cpp :-(
static DiscreteLookupTable::ADT Choose(const DiscreteLookupTable& conditional,
                                       const DiscreteValues& given,
                                       bool forceComplete = true) {
  // Get the big decision tree with all the levels, and then go down the
  // branches based on the value of the parent variables.
  DiscreteLookupTable::ADT adt(conditional);
  size_t value;
  for (Key j : conditional.parents()) {
    try {
      value = given.at(j);
      adt = adt.choose(j, value);  // ADT keeps getting smaller.
    } catch (std::out_of_range&) {
      if (forceComplete) {
        given.print("parentsValues: ");
        throw std::runtime_error(
            "DiscreteLookupTable::Choose: parent value missing");
      }
    }
  }
  return adt;
}

/* ************************************************************************** */
void DiscreteLookupTable::argmaxInPlace(DiscreteValues* values) const {
  ADT pFS = Choose(*this, *values);  // P(F|S=parentsValues)

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
  ADT pFS = Choose(*this, parentsValues);  // P(F|S=parentsValues)

  // Then, find the max over all remaining
  // TODO(Duy): only works for one key now, seems horribly slow this way
  size_t mpe = 0;
  DiscreteValues frontals;
  double maxP = 0;
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
DiscreteValues DiscreteLookupDAG::argmax() const {
  DiscreteValues result;
  return argmax(result);
}

DiscreteValues DiscreteLookupDAG::argmax(DiscreteValues result) const {
  // Argmax each node in turn in topological sort order (parents first).
  for (auto lookupTable : boost::adaptors::reverse(*this))
    lookupTable->argmaxInPlace(&result);
  return result;
}
/* ************************************************************************** */

}  // namespace gtsam
