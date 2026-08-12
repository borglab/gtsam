/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBayesNet.cpp
 * @date Feb 15, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/inference/FactorGraph-inst.h>

namespace gtsam {

// Instantiate base class
template class FactorGraph<DiscreteConditional>;

/* ************************************************************************* */
bool DiscreteBayesNet::equals(const This& bn, double tol) const {
  return Base::equals(bn, tol);
}

/* ************************************************************************* */
double DiscreteBayesNet::logProbability(const DiscreteValues& values) const {
  // evaluate all conditionals and add
  double result = 0.0;
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    result += conditional->logProbability(values);
  return result;
}

/* ************************************************************************* */
double DiscreteBayesNet::evaluate(const DiscreteValues& values) const {
  // evaluate all conditionals and multiply
  double result = 1.0;
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    result *= (*conditional)(values);
  return result;
}

/* ************************************************************************* */
DiscreteValues DiscreteBayesNet::sample(std::mt19937_64* rng) const {
  DiscreteValues result;
  return sample(result, rng);
}

DiscreteValues DiscreteBayesNet::sample(DiscreteValues result,
                                        std::mt19937_64* rng) const {
  // sample each node in turn in topological sort order (parents first)
  for (auto it = std::make_reverse_iterator(end());
       it != std::make_reverse_iterator(begin()); ++it) {
    const DiscreteConditional::shared_ptr& conditional = *it;
    // Sample the conditional only if value for j not already in result
    const Key j = conditional->firstFrontalKey();
    if (result.count(j) == 0) {
      conditional->sampleInPlace(&result, rng);
    }
  }
  return result;
}

/* ************************************************************************* */
// The implementation is: build the entire joint into one factor and then prune.
// NOTE(Frank): This can be quite expensive *unless* the factors have already
// been pruned before. Another, possibly faster approach is branch and bound
// search to find the K-best leaves and then create a single pruned conditional.
DiscreteBayesNet DiscreteBayesNet::prune(
    size_t maxNrLeaves, const std::optional<double>& marginalThreshold,
    DiscreteValues* fixedValues) const {
  // Multiply into one big conditional. NOTE: possibly quite expensive.
  DiscreteConditional joint = this->joint();

  // Prune the joint. NOTE: imperative and, again, possibly quite expensive.
  DiscreteConditional pruned = joint;
  pruned.prune(maxNrLeaves);

  DiscreteValues deadModesValues;
  // If we have a dead mode threshold and discrete variables left after pruning,
  // then we run dead mode removal.
  if (marginalThreshold && pruned.keys().size() > 0) {
    DiscreteMarginals marginals(DiscreteFactorGraph{pruned});
    for (auto dkey : pruned.discreteKeys()) {
      const Vector probabilities = marginals.marginalProbabilities(dkey);

      int index = -1;
      auto threshold = (probabilities.array() > *marginalThreshold);
      // If atleast 1 value is non-zero, then we can find the index
      // Else if all are zero, index would be set to 0 which is incorrect
      if (!threshold.isZero()) {
        threshold.maxCoeff(&index);
      }

      if (index >= 0) {
        deadModesValues.emplace(dkey.first, index);
      }
    }

    // Remove the modes (imperative)
    pruned.removeDiscreteModes(deadModesValues);

    // Set the fixed values if requested.
    if (fixedValues) {
      if (!fixedValues->empty()) {
        throw std::invalid_argument(
            "DiscreteBayesNet::prune: fixedValues should be empty since it is "
            "a purely output argument.");
      }

      *fixedValues = deadModesValues;
    }
  }

  // Return the resulting DiscreteBayesNet.
  DiscreteBayesNet result;
  if (pruned.keys().size() > 0) result.push_back(pruned);
  return result;
}

/* *********************************************************************** */
DiscreteConditional DiscreteBayesNet::joint() const {
  DiscreteConditional joint;
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    joint = joint * (*conditional);

  return joint;
}

/* *********************************************************************** */
std::string DiscreteBayesNet::markdown(
    const KeyFormatter& keyFormatter,
    const DiscreteFactor::Names& names) const {
  using std::endl;
  std::stringstream ss;
  ss << "`DiscreteBayesNet` of size " << size() << endl << endl;
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    ss << conditional->markdown(keyFormatter, names) << endl;
  return ss.str();
}

/* *********************************************************************** */
std::string DiscreteBayesNet::html(const KeyFormatter& keyFormatter,
                                   const DiscreteFactor::Names& names) const {
  using std::endl;
  std::stringstream ss;
  ss << "<div><p><tt>DiscreteBayesNet</tt> of size " << size() << "</p>";
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    ss << conditional->html(keyFormatter, names) << endl;
  return ss.str();
}

/* ************************************************************************* */
}  // namespace gtsam
