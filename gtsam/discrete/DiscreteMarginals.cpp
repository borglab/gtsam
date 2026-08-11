/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteMarginals.cpp
 * @brief A class for computing marginals in a DiscreteFactorGraph
 * @author Abhijit Kundu
 * @author Richard Roberts
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date June 4, 2012
 */

#include <gtsam/discrete/DiscreteMarginals.h>

namespace gtsam {

/* ************************************************************************* */
DiscreteMarginals::DiscreteMarginals(const DiscreteFactorGraph& graph) {
  bayesTree_ = graph.eliminateMultifrontal();
}

/* ************************************************************************* */
DiscreteFactor::shared_ptr DiscreteMarginals::operator()(Key variable) const {
  // Compute marginal
  DiscreteFactor::shared_ptr marginalFactor =
      bayesTree_->marginalFactor(variable, &EliminateDiscrete);
  return marginalFactor;
}

/* ************************************************************************* */
Vector DiscreteMarginals::marginalProbabilities(const DiscreteKey& key) const {
  // Compute marginal
  DiscreteFactor::shared_ptr marginalFactor = this->operator()(key.first);

  // Create result
  Vector vResult(key.second);
  for (size_t state = 0; state < key.second; ++state) {
    DiscreteValues values;
    values[key.first] = state;
    vResult(state) = (*marginalFactor)(values);
  }
  return vResult;
}

/* ************************************************************************* */
void DiscreteMarginals::print(const std::string& s,
                              const KeyFormatter formatter) const {
  std::cout << (s.empty() ? "Discrete Marginals of:" : s + " ") << std::endl;
  bayesTree_->print("", formatter);
}

} /* namespace gtsam */
