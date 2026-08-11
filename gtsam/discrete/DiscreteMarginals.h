/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteMarginals.h
 * @brief A class for computing marginals in a DiscreteFactorGraph
 * @author Abhijit Kundu
 * @author Richard Roberts
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date June 4, 2012
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>

namespace gtsam {

/**
 * A class for computing marginals of variables in a DiscreteFactorGraph
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteMarginals {
 protected:
  DiscreteBayesTree::shared_ptr bayesTree_;

 public:
  DiscreteMarginals() {}

  /** Construct a marginals class.
   * @param graph The factor graph defining the full joint
   * distribution on all variables.
   */
  DiscreteMarginals(const DiscreteFactorGraph& graph);

  /** Compute the marginal of a single variable */
  DiscreteFactor::shared_ptr operator()(Key variable) const;

  /** Compute the marginal of a single variable
   *   @param key DiscreteKey of the Variable
   *   @return Vector of marginal probabilities
   */
  Vector marginalProbabilities(const DiscreteKey& key) const;

  /// Print details
  void print(const std::string& s = "",
             const KeyFormatter formatter = DefaultKeyFormatter) const;
};

} /* namespace gtsam */
