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
 * @author Frank Dellaert
 * @date June 4, 2012
 */

#pragma once

#include <gtsam/discrete/DiscreteFactorGraph.h>

namespace gtsam {

  /**
   * A class for computing marginals of variables in a DiscreteFactorGraph
   */
  class DiscreteMarginals {

  protected:

    BayesTree<DiscreteConditional> bayesTree_;

  public:

    /** Construct a marginals class.
     * @param graph The factor graph defining the full joint density on all variables.
     */
    DiscreteMarginals(const DiscreteFactorGraph& graph) {
    }

    /** print */
    void print(const std::string& str = "DiscreteMarginals: ") const {
    }

    /** Compute the marginal of a single variable */
    DiscreteFactor::shared_ptr operator()(Index variable) const {
      DiscreteFactor::shared_ptr p;
      return p;
    }

    /** Compute the marginal of a single variable */
    Vector marginalProbabilities(Index variable) const {
      Vector v;
      return v;
    }

  };

} /* namespace gtsam */
