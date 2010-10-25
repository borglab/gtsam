/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericMultifrontalSolver.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/JunctionTree.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>

#include <utility>

namespace gtsam {

template<class FACTOR, class JUNCTIONTREE>
class GenericMultifrontalSolver {

protected:

  // Junction tree that performs elimination.
  JUNCTIONTREE junctionTree_;

public:

  /**
   * Construct the solver for a factor graph.  This builds the elimination
   * tree, which already does some of the symbolic work of elimination.
   */
  GenericMultifrontalSolver(const FactorGraph<FACTOR>& factorGraph);

  /**
   * Eliminate the factor graph sequentially.  Uses a column elimination tree
   * to recursively eliminate.
   */
  typename JUNCTIONTREE::BayesTree::shared_ptr eliminate() const;

  /**
   * Compute the marginal joint over a set of variables, by integrating out
   * all of the other variables.  This function returns the result as a factor
   * graph.
   */
  typename FactorGraph<FACTOR>::shared_ptr jointFactorGraph(const std::vector<Index>& js) const;

  /**
   * Compute the marginal density over a variable, by integrating out
   * all of the other variables.  This function returns the result as a factor.
   */
  typename FACTOR::shared_ptr marginalFactor(Index j) const;

};

}


