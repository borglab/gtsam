/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericSequentialSolver.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/EliminationTree.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>

#include <utility>

namespace gtsam {

template<class FACTOR>
class GenericSequentialSolver {

protected:

  // Store the original factors for computing marginals
  FactorGraph<FACTOR> factors_;

  // Column structure of the factor graph
  VariableIndex structure_;

  // Elimination tree that performs elimination.
  typename EliminationTree<FACTOR>::shared_ptr eliminationTree_;

public:

  /**
   * Construct the solver for a factor graph.  This builds the elimination
   * tree, which already does some of the symbolic work of elimination.
   */
  GenericSequentialSolver(const FactorGraph<FACTOR>& factorGraph);

  /**
   * Eliminate the factor graph sequentially.  Uses a column elimination tree
   * to recursively eliminate.
   */
  typename BayesNet<typename FACTOR::Conditional>::shared_ptr eliminate() const;

  /**
   * Compute the marginal Gaussian density over a variable, by integrating out
   * all of the other variables.  This function returns the result as a factor.
   */
  typename FACTOR::shared_ptr marginal(Index j) const;

  /**
   * Compute the marginal joint over a set of variables, by integrating out
   * all of the other variables.  This function returns the result as a factor
   * graph.
   */
  typename FactorGraph<FACTOR>::shared_ptr joint(const std::vector<Index>& js) const;

};

}

