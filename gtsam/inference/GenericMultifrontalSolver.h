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

/**
 * A Generic Multifrontal Solver class
 * Takes two template arguments:
 *   FACTOR the factor type, e.g., GaussianFactor, DiscreteFactor
 *   JUNCTIONTREE annoyingly, you also have to supply a compatible JT type
 *                i.e., one templated on a factor graph with the same factors
 *                TODO: figure why this is so and possibly fix it
 */
template<class FACTOR, class JUNCTIONTREE>
class GenericMultifrontalSolver {

protected:

  // Column structure of the factor graph
  VariableIndex::shared_ptr structure_;

  // Junction tree that performs elimination.
  typename JUNCTIONTREE::shared_ptr junctionTree_;

public:

  /**
   * Construct the solver for a factor graph.  This builds the junction
   * tree, which does the symbolic elimination, identifies the cliques,
   * and distributes all the factors to the right cliques.
   */
  GenericMultifrontalSolver(const FactorGraph<FACTOR>& factorGraph);

  /**
   * Construct the solver with a shared pointer to a factor graph and to a
   * VariableIndex.  The solver will store these pointers, so this constructor
   * is the fastest.
   */
  GenericMultifrontalSolver(const typename FactorGraph<FACTOR>::shared_ptr& factorGraph, const VariableIndex::shared_ptr& variableIndex);

  /**
   * Replace the factor graph with a new one having the same structure.  The
   * This function can be used if the numerical part of the factors changes,
   * such as during relinearization or adjusting of noise models.
   */
  void replaceFactors(const typename FactorGraph<FACTOR>::shared_ptr& factorGraph);

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


