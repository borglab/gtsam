/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    EliminateableFactorGraph.h
 * @brief   Variable elimination algorithms for factor graphs
 * @author  Richard Roberts
 * @date    Apr 21, 2013
 */

#pragma once

#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/inferenceExceptions.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class DERIVED, class ELIMINATIONTREE, class JUNCTIONTREE>
  boost::shared_ptr<typename ELIMINATIONTREE::BayesNetType>
    EliminateableFactorGraph<DERIVED,ELIMINATIONTREE,JUNCTIONTREE>::eliminateSequential(
    const Eliminate& function, OptionalOrdering ordering, const VariableIndexUnordered& variableIndex) const
  {
    // Do elimination
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> > result;
    if(ordering) {
      // Do elimination with given ordering
      result = EliminationTreeType(*this, variableIndex, *ordering).eliminate(function);
    } else {
      // Compute ordering
      OrderingUnordered colamdOrdering = variableIndex.orderingCOLAMD();
      result = EliminationTreeType(*this, variableIndex, colamdOrdering).eliminate(function);
    }

    // If any factors are remaining, the ordering was incomplete
    if(!result.second->empty())
      throw InconsistentEliminationRequested();

    // Return the Bayes net
    return result.first;
  }

  /* ************************************************************************* */
  template<class DERIVED, class ELIMINATIONTREE, class JUNCTIONTREE>
  boost::shared_ptr<typename JUNCTIONTREE::BayesTreeType>
    EliminateableFactorGraph<DERIVED,ELIMINATIONTREE,JUNCTIONTREE>::eliminateMultifrontal(
    const Eliminate& function, OptionalOrdering ordering, const VariableIndexUnordered& variableIndex) const
  {
    // Do elimination
    std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> > result;
    if(ordering) {
      // Do elimination with given ordering
      result = JunctionTreeType(*this, variableIndex, *ordering).eliminate(function);
    } else {
      // Compute ordering
      OrderingUnordered colamdOrdering = variableIndex.orderingCOLAMD();
      result = JunctionTreeType(*this, variableIndex, colamdOrdering).eliminate(function);
    }

    // If any factors are remaining, the ordering was incomplete
    if(!result.second->empty())
      throw InconsistentEliminationRequested();

    // Return the Bayes tree
    return result.first;
  }
}
