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
  template<class FACTOR, class FACTORGRAPH, class CONDITIONAL,
  class BAYESNET, class ELIMINATIONTREE, class BAYESTREE, class JUNCTIONTREE>
  boost::shared_ptr<BAYESNET>
    EliminateableFactorGraph<FACTOR, FACTORGRAPH, CONDITIONAL, BAYESNET, ELIMINATIONTREE, BAYESTREE, JUNCTIONTREE>::
    eliminateSequential(
    const Eliminate& function, OptionalOrdering ordering, const VariableIndexUnordered& variableIndex) const
  {
    // Do elimination
    std::pair<boost::shared_ptr<BAYESNET>, boost::shared_ptr<FACTORGRAPH> > result;
    if(ordering) {
      // Do elimination with given ordering
      result = ELIMINATIONTREE(asDerived(), variableIndex, *ordering).eliminate(function);
    } else {
      // Compute ordering
      OrderingUnordered colamdOrdering = OrderingUnordered::COLAMD(variableIndex);
      result = ELIMINATIONTREE(asDerived(), variableIndex, colamdOrdering).eliminate(function);
    }

    // If any factors are remaining, the ordering was incomplete
    if(!result.second->empty())
      throw InconsistentEliminationRequested();

    // Return the Bayes net
    return result.first;
  }

  /* ************************************************************************* */
  template<class FACTOR, class FACTORGRAPH, class CONDITIONAL,
  class BAYESNET, class ELIMINATIONTREE, class BAYESTREE, class JUNCTIONTREE>
  boost::shared_ptr<BAYESTREE>
    EliminateableFactorGraph<FACTOR, FACTORGRAPH, CONDITIONAL, BAYESNET, ELIMINATIONTREE, BAYESTREE, JUNCTIONTREE>::
    eliminateMultifrontal(
    const Eliminate& function, OptionalOrdering ordering, const VariableIndexUnordered& variableIndex) const
  {
    // Do elimination
    std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<FACTORGRAPH> > result;
    if(ordering) {
      // Do elimination with given ordering
      result = JUNCTIONTREE(ELIMINATIONTREE(asDerived(), variableIndex, *ordering)).eliminate(function);
    } else {
      // Compute ordering
      OrderingUnordered colamdOrdering = OrderingUnordered::COLAMD(variableIndex);
      result = JUNCTIONTREE(ELIMINATIONTREE(asDerived(), variableIndex, colamdOrdering)).eliminate(function);
    }

    // If any factors are remaining, the ordering was incomplete
    if(!result.second->empty())
      throw InconsistentEliminationRequested();

    // Return the Bayes tree
    return result.first;
  }

}
