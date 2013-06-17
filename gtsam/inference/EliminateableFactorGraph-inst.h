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
    const Eliminate& function, OptionalOrdering ordering, OptionalVariableIndex variableIndex) const
  {
    if(ordering && variableIndex) {
      // Do elimination
      std::pair<boost::shared_ptr<BAYESNET>, boost::shared_ptr<FACTORGRAPH> > result
        = ELIMINATIONTREE(asDerived(), *variableIndex, *ordering).eliminate(function);
      // If any factors are remaining, the ordering was incomplete
      if(!result.second->empty())
        throw InconsistentEliminationRequested();
      // Return the Bayes net
      return result.first;
    }
    else if(!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again IMPORTANT: we check
      // for no variable index first so that it's always computed if we need to call COLAMD because
      // no Ordering is provided.
      return eliminateSequential(function, ordering, VariableIndexUnordered(asDerived()));
    }
    else /*if(!ordering)*/ {
      // If no Ordering provided, compute one and call this function again.  We are guaranteed to
      // have a VariableIndex already here because we computed one if needed in the previous 'else'
      // block.
      return eliminateSequential(function, OrderingUnordered::COLAMD(*variableIndex));
    }
  }

  /* ************************************************************************* */
  template<class FACTOR, class FACTORGRAPH, class CONDITIONAL,
  class BAYESNET, class ELIMINATIONTREE, class BAYESTREE, class JUNCTIONTREE>
    boost::shared_ptr<BAYESTREE>
    EliminateableFactorGraph<FACTOR, FACTORGRAPH, CONDITIONAL, BAYESNET, ELIMINATIONTREE, BAYESTREE, JUNCTIONTREE>
    ::eliminateMultifrontal(const Eliminate& function, OptionalOrdering ordering, OptionalVariableIndex variableIndex) const
  {
    if(ordering && variableIndex) {
      // Do elimination with given ordering
      std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<FACTORGRAPH> > result
        = JUNCTIONTREE(ELIMINATIONTREE(asDerived(), *variableIndex, *ordering)).eliminate(function);
      // If any factors are remaining, the ordering was incomplete
      if(!result.second->empty())
        throw InconsistentEliminationRequested();
      // Return the Bayes tree
      return result.first;
    }
    else if(!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again IMPORTANT: we check
      // for no variable index first so that it's always computed if we need to call COLAMD because
      // no Ordering is provided.
      return eliminateMultifrontal(function, ordering, VariableIndexUnordered(asDerived()));
    }
    else /*if(!ordering)*/ {
      // If no Ordering provided, compute one and call this function again.  We are guaranteed to
      // have a VariableIndex already here because we computed one if needed in the previous 'else'
      // block.
      return eliminateMultifrontal(function, OrderingUnordered::COLAMD(*variableIndex));
    }
  }

  /* ************************************************************************* */
  template<class FACTOR, class FACTORGRAPH, class CONDITIONAL,
  class BAYESNET, class ELIMINATIONTREE, class BAYESTREE, class JUNCTIONTREE>
    std::pair<boost::shared_ptr<BAYESNET>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTOR, FACTORGRAPH, CONDITIONAL, BAYESNET, ELIMINATIONTREE, BAYESTREE, JUNCTIONTREE>
    ::eliminatePartialSequential(const Eliminate& function, const OrderingUnordered& ordering,
    OptionalVariableIndex variableIndex = boost::none) const
  {
    if(variableIndex) {
      // Do elimination
      return ELIMINATIONTREE(asDerived(), *variableIndex, ordering).eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      return eliminatePartialSequential(function, ordering, VariableIndexUnordered(asDerived()));
    }
  }

  /* ************************************************************************* */
  template<class FACTOR, class FACTORGRAPH, class CONDITIONAL,
  class BAYESNET, class ELIMINATIONTREE, class BAYESTREE, class JUNCTIONTREE>
    std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTOR, FACTORGRAPH, CONDITIONAL, BAYESNET, ELIMINATIONTREE, BAYESTREE, JUNCTIONTREE>
    ::eliminatePartialMultifrontal(const Eliminate& function, const OrderingUnordered& ordering,
    OptionalVariableIndex variableIndex = boost::none) const
  {
    if(variableIndex) {
      // Do elimination
      return JUNCTIONTREE(ELIMINATIONTREE(asDerived(), *variableIndex, ordering)).eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      return eliminatePartialMultifrontal(function, ordering, VariableIndexUnordered(asDerived()));
    }
  }


}
