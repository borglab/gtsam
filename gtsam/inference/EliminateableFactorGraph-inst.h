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
  template<class FACTORGRAPH>
  boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::eliminateSequential(
    OptionalOrdering ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(ordering && variableIndex) {
      // Do elimination
      std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> > result
        = EliminationTreeType(asDerived(), *variableIndex, *ordering).eliminate(function);
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
      return eliminateSequential(ordering, function, VariableIndexUnordered(asDerived()));
    }
    else /*if(!ordering)*/ {
      // If no Ordering provided, compute one and call this function again.  We are guaranteed to
      // have a VariableIndex already here because we computed one if needed in the previous 'else'
      // block.
      return eliminateSequential(OrderingUnordered::COLAMD(*variableIndex), function);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::eliminateMultifrontal(
    OptionalOrdering ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(ordering && variableIndex) {
      // Do elimination with given ordering
      std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> > result
        = JunctionTreeType(EliminationTreeType(asDerived(), *variableIndex, *ordering)).eliminate(function);
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
      return eliminateMultifrontal(ordering, function, VariableIndexUnordered(asDerived()));
    }
    else /*if(!ordering)*/ {
      // If no Ordering provided, compute one and call this function again.  We are guaranteed to
      // have a VariableIndex already here because we computed one if needed in the previous 'else'
      // block.
      return eliminateMultifrontal(OrderingUnordered::COLAMD(*variableIndex), function);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialSequential(
    const OrderingUnordered& ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      // Do elimination
      return EliminationTreeType(asDerived(), *variableIndex, ordering).eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      return eliminatePartialSequential(ordering, function, VariableIndexUnordered(asDerived()));
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialMultifrontal(
    const OrderingUnordered& ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      // Do elimination
      return JunctionTreeType(EliminationTreeType(asDerived(), *variableIndex, ordering)).eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      return eliminatePartialMultifrontal(ordering, function, VariableIndexUnordered(asDerived()));
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesNet(
    boost::variant<const OrderingUnordered&, const std::vector<Key>&> variables,
    OptionalOrdering marginalizedVariableOrdering,
    const Eliminate& function = EliminationTraits::DefaultEliminate,
    OptionalVariableIndex variableIndex = boost::none) const
  {
    if(variableIndex)
    {
      if(marginalizedVariableOrdering)
      {
        // An ordering was provided for the marginalized variables, so we can first eliminate them
        // in the order requested.
        std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> > eliminated =
          eliminatePartialMultifrontal(*marginalizedVariableOrdering, function, *variableIndex);

        if(const OrderingUnordered* varsAsOrdering = boost::get<const OrderingUnordered&>(&variables))
        {
          // An ordering was also provided for the unmarginalized variables, so we can also
          // eliminate them in the order requested.
          return eliminated.second->eliminateSequential(*varsAsOrdering, function);
        }
        else
        {
          // No ordering was provided for the unmarginalized variables, so order them with COLAMD.
          return eliminated.second->eliminateSequential(boost::none, function);
        }
      }
      else
      {
        // No ordering was provided for the marginalized variables, so order them using constrained
        // COLAMD.
        bool unmarginalizedAreOrdered = (boost::get<const OrderingUnordered&>(&variables) != 0);
        const std::vector<Key>* variablesOrOrdering =
          unmarginalizedAreOrdered ?
          boost::get<const OrderingUnordered&>(&variables) : boost::get<const std::vector<Key>&>(&variables);

        OrderingUnordered totalOrdering =
          OrderingUnordered::COLAMDConstrainedLast(*variableIndex, *variablesOrOrdering, unmarginalizedAreOrdered);

        // Split up ordering
        const size_t nVars = variablesOrOrdering->size();
        OrderingUnordered marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - nVars);
        OrderingUnordered marginalVarsOrdering(totalOrdering.end() - nVars, totalOrdering.end());

        // Call this function again with the computed orderings
        return marginalMultifrontalBayesNet(marginalVarsOrdering, marginalizationOrdering, function, *variableIndex);
      }
    } else {
      // If no variable index is provided, compute one and call this function again
      return marginalMultifrontalBayesNet(variables, marginalizedVariableOrdering, function, VariableIndexUnordered(asDerived()));
    }
  }

}
