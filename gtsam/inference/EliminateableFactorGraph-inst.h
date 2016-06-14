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
#include <boost/tuple/tuple.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::eliminateSequential(
    OptionalOrdering ordering, const Eliminate& function,
    OptionalVariableIndex variableIndex, OptionalOrderingType orderingType) const
  {
    if(ordering && variableIndex) {
      gttic(eliminateSequential);
      // Do elimination
      EliminationTreeType etree(asDerived(), *variableIndex, *ordering);
      boost::shared_ptr<BayesNetType> bayesNet;
      boost::shared_ptr<FactorGraphType> factorGraph;
      boost::tie(bayesNet,factorGraph) = etree.eliminate(function);
      // If any factors are remaining, the ordering was incomplete
      if(!factorGraph->empty())
        throw InconsistentEliminationRequested();
      // Return the Bayes net
      return bayesNet;
    }
    else if(!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again IMPORTANT: we check
      // for no variable index first so that it's always computed if we need to call COLAMD because
      // no Ordering is provided.
      VariableIndex computedVariableIndex(asDerived());
      return eliminateSequential(ordering, function, computedVariableIndex, orderingType);
    }
    else /*if(!ordering)*/ {
      // If no Ordering provided, compute one and call this function again.  We are guaranteed to
      // have a VariableIndex already here because we computed one if needed in the previous 'else'
      // block.
      if (orderingType == Ordering::METIS) {
        Ordering computedOrdering = Ordering::Metis(asDerived());
        return eliminateSequential(computedOrdering, function, variableIndex, orderingType);
      } else {
        Ordering computedOrdering = Ordering::Colamd(*variableIndex);
        return eliminateSequential(computedOrdering, function, variableIndex, orderingType);
      }
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::eliminateMultifrontal(
    OptionalOrdering ordering, const Eliminate& function,
    OptionalVariableIndex variableIndex, OptionalOrderingType orderingType) const
  {
    if(ordering && variableIndex) {
      gttic(eliminateMultifrontal);
      // Do elimination with given ordering
      EliminationTreeType etree(asDerived(), *variableIndex, *ordering);
      JunctionTreeType junctionTree(etree);
      boost::shared_ptr<BayesTreeType> bayesTree;
      boost::shared_ptr<FactorGraphType> factorGraph;
      boost::tie(bayesTree,factorGraph) = junctionTree.eliminate(function);
      // If any factors are remaining, the ordering was incomplete
      if(!factorGraph->empty())
        throw InconsistentEliminationRequested();
      // Return the Bayes tree
      return bayesTree;
    }
    else if(!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again IMPORTANT: we check
      // for no variable index first so that it's always computed if we need to call COLAMD because
      // no Ordering is provided.
      VariableIndex computedVariableIndex(asDerived());
      return eliminateMultifrontal(ordering, function, computedVariableIndex, orderingType);
    }
    else /*if(!ordering)*/ {
      // If no Ordering provided, compute one and call this function again.  We are guaranteed to
      // have a VariableIndex already here because we computed one if needed in the previous 'else'
      // block.
      if (orderingType == Ordering::METIS) {
        Ordering computedOrdering = Ordering::Metis(asDerived());
        return eliminateMultifrontal(computedOrdering, function, variableIndex, orderingType);
      } else {
        Ordering computedOrdering = Ordering::Colamd(*variableIndex);
        return eliminateMultifrontal(computedOrdering, function, variableIndex, orderingType);
      }
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialSequential(
    const Ordering& ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialSequential);
      // Do elimination
      EliminationTreeType etree(asDerived(), *variableIndex, ordering);
      return etree.eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialSequential(ordering, function, computedVariableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialSequential(
    const std::vector<Key>& variables, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialSequential);
      // Compute full ordering
      Ordering fullOrdering = Ordering::ColamdConstrainedFirst(*variableIndex, variables);

      // Split off the part of the ordering for the variables being eliminated
      Ordering ordering(fullOrdering.begin(), fullOrdering.begin() + variables.size());
      return eliminatePartialSequential(ordering, function, variableIndex);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialSequential(variables, function, computedVariableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialMultifrontal(
    const Ordering& ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialMultifrontal);
      // Do elimination
      EliminationTreeType etree(asDerived(), *variableIndex, ordering);
      JunctionTreeType junctionTree(etree);
      return junctionTree.eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialMultifrontal(ordering, function, computedVariableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>, boost::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialMultifrontal(
    const std::vector<Key>& variables, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialMultifrontal);
      // Compute full ordering
      Ordering fullOrdering = Ordering::ColamdConstrainedFirst(*variableIndex, variables);

      // Split off the part of the ordering for the variables being eliminated
      Ordering ordering(fullOrdering.begin(), fullOrdering.begin() + variables.size());
      return eliminatePartialMultifrontal(ordering, function, variableIndex);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialMultifrontal(variables, function, computedVariableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesNet(
    boost::variant<const Ordering&, const std::vector<Key>&> variables,
    OptionalOrdering marginalizedVariableOrdering,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex)
    {
      if(marginalizedVariableOrdering)
      {
        gttic(marginalMultifrontalBayesNet);
        // An ordering was provided for the marginalized variables, so we can first eliminate them
        // in the order requested.
        boost::shared_ptr<BayesTreeType> bayesTree;
        boost::shared_ptr<FactorGraphType> factorGraph;
        boost::tie(bayesTree,factorGraph) =
          eliminatePartialMultifrontal(*marginalizedVariableOrdering, function, *variableIndex);

        if(const Ordering* varsAsOrdering = boost::get<const Ordering&>(&variables))
        {
          // An ordering was also provided for the unmarginalized variables, so we can also
          // eliminate them in the order requested.
          return factorGraph->eliminateSequential(*varsAsOrdering, function);
        }
        else
        {
          // No ordering was provided for the unmarginalized variables, so order them with COLAMD.
          return factorGraph->eliminateSequential(boost::none, function);
        }
      }
      else
      {
        // No ordering was provided for the marginalized variables, so order them using constrained
        // COLAMD.
        bool unmarginalizedAreOrdered = (boost::get<const Ordering&>(&variables) != 0);
        const std::vector<Key>* variablesOrOrdering =
          unmarginalizedAreOrdered ?
          boost::get<const Ordering&>(&variables) : boost::get<const std::vector<Key>&>(&variables);

        Ordering totalOrdering =
          Ordering::ColamdConstrainedLast(*variableIndex, *variablesOrOrdering, unmarginalizedAreOrdered);

        // Split up ordering
        const size_t nVars = variablesOrOrdering->size();
        Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - nVars);
        Ordering marginalVarsOrdering(totalOrdering.end() - nVars, totalOrdering.end());

        // Call this function again with the computed orderings
        return marginalMultifrontalBayesNet(marginalVarsOrdering, marginalizationOrdering, function, *variableIndex);
      }
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex index(asDerived());
      return marginalMultifrontalBayesNet(variables, marginalizedVariableOrdering, function, index);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  boost::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesTree(
    boost::variant<const Ordering&, const std::vector<Key>&> variables,
    OptionalOrdering marginalizedVariableOrdering,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex)
    {
      if(marginalizedVariableOrdering)
      {
        gttic(marginalMultifrontalBayesTree);
        // An ordering was provided for the marginalized variables, so we can first eliminate them
        // in the order requested.
        boost::shared_ptr<BayesTreeType> bayesTree;
        boost::shared_ptr<FactorGraphType> factorGraph;
        boost::tie(bayesTree,factorGraph) =
          eliminatePartialMultifrontal(*marginalizedVariableOrdering, function, *variableIndex);

        if(const Ordering* varsAsOrdering = boost::get<const Ordering&>(&variables))
        {
          // An ordering was also provided for the unmarginalized variables, so we can also
          // eliminate them in the order requested.
          return factorGraph->eliminateMultifrontal(*varsAsOrdering, function);
        }
        else
        {
          // No ordering was provided for the unmarginalized variables, so order them with COLAMD.
          return factorGraph->eliminateMultifrontal(boost::none, function);
        }
      }
      else
      {
        // No ordering was provided for the marginalized variables, so order them using constrained
        // COLAMD.
        bool unmarginalizedAreOrdered = (boost::get<const Ordering&>(&variables) != 0);
        const std::vector<Key>* variablesOrOrdering =
          unmarginalizedAreOrdered ?
          boost::get<const Ordering&>(&variables) : boost::get<const std::vector<Key>&>(&variables);

        Ordering totalOrdering =
          Ordering::ColamdConstrainedLast(*variableIndex, *variablesOrOrdering, unmarginalizedAreOrdered);

        // Split up ordering
        const size_t nVars = variablesOrOrdering->size();
        Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - nVars);
        Ordering marginalVarsOrdering(totalOrdering.end() - nVars, totalOrdering.end());

        // Call this function again with the computed orderings
        return marginalMultifrontalBayesTree(marginalVarsOrdering, marginalizationOrdering, function, *variableIndex);
      }
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return marginalMultifrontalBayesTree(variables, marginalizedVariableOrdering, function, computedVariableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  boost::shared_ptr<FACTORGRAPH>
    EliminateableFactorGraph<FACTORGRAPH>::marginal(
    const std::vector<Key>& variables,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex)
    {
      // Compute a total ordering for all variables
      Ordering totalOrdering = Ordering::ColamdConstrainedLast(*variableIndex, variables);

      // Split out the part for the marginalized variables
      Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - variables.size());

      // Eliminate and return the remaining factor graph
      return eliminatePartialMultifrontal(marginalizationOrdering, function, *variableIndex).second;
    }
    else
    {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return marginal(variables, function, computedVariableIndex);
    }
  }


}
