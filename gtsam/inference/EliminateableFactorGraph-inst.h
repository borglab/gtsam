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
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::eliminateSequential(
    OptionalOrderingType orderingType, const Eliminate& function,
    OptionalVariableIndex variableIndex) const {
    if(!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again IMPORTANT: we check
      // for no variable index first so that it's always computed if we need to call COLAMD because
      // no Ordering is provided.  When removing optional from VariableIndex, create VariableIndex
      // before creating ordering.
      VariableIndex computedVariableIndex(asDerived());
      return eliminateSequential(orderingType, function, std::cref(computedVariableIndex));
    }
    else {
      // Compute an ordering and call this function again.  We are guaranteed to have a 
      // VariableIndex already here because we computed one if needed in the previous 'if' block.
      if (orderingType == Ordering::METIS) {
        Ordering computedOrdering = Ordering::Metis(asDerived());
        return eliminateSequential(computedOrdering, function, variableIndex);
      } else if (orderingType == Ordering::COLAMD) {
        Ordering computedOrdering = Ordering::Colamd((*variableIndex).get());
        return eliminateSequential(computedOrdering, function, variableIndex);
      } else if (orderingType == Ordering::NATURAL) {
        Ordering computedOrdering = Ordering::Natural(asDerived());
        return eliminateSequential(computedOrdering, function, variableIndex);
      } else {
        Ordering computedOrdering = EliminationTraitsType::DefaultOrderingFunc(
            asDerived(), *variableIndex);
        return eliminateSequential(computedOrdering, function, variableIndex);
      }
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::eliminateSequential(
    const Ordering& ordering, const Eliminate& function,
    OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminateSequential(ordering, function, std::cref(computedVariableIndex));
    } else {
      gttic(eliminateSequential);
      // Do elimination
      EliminationTreeType etree(asDerived(), (*variableIndex).get(), ordering);
      const auto [bayesNet, factorGraph] = etree.eliminate(function);
      // If any factors are remaining, the ordering was incomplete
      if(!factorGraph->empty())
        throw InconsistentEliminationRequested();
      // Return the Bayes net
      return bayesNet;
    }
  }

  /* ************************************************************************* */
  template <class FACTORGRAPH>
  std::shared_ptr<
      typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
  EliminateableFactorGraph<FACTORGRAPH>::eliminateMultifrontal(
      OptionalOrderingType orderingType, const Eliminate& function,
      OptionalVariableIndex variableIndex) const {
    if (!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again
      // IMPORTANT: we check for no variable index first so that it's always
      // computed if we need to call COLAMD because no Ordering is provided.
      // When removing optional from VariableIndex, create VariableIndex before
      // creating ordering.
      VariableIndex computedVariableIndex(asDerived());
      return eliminateMultifrontal(orderingType, function,
                                   std::cref(computedVariableIndex));
    } else {
      // Compute an ordering and call this function again.  We are guaranteed to
      // have a VariableIndex already here because we computed one if needed in
      // the previous 'if' block.
      if (orderingType == Ordering::METIS) {
        Ordering computedOrdering = Ordering::Metis(asDerived());
        return eliminateMultifrontal(computedOrdering, function, variableIndex);
      } else if (orderingType == Ordering::COLAMD) {
        Ordering computedOrdering = Ordering::Colamd((*variableIndex).get());
        return eliminateMultifrontal(computedOrdering, function, variableIndex);
      } else if (orderingType == Ordering::NATURAL) {
        Ordering computedOrdering = Ordering::Natural(asDerived());
        return eliminateMultifrontal(computedOrdering, function, variableIndex);
      } else {
        Ordering computedOrdering = EliminationTraitsType::DefaultOrderingFunc(
            asDerived(), *variableIndex);
        return eliminateMultifrontal(computedOrdering, function, variableIndex);
      }
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::eliminateMultifrontal(
    const Ordering& ordering, const Eliminate& function,
    OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no VariableIndex provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminateMultifrontal(ordering, function, std::cref(computedVariableIndex));
    } else {
      gttic(eliminateMultifrontal);
      // Do elimination with given ordering
      EliminationTreeType etree(asDerived(), (*variableIndex).get(), ordering);
      JunctionTreeType junctionTree(etree);
      const auto [bayesTree, factorGraph] = junctionTree.eliminate(function);
      // If any factors are remaining, the ordering was incomplete
      if(!factorGraph->empty())
        throw InconsistentEliminationRequested();
      // Return the Bayes tree
      return bayesTree;
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>, std::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialSequential(
    const Ordering& ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialSequential);
      // Do elimination
      EliminationTreeType etree(asDerived(), (*variableIndex).get(), ordering);
      return etree.eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialSequential(ordering, function, std::cref(computedVariableIndex));
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>, std::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialSequential(
    const KeyVector& variables, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialSequential);
      // Compute full ordering
      Ordering fullOrdering = Ordering::ColamdConstrainedFirst((*variableIndex).get(), variables);

      // Split off the part of the ordering for the variables being eliminated
      Ordering ordering(fullOrdering.begin(), fullOrdering.begin() + variables.size());
      return eliminatePartialSequential(ordering, function, variableIndex);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialSequential(variables, function, std::cref(computedVariableIndex));
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>, std::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialMultifrontal(
    const Ordering& ordering, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialMultifrontal);
      // Do elimination
      EliminationTreeType etree(asDerived(), (*variableIndex).get(), ordering);
      JunctionTreeType junctionTree(etree);
      return junctionTree.eliminate(function);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialMultifrontal(ordering, function, std::cref(computedVariableIndex));
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::pair<std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>, std::shared_ptr<FACTORGRAPH> >
    EliminateableFactorGraph<FACTORGRAPH>::eliminatePartialMultifrontal(
    const KeyVector& variables, const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex) {
      gttic(eliminatePartialMultifrontal);
      // Compute full ordering
      Ordering fullOrdering = Ordering::ColamdConstrainedFirst((*variableIndex).get(), variables);

      // Split off the part of the ordering for the variables being eliminated
      Ordering ordering(fullOrdering.begin(), fullOrdering.begin() + variables.size());
      return eliminatePartialMultifrontal(ordering, function, variableIndex);
    } else {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return eliminatePartialMultifrontal(variables, function, std::cref(computedVariableIndex));
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesNet(
    const Ordering& variables,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex index(asDerived());
      return marginalMultifrontalBayesNet(variables, function, std::cref(index));
    } else {
      // No ordering was provided for the marginalized variables, so order them using constrained
      // COLAMD.
      constexpr bool forceOrder = true;
      Ordering totalOrdering =
        Ordering::ColamdConstrainedLast((*variableIndex).get(), variables, forceOrder);

      // Split up ordering
      const size_t nVars = variables.size();
      Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - nVars);
      Ordering marginalVarsOrdering(totalOrdering.end() - nVars, totalOrdering.end());

      // Call this function again with the computed orderings
      return marginalMultifrontalBayesNet(marginalVarsOrdering, marginalizationOrdering, function, variableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesNet(
    const KeyVector& variables,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex index(asDerived());
      return marginalMultifrontalBayesNet(variables, function, std::cref(index));
    } else {
      // No ordering was provided for the marginalized variables, so order them using constrained
      // COLAMD.
      const constexpr bool forceOrder = false;
      Ordering totalOrdering =
        Ordering::ColamdConstrainedLast((*variableIndex).get(), variables, forceOrder);

      // Split up ordering
      const size_t nVars = variables.size();
      Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - nVars);
      Ordering marginalVarsOrdering(totalOrdering.end() - nVars, totalOrdering.end());

      // Call this function again with the computed orderings
      return marginalMultifrontalBayesNet(marginalVarsOrdering, marginalizationOrdering, function, variableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesNet(
    const Ordering& variables,
    const Ordering& marginalizedVariableOrdering,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex index(asDerived());
      return marginalMultifrontalBayesNet(variables, marginalizedVariableOrdering, function, index);
    } else {
      gttic(marginalMultifrontalBayesNet);
      // An ordering was provided for the marginalized variables, so we can first eliminate them
      // in the order requested.
      const auto [bayesTree, factorGraph] =
        eliminatePartialMultifrontal(marginalizedVariableOrdering, function, variableIndex);

      // An ordering was also provided for the unmarginalized variables, so we can also
      // eliminate them in the order requested.
      return factorGraph->eliminateSequential(variables, function);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesNetType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesNet(
    const KeyVector& variables,
    const Ordering& marginalizedVariableOrdering,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex index(asDerived());
      return marginalMultifrontalBayesNet(variables, marginalizedVariableOrdering, function, index);
    } else {
      gttic(marginalMultifrontalBayesNet);
      // An ordering was provided for the marginalized variables, so we can first eliminate them
      // in the order requested.
      const auto [bayesTree, factorGraph] =
        eliminatePartialMultifrontal(marginalizedVariableOrdering, function, variableIndex);

      // No ordering was provided for the unmarginalized variables, so order them with COLAMD.
      return factorGraph->eliminateSequential(Ordering::COLAMD, function);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesTree(
    const Ordering& variables,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return marginalMultifrontalBayesTree(variables, function, std::cref(computedVariableIndex));
    } else {
      // No ordering was provided for the marginalized variables, so order them using constrained
      // COLAMD.
      constexpr bool forceOrder = true;
      Ordering totalOrdering =
        Ordering::ColamdConstrainedLast((*variableIndex).get(), variables, forceOrder);

      // Split up ordering
      const size_t nVars = variables.size();
      Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - nVars);
      Ordering marginalVarsOrdering(totalOrdering.end() - nVars, totalOrdering.end());

      // Call this function again with the computed orderings
      return marginalMultifrontalBayesTree(marginalVarsOrdering, marginalizationOrdering, function, variableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesTree(
    const KeyVector& variables,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return marginalMultifrontalBayesTree(variables, function, std::cref(computedVariableIndex));
    } else {
      // No ordering was provided for the marginalized variables, so order them using constrained
      // COLAMD.
      constexpr bool forceOrder = false;
      Ordering totalOrdering =
        Ordering::ColamdConstrainedLast((*variableIndex).get(), variables, forceOrder);

      // Split up ordering
      const size_t nVars = variables.size();
      Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - nVars);
      Ordering marginalVarsOrdering(totalOrdering.end() - nVars, totalOrdering.end());

      // Call this function again with the computed orderings
      return marginalMultifrontalBayesTree(marginalVarsOrdering, marginalizationOrdering, function, variableIndex);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesTree(
    const Ordering& variables,
    const Ordering& marginalizedVariableOrdering,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return marginalMultifrontalBayesTree(variables, marginalizedVariableOrdering, function, std::cref(computedVariableIndex));
    } else {
      gttic(marginalMultifrontalBayesTree);
      // An ordering was provided for the marginalized variables, so we can first eliminate them
      // in the order requested.
      const auto [bayesTree, factorGraph] =
        eliminatePartialMultifrontal(marginalizedVariableOrdering, function, variableIndex);

      // An ordering was also provided for the unmarginalized variables, so we can also
      // eliminate them in the order requested.
      return factorGraph->eliminateMultifrontal(variables, function);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
    EliminateableFactorGraph<FACTORGRAPH>::marginalMultifrontalBayesTree(
    const KeyVector& variables,
    const Ordering& marginalizedVariableOrdering,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(!variableIndex) {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return marginalMultifrontalBayesTree(variables, marginalizedVariableOrdering, function, std::cref(computedVariableIndex));
    } else {
      gttic(marginalMultifrontalBayesTree);
      // An ordering was provided for the marginalized variables, so we can first eliminate them
      // in the order requested.
      const auto [bayesTree, factorGraph] =
        eliminatePartialMultifrontal(marginalizedVariableOrdering, function, variableIndex);

      // No ordering was provided for the unmarginalized variables, so order them with COLAMD.
      return factorGraph->eliminateMultifrontal(Ordering::COLAMD, function);
    }
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  std::shared_ptr<FACTORGRAPH>
    EliminateableFactorGraph<FACTORGRAPH>::marginal(
    const KeyVector& variables,
    const Eliminate& function, OptionalVariableIndex variableIndex) const
  {
    if(variableIndex)
    {
      // Compute a total ordering for all variables
      Ordering totalOrdering = Ordering::ColamdConstrainedLast((*variableIndex).get(), variables);

      // Split out the part for the marginalized variables
      Ordering marginalizationOrdering(totalOrdering.begin(), totalOrdering.end() - variables.size());

      // Eliminate and return the remaining factor graph
      return eliminatePartialMultifrontal(marginalizationOrdering, function, variableIndex).second;
    }
    else
    {
      // If no variable index is provided, compute one and call this function again
      VariableIndex computedVariableIndex(asDerived());
      return marginal(variables, function, std::cref(computedVariableIndex));
    }
  }


}
