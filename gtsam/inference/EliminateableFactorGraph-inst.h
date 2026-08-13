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
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/symbolic/IndexedJunctionTree.h>

#ifdef GTSAM_USE_TBB
#include <mutex>
#endif
#include <unordered_set>

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
      if(!factorGraph->empty()) {
        throw InconsistentEliminationRequested(factorGraph->keys());
      }
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
      if(!factorGraph->empty()) {
        throw InconsistentEliminationRequested(factorGraph->keys());
      }
      // Return the Bayes tree
      return bayesTree;
    }
  }

  /* ************************************************************************* */
  template <class FACTORGRAPH>
  IndexedJunctionTree
  EliminateableFactorGraph<FACTORGRAPH>::buildIndexedJunctionTree(
      const Ordering& ordering,
      const std::unordered_set<Key>& fixedKeys) const {
    return IndexedJunctionTree(asDerived(), ordering, fixedKeys);
  }

  /* ************************************************************************* */
  template <class FACTORGRAPH>
  std::shared_ptr<typename EliminateableFactorGraph<FACTORGRAPH>::BayesTreeType>
  EliminateableFactorGraph<FACTORGRAPH>::eliminateMultifrontal(
      const IndexedJunctionTree& indexedJunctionTree,
      const Eliminate& function) const {
    gttic(eliminateMultifrontal);

    using BayesTreeNode = typename BayesTreeType::Node;
    using SharedFactor = typename FactorGraphType::sharedFactor;

    // Elimination traversal data - stores a pointer to the parent data and collects
    // the factors resulting from elimination of the children.  Also sets up BayesTree
    // cliques with parent and child pointers.
    struct ClusterEliminationData {
      ClusterEliminationData* const parentData;
      size_t myIndexInParent;
      FastVector<SharedFactor> childFactors;
      std::shared_ptr<BayesTreeNode> bayesTreeNode;
#ifdef GTSAM_USE_TBB
      std::shared_ptr<std::mutex> writeLock;
#endif

      ClusterEliminationData(ClusterEliminationData* _parentData, size_t nChildren)
          : parentData(_parentData), bayesTreeNode(std::make_shared<BayesTreeNode>())
#ifdef GTSAM_USE_TBB
            , writeLock(std::make_shared<std::mutex>())
#endif
      {
        if (parentData) {
#ifdef GTSAM_USE_TBB
          parentData->writeLock->lock();
#endif
          myIndexInParent = parentData->childFactors.size();
          parentData->childFactors.push_back(SharedFactor());
#ifdef GTSAM_USE_TBB
          parentData->writeLock->unlock();
#endif
        } else {
          myIndexInParent = 0;
        }
        if (parentData) {
          if (parentData->parentData)
            bayesTreeNode->parent_ = parentData->bayesTreeNode;
          parentData->bayesTreeNode->children.push_back(bayesTreeNode);
        }
      }

      static ClusterEliminationData EliminationPreOrderVisitor(
          const SymbolicJunctionTree::sharedNode& node,
          ClusterEliminationData& parentData) {
        assert(node);
        ClusterEliminationData myData(&parentData, node->nrChildren());
        myData.bayesTreeNode->problemSize_ = node->problemSize();
        return myData;
      }
    };

    // Elimination post-order visitor - gather factors, eliminate, store results.
    class EliminationPostOrderVisitor {
      const FactorGraphType& graph_;
      const Eliminate& eliminationFunction_;

     public:
      EliminationPostOrderVisitor(
          const FactorGraphType& graph,
          const Eliminate& eliminationFunction)
          : graph_(graph), eliminationFunction_(eliminationFunction) {}

      void operator()(const SymbolicJunctionTree::sharedNode& node,
                      ClusterEliminationData& myData) {
        assert(node);

        FactorGraphType gatheredFactors;
        gatheredFactors.reserve(node->factors.size() + node->nrChildren());

        for (const auto& factor : node->factors) {
          auto indexed =
              std::static_pointer_cast<internal::IndexedSymbolicFactor>(factor);
          gatheredFactors.push_back(graph_.at(indexed->index_));
        }
        gatheredFactors.push_back(myData.childFactors);

        auto eliminationResult =
            eliminationFunction_(gatheredFactors, node->orderedFrontalKeys);

        myData.bayesTreeNode->setEliminationResult(eliminationResult);

        if (!eliminationResult.second->empty()) {
#ifdef GTSAM_USE_TBB
          myData.parentData->writeLock->lock();
#endif
          myData.parentData->childFactors[myData.myIndexInParent] =
              eliminationResult.second;
#ifdef GTSAM_USE_TBB
          myData.parentData->writeLock->unlock();
#endif
        }
      }
    };

    // Do elimination (depth-first traversal).  The rootsContainer stores a 'dummy'
    // BayesTree node that contains all of the roots as its children.  rootsContainer
    // also stores the remaining un-eliminated factors passed up from the roots.
    std::shared_ptr<BayesTreeType> result = std::make_shared<BayesTreeType>();

    ClusterEliminationData rootsContainer(0, indexedJunctionTree.nrRoots());

    EliminationPostOrderVisitor visitorPost(asDerived(), function);
    {
      TbbOpenMPMixedScope threadLimiter;
      treeTraversal::DepthFirstForestParallel(
          indexedJunctionTree, rootsContainer,
          ClusterEliminationData::EliminationPreOrderVisitor, visitorPost, 10);
    }

    // Create BayesTree from roots stored in the dummy BayesTree node.
    for (const auto& rootClique : rootsContainer.bayesTreeNode->children)
      result->insertRoot(rootClique);

    // If any factors are remaining, the ordering was incomplete.
    KeySet remainingKeys;
    for (const auto& factor : rootsContainer.childFactors) {
      if (!factor || factor->empty()) continue;
      remainingKeys.insert(factor->begin(), factor->end());
    }
    if (!remainingKeys.empty()) {
      throw InconsistentEliminationRequested(remainingKeys);
    }

    return result;
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
