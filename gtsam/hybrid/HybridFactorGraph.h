/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.h
 * @brief  Hybrid factor graph that uses type erasure
 * @author Fan Jiang
 * @date   Mar 11, 2022
 */

#pragma once

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Ordering.h>

namespace gtsam {

// Forward declarations
class HybridFactorGraph;
class HybridConditional;
class HybridBayesNet;
class HybridEliminationTree;
class HybridBayesTree;
class HybridJunctionTree;

class JacobianFactor;

/** Main elimination function for HybridFactorGraph */
GTSAM_EXPORT std::pair<boost::shared_ptr<HybridConditional>, HybridFactor::shared_ptr>
EliminateHybrid(const HybridFactorGraph& factors, const Ordering& keys);

/* ************************************************************************* */
template<> struct EliminationTraits<HybridFactorGraph>
{
  typedef HybridFactor FactorType;                   ///< Type of factors in factor graph
  typedef HybridFactorGraph FactorGraphType;         ///< Type of the factor graph (e.g. HybridFactorGraph)
  typedef HybridConditional ConditionalType;         ///< Type of conditionals from elimination
  typedef HybridBayesNet BayesNetType;               ///< Type of Bayes net from sequential elimination
  typedef HybridEliminationTree EliminationTreeType; ///< Type of elimination tree
  typedef HybridBayesTree BayesTreeType;             ///< Type of Bayes tree
  typedef HybridJunctionTree JunctionTreeType;       ///< Type of Junction tree
  /// The default dense elimination function
  static std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<FactorType> >
  DefaultEliminate(const FactorGraphType& factors, const Ordering& keys) {
    return EliminateHybrid(factors, keys); }
};


class HybridFactorGraph : public FactorGraph<HybridFactor>, public EliminateableFactorGraph<HybridFactorGraph> {
 public:
  using Base = FactorGraph<HybridFactor>;
  using This = HybridFactorGraph;          ///< this class
  using BaseEliminateable =
  EliminateableFactorGraph<This>;          ///< for elimination
  using shared_ptr = boost::shared_ptr<This>;  ///< shared_ptr to This

  using Values = gtsam::Values;  ///< backwards compatibility
  using Indices = KeyVector;  ///> map from keys to values

 public:
  HybridFactorGraph() = default;

  /** Construct from container of factors (shared_ptr or plain objects) */
  template <class CONTAINER>
  explicit HybridFactorGraph(const CONTAINER& factors) : Base(factors) {}

  /** Implicit copy/downcast constructor to override explicit template container
   * constructor */
  template <class DERIVEDFACTOR>
  HybridFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph) : Base(graph) {}

  using FactorGraph::add;
  
  /// Add a factor directly using a shared_ptr.
  void add(JacobianFactor &&factor);
};

}

