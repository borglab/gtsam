/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianHybridFactorGraph.h
 * @brief  Linearized Hybrid factor graph that uses type erasure
 * @author Fan Jiang
 * @date   Mar 11, 2022
 */

#pragma once

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Ordering.h>

namespace gtsam {

// Forward declarations
class GaussianHybridFactorGraph;
class HybridConditional;
class HybridBayesNet;
class HybridEliminationTree;
class HybridBayesTree;
class HybridJunctionTree;
class DecisionTreeFactor;

class JacobianFactor;

/** Main elimination function for GaussianHybridFactorGraph */
GTSAM_EXPORT
std::pair<boost::shared_ptr<HybridConditional>, HybridFactor::shared_ptr>
EliminateHybrid(const GaussianHybridFactorGraph& factors, const Ordering& keys);

/* ************************************************************************* */
template <>
struct EliminationTraits<GaussianHybridFactorGraph> {
  typedef HybridFactor FactorType;  ///< Type of factors in factor graph
  typedef GaussianHybridFactorGraph
      FactorGraphType;  ///< Type of the factor graph (e.g.
                        ///< GaussianHybridFactorGraph)
  typedef HybridConditional
      ConditionalType;  ///< Type of conditionals from elimination
  typedef HybridBayesNet
      BayesNetType;  ///< Type of Bayes net from sequential elimination
  typedef HybridEliminationTree
      EliminationTreeType;                      ///< Type of elimination tree
  typedef HybridBayesTree BayesTreeType;        ///< Type of Bayes tree
  typedef HybridJunctionTree JunctionTreeType;  ///< Type of Junction tree
  /// The default dense elimination function
  static std::pair<boost::shared_ptr<ConditionalType>,
                   boost::shared_ptr<FactorType> >
  DefaultEliminate(const FactorGraphType& factors, const Ordering& keys) {
    return EliminateHybrid(factors, keys);
  }
};

/**
 * Gaussian Hybrid Factor Graph
 * -----------------------
 * This is the linearized version of a hybrid factor graph.
 * Everything inside needs to be hybrid factor or hybrid conditional.
 */
class GaussianHybridFactorGraph
    : public HybridFactorGraph,
      public EliminateableFactorGraph<GaussianHybridFactorGraph> {
 public:
  using Base = HybridFactorGraph;
  using This = GaussianHybridFactorGraph;  ///< this class
  using BaseEliminateable =
      EliminateableFactorGraph<This>;          ///< for elimination
  using shared_ptr = boost::shared_ptr<This>;  ///< shared_ptr to This

  using Values = gtsam::Values;  ///< backwards compatibility
  using Indices = KeyVector;     ///> map from keys to values

  /// @name Constructors
  /// @{

  GaussianHybridFactorGraph() = default;

  /**
   * Implicit copy/downcast constructor to override explicit template container
   * constructor. In BayesTree this is used for:
   * `cachedSeparatorMarginal_.reset(*separatorMarginal)`
   * */
  template <class DERIVEDFACTOR>
  GaussianHybridFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph)
      : Base(graph) {}

  /// @}

  using Base::empty;
  using Base::reserve;
  using Base::size;
  using Base::operator[];
  using Base::add;
  using Base::push_back;
  using Base::resize;

  /// Add a Jacobian factor to the factor graph.
  void add(JacobianFactor&& factor);

  /// Add a Jacobian factor as a shared ptr.
  void add(boost::shared_ptr<JacobianFactor> factor);

  /// Add a DecisionTreeFactor to the factor graph.
  void add(DecisionTreeFactor&& factor);

  /// Add a DecisionTreeFactor as a shared ptr.
  void add(boost::shared_ptr<DecisionTreeFactor> factor);
};

}  // namespace gtsam
