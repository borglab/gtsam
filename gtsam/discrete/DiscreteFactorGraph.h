/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteFactorGraph.h
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/base/FastSet.h>
#include <boost/make_shared.hpp>

namespace gtsam {

// Forward declarations
class DiscreteFactorGraph;
class DiscreteFactor;
class DiscreteConditional;
class DiscreteBayesNet;
class DiscreteEliminationTree;
class DiscreteBayesTree;
class DiscreteJunctionTree;

/** Main elimination function for DiscreteFactorGraph */
GTSAM_EXPORT std::pair<boost::shared_ptr<DiscreteConditional>, DecisionTreeFactor::shared_ptr>
EliminateDiscrete(const DiscreteFactorGraph& factors, const Ordering& keys);

/* ************************************************************************* */
template<> struct EliminationTraits<DiscreteFactorGraph>
{
  typedef DiscreteFactor FactorType;                   ///< Type of factors in factor graph
  typedef DiscreteFactorGraph FactorGraphType;         ///< Type of the factor graph (e.g. DiscreteFactorGraph)
  typedef DiscreteConditional ConditionalType;         ///< Type of conditionals from elimination
  typedef DiscreteBayesNet BayesNetType;               ///< Type of Bayes net from sequential elimination
  typedef DiscreteEliminationTree EliminationTreeType; ///< Type of elimination tree
  typedef DiscreteBayesTree BayesTreeType;             ///< Type of Bayes tree
  typedef DiscreteJunctionTree JunctionTreeType;       ///< Type of Junction tree
  /// The default dense elimination function
  static std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<FactorType> >
  DefaultEliminate(const FactorGraphType& factors, const Ordering& keys) {
    return EliminateDiscrete(factors, keys); }
};

/* ************************************************************************* */
/**
 * A Discrete Factor Graph is a factor graph where all factors are Discrete, i.e.
 *   Factor == DiscreteFactor
 */
class GTSAM_EXPORT DiscreteFactorGraph: public FactorGraph<DiscreteFactor>,
public EliminateableFactorGraph<DiscreteFactorGraph> {
public:

  typedef DiscreteFactorGraph This; ///< Typedef to this class
  typedef FactorGraph<DiscreteFactor> Base; ///< Typedef to base factor graph type
  typedef EliminateableFactorGraph<This> BaseEliminateable; ///< Typedef to base elimination class
  typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

  using Values = DiscreteValues; ///< backwards compatibility

  /** A map from keys to values */
  typedef KeyVector Indices;

  /** Default constructor */
  DiscreteFactorGraph() {}

  /** Construct from iterator over factors */
  template<typename ITERATOR>
  DiscreteFactorGraph(ITERATOR firstFactor, ITERATOR lastFactor) : Base(firstFactor, lastFactor) {}

  /** Construct from container of factors (shared_ptr or plain objects) */
  template<class CONTAINER>
  explicit DiscreteFactorGraph(const CONTAINER& factors) : Base(factors) {}

  /** Implicit copy/downcast constructor to override explicit template container constructor */
  template<class DERIVEDFACTOR>
  DiscreteFactorGraph(const FactorGraph<DERIVEDFACTOR>& graph) : Base(graph) {}

  /// Destructor
  virtual ~DiscreteFactorGraph() {}

  /// @name Testable
  /// @{

  bool equals(const This& fg, double tol = 1e-9) const;

  /// @}

  /** Add a decision-tree factor */
  template <typename... Args>
  void add(Args&&... args) {
    emplace_shared<DecisionTreeFactor>(std::forward<Args>(args)...);
  }
      
  /** Return the set of variables involved in the factors (set union) */
  KeySet keys() const;

  /** return product of all factors as a single factor */
  DecisionTreeFactor product() const;

  /** 
   * Evaluates the factor graph given values, returns the joint probability of
   * the factor graph given specific instantiation of values
   */
  double operator()(const DiscreteValues& values) const;

  /// print
  void print(
      const std::string& s = "DiscreteFactorGraph",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /** Solve the factor graph by performing variable elimination in COLAMD order using
   *  the dense elimination function specified in \c function,
   *  followed by back-substitution resulting from elimination.  Is equivalent
   *  to calling graph.eliminateSequential()->optimize(). */
  DiscreteValues optimize() const;


//  /** Permute the variables in the factors */
//  GTSAM_EXPORT void permuteWithInverse(const Permutation& inversePermutation);
//
//  /** Apply a reduction, which is a remapping of variable indices. */
//  GTSAM_EXPORT void reduceWithInverse(const internal::Reduction& inverseReduction);

  /// @name Wrapper support
  /// @{

  /// Render as markdown table.
  std::string markdown(
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @}
}; // \ DiscreteFactorGraph

/// traits
template<> struct traits<DiscreteFactorGraph> : public Testable<DiscreteFactorGraph> {};

} // \ namespace gtsam
