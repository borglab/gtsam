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

#include <boost/shared_ptr.hpp>
#include <cstddef>
#include <functional>
#include <optional>
#include <boost/variant.hpp>

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/VariableIndex.h>

namespace gtsam {

  /// Traits class for eliminateable factor graphs, specifies the types that result from
  /// elimination, etc.  This must be defined for each factor graph that inherits from
  /// EliminateableFactorGraph.
  template<class GRAPH>
  struct EliminationTraits
  {
    // Template for deriving:
    // typedef MyFactor FactorType;                   ///< Type of factors in factor graph (e.g. GaussianFactor)
    // typedef MyFactorGraphType FactorGraphType;     ///< Type of the factor graph (e.g. GaussianFactorGraph)
    // typedef MyConditional ConditionalType;         ///< Type of conditionals from elimination (e.g. GaussianConditional)
    // typedef MyBayesNet BayesNetType;               ///< Type of Bayes net from sequential elimination (e.g. GaussianBayesNet)
    // typedef MyEliminationTree EliminationTreeType; ///< Type of elimination tree (e.g. GaussianEliminationTree)
    // typedef MyBayesTree BayesTreeType;             ///< Type of Bayes tree (e.g. GaussianBayesTree)
    // typedef MyJunctionTree JunctionTreeType;       ///< Type of Junction tree (e.g. GaussianJunctionTree)
    // static pair<shared_ptr<ConditionalType>, shared_ptr<FactorType>
    //   DefaultEliminate(
    //   const MyFactorGraph& factors, const Ordering& keys); ///< The default dense elimination function
  };


  /** EliminateableFactorGraph is a base class for factor graphs that contains elimination
   *  algorithms.  Any factor graph holding eliminateable factors can derive from this class to
   *  expose functions for computing marginals, conditional marginals, doing multifrontal and
   *  sequential elimination, etc. */
  template<class FACTORGRAPH>
  class EliminateableFactorGraph
  {
  private:
    typedef EliminateableFactorGraph<FACTORGRAPH> This; ///< Typedef to this class.
    typedef FACTORGRAPH FactorGraphType; ///< Typedef to factor graph type
    // Base factor type stored in this graph (private because derived classes will get this from
    // their FactorGraph base class)
    typedef typename EliminationTraits<FactorGraphType>::FactorType _FactorType;

  public:
    /// Typedef to the specific EliminationTraits for this graph
    typedef EliminationTraits<FactorGraphType> EliminationTraitsType;

    /// Conditional type stored in the Bayes net produced by elimination
    typedef typename EliminationTraitsType::ConditionalType ConditionalType;

    /// Bayes net type produced by sequential elimination
    typedef typename EliminationTraitsType::BayesNetType BayesNetType;

    /// Elimination tree type that can do sequential elimination of this graph
    typedef typename EliminationTraitsType::EliminationTreeType EliminationTreeType;

    /// Bayes tree type produced by multifrontal elimination
    typedef typename EliminationTraitsType::BayesTreeType BayesTreeType;

    /// Junction tree type that can do multifrontal elimination of this graph
    typedef typename EliminationTraitsType::JunctionTreeType JunctionTreeType;

    /// The pair of conditional and remaining factor produced by a single dense elimination step on
    /// a subgraph.
    typedef std::pair<boost::shared_ptr<ConditionalType>, boost::shared_ptr<_FactorType> > EliminationResult;

    /// The function type that does a single dense elimination step on a subgraph.
    typedef std::function<EliminationResult(const FactorGraphType&, const Ordering&)> Eliminate;

    /// Typedef for an optional variable index as an argument to elimination functions
    typedef const VariableIndex* OptionalVariableIndex;

    /// Typedef for an optional ordering type
    typedef std::optional<Ordering::OrderingType> OptionalOrderingType;

    /** Do sequential elimination of all variables to produce a Bayes net.  If an ordering is not
     *  provided, the ordering provided by COLAMD will be used.
     *
     *  <b> Example - Full Cholesky elimination in COLAMD order: </b>
     *  \code
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(EliminateCholesky);
     *  \endcode
     *
     *  <b> Example - METIS ordering for elimination
     *  \code
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(OrderingType::METIS);
     *  \endcode
     *
     *  <b> Example - Reusing an existing VariableIndex to improve performance, and using COLAMD ordering: </b>
     *  \code
     *  VariableIndex varIndex(graph); // Build variable index
     *  Data data = otherFunctionUsingVariableIndex(graph, varIndex); // Other code that uses variable index
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(EliminateQR, varIndex, std::nullopt);
     *  \endcode
     *  */
    boost::shared_ptr<BayesNetType> eliminateSequential(
      OptionalOrderingType orderingType = {},
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Do sequential elimination of all variables to produce a Bayes net.
     *
     *  <b> Example - Full QR elimination in specified order:
     *  \code
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(myOrdering, EliminateQR);
     *  \endcode
     *
     *  <b> Example - Reusing an existing VariableIndex to improve performance: </b>
     *  \code
     *  VariableIndex varIndex(graph); // Build variable index
     *  Data data = otherFunctionUsingVariableIndex(graph, varIndex); // Other code that uses variable index
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(myOrdering, EliminateQR, varIndex, std::nullopt);
     *  \endcode
     *  */
    boost::shared_ptr<BayesNetType> eliminateSequential(
      const Ordering& ordering,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Do multifrontal elimination of all variables to produce a Bayes tree.  If an ordering is not
     *  provided, the ordering will be computed using either COLAMD or METIS, dependeing on
     *  the parameter orderingType (Ordering::COLAMD or Ordering::METIS)
     *
     *  <b> Example - Full Cholesky elimination in COLAMD order: </b>
     *  \code
     *  boost::shared_ptr<GaussianBayesTree> result = graph.eliminateMultifrontal(EliminateCholesky);
     *  \endcode
     *
     *  <b> Example - Reusing an existing VariableIndex to improve performance, and using COLAMD ordering: </b>
     *  \code
     *  VariableIndex varIndex(graph); // Build variable index
     *  Data data = otherFunctionUsingVariableIndex(graph, varIndex); // Other code that uses variable index
     *  boost::shared_ptr<GaussianBayesTree> result = graph.eliminateMultifrontal(EliminateQR, {}, varIndex);
     *  \endcode
     *  */
    boost::shared_ptr<BayesTreeType> eliminateMultifrontal(
      OptionalOrderingType orderingType = {},
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Do multifrontal elimination of all variables to produce a Bayes tree.  If an ordering is not
     *  provided, the ordering will be computed using either COLAMD or METIS, dependeing on
     *  the parameter orderingType (Ordering::COLAMD or Ordering::METIS)
     *
     *  <b> Example - Full QR elimination in specified order:
     *  \code
     *  boost::shared_ptr<GaussianBayesTree> result = graph.eliminateMultifrontal(EliminateQR, myOrdering);
     *  \endcode
     *  */
    boost::shared_ptr<BayesTreeType> eliminateMultifrontal(
      const Ordering& ordering,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Do sequential elimination of some variables, in \c ordering provided, to produce a Bayes net
     *  and a remaining factor graph.  This computes the factorization \f$ p(X) = p(A|B) p(B) \f$,
     *  where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the factor graph, and \f$
     *  B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialSequential(
      const Ordering& ordering,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Do sequential elimination of the given \c variables in an ordering computed by COLAMD to
     *  produce a Bayes net and a remaining factor graph.  This computes the factorization \f$ p(X)
     *  = p(A|B) p(B) \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the
     *  factor graph, and \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialSequential(
      const KeyVector& variables,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Do multifrontal elimination of some variables, in \c ordering provided, to produce a Bayes
     *  tree and a remaining factor graph.  This computes the factorization \f$ p(X) = p(A|B) p(B)
     *  \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the factor graph, and
     *  \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialMultifrontal(
      const Ordering& ordering,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Do multifrontal elimination of the given \c variables in an ordering computed by COLAMD to
     *  produce a Bayes tree and a remaining factor graph.  This computes the factorization \f$ p(X)
     *  = p(A|B) p(B) \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the
     *  factor graph, and \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialMultifrontal(
      const KeyVector& variables,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Compute the marginal of the requested variables and return the result as a Bayes net.  Uses
     *  COLAMD marginalization ordering by default
     *  @param variables Determines the variables whose marginal to compute, if provided as an
     *         Ordering they will be ordered in the returned BayesNet as specified, and if provided
     *         as a KeyVector they will be ordered using constrained COLAMD.
     *  @param function Optional dense elimination function, if not provided the default will be
     *         used.
     *  @param variableIndex Optional pre-computed VariableIndex for the factor graph, if not
     *         provided one will be computed. */
    boost::shared_ptr<BayesNetType> marginalMultifrontalBayesNet(
      boost::variant<const Ordering&, const KeyVector&> variables,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Compute the marginal of the requested variables and return the result as a Bayes net.
     *  @param variables Determines the variables whose marginal to compute, if provided as an
     *         Ordering they will be ordered in the returned BayesNet as specified, and if provided
     *         as a KeyVector they will be ordered using constrained COLAMD.
     *  @param marginalizedVariableOrdering Ordering for the variables being marginalized out,
     *         i.e. all variables not in \c variables.
     *  @param function Optional dense elimination function, if not provided the default will be
     *         used.
     *  @param variableIndex Optional pre-computed VariableIndex for the factor graph, if not
     *         provided one will be computed. */
    boost::shared_ptr<BayesNetType> marginalMultifrontalBayesNet(
      boost::variant<const Ordering&, const KeyVector&> variables,
      const Ordering& marginalizedVariableOrdering,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Compute the marginal of the requested variables and return the result as a Bayes tree.  Uses
     *  COLAMD marginalization order by default
     *  @param variables Determines the variables whose marginal to compute, if provided as an
     *         Ordering they will be ordered in the returned BayesNet as specified, and if provided
     *         as a KeyVector they will be ordered using constrained COLAMD.
     *  @param function Optional dense elimination function, if not provided the default will be
     *         used.
     *  @param variableIndex Optional pre-computed VariableIndex for the factor graph, if not
     *         provided one will be computed. */
    boost::shared_ptr<BayesTreeType> marginalMultifrontalBayesTree(
      boost::variant<const Ordering&, const KeyVector&> variables,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Compute the marginal of the requested variables and return the result as a Bayes tree.
     *  @param variables Determines the variables whose marginal to compute, if provided as an
     *         Ordering they will be ordered in the returned BayesNet as specified, and if provided
     *         as a KeyVector they will be ordered using constrained COLAMD.
     *  @param marginalizedVariableOrdering Ordering for the variables being marginalized out,
     *         i.e. all variables not in \c variables.
     *  @param function Optional dense elimination function, if not provided the default will be
     *         used.
     *  @param variableIndex Optional pre-computed VariableIndex for the factor graph, if not
     *         provided one will be computed. */
    boost::shared_ptr<BayesTreeType> marginalMultifrontalBayesTree(
      boost::variant<const Ordering&, const KeyVector&> variables,
      const Ordering& marginalizedVariableOrdering,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

    /** Compute the marginal factor graph of the requested variables. */
    boost::shared_ptr<FactorGraphType> marginal(
      const KeyVector& variables,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const;

  private:

    // Access the derived factor graph class
    const FactorGraphType& asDerived() const { return static_cast<const FactorGraphType&>(*this); }

    // Access the derived factor graph class
    FactorGraphType& asDerived() { return static_cast<FactorGraphType&>(*this); }

  public:
  #ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
    /** @deprecated ordering and orderingType shouldn't both be specified */
    boost::shared_ptr<BayesNetType> GTSAM_DEPRECATED eliminateSequential(
      const Ordering& ordering,
      const Eliminate& function,
      OptionalVariableIndex variableIndex,
      OptionalOrderingType orderingType) const {
        return eliminateSequential(ordering, function, variableIndex);
      }
    
    /** @deprecated orderingType specified first for consistency */
    boost::shared_ptr<BayesNetType> GTSAM_DEPRECATED eliminateSequential(
      const Eliminate& function,
      OptionalVariableIndex variableIndex = {},
      OptionalOrderingType orderingType = {}) const {
        return eliminateSequential(orderingType, function, variableIndex);
      }

    /** @deprecated ordering and orderingType shouldn't both be specified */
    boost::shared_ptr<BayesTreeType> GTSAM_DEPRECATED eliminateMultifrontal(
      const Ordering& ordering,
      const Eliminate& function,
      OptionalVariableIndex variableIndex,
      OptionalOrderingType orderingType) const {
        return eliminateMultifrontal(ordering, function, variableIndex);
      }

    /** @deprecated orderingType specified first for consistency */
    boost::shared_ptr<BayesTreeType> GTSAM_DEPRECATED eliminateMultifrontal(
      const Eliminate& function,
      OptionalVariableIndex variableIndex = {},
      OptionalOrderingType orderingType = {}) const {
        return eliminateMultifrontal(orderingType, function, variableIndex);
      }

    /** @deprecated */
    boost::shared_ptr<BayesNetType> GTSAM_DEPRECATED marginalMultifrontalBayesNet(
      boost::variant<const Ordering&, const KeyVector&> variables,
      std::nullptr_t,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const {
          return marginalMultifrontalBayesNet(variables, function, variableIndex);
      }

    /** @deprecated */
    boost::shared_ptr<BayesTreeType> GTSAM_DEPRECATED marginalMultifrontalBayesTree(
      boost::variant<const Ordering&, const KeyVector&> variables,
      std::nullptr_t,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate,
      OptionalVariableIndex variableIndex = {}) const {
          return marginalMultifrontalBayesTree(variables, function, variableIndex);
      }
  #endif
  };

}
