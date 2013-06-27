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
#include <boost/function.hpp>

#include <gtsam/inference/OrderingUnordered.h>
#include <gtsam/inference/VariableIndexUnordered.h>

namespace gtsam {

  /// Traits class for eliminateable factor graphs, specifies the types that result from
  /// elimination, etc.  This must be defined for each factor graph that inherits from
  /// EliminateableFactorGraph.
  template<class GRAPH>
  class EliminationTraits
  {
    // Template for deriving:
    // typedef MyFactor FactorType;                   // Type of factors in factor graph (e.g. GaussianFactor, SymbolicFactor)
    // typedef MyBayesNet BayesNetType;               // Type of Bayes net from sequential elimination (e.g. GaussianBayesNet)
    // typedef MyEliminationTree EliminationTreeType; // Type of elimination tree (e.g. GaussianEliminationTree)
    // typedef MyBayesTree BayesTreeType;             // Type of Bayes tree (e.g. GaussianBayesTree)
    // typedef MyJunctionTree JunctionTreeType;       // Type of Junction tree (e.g. GaussianJunctionTree)
  };


  /** EliminateableFactorGraph is a base class for factor graphs that contains elimination
   *  algorithms.  Any factor graph holding eliminateable factors can derive from this class to
   *  expose functions for computing marginals, conditional marginals, doing multifrontal and
   *  sequential elimination, etc. */
  template<class FACTOR, class FACTORGRAPH, class CONDITIONAL,
  class BAYESNET, class ELIMINATIONTREE, class BAYESTREE, class JUNCTIONTREE>
  class EliminateableFactorGraph {
  public:
    typedef EliminateableFactorGraph<FACTOR, FACTORGRAPH, CONDITIONAL, BAYESNET, ELIMINATIONTREE, BAYESTREE, JUNCTIONTREE> This;
    typedef boost::optional<const OrderingUnordered&> OptionalOrdering;
    typedef boost::optional<const VariableIndexUnordered&> OptionalVariableIndex;
    typedef boost::function<std::pair<boost::shared_ptr<CONDITIONAL>, boost::shared_ptr<FACTOR> >(
      std::vector<boost::shared_ptr<FACTOR> >, std::vector<Key>)>
      Eliminate; ///< Typedef for an eliminate subroutine


    /** Do sequential elimination of all variables to produce a Bayes net.  If an ordering is not
     *  provided, the ordering provided by COLAMD will be used.
     *  
     *  <b> Example - Full Cholesky elimination in COLAMD order: </b>
     *  \code
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(EliminateCholesky);
     *  \endcode
     *  
     *  <b> Example - Full QR elimination in specified order:
     *  \code
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(EliminateQR, myOrdering);
     *  \endcode
     *  
     *  <b> Example - Reusing an existing VariableIndex to improve performance, and using COLAMD ordering: </b>
     *  \code
     *  VariableIndex varIndex(graph); // Build variable index
     *  Data data = otherFunctionUsingVariableIndex(graph, varIndex); // Other code that uses variable index
     *  boost::shared_ptr<GaussianBayesNet> result = graph.eliminateSequential(EliminateQR, boost::none, varIndex);
     *  \endcode
     *  */
    boost::shared_ptr<BAYESNET>
      eliminateSequential(const Eliminate& function, OptionalOrdering ordering = boost::none,
      OptionalVariableIndex variableIndex = boost::none) const;

    /** Do multifrontal elimination of all variables to produce a Bayes tree.  If an ordering is not
     *  provided, the ordering provided by COLAMD will be used.
     *  
     *  <b> Example - Full Cholesky elimination in COLAMD order: </b>
     *  \code
     *  boost::shared_ptr<GaussianBayesTree> result = graph.eliminateMultifrontal(EliminateCholesky);
     *  \endcode
     *  
     *  <b> Example - Full QR elimination in specified order:
     *  \code
     *  boost::shared_ptr<GaussianBayesTree> result = graph.eliminateMultifrontal(EliminateQR, myOrdering);
     *  \endcode
     *  
     *  <b> Example - Reusing an existing VariableIndex to improve performance, and using COLAMD ordering: </b>
     *  \code
     *  VariableIndex varIndex(graph); // Build variable index
     *  Data data = otherFunctionUsingVariableIndex(graph, varIndex); // Other code that uses variable index
     *  boost::shared_ptr<GaussianBayesTree> result = graph.eliminateMultifrontal(EliminateQR, boost::none, varIndex);
     *  \endcode
     *  */
    boost::shared_ptr<BAYESTREE>
      eliminateMultifrontal(const Eliminate& function, OptionalOrdering ordering = boost::none,
      OptionalVariableIndex variableIndex = boost::none) const;

    /** Do sequential elimination of some variables in the given \c ordering to produce a Bayes net
     *  and a remaining factor graph.  This computes the factorization \f$ p(X) = p(A|B) p(B) \f$,
     *  where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the factor graph, and \f$
     *  B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BAYESNET>, boost::shared_ptr<FACTORGRAPH> >
      eliminatePartialSequential(const Eliminate& function, const OrderingUnordered& ordering,
      OptionalVariableIndex variableIndex = boost::none) const;

    /** Do sequential elimination of the given \c variables in an ordering computed by COLAMD to
     *  produce a Bayes net and a remaining factor graph.  This computes the factorization \f$ p(X)
     *  = p(A|B) p(B) \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the
     *  factor graph, and \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BAYESNET>, boost::shared_ptr<FACTORGRAPH> >
      eliminatePartialSequential(const Eliminate& function, const std::vector<Key>& variables,
      OptionalVariableIndex variableIndex = boost::none) const;

    /** Do multifrontal elimination of the given \c variables in an ordering computed by COLAMD to
     *  produce a Bayes net and a remaining factor graph.  This computes the factorization \f$ p(X)
     *  = p(A|B) p(B) \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the
     *  factor graph, and \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<FACTORGRAPH> >
      eliminatePartialMultifrontal(const Eliminate& function, const OrderingUnordered& ordering,
      OptionalVariableIndex variableIndex = boost::none) const;
    
    /** Do multifrontal elimination of some variables in the given \c ordering to produce a Bayes
     *  tree and a remaining factor graph.  This computes the factorization \f$ p(X) = p(A|B) p(B)
     *  \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the factor graph, and
     *  \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BAYESTREE>, boost::shared_ptr<FACTORGRAPH> >
      eliminatePartialMultifrontal(const Eliminate& function, const std::vector<Key>& variables,
      OptionalVariableIndex variableIndex = boost::none) const;

  private:

    // Access the derived factor graph class
    const FACTORGRAPH& asDerived() const { return static_cast<const FACTORGRAPH&>(*this); }

    // Access the derived factor graph class
    FACTORGRAPH& asDerived() { return static_cast<FACTORGRAPH&>(*this); }
  };

}
