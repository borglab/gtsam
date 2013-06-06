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

  /** EliminateableFactorGraph is a base class for factor graphs that contains elimination
   *  algorithms.  Any factor graph holding eliminateable factors can derive from this class to
   *  expose functions for computing marginals, conditional marginals, doing multifrontal and
   *  sequential elimination, etc. */
  template<class DERIVED, class ELIMINATIONTREE, class JUNCTIONTREE>
  class EliminateableFactorGraph {
  public:
    typedef EliminateableFactorGraph<DERIVED, ELIMINATIONTREE, JUNCTIONTREE> This;
    typedef DERIVED FactorGraphType;
    typedef ELIMINATIONTREE EliminationTreeType;
    typedef JUNCTIONTREE JunctionTreeType;
    typedef typename EliminationTreeType::FactorType FactorType;
    typedef typename EliminationTreeType::BayesNetType BayesNetType;
    typedef typename JunctionTreeType::BayesTreeType BayesTreeType;
    typedef typename BayesNetType::ConditionalType ConditionalType;
    typedef boost::shared_ptr<FactorType> sharedFactor;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;
    typedef boost::function<std::pair<sharedConditional,sharedFactor>(
      std::vector<sharedFactor>, std::vector<Key>)>
      Eliminate; ///< Typedef for a dense eliminate subroutine
    typedef boost::optional<const OrderingUnordered&> OptionalOrdering;

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
    boost::shared_ptr<BayesNetType>
      eliminateSequential(const Eliminate& function, OptionalOrdering ordering = boost::none,
      const VariableIndexUnordered& variableIndex = VariableIndexUnordered(*this)) const;

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
    boost::shared_ptr<BayesTreeType>
      eliminateMultifrontal(const Eliminate& function, OptionalOrdering ordering = boost::none,
      const VariableIndexUnordered& variableIndex = VariableIndexUnordered(*this)) const;

    /** Do sequential elimination of some variables in the given \c ordering to produce a Bayes net
     *  and a remaining factor graph.  This computes the factorization \f$ p(X) = p(A|B) p(B) \f$,
     *  where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the factor graph, and \f$
     *  B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialSequential(const Eliminate& function, const Ordering& ordering,
      const VariableIndexUnordered& variableIndex = VariableIndexUnordered(*this));

    /** Do sequential elimination of the given \c variables in an ordering computed by COLAMD to
     *  produce a Bayes net and a remaining factor graph.  This computes the factorization \f$ p(X)
     *  = p(A|B) p(B) \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the
     *  factor graph, and \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialSequential(const Eliminate& function, const std::vector& variables,
      const VariableIndexUnordered& variableIndex = VariableIndexUnordered(*this));

    /** Do multifrontal elimination of the given \c variables in an ordering computed by COLAMD to
     *  produce a Bayes net and a remaining factor graph.  This computes the factorization \f$ p(X)
     *  = p(A|B) p(B) \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the
     *  factor graph, and \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialMultifrontal(const Eliminate& function, const Ordering& ordering,
      const VariableIndexUnordered& variableIndex = VariableIndexUnordered(*this));
    
    /** Do multifrontal elimination of some variables in the given \c ordering to produce a Bayes
     *  tree and a remaining factor graph.  This computes the factorization \f$ p(X) = p(A|B) p(B)
     *  \f$, where \f$ A = \f$ \c variables, \f$ X \f$ is all the variables in the factor graph, and
     *  \f$ B = X\backslash A \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialMultifrontal(const Eliminate& function, const std::vector& ordering,
      const VariableIndexUnordered& variableIndex = VariableIndexUnordered(*this));

  };

}
