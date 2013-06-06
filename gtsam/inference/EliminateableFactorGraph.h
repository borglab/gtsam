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
 * @date    Oct 21, 2010
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <gtsam/inference/OrderingUnordered.h>

namespace gtsam {

  /** EliminateableFactorGraph is a base class for factor graphs that contains elimination
   *  algorithms.  Any factor graph holding eliminateable factors can derive from this class to
   *  expose functions for computing marginals, conditional marginals, doing full multifrontal and
   *  sequential elimination, etc. */
  template<class DERIVED, class FACTOR, class BAYESNET, class BAYESTREE>
  class EliminateableFactorGraph {
  public:
    typedef EliminateableFactorGraph<DERIVED, FACTOR, BAYESNET, BAYESTREE> This;
    typedef DERIVED FactorGraphType;
    typedef FACTOR FactorType;
    typedef BAYESNET BayesNetType;
    typedef BAYESTREE BayesTreeType;
    typedef typename BayesNetType::ConditionalType ConditionalType;
    typedef boost::shared_ptr<FactorType> sharedFactor;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;
    typedef boost::function<std::pair<sharedConditional,sharedFactor>(
      std::vector<sharedFactor>, std::vector<Key>)>
      Eliminate; ///< Typedef for a dense eliminate subroutine
    typedef boost::optional<const OrderingUnordered&> OptionalOrdering;

    /** Do sequential elimination of all variables to produce a Bayes net.  If an ordering is not
     *  provided, the ordering provided by COLAMD will be used. */
    boost::shared_ptr<BayesNetType>
      eliminateSequential(const Eliminate& function, OptionalOrdering ordering = boost::none) const;

    /** Do multifrontal elimination of all variables to produce a Bayes tree.  If an ordering is not
     *  provided, the ordering provided by COLAMD will be used. */
    boost::shared_ptr<BayesNetType>
      eliminateMultifrontal(const Eliminate& function, OptionalOrdering ordering = boost::none) const;

    /** Do sequential elimination of some variables to produce a Bayes net and a remaining factor
     *  graph.  This computes the factorization \f$ p(X) = p(A|B) p(B) \f$, where \f$ A = \f$ \c
     *  variables, \f$ X \f$ is all the variables in the factor graph, and \f$ B = X\backslash A
     *  \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialSequential(const Eliminate& function, OptionalOrdering ordering = boost::none);

    /** Do multifrontal elimination of some variables to produce a Bayes tree and a remaining factor
     *  graph.  This computes the factorization \f$ p(X) = p(A|B) p(B) \f$, where \f$ A = \f$ \c
     *  variables, \f$ X \f$ is all the variables in the factor graph, and \f$ B = X\backslash A
     *  \f$. */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminatePartialMultifrontal(const Eliminate& function, OptionalOrdering ordering = boost::none);
  };

}
