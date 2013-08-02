/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBayesNet.h
 * @date Feb 15, 2011
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/FactorGraph.h>

#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>

namespace gtsam {

  class GTSAM_EXPORT DiscreteBayesNet : public FactorGraph<DiscreteConditional>
  {
  public:
    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    DiscreteBayesNet() {}

    /** Construct from iterator over conditionals */
    template<typename ITERATOR>
    DiscreteBayesNet(ITERATOR firstConditional, ITERATOR lastConditional) : Base(firstConditional, lastConditional) {}

    /** Construct from container of factors (shared_ptr or plain objects) */
    template<class CONTAINER>
    explicit DiscreteBayesNet(const CONTAINER& conditionals) : Base(conditionals) {}

    /** Implicit copy/downcast constructor to override explicit template container constructor */
    template<class DERIVEDCONDITIONAL>
    DiscreteBayesNet(const FactorGraph<DERIVEDCONDITIONAL>& graph) : Base(graph) {}

    /// @}

    /** Add a DiscreteCondtional */
    void add(const Signature& s);

    //** evaluate for given Values */
    double evaluate(const DiscreteConditional::Values& values);

    /** Optimize function for back-substitution. */
    DiscreteFactor::sharedValues optimize();

    /** Do ancestral sampling */
    DiscreteFactor::sharedValues sample();
  };

} // namespace

