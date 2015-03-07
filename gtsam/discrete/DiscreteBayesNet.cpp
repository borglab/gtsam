/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBayesNet.cpp
 * @date Feb 15, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <boost/make_shared.hpp>

namespace gtsam {

  // Instantiate base class
  template class FactorGraph<DiscreteConditional>;

  /* ************************************************************************* */
  bool DiscreteBayesNet::equals(const This& bn, double tol) const
  {
    return Base::equals(bn, tol);
  }

  /* ************************************************************************* */
//  void DiscreteBayesNet::add_front(const Signature& s) {
//    push_front(boost::make_shared<DiscreteConditional>(s));
//  }

  /* ************************************************************************* */
  void DiscreteBayesNet::add(const Signature& s) {
    push_back(boost::make_shared<DiscreteConditional>(s));
  }

  /* ************************************************************************* */
  double DiscreteBayesNet::evaluate(const DiscreteConditional::Values & values) const {
    // evaluate all conditionals and multiply
    double result = 1.0;
    BOOST_FOREACH(DiscreteConditional::shared_ptr conditional, *this)
      result *= (*conditional)(values);
    return result;
  }

  /* ************************************************************************* */
  DiscreteFactor::sharedValues DiscreteBayesNet::optimize() const {
    // solve each node in turn in topological sort order (parents first)
    DiscreteFactor::sharedValues result(new DiscreteFactor::Values());
    BOOST_REVERSE_FOREACH (DiscreteConditional::shared_ptr conditional, *this)
      conditional->solveInPlace(*result);
    return result;
  }

  /* ************************************************************************* */
  DiscreteFactor::sharedValues DiscreteBayesNet::sample() const {
    // sample each node in turn in topological sort order (parents first)
    DiscreteFactor::sharedValues result(new DiscreteFactor::Values());
    BOOST_REVERSE_FOREACH(DiscreteConditional::shared_ptr conditional, *this)
      conditional->sampleInPlace(*result);
    return result;
  }

/* ************************************************************************* */
} // namespace
