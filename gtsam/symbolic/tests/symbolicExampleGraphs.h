/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file symbolicExampleGraphs.cpp
 * @date sept 15, 2012
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/nonlinear/Symbol.h>
#include <boost/assign/list_of.hpp>

namespace gtsam {
  namespace {

    const SymbolicFactorGraphUnordered simpleTestGraph1 = boost::assign::list_of
      (boost::make_shared<SymbolicFactorUnordered>(0,1))
      (boost::make_shared<SymbolicFactorUnordered>(0,2))
      (boost::make_shared<SymbolicFactorUnordered>(1,4))
      (boost::make_shared<SymbolicFactorUnordered>(2,4))
      (boost::make_shared<SymbolicFactorUnordered>(3,4));

    const SymbolicBayesNetUnordered simpleTestGraph1BayesNet = boost::assign::list_of
      (boost::make_shared<SymbolicConditionalUnordered>(0,1,2))
      (boost::make_shared<SymbolicConditionalUnordered>(1,2,4))
      (boost::make_shared<SymbolicConditionalUnordered>(2,4))
      (boost::make_shared<SymbolicConditionalUnordered>(3,4))
      (boost::make_shared<SymbolicConditionalUnordered>(4));

    const SymbolicFactorGraphUnordered simpleTestGraph2 = boost::assign::list_of
      (boost::make_shared<SymbolicFactorUnordered>(0,1))
      (boost::make_shared<SymbolicFactorUnordered>(0,2))
      (boost::make_shared<SymbolicFactorUnordered>(1,3))
      (boost::make_shared<SymbolicFactorUnordered>(1,4))
      (boost::make_shared<SymbolicFactorUnordered>(2,3))
      (boost::make_shared<SymbolicFactorUnordered>(4,5));

    /** 0 - 1 - 2 - 3 */
    const SymbolicFactorGraphUnordered simpleChain = boost::assign::list_of
      (boost::make_shared<SymbolicFactorUnordered>(0,1))
      (boost::make_shared<SymbolicFactorUnordered>(1,2))
      (boost::make_shared<SymbolicFactorUnordered>(2,3));

    SymbolicBayesTreeUnordered __simpleChainBayesTree() {
      SymbolicBayesTreeUnordered result;
      SymbolicBayesTreeCliqueUnordered::shared_ptr root =
        boost::make_shared<SymbolicBayesTreeCliqueUnordered>(
        boost::make_shared<SymbolicConditionalUnordered>(SymbolicConditionalUnordered::FromKeys(list_of(2)(3),2)));
    }

    const SymbolicBayesTreeUnordered simpleChainBayesTree = __simpleChainBayesTree();

    /* ************************************************************************* */
    // Keys for ASIA example from the tutorial with A and D evidence
    const Key _X_=gtsam::symbol_shorthand::X(0), _T_=gtsam::symbol_shorthand::T(0),
      _S_=gtsam::symbol_shorthand::S(0), _E_=gtsam::symbol_shorthand::E(0),
      _L_=gtsam::symbol_shorthand::L(0), _B_=gtsam::symbol_shorthand::B(0);

    // Factor graph for Asia example
    const SymbolicFactorGraphUnordered asiaGraph = boost::assign::list_of
      (boost::make_shared<SymbolicFactorUnordered>(_T_))
      (boost::make_shared<SymbolicFactorUnordered>(_S_))
      (boost::make_shared<SymbolicFactorUnordered>(_T_, _E_, _L_))
      (boost::make_shared<SymbolicFactorUnordered>(_L_, _S_))
      (boost::make_shared<SymbolicFactorUnordered>(_S_, _B_))
      (boost::make_shared<SymbolicFactorUnordered>(_E_, _B_))
      (boost::make_shared<SymbolicFactorUnordered>(_E_, _X_));

    const SymbolicBayesNetUnordered asiaBayesNet = boost::assign::list_of
      (boost::make_shared<SymbolicConditionalUnordered>(_T_, _E_, _L_))
      (boost::make_shared<SymbolicConditionalUnordered>(_X_, _E_))
      (boost::make_shared<SymbolicConditionalUnordered>(_E_, _B_, _L_))
      (boost::make_shared<SymbolicConditionalUnordered>(_S_, _B_, _L_))
      (boost::make_shared<SymbolicConditionalUnordered>(_L_, _B_))
      (boost::make_shared<SymbolicConditionalUnordered>(_B_));

    /* ************************************************************************* */
    const OrderingUnordered asiaOrdering = boost::assign::list_of(_X_)(_T_)(_S_)(_E_)(_L_)(_B_);

  }
}
