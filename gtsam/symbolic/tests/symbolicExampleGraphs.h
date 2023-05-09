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

#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <boost/assign/list_of.hpp>

namespace gtsam {
  namespace {

    const SymbolicFactorGraph simpleTestGraph1 = boost::assign::list_of
      (boost::make_shared<SymbolicFactor>(0,1))
      (boost::make_shared<SymbolicFactor>(0,2))
      (boost::make_shared<SymbolicFactor>(1,4))
      (boost::make_shared<SymbolicFactor>(2,4))
      (boost::make_shared<SymbolicFactor>(3,4));

    const SymbolicBayesNet simpleTestGraph1BayesNet = boost::assign::list_of
      (boost::make_shared<SymbolicConditional>(0,1,2))
      (boost::make_shared<SymbolicConditional>(1,2,4))
      (boost::make_shared<SymbolicConditional>(2,4))
      (boost::make_shared<SymbolicConditional>(3,4))
      (boost::make_shared<SymbolicConditional>(4));

    const SymbolicFactorGraph simpleTestGraph2 = boost::assign::list_of
      (boost::make_shared<SymbolicFactor>(0,1))
      (boost::make_shared<SymbolicFactor>(0,2))
      (boost::make_shared<SymbolicFactor>(1,3))
      (boost::make_shared<SymbolicFactor>(1,4))
      (boost::make_shared<SymbolicFactor>(2,3))
      (boost::make_shared<SymbolicFactor>(4,5));

    /** 1 - 0 - 2 - 3 */
    const SymbolicFactorGraph simpleChain = boost::assign::list_of
      (boost::make_shared<SymbolicFactor>(1,0))
      (boost::make_shared<SymbolicFactor>(0,2))
      (boost::make_shared<SymbolicFactor>(2,3));

    /* ************************************************************************* *
     * 2 3
     *   0 1 : 2
     ****************************************************************************/
    SymbolicBayesTree __simpleChainBayesTree() {
      SymbolicBayesTree result;
      result.insertRoot(boost::make_shared<SymbolicBayesTreeClique>(
        boost::make_shared<SymbolicConditional>(
        SymbolicConditional::FromKeys(boost::assign::list_of(2)(3), 2))));
      result.addClique(boost::make_shared<SymbolicBayesTreeClique>(
        boost::make_shared<SymbolicConditional>(
        SymbolicConditional::FromKeys(boost::assign::list_of(0)(1)(2), 2))),
        result.roots().front());
      return result;
    }

    const SymbolicBayesTree simpleChainBayesTree = __simpleChainBayesTree();

    /* ************************************************************************* */
    // Keys for ASIA example from the tutorial with A and D evidence
    const Key _X_=gtsam::symbol_shorthand::X(0), _T_=gtsam::symbol_shorthand::T(0),
      _S_=gtsam::symbol_shorthand::S(0), _E_=gtsam::symbol_shorthand::E(0),
      _L_=gtsam::symbol_shorthand::L(0), _B_=gtsam::symbol_shorthand::B(0);

    // Factor graph for Asia example
    const SymbolicFactorGraph asiaGraph = boost::assign::list_of
      (boost::make_shared<SymbolicFactor>(_T_))
      (boost::make_shared<SymbolicFactor>(_S_))
      (boost::make_shared<SymbolicFactor>(_T_, _E_, _L_))
      (boost::make_shared<SymbolicFactor>(_L_, _S_))
      (boost::make_shared<SymbolicFactor>(_S_, _B_))
      (boost::make_shared<SymbolicFactor>(_E_, _B_))
      (boost::make_shared<SymbolicFactor>(_E_, _X_));

    const SymbolicBayesNet asiaBayesNet = boost::assign::list_of
      (boost::make_shared<SymbolicConditional>(_T_, _E_, _L_))
      (boost::make_shared<SymbolicConditional>(_X_, _E_))
      (boost::make_shared<SymbolicConditional>(_E_, _B_, _L_))
      (boost::make_shared<SymbolicConditional>(_S_, _B_, _L_))
      (boost::make_shared<SymbolicConditional>(_L_, _B_))
      (boost::make_shared<SymbolicConditional>(_B_));

    SymbolicBayesTree __asiaBayesTree() {
      SymbolicBayesTree result;
      result.insertRoot(boost::make_shared<SymbolicBayesTreeClique>(
        boost::make_shared<SymbolicConditional>(
        SymbolicConditional::FromKeys(boost::assign::list_of(_E_)(_L_)(_B_), 3))));
      result.addClique(boost::make_shared<SymbolicBayesTreeClique>(
        boost::make_shared<SymbolicConditional>(
        SymbolicConditional::FromKeys(boost::assign::list_of(_S_)(_B_) (_L_), 1))),
        result.roots().front());
      result.addClique(boost::make_shared<SymbolicBayesTreeClique>(
        boost::make_shared<SymbolicConditional>(
        SymbolicConditional::FromKeys(boost::assign::list_of(_T_)(_E_)(_L_), 1))),
        result.roots().front());
      result.addClique(boost::make_shared<SymbolicBayesTreeClique>(
        boost::make_shared<SymbolicConditional>(
        SymbolicConditional::FromKeys(boost::assign::list_of(_X_)(_E_), 1))),
        result.roots().front());
      return result;
    }

    const SymbolicBayesTree asiaBayesTree = __asiaBayesTree();

    /* ************************************************************************* */
    const Ordering asiaOrdering = boost::assign::list_of(_X_)(_T_)(_S_)(_E_)(_L_)(_B_);

  }
}
