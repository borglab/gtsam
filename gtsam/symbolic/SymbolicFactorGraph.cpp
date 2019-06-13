/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicFactorGraph.cpp
 * @date Oct 29, 2009
 * @author Frank Dellaert
 */

#include <boost/make_shared.hpp>

#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/symbolic/SymbolicConditional.h>

namespace gtsam {

  // Instantiate base classes
  template class FactorGraph<SymbolicFactor>;
  template class EliminateableFactorGraph<SymbolicFactorGraph>;

  using namespace std;

  /* ************************************************************************* */
  bool SymbolicFactorGraph::equals(const This& fg, double tol) const
  {
    return Base::equals(fg, tol);
  }

  /* ************************************************************************* */
  void SymbolicFactorGraph::push_factor(Key key) {
    push_back(boost::make_shared<SymbolicFactor>(key));
  }

  /* ************************************************************************* */
  void SymbolicFactorGraph::push_factor(Key key1, Key key2) {
    push_back(boost::make_shared<SymbolicFactor>(key1,key2));
  }

  /* ************************************************************************* */
  void SymbolicFactorGraph::push_factor(Key key1, Key key2, Key key3) {
    push_back(boost::make_shared<SymbolicFactor>(key1,key2,key3));
  }

  /* ************************************************************************* */
  void SymbolicFactorGraph::push_factor(Key key1, Key key2, Key key3, Key key4) {
    push_back(boost::make_shared<SymbolicFactor>(key1,key2,key3,key4));
  }

  /* ************************************************************************* */
}
