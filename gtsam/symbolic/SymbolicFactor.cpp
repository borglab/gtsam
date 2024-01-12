/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicFactor.cpp
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#include <gtsam/base/FastVector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicFactor-inst.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  double SymbolicFactor::error(const HybridValues& c) const {
    throw std::runtime_error("SymbolicFactor::error is not implemented");
  }

  /* ************************************************************************* */
  std::pair<std::shared_ptr<SymbolicConditional>, std::shared_ptr<SymbolicFactor> >
    EliminateSymbolic(const SymbolicFactorGraph& factors, const Ordering& keys)
  {
    return internal::EliminateSymbolic(factors, keys);
  }

  /* ************************************************************************* */
  bool SymbolicFactor::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

  /* ************************************************************************* */
  std::pair<std::shared_ptr<SymbolicConditional>, std::shared_ptr<SymbolicFactor> >
    SymbolicFactor::eliminate(const Ordering& keys) const
  {
    SymbolicFactorGraph graph;
    graph.push_back(*this); // TODO: Is there a way to avoid copying this factor?
    return EliminateSymbolic(graph, keys);
  }

} // gtsam
