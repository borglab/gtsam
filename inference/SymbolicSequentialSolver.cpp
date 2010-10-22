/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicSequentialSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#include <gtsam/inference/SymbolicSequentialSolver.h>
#include <gtsam/inference/GenericSequentialSolver-inl.h>

namespace gtsam {

// An explicit instantiation to be compiled into the library
template class GenericSequentialSolver<IndexFactor>;

///* ************************************************************************* */
//SymbolicSequentialSolver::SymbolicSequentialSolver(const FactorGraph<IndexFactor>& factorGraph) :
//    Base(factorGraph) {}
//
///* ************************************************************************* */
//BayesNet<IndexConditional>::shared_ptr SymbolicSequentialSolver::eliminate() const {
//  return Base::eliminate();
//}
//
///* ************************************************************************* */
//SymbolicFactorGraph::shared_ptr SymbolicSequentialSolver::joint(const std::vector<Index>& js) const {
//  return Base::joint(js);
//}

}
