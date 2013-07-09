/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicEliminationTreeUnordered.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/EliminationTreeUnordered-inst.h>
#include <gtsam/symbolic/SymbolicEliminationTreeUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  SymbolicEliminationTreeUnordered::SymbolicEliminationTreeUnordered(
    const SymbolicFactorGraphUnordered& factorGraph, const VariableIndexUnordered& structure,
    const OrderingUnordered& order) :
  Base(factorGraph, structure, order) {}

  /* ************************************************************************* */
  SymbolicEliminationTreeUnordered::SymbolicEliminationTreeUnordered(
    const SymbolicFactorGraphUnordered& factorGraph, const OrderingUnordered& order) :
  Base(factorGraph, order) {}

  /* ************************************************************************* */
  SymbolicEliminationTreeUnordered::SymbolicEliminationTreeUnordered(
    const This& other) :
  Base(other) {}

  /* ************************************************************************* */
  SymbolicEliminationTreeUnordered& SymbolicEliminationTreeUnordered::operator=(const This& other)
  {
    (void) Base::operator=(other);
    return *this;
  }

  /* ************************************************************************* */
  bool SymbolicEliminationTreeUnordered::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

}
