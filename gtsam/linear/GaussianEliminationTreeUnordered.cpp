/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianEliminationTreeUnordered.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/EliminationTreeUnordered-inst.h>
#include <gtsam/linear/GaussianEliminationTreeUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  GaussianEliminationTreeUnordered::GaussianEliminationTreeUnordered(
    const GaussianFactorGraphUnordered& factorGraph, const VariableIndexUnordered& structure,
    const OrderingUnordered& order) :
  Base(factorGraph, structure, order) {}

  /* ************************************************************************* */
  GaussianEliminationTreeUnordered::GaussianEliminationTreeUnordered(
    const GaussianFactorGraphUnordered& factorGraph, const OrderingUnordered& order) :
  Base(factorGraph, order) {}

  /* ************************************************************************* */
  GaussianEliminationTreeUnordered::GaussianEliminationTreeUnordered(
    const This& other) :
  Base(other) {}

  /* ************************************************************************* */
  GaussianEliminationTreeUnordered& GaussianEliminationTreeUnordered::operator=(const This& other)
  {
    (void) Base::operator=(other);
    return *this;
  }

  /* ************************************************************************* */
  bool GaussianEliminationTreeUnordered::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

}
