/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesTree.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <boost/foreach.hpp>

#include <gtsam/symbolic/SymbolicBayesTreeUnordered.h>
#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/symbolic/SymbolicBayesNetUnordered.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>
#include <gtsam/inference/BayesTreeUnordered-inst.h>
#include <gtsam/inference/BayesTreeCliqueBaseUnordered-inst.h>

namespace gtsam {

  /* ************************************************************************* */
  SymbolicBayesTreeUnordered::SymbolicBayesTreeUnordered(const SymbolicBayesTreeUnordered& other) :
    Base(other) {}

  /* ************************************************************************* */
  SymbolicBayesTreeUnordered& SymbolicBayesTreeUnordered::operator=(const SymbolicBayesTreeUnordered& other)
  {
    (void) Base::operator=(other);
    return *this;
  }

  /* ************************************************************************* */\
  bool SymbolicBayesTreeUnordered::equals(const This& other, double tol /* = 1e-9 */) const
  {
    return Base::equals(other, tol);
  }

}
