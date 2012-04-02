/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DirectOptimizer.cpp
 * @brief 
 * @author Richard Roberts
 * @date Apr 1, 2012
 */

#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
NonlinearOptimizer::shared_ptr DirectOptimizer::update(const SharedGraph& newGraph) const {

  // Call update on the base class
  shared_ptr result = boost::static_pointer_cast<DirectOptimizer>();

  // Need to recompute the ordering if we're using an automatic COLAMD ordering
  if(result->colamdOrdering_) {
    result->ordering_ = result->graph_->orderingCOLAMD(*result->values_);
  }

  return result;
}

/* ************************************************************************* */
DirectOptimizer::shared_ptr DirectOptimizer::update(const SharedOrdering& newOrdering) const {
  return update(graph_, newOrdering);
}

/* ************************************************************************* */
DirectOptimizer::shared_ptr DirectOptimizer::update(const SharedGraph& newGraph, const SharedOrdering& newOrdering) const {
  // Call update on base class
  shared_ptr result = boost::static_pointer_cast<DirectOptimizer>(NonlinearOptimizer::update(newGraph));

  if(newOrdering && newOrdering->size() > 0) {
    result->colamdOrdering_ = false;
    result->ordering_ = newOrdering;
  } else {
    result->colamdOrdering_ = true;
    result->ordering_ = result->graph_->orderingCOLAMD(*result->values_);
  }

  return result;
}

} /* namespace gtsam */
