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

#include <gtsam/nonlinear/SuccessiveLinearizationOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
const Ordering& SuccessiveLinearizationOptimizer::ordering(const Values& values) const {

  if(!ordering_) {
    SharedParams params =
        boost::dynamic_pointer_cast<const SuccessiveLinearizationParams>(this->params());

    // If we're using a COLAMD ordering, compute it
    if(params->ordering)
      ordering_ = params->ordering;
    else
      ordering_ = *graph_->orderingCOLAMD(values);
  }

  return *ordering_;
}

} /* namespace gtsam */
