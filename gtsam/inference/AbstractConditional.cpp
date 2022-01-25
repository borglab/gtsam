/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AbstractConditional.cpp
 * @brief   Concrete base class for conditional densities
 * @author  Fan Jiang
 */


#include <gtsam/inference/AbstractConditional.h>

namespace gtsam {
/* ************************************************************************* */
bool AbstractConditional::equals(const AbstractConditional &c,
                                 double tol) const {
  return nrFrontals_ == c.nrFrontals_;
}

}
