/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CustomFactor.cpp
 * @brief   Class to enable arbitrary factors with runtime swappable error function.
 * @author  Fan Jiang
 */

#include <gtsam/nonlinear/CustomFactor.h>

namespace gtsam {

Vector CustomFactor::unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H) const {
  if(this->active(x)) {
    if(H) {
      return this->errorFunction(*this, x, H.get_ptr());
    } else {
      JacobianVector dummy;
      return this->errorFunction(*this, x, &dummy);
    }
  } else {
    return Vector::Zero(this->dim());
  }
}

}
