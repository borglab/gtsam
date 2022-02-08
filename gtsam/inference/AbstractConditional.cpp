/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AbstractConditional.cpp
 * @brief   Abstract base class for conditional densities
 * @author  Fan Jiang
 */

#include <gtsam/inference/AbstractConditional.h>

namespace gtsam {
/* ************************************************************************* */
void AbstractConditional::print(const std::string &s,
                                const KeyFormatter &formatter) const {
  throw std::runtime_error("AbstractConditional::print not implemented!");
}

/* ************************************************************************* */
bool AbstractConditional::equals(const AbstractConditional &c,
                                 double tol) const {
  throw std::invalid_argument(
      "You are calling the base AbstractConditional's"
      " equality, which is illegal.");
  return nrFrontals_ == c.nrFrontals_;
}

}  // namespace gtsam
