/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    linearExceptions.cpp
 * @brief   Exceptions that may be thrown by linear solver components
 * @author  Richard Roberts
 * @date    Aug 17, 2012
 */

#include <gtsam/linear/linearExceptions.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {

  /* ************************************************************************* */
  const char* IndeterminateSystemException::what() const noexcept
  {
    if(!description_) {
      description_ = String(
          "\nIndeterminate linear system detected while working near variable\n"
          + std::to_string(j_) +
          + " (Symbol: " + gtsam::DefaultKeyFormatter(gtsam::Symbol(j_)) + ").\n"
          "\n\
Thrown when a linear system is ill-posed.  The most common cause for this\n\
error is having underconstrained variables, but the exception is also raised\n\
for nearly indeterminate systems that are too poorly conditioned to solve\n\
reliably.  See the GTSAM Doxygen documentation for\n\
gtsam::IndeterminateSystemException and the user\n\
guide at\n\
https://github.com/borglab/gtsam/blob/develop/gtsam/linear/doc/IndeterminateSystemException.ipynb\n\
for more information.");
    }
    return description_->c_str();
  }

  /* ************************************************************************* */
  const char* InvalidNoiseModel::what() const noexcept {
    if(description_->empty())
      description_ = "A JacobianFactor was attempted to be constructed or modified to use a\n"
                     "noise model of incompatible dimension.  The JacobianFactor has\n"
                     "dimensionality (i.e. length of error vector) " + std::to_string(factorDims) +
                     " but the provided noise model has dimensionality " + std::to_string(noiseModelDims) + ".";
    return description_->c_str();
  }

  /* ************************************************************************* */
  const char* InvalidMatrixBlock::what() const noexcept {
    if(description_->empty()) {
      description_ = "A JacobianFactor was attempted to be constructed with a matrix block of\n"
                     "inconsistent dimension.  The JacobianFactor has " + std::to_string(factorRows) +
                     " rows (i.e. length of error vector) but the provided matrix block has " +
                     std::to_string(blockRows) + " rows.";
    }
    return description_->c_str();
  }

 }
