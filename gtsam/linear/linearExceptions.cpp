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
  const char* IndeterminantLinearSystemException::what() const noexcept
  {
    if(!description_) {
      description_ = String(
          "\nIndeterminant linear system detected while working near variable\n"
          + std::to_string(j_) +
          + " (Symbol: " + gtsam::DefaultKeyFormatter(gtsam::Symbol(j_)) + ").\n"
          "\n\
Thrown when a linear system is ill-posed.  The most common cause for this\n\
error is having underconstrained variables.  Mathematically, the system is\n\
underdetermined.  See the GTSAM Doxygen documentation at\n\
http://borg.cc.gatech.edu/ on gtsam::IndeterminantLinearSystemException for\n\
more information.");
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
