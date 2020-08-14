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
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

namespace gtsam {

  /* ************************************************************************* */
  const char* IndeterminantLinearSystemException::what() const noexcept
  {
    if(!description_) {
      description_ = String(
          "\nIndeterminant linear system detected while working near variable\n"
          + boost::lexical_cast<String>(j_) +
          + " (Symbol: " + boost::lexical_cast<String>(
              gtsam::DefaultKeyFormatter(gtsam::Symbol(j_))) + ").\n"
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
    if(description_.empty())
      description_ = (boost::format(
      "A JacobianFactor was attempted to be constructed or modified to use a\n"
      "noise model of incompatible dimension.  The JacobianFactor has\n"
      "dimensionality (i.e. length of error vector) %d but the provided noise\n"
      "model has dimensionality %d.") % factorDims % noiseModelDims).str();
    return description_.c_str();
  }

  /* ************************************************************************* */
  const char* InvalidMatrixBlock::what() const noexcept {
    if(description_.empty())
      description_ = (boost::format(
      "A JacobianFactor was attempted to be constructed with a matrix block of\n"
      "inconsistent dimension.  The JacobianFactor has %d rows (i.e. length of\n"
      "error vector) but the provided matrix block has %d rows.")
      % factorRows % blockRows).str();
    return description_.c_str();
  }

 }
