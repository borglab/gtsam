/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    inferenceExceptions.h
 * @brief   Exceptions that may be thrown by inference algorithms
 * @author  Richard Roberts
 * @date    Apr 25, 2013
 */
#pragma once

#include <gtsam/global_includes.h>
#include <boost/lexical_cast.hpp>
#include <exception>

namespace gtsam {

  /** An inference algorithm was called with inconsistent arguments.  The factor graph, ordering, or
   *  variable index were inconsistent with each other, or a full elimination routine was called
   *  with an ordering that does not include all of the variables. */
  class InconsistentEliminationRequested : public std::exception {
  public:
    InconsistentEliminationRequested() noexcept {}
    ~InconsistentEliminationRequested() noexcept override {}
    const char* what() const noexcept override {
      return
        "An inference algorithm was called with inconsistent arguments.  The\n"
        "factor graph, ordering, or variable index were inconsistent with each\n"
        "other, or a full elimination routine was called with an ordering that\n"
        "does not include all of the variables.";
    }
  };

}
