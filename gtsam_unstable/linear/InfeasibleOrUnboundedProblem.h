/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     InfeasibleOrUnboundedProblem.h
 * @brief    Throw when the problem is either infeasible or unbounded
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#pragma once

namespace gtsam {

class InfeasibleOrUnboundedProblem: public ThreadsafeException<
    InfeasibleOrUnboundedProblem> {
public:
  InfeasibleOrUnboundedProblem() {
  }
  ~InfeasibleOrUnboundedProblem() noexcept override {
  }

  const char* what() const noexcept override {
    if (description_->empty())
      description_ = "The problem is either infeasible or unbounded.\n";
    return description_->c_str();
  }
};
}
