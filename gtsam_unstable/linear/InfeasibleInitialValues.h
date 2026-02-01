/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file InfeasibleInitialValues.h
 * @brief Exception thrown when given Infeasible Initial Values.
 * @date jan 24, 2015
 * @author Duy-Nguyen Ta
 */

#pragma once

namespace gtsam {
/* ************************************************************************* */
/** An exception indicating that the provided initial value is infeasible
 * Also used to inzdicatethat the noise model dimension passed into a
 * JacobianFactor has a different dimensionality than the factor. */
class InfeasibleInitialValues: public ThreadsafeException<
    InfeasibleInitialValues> {
public:
  InfeasibleInitialValues() {
  }

  ~InfeasibleInitialValues() noexcept override {
  }

  const char *what() const noexcept override {
    if (description_->empty())
      description_ =
          "An infeasible initial value was provided for the solver.\n";
    return description_->c_str();
  }
};
}
