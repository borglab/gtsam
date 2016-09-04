/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPSolver.h
 * @brief    Policy of ActiveSetSolver to solve Quadratic Programming Problems
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     6/16/16
 */

#include <gtsam_unstable/linear/QP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/QPInitSolver.h>
#include <limits>
#include <algorithm>

namespace gtsam {

/// Policy for ActivetSetSolver to solve Linear Programming \sa QP problems
struct QPPolicy {
  /// Maximum alpha for line search x'=xk + alpha*p, where p is the cost gradient
  /// For QP, maxAlpha = 1 is the minimum point of the quadratic cost
  static constexpr double maxAlpha = 1.0;

  /// Simply the cost of the QP problem
  static const GaussianFactorGraph buildCostFunction(const QP& qp,
      const VectorValues& xk = VectorValues()) {
    GaussianFactorGraph no_constant_factor;
    for (auto factor : qp.cost) {
      HessianFactor hf = static_cast<HessianFactor>(*factor);
      //a trick to ensure  that the augmented matrix is always symmetric. Should only be an issue when dealing
      // with the manifold.
      hf.augmentedInformation() = (hf.augmentedInformation() + hf.augmentedInformation().transpose())/2;
      if (hf.constantTerm() < 0) // Hessian Factors cannot deal
        // with negative constant terms replace with zero in this case
        //TODO: Perhaps there is a smarter way to set the constant term such that the resulting matrix is almost always
        // Positive definite.
        hf.constantTerm() = 0.0;
      no_constant_factor.push_back(hf);
    }
    return no_constant_factor;
  }
};

using QPSolver = ActiveSetSolver<QP, QPPolicy, QPInitSolver>;

}