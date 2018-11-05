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
      no_constant_factor.push_back(hf);
    }
    return no_constant_factor;
  }
};

using QPSolver = ActiveSetSolver<QP, QPPolicy, QPInitSolver>;

}