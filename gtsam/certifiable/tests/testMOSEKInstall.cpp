/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMOSEKInstall.cpp
 * @brief This tests if MOSEK is installed and running correctly. 
 */

#include <fusion.h>

#include <cmath>
#include <iostream>

using namespace mosek::fusion;
using namespace monty;

namespace {

struct SimpleSdpSolution {
  double x00 = 0.0;
  double x10 = 0.0;
  double x01 = 0.0;
  double x11 = 0.0;
};

/**
 * Solve minimize trace(X) subject to X in S_+^2 and X(0,0)=1. The optimum is
 * X = [1 0; 0 0], which is enough to verify MOSEK Fusion links and runs.
 */
SimpleSdpSolution SolveSimpleSdp() {
  Model::t model = new Model("SimpleSDP");
  auto cleanup = finally([&]() { model->dispose(); });

  auto X = model->variable("X", Domain::inPSDCone(2));
  model->constraint(X->index(0, 0), Domain::equalsTo(1.0));
  model->objective(ObjectiveSense::Minimize,
                   Expr::add(X->index(0, 0), X->index(1, 1)));
  model->solve();

  auto level = X->level();
  return {(*level)[0], (*level)[1], (*level)[2], (*level)[3]};
}

}  // namespace

int main() {
  const SimpleSdpSolution solution = SolveSimpleSdp();
  const double trace = solution.x00 + solution.x11;

  std::cout << "Simple MOSEK SDP solution" << std::endl;
  std::cout << "X = [[" << solution.x00 << ", " << solution.x01 << "], ["
            << solution.x10 << ", " << solution.x11 << "]]" << std::endl;
  std::cout << "trace(X) = " << trace << std::endl;

  constexpr double tol = 1e-6;
  if (std::fabs(solution.x00 - 1.0) > tol ||
      std::fabs(solution.x10) > tol || std::fabs(solution.x01) > tol ||
      std::fabs(solution.x11) > tol || std::fabs(trace - 1.0) > tol) {
    return 1;
  }
  return 0;
}
