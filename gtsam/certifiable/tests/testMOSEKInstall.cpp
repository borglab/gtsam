/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMOSEKInstall.cpp
 * @brief Test that MOSEK Fusion is installed and can solve an SDP.
 */

#include <CppUnitLite/TestHarness.h>
#include <fusion.h>

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

/* ************************************************************************* */
TEST(MOSEKInstall, SimpleSDP) {
  const SimpleSdpSolution solution = SolveSimpleSdp();
  const double trace = solution.x00 + solution.x11;

  constexpr double tol = 1e-6;
  EXPECT_DOUBLES_EQUAL(1.0, solution.x00, tol);
  EXPECT_DOUBLES_EQUAL(0.0, solution.x10, tol);
  EXPECT_DOUBLES_EQUAL(0.0, solution.x01, tol);
  EXPECT_DOUBLES_EQUAL(0.0, solution.x11, tol);
  EXPECT_DOUBLES_EQUAL(1.0, trace, tol);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
