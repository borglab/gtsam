/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMosekSDP.cpp
 * @brief Test that MOSEK Fusion is installed and can solve an SDP.
 */

#include <CppUnitLite/TestHarness.h>
#include <fusion.h>

using namespace mosek::fusion;
using namespace monty;

/* ************************************************************************* */
namespace mosek_sdp_tests {

struct SimpleSdpSolution {
  double x00 = 0.0;
  double x10 = 0.0;
  double x01 = 0.0;
  double x11 = 0.0;
};

// Solve a two-dimensional SDP whose unique optimum is diag(1, 0).
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

// Verifies that MOSEK Fusion links and solves a positive semidefinite program.
TEST(MosekSDP, SimplePSD) {
  const SimpleSdpSolution solution = SolveSimpleSdp();
  const double trace = solution.x00 + solution.x11;

  constexpr double tol = 1e-6;
  EXPECT_DOUBLES_EQUAL(1.0, solution.x00, tol);
  EXPECT_DOUBLES_EQUAL(0.0, solution.x10, tol);
  EXPECT_DOUBLES_EQUAL(0.0, solution.x01, tol);
  EXPECT_DOUBLES_EQUAL(0.0, solution.x11, tol);
  EXPECT_DOUBLES_EQUAL(1.0, trace, tol);
}

}  // namespace mosek_sdp_tests
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
