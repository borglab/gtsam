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

// Indexed views preserve row-major rectangular layout and repeated coordinates.
TEST(MosekSDP, IndexedPSDViews) {
  Model::t model = new Model("IndexedPSDViews");
  auto cleanup = finally([&]() { model->dispose(); });
  auto Z = model->variable("Z", Domain::inPSDCone(3));
  // Positive definite, with distinct entries in its lower triangle.
  auto expected = new_array_ptr<double, 1>({10, 2, 3, 2, 20, 4, 3, 4, 30});
  model->constraint(Z, Domain::equalsTo(Matrix::dense(3, 3, expected)));

  // Rows [0, 2, 0], columns [0, 1]: the first homogeneous row repeats.
  auto coordinates = new_array_ptr<int, 2>(shape(6, 2));
  const int rows[] = {0, 2, 0};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 2; ++c) {
      (*coordinates)(2 * r + c, 0) = rows[r];
      (*coordinates)(2 * r + c, 1) = c;
    }
  }
  auto view = Z->pick(coordinates)->reshape(3, 2);
  // Exercise the view as an expression as well as a solved variable.
  model->objective(ObjectiveSense::Minimize, Expr::sum(view->asExpr()));
  model->solve();

  const double expectedView[] = {10, 2, 3, 4, 10, 2};
  const auto level = view->level();
  EXPECT_LONGS_EQUAL(6, level->size());
  for (int index = 0; index < 6; ++index) {
    EXPECT_DOUBLES_EQUAL(expectedView[index], (*level)[index], 1e-7);
  }
  EXPECT_DOUBLES_EQUAL(31.0, model->primalObjValue(), 1e-7);
  EXPECT_DOUBLES_EQUAL(4.0, (*view->index(1, 1)->level())[0], 1e-7);
}

}  // namespace mosek_sdp_tests
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
