/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMosekSDP.cpp
 * @brief Minimal MOSEK Fusion SDP smoke test.
 */

#include <CppUnitLite/TestHarness.h>
#include <fusion.h>

#include <iostream>

using namespace mosek::fusion;
using namespace monty;

/* ************************************************************************* */
// Solves minimize trace(X) subject to X in S_+^2 and X(0,0)=1. This tests only
// optional MOSEK wiring; it is intentionally independent of lifted QCQP SDP code.
TEST(MosekSDP, SimplePSD) {
  Model::t model = new Model("SimpleSDP");
  auto cleanup = finally([&]() { model->dispose(); });

  auto X = model->variable("X", Domain::inPSDCone(2));
  model->constraint(X->index(0, 0), Domain::equalsTo(1.0));
  model->objective(ObjectiveSense::Minimize,
                   Expr::add(X->index(0, 0), X->index(1, 1)));
  model->solve();

  std::cout << "testMosekSDP problem status: " << model->getProblemStatus()
            << std::endl;
  std::cout << "testMosekSDP primal solution status: "
            << model->getPrimalSolutionStatus() << std::endl;
  std::cout << "testMosekSDP dual solution status: "
            << model->getDualSolutionStatus() << std::endl;

  auto level = X->level();
  const double X00 = (*level)[0];
  const double X10 = (*level)[1];
  const double X01 = (*level)[2];
  const double X11 = (*level)[3];

  EXPECT_DOUBLES_EQUAL(1.0, X00, 1e-6);
  EXPECT_DOUBLES_EQUAL(0.0, X10, 1e-6);
  EXPECT_DOUBLES_EQUAL(0.0, X01, 1e-6);
  EXPECT_DOUBLES_EQUAL(0.0, X11, 1e-6);
  EXPECT_DOUBLES_EQUAL(1.0, X00 + X11, 1e-6);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
