/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testSubgraphConditioner.cpp
 *  @brief  Unit tests for SubgraphPreconditioner
 *  @author Frank Dellaert
 **/

#include <tests/smallExample.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/linear/iterative.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <CppUnitLite/TestHarness.h>

#include <fstream>

using namespace std;
using namespace gtsam;
using namespace example;

// define keys
// Create key for simulated planar graph
Symbol key(int x, int y) { return symbol_shorthand::X(1000 * x + y); }

/* ************************************************************************* */
TEST(SubgraphPreconditioner, planarOrdering) {
  // Check canonical ordering
  Ordering expected, ordering = planarOrdering(3);
  expected +=
      key(3, 3), key(2, 3), key(1, 3),
      key(3, 2), key(2, 2), key(1, 2),
      key(3, 1), key(2, 1), key(1, 1);
  EXPECT(assert_equal(expected, ordering));
}

/* ************************************************************************* */
/** unnormalized error */
static double error(const GaussianFactorGraph& fg, const VectorValues& x) {
  double total_error = 0.;
  for (const GaussianFactor::shared_ptr& factor : fg)
    total_error += factor->error(x);
  return total_error;
}

/* ************************************************************************* */
TEST(SubgraphPreconditioner, planarGraph) {
  // Check planar graph construction
  GaussianFactorGraph A;
  VectorValues xtrue;
  std::tie(A, xtrue) = planarGraph(3);
  LONGS_EQUAL(13, A.size());
  LONGS_EQUAL(9, xtrue.size());
  DOUBLES_EQUAL(0, error(A, xtrue), 1e-9);  // check zero error for xtrue

  // Check that xtrue is optimal
  GaussianBayesNet R1 = *A.eliminateSequential();
  VectorValues actual = R1.optimize();
  EXPECT(assert_equal(xtrue, actual));
}

/* ************************************************************************* */
TEST(SubgraphPreconditioner, splitOffPlanarTree) {
  // Build a planar graph
  GaussianFactorGraph A;
  VectorValues xtrue;
  std::tie(A, xtrue) = planarGraph(3);

  // Get the spanning tree and constraints, and check their sizes
  GaussianFactorGraph T, C;
  std::tie(T, C) = splitOffPlanarTree(3, A);
  LONGS_EQUAL(9, T.size());
  LONGS_EQUAL(4, C.size());

  // Check that the tree can be solved to give the ground xtrue
  GaussianBayesNet R1 = *T.eliminateSequential();
  VectorValues xbar = R1.optimize();
  EXPECT(assert_equal(xtrue, xbar));
}

/* ************************************************************************* */
TEST(SubgraphPreconditioner, system) {
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  std::tie(Ab, xtrue) = planarGraph(N);  // A*x-b

  // Get the spanning tree and remaining graph
  GaussianFactorGraph Ab1, Ab2;  // A1*x-b1 and A2*x-b2
  std::tie(Ab1, Ab2) = splitOffPlanarTree(N, Ab);

  // Eliminate the spanning tree to build a prior
  const Ordering ord = planarOrdering(N);
  auto Rc1 = *Ab1.eliminateSequential(ord);  // R1*x-c1
  VectorValues xbar = Rc1.optimize();       // xbar = inv(R1)*c1

  // Create Subgraph-preconditioned system
  const SubgraphPreconditioner system(Ab2, Rc1, xbar);

  // Get corresponding matrices for tests. Add dummy factors to Ab2 to make
  // sure it works with the ordering.
  Ordering ordering = Rc1.ordering();  // not ord in general!
  Ab2.add(key(1, 1), Z_2x2, Z_2x1);
  Ab2.add(key(1, 2), Z_2x2, Z_2x1);
  Ab2.add(key(1, 3), Z_2x2, Z_2x1);
  Matrix A, A1, A2;
  Vector b, b1, b2;
  std::tie(A, b) = Ab.jacobian(ordering);
  std::tie(A1, b1) = Ab1.jacobian(ordering);
  std::tie(A2, b2) = Ab2.jacobian(ordering);
  Matrix R1 = Rc1.matrix(ordering).first;
  Matrix Abar(13 * 2, 9 * 2);
  Abar.topRows(9 * 2) = Matrix::Identity(9 * 2, 9 * 2);
  Abar.bottomRows(8) = A2.topRows(8) * R1.inverse();

  // Helper function to vectorize in correct order, which is the order in which
  // we eliminated the spanning tree.
  auto vec = [ordering](const VectorValues& x) { return x.vector(ordering); };

  // Set up y0 as all zeros
  const VectorValues y0 = system.zero();

  // y1 = perturbed y0
  VectorValues y1 = system.zero();
  y1[key(3, 3)] = Vector2(1.0, -1.0);

  // Check backSubstituteTranspose works with R1
  VectorValues actual = Rc1.backSubstituteTranspose(y1);
  Vector expected = R1.transpose().inverse() * vec(y1);
  EXPECT(assert_equal(expected, vec(actual)));

  // Check corresponding x values
  // for y = 0, we get xbar:
  EXPECT(assert_equal(xbar, system.x(y0)));
  // for non-zero y, answer is x = xbar + inv(R1)*y
  const Vector expected_x1 = vec(xbar) + R1.inverse() * vec(y1);
  const VectorValues x1 = system.x(y1);
  EXPECT(assert_equal(expected_x1, vec(x1)));

  // Check errors
  DOUBLES_EQUAL(0, error(Ab, xbar), 1e-9);
  DOUBLES_EQUAL(0, system.error(y0), 1e-9);
  DOUBLES_EQUAL(2, error(Ab, x1), 1e-9);
  DOUBLES_EQUAL(2, system.error(y1), 1e-9);

  // Check that transposeMultiplyAdd <=> y += alpha * Abar' * e
  // We check for e1 =[1;0] and e2=[0;1] corresponding to T and C
  const double alpha = 0.5;
  Errors e1, e2;
  for (size_t i = 0; i < 13; i++) {
    e1.push_back(i < 9 ? Vector2(1, 1) : Vector2(0, 0));
    e2.push_back(i >= 9 ? Vector2(1, 1) : Vector2(0, 0));
  }
  Vector ee1(13 * 2), ee2(13 * 2);
  ee1 << Vector::Ones(9 * 2), Vector::Zero(4 * 2);
  ee2 << Vector::Zero(9 * 2), Vector::Ones(4 * 2);

  // Check transposeMultiplyAdd for e1
  VectorValues y = system.zero();
  system.transposeMultiplyAdd(alpha, e1, y);
  Vector expected_y = alpha * Abar.transpose() * ee1;
  EXPECT(assert_equal(expected_y, vec(y)));

  // Check transposeMultiplyAdd for e2
  y = system.zero();
  system.transposeMultiplyAdd(alpha, e2, y);
  expected_y = alpha * Abar.transpose() * ee2;
  EXPECT(assert_equal(expected_y, vec(y)));

  // Test gradient in y
  auto g = system.gradient(y0);
  Vector expected_g = Vector::Zero(18);
  EXPECT(assert_equal(expected_g, vec(g)));
}

/* ************************************************************************* */
TEST(SubgraphPreconditioner, conjugateGradients) {
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  std::tie(Ab, xtrue) = planarGraph(N);  // A*x-b

  // Get the spanning tree
  GaussianFactorGraph Ab1, Ab2;  // A1*x-b1 and A2*x-b2
  std::tie(Ab1, Ab2) = splitOffPlanarTree(N, Ab);

  // Eliminate the spanning tree to build a prior
  GaussianBayesNet Rc1 = *Ab1.eliminateSequential();  // R1*x-c1
  VectorValues xbar = Rc1.optimize();  // xbar = inv(R1)*c1

  // Create Subgraph-preconditioned system
  SubgraphPreconditioner system(Ab2, Rc1, xbar);

  // Create zero config y0 and perturbed config y1
  VectorValues y0 = VectorValues::Zero(xbar);

  VectorValues y1 = y0;
  y1[key(2, 2)] = Vector2(1.0, -1.0);
  VectorValues x1 = system.x(y1);

  // Solve for the remaining constraints using PCG
  ConjugateGradientParameters parameters;
  VectorValues actual = conjugateGradients<SubgraphPreconditioner,
      VectorValues, Errors>(system, y1, parameters);
  EXPECT(assert_equal(y0,actual));

  // Compare with non preconditioned version:
  VectorValues actual2 = conjugateGradientDescent(Ab, x1, parameters);
  EXPECT(assert_equal(xtrue, actual2, 1e-4));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
