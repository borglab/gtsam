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

#include <boost/archive/xml_iarchive.hpp>
#include <boost/assign/std/list.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/serialization/export.hpp>
#include <boost/tuple/tuple.hpp>
using namespace boost::assign;

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
  boost::tie(A, xtrue) = planarGraph(3);
  LONGS_EQUAL(13, A.size());
  LONGS_EQUAL(9, xtrue.size());
  DOUBLES_EQUAL(0, error(A, xtrue), 1e-9);  // check zero error for xtrue

  // Check that xtrue is optimal
  GaussianBayesNet::shared_ptr R1 = A.eliminateSequential();
  VectorValues actual = R1->optimize();
  EXPECT(assert_equal(xtrue, actual));
}

/* ************************************************************************* */
TEST(SubgraphPreconditioner, splitOffPlanarTree) {
  // Build a planar graph
  GaussianFactorGraph A;
  VectorValues xtrue;
  boost::tie(A, xtrue) = planarGraph(3);

  // Get the spanning tree and constraints, and check their sizes
  GaussianFactorGraph::shared_ptr T, C;
  boost::tie(T, C) = splitOffPlanarTree(3, A);
  LONGS_EQUAL(9, T->size());
  LONGS_EQUAL(4, C->size());

  // Check that the tree can be solved to give the ground xtrue
  GaussianBayesNet::shared_ptr R1 = T->eliminateSequential();
  VectorValues xbar = R1->optimize();
  EXPECT(assert_equal(xtrue, xbar));
}

/* ************************************************************************* */
TEST(SubgraphPreconditioner, system) {
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = planarGraph(N);  // A*x-b

  // Get the spanning tree and remaining graph
  GaussianFactorGraph::shared_ptr Ab1, Ab2;  // A1*x-b1 and A2*x-b2
  boost::tie(Ab1, Ab2) = splitOffPlanarTree(N, Ab);

  // Eliminate the spanning tree to build a prior
  const Ordering ord = planarOrdering(N);
  auto Rc1 = Ab1->eliminateSequential(ord);  // R1*x-c1
  VectorValues xbar = Rc1->optimize();       // xbar = inv(R1)*c1

  // Create Subgraph-preconditioned system
  VectorValues::shared_ptr xbarShared(
      new VectorValues(xbar));  // TODO: horrible
  const SubgraphPreconditioner system(Ab2, Rc1, xbarShared);

  // Get corresponding matrices for tests. Add dummy factors to Ab2 to make
  // sure it works with the ordering.
  Ordering ordering = Rc1->ordering();  // not ord in general!
  Ab2->add(key(1, 1), Z_2x2, Z_2x1);
  Ab2->add(key(1, 2), Z_2x2, Z_2x1);
  Ab2->add(key(1, 3), Z_2x2, Z_2x1);
  Matrix A, A1, A2;
  Vector b, b1, b2;
  std::tie(A, b) = Ab.jacobian(ordering);
  std::tie(A1, b1) = Ab1->jacobian(ordering);
  std::tie(A2, b2) = Ab2->jacobian(ordering);
  Matrix R1 = Rc1->matrix(ordering).first;
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
  VectorValues actual = Rc1->backSubstituteTranspose(y1);
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
    e1 += i < 9 ? Vector2(1, 1) : Vector2(0, 0);
    e2 += i >= 9 ? Vector2(1, 1) : Vector2(0, 0);
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
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "JacobianFactor")

// Read from XML file
static GaussianFactorGraph read(const string& name) {
  auto inputFile = findExampleDataFile(name);
  ifstream is(inputFile);
  if (!is.is_open()) throw runtime_error("Cannot find file " + inputFile);
  boost::archive::xml_iarchive in_archive(is);
  GaussianFactorGraph Ab;
  in_archive >> boost::serialization::make_nvp("graph", Ab);
  return Ab;
}

TEST(SubgraphSolver, Solves) {
  // Create preconditioner
  SubgraphPreconditioner system;

  // We test on three different graphs
  const auto Ab1 = planarGraph(3).first;
  const auto Ab2 = read("toy3D");
  const auto Ab3 = read("randomGrid3D");

  // For all graphs, test solve and solveTranspose
  for (const auto& Ab : {Ab1, Ab2, Ab3}) {
    // Call build, a non-const method needed to make solve work :-(
    KeyInfo keyInfo(Ab);
    std::map<Key, Vector> lambda;
    system.build(Ab, keyInfo, lambda);

    // Create a perturbed (non-zero) RHS
    const auto xbar = system.Rc1()->optimize();  // merely for use in zero below
    auto values_y = VectorValues::Zero(xbar);
    auto it = values_y.begin();
    it->second.setConstant(100);
    ++it;
    it->second.setConstant(-100);

    // Solve the VectorValues way
    auto values_x = system.Rc1()->backSubstitute(values_y);

    // Solve the matrix way, this really just checks BN::backSubstitute
    // This only works with Rc1 ordering, not with keyInfo !
    // TODO(frank): why does this not work with an arbitrary ordering?
    const auto ord = system.Rc1()->ordering();
    const Matrix R1 = system.Rc1()->matrix(ord).first;
    auto ord_y = values_y.vector(ord);
    auto vector_x = R1.inverse() * ord_y;
    EXPECT(assert_equal(vector_x, values_x.vector(ord)));

    // Test that 'solve' does implement x = R^{-1} y
    // We do this by asserting it gives same answer as backSubstitute
    // Only works with keyInfo ordering:
    const auto ordering = keyInfo.ordering();
    auto vector_y = values_y.vector(ordering);
    const size_t N = R1.cols();
    Vector solve_x = Vector::Zero(N);
    system.solve(vector_y, solve_x);
    EXPECT(assert_equal(values_x.vector(ordering), solve_x));

    // Test that transposeSolve does implement x = R^{-T} y
    // We do this by asserting it gives same answer as backSubstituteTranspose
    auto values_x2 = system.Rc1()->backSubstituteTranspose(values_y);
    Vector solveT_x = Vector::Zero(N);
    system.transposeSolve(vector_y, solveT_x);
    EXPECT(assert_equal(values_x2.vector(ordering), solveT_x));
  }
}

/* ************************************************************************* */
TEST(SubgraphPreconditioner, conjugateGradients) {
  // Build a planar graph
  GaussianFactorGraph Ab;
  VectorValues xtrue;
  size_t N = 3;
  boost::tie(Ab, xtrue) = planarGraph(N);  // A*x-b

  // Get the spanning tree
  GaussianFactorGraph::shared_ptr Ab1, Ab2;  // A1*x-b1 and A2*x-b2
  boost::tie(Ab1, Ab2) = splitOffPlanarTree(N, Ab);

  // Eliminate the spanning tree to build a prior
  SubgraphPreconditioner::sharedBayesNet Rc1 =
      Ab1->eliminateSequential();       // R1*x-c1
  VectorValues xbar = Rc1->optimize();  // xbar = inv(R1)*c1

  // Create Subgraph-preconditioned system
  VectorValues::shared_ptr xbarShared(
      new VectorValues(xbar));  // TODO: horrible
  SubgraphPreconditioner system(Ab2, Rc1, xbarShared);

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
