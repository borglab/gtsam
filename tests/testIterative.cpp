/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testIterative.cpp
 *  @brief  Unit tests for iterative methods
 *  @author Frank Dellaert
 **/

#include <tests/smallExample.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/iterative.h>
#include <gtsam/geometry/Pose2.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace example;
using symbol_shorthand::X; // to create pose keys
using symbol_shorthand::L; // to create landmark keys

static ConjugateGradientParameters parameters;
// add following below to add printing:
// parameters.verbosity_ = ConjugateGradientParameters::COMPLEXITY;

/* ************************************************************************* */
TEST( Iterative, steepestDescent )
{
  // Create factor graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // eliminate and solve
  VectorValues expected = fg.optimize();

  // Do gradient descent
  VectorValues zero = VectorValues::Zero(expected); // TODO, how do we do this normally?
  VectorValues actual = steepestDescent(fg, zero, parameters);
  CHECK(assert_equal(expected,actual,1e-2));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent )
{
  // Create factor graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // eliminate and solve
  VectorValues expected = fg.optimize();

  // get matrices
  Vector x0 = Z_6x1;
  const auto [A, b] = fg.jacobian();
  Vector expectedX = (Vector(6) << -0.1, 0.1, -0.1, -0.1, 0.1, -0.2).finished();

  // Do conjugate gradient descent, System version
  System Ab(A, b);
  Vector actualX = conjugateGradientDescent(Ab, x0, parameters);
  CHECK(assert_equal(expectedX,actualX,1e-9));

  // Do conjugate gradient descent, Matrix version
  Vector actualX2 = conjugateGradientDescent(A, b, x0, parameters);
  CHECK(assert_equal(expectedX,actualX2,1e-9));

  // Do conjugate gradient descent on factor graph
  VectorValues zero = VectorValues::Zero(expected);
  VectorValues actual = conjugateGradientDescent(fg, zero, parameters);
  CHECK(assert_equal(expected,actual,1e-2));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent_hard_constraint )
{
  Values config;
  Pose2 pose1 = Pose2(0.,0.,0.);
  config.insert(X(1), pose1);
  config.insert(X(2), Pose2(1.5,0.,0.));

  NonlinearFactorGraph graph;
  graph.emplace_shared<NonlinearEquality<Pose2>>(X(1), pose1);
  graph.emplace_shared<BetweenFactor<Pose2>>(X(1),X(2), Pose2(1.,0.,0.), noiseModel::Isotropic::Sigma(3, 1));

  std::shared_ptr<GaussianFactorGraph> fg = graph.linearize(config);

  VectorValues zeros = config.zeroVectors();

  ConjugateGradientParameters parameters;
  parameters.setEpsilon_abs(1e-3);
  parameters.setEpsilon_rel(1e-5);
  parameters.setMaxIterations(100);
  VectorValues actual = conjugateGradientDescent(*fg, zeros, parameters);

  VectorValues expected;
  expected.insert(X(1), Z_3x1);
  expected.insert(X(2), Vector3(-0.5,0.,0.));
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent_soft_constraint )
{
  Values config;
  config.insert(X(1), Pose2(0.,0.,0.));
  config.insert(X(2), Pose2(1.5,0.,0.));

  NonlinearFactorGraph graph;
  graph.addPrior(X(1), Pose2(0.,0.,0.), noiseModel::Isotropic::Sigma(3, 1e-10));
  graph.emplace_shared<BetweenFactor<Pose2>>(X(1),X(2), Pose2(1.,0.,0.), noiseModel::Isotropic::Sigma(3, 1));

  std::shared_ptr<GaussianFactorGraph> fg = graph.linearize(config);

  VectorValues zeros = config.zeroVectors();

  ConjugateGradientParameters parameters;
  parameters.setEpsilon_abs(1e-3);
  parameters.setEpsilon_rel(1e-5);
  parameters.setMaxIterations(100);
  VectorValues actual = conjugateGradientDescent(*fg, zeros, parameters);

  VectorValues expected;
  expected.insert(X(1), Z_3x1);
  expected.insert(X(2), Vector3(-0.5,0.,0.));
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
