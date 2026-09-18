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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/VectorConstants.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/linear/iterative.h>
#include <gtsam/constrained/NonlinearEquality.h>
#include <gtsam/slam/BetweenFactor.h>
#include <tests/smallExample.h>

#include <limits>
#include <map>

using namespace std;
using namespace gtsam;
using namespace example;
using symbol_shorthand::X; // to create pose keys
using symbol_shorthand::L; // to create landmark keys

static ConjugateGradientParameters parameters;
// add following below to add printing:
// parameters.verbosity_ = ConjugateGradientParameters::COMPLEXITY;

/* ************************************************************************* */
// Verifies natural-order construction from a key-to-dimension map.
TEST(KeyInfo, ConstructsFromDimensionsInNaturalOrder) {
  const std::map<Key, size_t> dimensions{{X(2), 3}, {X(1), 2}};
  const KeyInfo info(dimensions);

  EXPECT((info.ordering() == Ordering{X(1), X(2)}));
  LONGS_EQUAL(5, info.numCols());
  LONGS_EQUAL(0, info.at(X(1)).index);
  LONGS_EQUAL(0, info.at(X(1)).start);
  LONGS_EQUAL(2, info.at(X(1)).dim);
  LONGS_EQUAL(1, info.at(X(2)).index);
  LONGS_EQUAL(2, info.at(X(2)).start);
  LONGS_EQUAL(3, info.at(X(2)).dim);
}

/* ************************************************************************* */
// Verifies explicit-order construction from a key-to-dimension map.
TEST(KeyInfo, ConstructsFromDimensionsInExplicitOrder) {
  const std::map<Key, size_t> dimensions{{X(1), 2}, {X(2), 3}};
  const KeyInfo info(dimensions, Ordering{X(2), X(1)});

  EXPECT((info.ordering() == Ordering{X(2), X(1)}));
  LONGS_EQUAL(0, info.at(X(2)).index);
  LONGS_EQUAL(0, info.at(X(2)).start);
  LONGS_EQUAL(1, info.at(X(1)).index);
  LONGS_EQUAL(3, info.at(X(1)).start);
  LONGS_EQUAL(5, info.numCols());
}

/* ************************************************************************* */
// Verifies incomplete, duplicate, and unknown explicit ordering rejection.
TEST(KeyInfo, RejectsMalformedDimensionOrdering) {
  const std::map<Key, size_t> dimensions{{X(1), 2}, {X(2), 3}};

  CHECK_EXCEPTION(KeyInfo(dimensions, Ordering{X(1)}),
                  std::invalid_argument);
  CHECK_EXCEPTION(KeyInfo(dimensions, Ordering{X(1), X(1)}),
                  std::invalid_argument);
  CHECK_EXCEPTION(KeyInfo(dimensions, Ordering{X(1), X(3)}),
                  std::invalid_argument);
}

/* ************************************************************************* */
// Verifies scalar prefix sums cannot overflow size_t.
TEST(KeyInfo, RejectsScalarDimensionOverflow) {
  const std::map<Key, size_t> dimensions{
      {X(1), std::numeric_limits<size_t>::max()}, {X(2), 1}};

  CHECK_EXCEPTION(KeyInfo{dimensions}, std::overflow_error);
}

/* ************************************************************************* */
// Verifies flat-vector conversion and exact scalar-dimension validation.
TEST(KeyInfo, BuildsVectorValuesAndRejectsWrongDimension) {
  const KeyInfo info(std::map<Key, size_t>{{X(1), 2}, {X(2), 1}},
                     Ordering{X(2), X(1)});
  const Vector flat = (Vector(3) << 4.0, 2.0, 3.0).finished();
  const VectorValues values = buildVectorValues(flat, info);

  EXPECT(assert_equal(Vector1(4.0), values.at(X(2))));
  EXPECT(assert_equal(Vector2(2.0, 3.0), values.at(X(1))));
  CHECK_EXCEPTION(buildVectorValues(Vector::Zero(2), info),
                  std::invalid_argument);
}

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
  Vector expectedX{{-0.1, 0.1, -0.1, -0.1, 0.1, -0.2}};

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
  parameters.epsilon_abs = 1e-3;
  parameters.epsilon_rel = 1e-5;
  parameters.maxIterations = 100;
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
  parameters.epsilon_abs = 1e-3;
  parameters.epsilon_rel = 1e-5;
  parameters.maxIterations = 100;
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
