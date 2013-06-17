/**
 * @file testSummarization.cpp
 *
 * @brief Test ported from MastSLAM for a simple batch summarization technique
 *
 * @date May 7, 2013
 * @author Alex Cunningham
 */

#include <boost/assign/std/set.hpp>
#include <boost/assign/std/vector.hpp>

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/TestableAssertions.h>

#include <gtsam/geometry/Pose2.h>

#include <gtsam/nonlinear/LabeledSymbol.h>
#include <gtsam/nonlinear/summarization.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

const double tol=1e-5;

typedef gtsam::PriorFactor<gtsam::Pose2> PosePrior;
typedef gtsam::BetweenFactor<gtsam::Pose2> PoseBetween;
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> PosePointBearingRange;

gtsam::noiseModel::Base::shared_ptr model2 = noiseModel::Unit::Create(2);
gtsam::noiseModel::Base::shared_ptr model3 = noiseModel::Unit::Create(3);

/* ************************************************************************* */
TEST( testSummarization, example_from_ddf1 ) {
  Key xA0 = LabeledSymbol('x', 'A', 0),
      xA1 = LabeledSymbol('x', 'A', 1),
      xA2 = LabeledSymbol('x', 'A', 2);
  Key lA3 = LabeledSymbol('l', 'A', 3), lA5 = LabeledSymbol('l', 'A', 5);

  SharedDiagonal diagmodel2 = noiseModel::Unit::Create(2);
  SharedDiagonal diagmodel4 = noiseModel::Unit::Create(4);

  Pose2 pose0;
  Pose2 pose1(1.0, 0.0, 0.0);
  Pose2 pose2(2.0, 0.0, 0.0);
  Point2 landmark3(3.0, 3.0);
  Point2 landmark5(5.0, 5.0);

  Values values;
  values.insert(xA0, pose0);
  values.insert(xA1, pose1);
  values.insert(xA2, pose2);
  values.insert(lA3, landmark3);
  values.insert(lA5, landmark5);

  // build from nonlinear graph/values
  NonlinearFactorGraph graph;
  graph.add(PosePrior(xA0, Pose2(), model3));
  graph.add(PoseBetween(xA0, xA1, pose0.between(pose1), model3));
  graph.add(PoseBetween(xA1, xA2, pose1.between(pose2), model3));
  graph.add(PosePointBearingRange(xA0, lA3, pose0.bearing(landmark3), pose0.range(landmark3), model2));
  graph.add(PosePointBearingRange(xA1, lA3, pose1.bearing(landmark3), pose1.range(landmark3), model2));
  graph.add(PosePointBearingRange(xA2, lA5, pose2.bearing(landmark5), pose2.range(landmark5), model2));

  KeySet saved_keys;
  saved_keys += lA3, lA5;

  {
    // Summarize to a linear system
    GaussianFactorGraph actLinGraph; Ordering actOrdering;
    SummarizationMode mode = PARTIAL_QR;
    boost::tie(actLinGraph, actOrdering) = summarize(graph, values, saved_keys, mode);

    Ordering expSumOrdering; expSumOrdering += xA0, xA1, xA2, lA3, lA5;
    EXPECT(assert_equal(expSumOrdering, actOrdering));

    // Does not split out subfactors where possible
    GaussianFactorGraph expLinGraph;
    expLinGraph.add(
      expSumOrdering[lA3],
      Matrix_(4,2,
        0.595867,  0.605092,
        0.0, -0.406109,
        0.0,       0.0,
        0.0,       0.0),
      expSumOrdering[lA5],
      Matrix_(4,2,
        -0.125971, -0.160052,
        0.13586,  0.301096,
        0.268667,   0.31703,
        0.0, -0.131698),
      zero(4), diagmodel4);
    EXPECT(assert_equal(expLinGraph, actLinGraph, tol));

    // Summarize directly from a nonlinear graph to another nonlinear graph
    NonlinearFactorGraph actContainerGraph = summarizeAsNonlinearContainer(graph, values, saved_keys, mode);
    NonlinearFactorGraph expContainerGraph = LinearContainerFactor::convertLinearGraph(expLinGraph, expSumOrdering);

    EXPECT(assert_equal(expContainerGraph, actContainerGraph, tol));
  }

  {
    // Summarize to a linear system using cholesky - compare to previous version
    GaussianFactorGraph actLinGraph; Ordering actOrdering;
    SummarizationMode mode = PARTIAL_CHOLESKY;
    boost::tie(actLinGraph, actOrdering) = summarize(graph, values, saved_keys, mode);

    Ordering expSumOrdering; expSumOrdering += xA0, xA1, xA2, lA3, lA5;
    EXPECT(assert_equal(expSumOrdering, actOrdering));

    // Does not split out subfactors where possible
    GaussianFactorGraph expLinGraph;
    expLinGraph.add(HessianFactor(JacobianFactor(
        expSumOrdering[lA3],
        Matrix_(4,2,
          0.595867,  0.605092,
          0.0, -0.406109,
          0.0,       0.0,
          0.0,       0.0),
        expSumOrdering[lA5],
        Matrix_(4,2,
          -0.125971, -0.160052,
          0.13586,  0.301096,
          0.268667,   0.31703,
          0.0, -0.131698),
        zero(4), diagmodel4)));
    EXPECT(assert_equal(expLinGraph, actLinGraph, tol));

    // Summarize directly from a nonlinear graph to another nonlinear graph
    NonlinearFactorGraph actContainerGraph = summarizeAsNonlinearContainer(graph, values, saved_keys, mode);
    NonlinearFactorGraph expContainerGraph = LinearContainerFactor::convertLinearGraph(expLinGraph, expSumOrdering);

    EXPECT(assert_equal(expContainerGraph, actContainerGraph, tol));
  }

  {
    // Summarize to a linear system with joint factor graph version
    GaussianFactorGraph actLinGraph; Ordering actOrdering;
    SummarizationMode mode = SEQUENTIAL_QR;
    boost::tie(actLinGraph, actOrdering) = summarize(graph, values, saved_keys, mode);

    Ordering expSumOrdering; expSumOrdering += xA0, xA1, xA2, lA3, lA5;
    EXPECT(assert_equal(expSumOrdering, actOrdering));

    // Does not split out subfactors where possible
    GaussianFactorGraph expLinGraph;
    expLinGraph.add(
        expSumOrdering[lA3],
        Matrix_(2,2,
          0.595867, 0.605092,
          0.0, 0.406109),
        expSumOrdering[lA5],
        Matrix_(2,2,
          -0.125971, -0.160052,
          -0.13586, -0.301096),
        zero(2), diagmodel2);

    expLinGraph.add(
        expSumOrdering[lA5],
        Matrix_(2,2,
          0.268667,  0.31703,
          0.0, 0.131698),
        zero(2), diagmodel2);

    EXPECT(assert_equal(expLinGraph, actLinGraph, tol));

    // Summarize directly from a nonlinear graph to another nonlinear graph
    NonlinearFactorGraph actContainerGraph = summarizeAsNonlinearContainer(graph, values, saved_keys, mode);
    NonlinearFactorGraph expContainerGraph = LinearContainerFactor::convertLinearGraph(expLinGraph, expSumOrdering);

    EXPECT(assert_equal(expContainerGraph, actContainerGraph, tol));
  }

  {
    // Summarize to a linear system with joint factor graph version
    GaussianFactorGraph actLinGraph; Ordering actOrdering;
    SummarizationMode mode = SEQUENTIAL_CHOLESKY;
    boost::tie(actLinGraph, actOrdering) = summarize(graph, values, saved_keys, mode);

    Ordering expSumOrdering; expSumOrdering += xA0, xA1, xA2, lA3, lA5;
    EXPECT(assert_equal(expSumOrdering, actOrdering));

    // Does not split out subfactors where possible
    GaussianFactorGraph expLinGraph;
    expLinGraph.add(
        expSumOrdering[lA3],
        Matrix_(2,2,
          0.595867, 0.605092,
          0.0, 0.406109),
        expSumOrdering[lA5],
        Matrix_(2,2,
          -0.125971, -0.160052,
          -0.13586, -0.301096),
        zero(2), diagmodel2);

    expLinGraph.add(
        expSumOrdering[lA5],
        Matrix_(2,2,
          0.268667,  0.31703,
          0.0, 0.131698),
        zero(2), diagmodel2);

    EXPECT(assert_equal(expLinGraph, actLinGraph, tol));

    // Summarize directly from a nonlinear graph to another nonlinear graph
    NonlinearFactorGraph actContainerGraph = summarizeAsNonlinearContainer(graph, values, saved_keys, mode);
    NonlinearFactorGraph expContainerGraph = LinearContainerFactor::convertLinearGraph(expLinGraph, expSumOrdering);

    EXPECT(assert_equal(expContainerGraph, actContainerGraph, tol));
  }
}

/* ************************************************************************* */
TEST( testSummarization, no_summarize_case ) {
  // Checks a corner case in which no variables are being eliminated
  gtsam::Key key = 7;
  gtsam::KeySet saved_keys; saved_keys.insert(key);
  NonlinearFactorGraph graph;
  graph.add(PosePrior(key, Pose2(1.0, 2.0, 0.3), model3));
  graph.add(PosePrior(key, Pose2(2.0, 3.0, 0.4), model3));
  Values values;
  values.insert(key, Pose2(0.0, 0.0, 0.1));

  SummarizationMode mode = SEQUENTIAL_CHOLESKY;
  GaussianFactorGraph actLinGraph; Ordering actOrdering;
  boost::tie(actLinGraph, actOrdering) = summarize(graph, values, saved_keys, mode);
  Ordering expOrdering; expOrdering += key;
  GaussianFactorGraph expLinGraph = *graph.linearize(values, expOrdering);
  EXPECT(assert_equal(expOrdering, actOrdering));
  EXPECT(assert_equal(expLinGraph, actLinGraph));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
