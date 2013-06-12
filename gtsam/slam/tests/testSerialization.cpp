/**
 * @file testSerialization.cpp
 *
 * @brief Tests for serialization global functions using boost.serialization
 *
 * @date Jun 12, 2013
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/serialization.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

Values exampleValues() {
  Values result;
  result.insert(234, gtsam::Rot2::fromAngle(0.1));
  result.insert(123, gtsam::Point2(1.0, 2.0));
  result.insert(254, gtsam::Pose2(1.0, 2.0, 0.3));
  result.insert(678, gtsam::Rot3::Rx(0.1));
  result.insert(498, gtsam::Point3(1.0, 2.0, 3.0));
  result.insert(345, gtsam::Pose3(Rot3::Rx(0.1), Point3(1.0, 2.0, 3.0)));
  return result;
}

NonlinearFactorGraph exampleGraph() {
  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose2>(234, Pose2(1.0, 2.0, 0.3), noiseModel::Diagonal::Sigmas(ones(3))));
  graph.add(BetweenFactor<Pose2>(234, 567, Pose2(1.0, 2.0, 0.3), noiseModel::Diagonal::Sigmas(ones(3))));
  return graph;
}

/* ************************************************************************* */
TEST( testSerialization, text_graph_serialization ) {
  NonlinearFactorGraph graph = exampleGraph();
  string serialized = serializeGraph(graph);
  NonlinearFactorGraph actGraph = deserializeGraph(serialized);
  EXPECT(assert_equal(graph, actGraph));
}

/* ************************************************************************* */
TEST( testSerialization, xml_graph_serialization ) {
  NonlinearFactorGraph graph = exampleGraph();
  string serialized = serializeGraphXML(graph, "graph1");
  NonlinearFactorGraph actGraph = deserializeGraphXML(serialized, "graph1");
  EXPECT(assert_equal(graph, actGraph));
}

/* ************************************************************************* */
TEST( testSerialization, text_values_serialization ) {
  Values values = exampleValues();
  string serialized = serializeValues(values);
  Values actValues = deserializeValues(serialized);
  EXPECT(assert_equal(values, actValues));
}

/* ************************************************************************* */
TEST( testSerialization, xml_values_serialization ) {
  Values values = exampleValues();
  string serialized = serializeValuesXML(values, "values1");
  Values actValues = deserializeValuesXML(serialized, "values1");
  EXPECT(assert_equal(values, actValues, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
