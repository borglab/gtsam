/**
 * @file testSerialization.cpp
 *
 * @brief Tests for serialization global functions using boost.serialization
 *
 * @date Jun 12, 2013
 * @author Alex Cunningham
 */

#include <gtsam_unstable/slam/serialization.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/vector.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>

using namespace std;
using namespace gtsam;
using namespace boost::assign;
namespace fs = boost::filesystem;
#ifdef TOPSRCDIR
static string topdir = TOPSRCDIR;
#else
static string topdir = "TOPSRCDIR_NOT_CONFIGURED"; // If TOPSRCDIR is not defined, we error
#endif

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
  graph.addPrior(234, Pose2(1.0, 2.0, 0.3), noiseModel::Diagonal::Sigmas(Vector::Ones(3)));
  graph.add(BetweenFactor<Pose2>(234, 567, Pose2(1.0, 2.0, 0.3), noiseModel::Diagonal::Sigmas(Vector::Ones(3))));
  graph.add(BearingRangeFactor<Pose2,Point2>(234, 567, Rot2::fromAngle(0.3), 2.0, noiseModel::Diagonal::Sigmas(Vector::Ones(2))));
  return graph;
}

/* ************************************************************************* */
TEST( testSerialization, text_graph_serialization ) {
  NonlinearFactorGraph graph = exampleGraph();
  string serialized = serializeGraph(graph);
  NonlinearFactorGraph actGraph = *deserializeGraph(serialized);
  EXPECT(assert_equal(graph, actGraph));
}

/* ************************************************************************* */
TEST( testSerialization, xml_graph_serialization ) {
  NonlinearFactorGraph graph = exampleGraph();
  string serialized = serializeGraphXML(graph, "graph1");
  NonlinearFactorGraph actGraph = *deserializeGraphXML(serialized, "graph1");
  EXPECT(assert_equal(graph, actGraph));
}

/* ************************************************************************* */
TEST( testSerialization, text_values_serialization ) {
  Values values = exampleValues();
  string serialized = serializeValues(values);
  Values actValues = *deserializeValues(serialized);
  EXPECT(assert_equal(values, actValues));
}

/* ************************************************************************* */
TEST( testSerialization, xml_values_serialization ) {
  Values values = exampleValues();
  string serialized = serializeValuesXML(values, "values1");
  Values actValues = *deserializeValuesXML(serialized, "values1");
  EXPECT(assert_equal(values, actValues, 1e-5));
}

/* ************************************************************************* */
TEST( testSerialization, serialization_file ) {
  // Create files in folder in build folder
  fs::remove_all("actual");
  fs::create_directory("actual");
  string path = "actual/";

  NonlinearFactorGraph graph = exampleGraph();
  Values values = exampleValues();

  // Serialize objects using each configuration
  EXPECT(serializeGraphToFile(graph, path + "graph.dat"));
  EXPECT(serializeGraphToXMLFile(graph, path + "graph.xml", "graph1"));

  EXPECT(serializeValuesToFile(values, path + "values.dat"));
  EXPECT(serializeValuesToXMLFile(values, path + "values.xml", "values1"));

  // Deserialize
  NonlinearFactorGraph actGraph = *deserializeGraphFromFile(path + "graph.dat");
  NonlinearFactorGraph actGraphXML = *deserializeGraphFromXMLFile(path + "graph.xml", "graph1");

  Values actValues = *deserializeValuesFromFile(path + "values.dat");
  Values actValuesXML = *deserializeValuesFromXMLFile(path + "values.xml", "values1");

  // Verify
  EXPECT(assert_equal(graph, actGraph));
  EXPECT(assert_equal(graph, actGraphXML));

  EXPECT(assert_equal(values, actValues));
  EXPECT(assert_equal(values, actValuesXML));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
