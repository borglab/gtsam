/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDataset.cpp
 * @brief   Unit test for dataset.cpp
 * @author  Richard Roberts, Luca Carlone
 */


#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/TestableAssertions.h>

#include <boost/algorithm/string.hpp>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <sstream>

using namespace gtsam::symbol_shorthand;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(dataSet, findExampleDataFile) {
  const string expected_end = "examples/Data/example.graph";
  const string actual = findExampleDataFile("example");
  string actual_end = actual.substr(actual.size() - expected_end.size(), expected_end.size());
  boost::replace_all(actual_end, "\\", "/"); // Convert directory separators to forward-slash
  EXPECT(assert_equal(expected_end, actual_end));
}

/* ************************************************************************* */
TEST( dataSet, parseVertexPose)
{
  const string str = "VERTEX2 1 2.000000 3.000000 4.000000";
  istringstream is(str);
  string tag;
  EXPECT(is >> tag);
  const auto actual = parseVertexPose(is, tag);
  EXPECT(actual);
  if (actual) {
    EXPECT_LONGS_EQUAL(1, actual->first);
    EXPECT(assert_equal(Pose2(2, 3, 4), actual->second));
  }
}

/* ************************************************************************* */
TEST( dataSet, parseVertexLandmark)
{
  const string str = "VERTEX_XY 1 2.000000 3.000000";
  istringstream is(str);
  string tag;
  EXPECT(is >> tag);
  const auto actual = parseVertexLandmark(is, tag);
  EXPECT(actual);
  if (actual) {
    EXPECT_LONGS_EQUAL(1, actual->first);
    EXPECT(assert_equal(Point2(2, 3), actual->second));
  }
}

/* ************************************************************************* */
TEST( dataSet, parseEdge)
{
  const string str = "EDGE2 0 1 2.000000 3.000000 4.000000";
  istringstream is(str);
  string tag;
  EXPECT(is >> tag);
  const auto actual = parseEdge(is, tag);
  EXPECT(actual);
  if (actual) {
    pair<size_t, size_t> expected(0, 1);
    EXPECT(expected == actual->first);
    EXPECT(assert_equal(Pose2(2, 3, 4), actual->second));
  }
}

/* ************************************************************************* */
TEST(dataSet, load2D) {
  ///< The structure where we will save the SfM data
  const string filename = findExampleDataFile("w100.graph");
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  boost::tie(graph, initial) = load2D(filename);
  EXPECT_LONGS_EQUAL(300, graph->size());
  EXPECT_LONGS_EQUAL(100, initial->size());
  auto model = noiseModel::Unit::Create(3);
  BetweenFactor<Pose2> expected(1, 0, Pose2(-0.99879, 0.0417574, -0.00818381),
                                model);
  BetweenFactor<Pose2>::shared_ptr actual =
      boost::dynamic_pointer_cast<BetweenFactor<Pose2>>(graph->at(0));
  EXPECT(assert_equal(expected, *actual));

  // Check binary measurements, Pose2
  size_t maxIndex = 5;
  auto measurements = parseMeasurements<Pose2>(filename, nullptr, maxIndex);
  EXPECT_LONGS_EQUAL(5, measurements.size());

  // Check binary measurements, Rot2
  auto measurements2 = parseMeasurements<Rot2>(filename);
  EXPECT_LONGS_EQUAL(300, measurements2.size());

  // // Check factor parsing
  const auto actualFactors = parseFactors<Pose2>(filename);
  for (size_t i : {0, 1, 2, 3, 4, 5}) {
    EXPECT(assert_equal(
        *boost::dynamic_pointer_cast<BetweenFactor<Pose2>>(graph->at(i)),
        *actualFactors[i], 1e-5));
  }

  // Check pose parsing
  const auto actualPoses = parseVariables<Pose2>(filename);
  for (size_t j : {0, 1, 2, 3, 4}) {
    EXPECT(assert_equal(initial->at<Pose2>(j), actualPoses.at(j), 1e-5));
  }

  // Check landmark parsing
  const auto actualLandmarks = parseVariables<Point2>(filename);
  EXPECT_LONGS_EQUAL(0, actualLandmarks.size());
}

/* ************************************************************************* */
TEST(dataSet, load2DVictoriaPark) {
  const string filename = findExampleDataFile("victoria_park.txt");
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;

  // Load all
  boost::tie(graph, initial) = load2D(filename);
  EXPECT_LONGS_EQUAL(10608, graph->size());
  EXPECT_LONGS_EQUAL(7120, initial->size());

  // Restrict keys
  size_t maxIndex = 5;
  boost::tie(graph, initial) = load2D(filename, nullptr, maxIndex);
  EXPECT_LONGS_EQUAL(5, graph->size());
  EXPECT_LONGS_EQUAL(6, initial->size()); // file has 0 as well
  EXPECT_LONGS_EQUAL(L(5), graph->at(4)->keys()[1]);
}

/* ************************************************************************* */
TEST( dataSet, Balbianello)
{
  ///< The structure where we will save the SfM data
  const string filename = findExampleDataFile("Balbianello");
  SfmData mydata;
  CHECK(readBundler(filename, mydata));

  // Check number of things
  EXPECT_LONGS_EQUAL(5,mydata.number_cameras());
  EXPECT_LONGS_EQUAL(544,mydata.number_tracks());
  const SfmTrack& track0 = mydata.tracks[0];
  EXPECT_LONGS_EQUAL(3,track0.number_measurements());

  // Check projection of a given point
  EXPECT_LONGS_EQUAL(0,track0.measurements[0].first);
  const SfmCamera& camera0 = mydata.cameras[0];
  Point2 expected = camera0.project(track0.p), actual = track0.measurements[0].second;
  EXPECT(assert_equal(expected,actual,1));
}

/* ************************************************************************* */
TEST(dataSet, readG2o3D) {
  const string g2oFile = findExampleDataFile("pose3example");
  auto model = noiseModel::Isotropic::Precision(6, 10000);

  // Initialize 6 relative measurements with quaternion/point3 values:
  const std::vector<Pose3> relative_poses = {
      {{0.854230, 0.190253, 0.283162, -0.392318},
       {1.001367, 0.015390, 0.004948}},
      {{0.105373, 0.311512, 0.656877, -0.678505},
       {0.523923, 0.776654, 0.326659}},
      {{0.568551, 0.595795, -0.561677, 0.079353},
       {0.910927, 0.055169, -0.411761}},
      {{0.542221, -0.592077, 0.303380, -0.513226},
       {0.775288, 0.228798, -0.596923}},
      {{0.327419, -0.125250, -0.534379, 0.769122},
       {-0.577841, 0.628016, -0.543592}},
      {{0.083672, 0.104639, 0.627755, 0.766795},
       {-0.623267, 0.086928, 0.773222}},
  };

  // Initialize 5 poses with quaternion/point3 values:
  const std::vector<Pose3> poses = {
      {{1.000000, 0.000000, 0.000000, 0.000000}, {0, 0, 0}},
      {{0.854230, 0.190253, 0.283162, -0.392318},
       {1.001367, 0.015390, 0.004948}},
      {{0.421446, -0.351729, -0.597838, 0.584174},
       {1.993500, 0.023275, 0.003793}},
      {{0.067024, 0.331798, -0.200659, 0.919323},
       {2.004291, 1.024305, 0.018047}},
      {{0.765488, -0.035697, -0.462490, 0.445933},
       {0.999908, 1.055073, 0.020212}},
  };

  // Specify connectivity:
  using KeyPair = pair<Key, Key>;
  std::vector<KeyPair> edges = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {1, 4}, {3, 0}};

  // Created expected factor graph
  size_t i = 0;
  NonlinearFactorGraph expectedGraph;
  for (const auto& keys : edges) {
    expectedGraph.emplace_shared<BetweenFactor<Pose3>>(
        keys.first, keys.second, relative_poses[i++], model);
  }

  // Check factor parsing
  const auto actualFactors = parseFactors<Pose3>(g2oFile);
  for (size_t i : {0, 1, 2, 3, 4, 5}) {
    EXPECT(assert_equal(
        *boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(expectedGraph[i]),
        *actualFactors[i], 1e-5));
  }

  // Check pose parsing
  const auto actualPoses = parseVariables<Pose3>(g2oFile);
  for (size_t j : {0, 1, 2, 3, 4}) {
    EXPECT(assert_equal(poses[j], actualPoses.at(j), 1e-5));
  }

  // Check landmark parsing
  const auto actualLandmarks = parseVariables<Point3>(g2oFile);
  for (size_t j : {0, 1, 2, 3, 4}) {
    EXPECT(assert_equal(poses[j], actualPoses.at(j), 1e-5));
  }

  // Check graph version
  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  bool is3D = true;
  boost::tie(actualGraph, actualValues) = readG2o(g2oFile, is3D);
  EXPECT(assert_equal(expectedGraph, *actualGraph, 1e-5));
  for (size_t j : {0, 1, 2, 3, 4}) {
    EXPECT(assert_equal(poses[j], actualValues->at<Pose3>(j), 1e-5));
  }
}

/* ************************************************************************* */
TEST( dataSet, readG2o3DNonDiagonalNoise)
{
  const string g2oFile = findExampleDataFile("pose3example-offdiagonal.txt");
  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  bool is3D = true;
  boost::tie(actualGraph, actualValues) = readG2o(g2oFile, is3D);

  Values expectedValues;
  Rot3 R0 = Rot3::Quaternion(1.000000, 0.000000, 0.000000, 0.000000 );
  Point3 p0 = Point3(0.000000, 0.000000, 0.000000);
  expectedValues.insert(0, Pose3(R0, p0));

  Rot3 R1 = Rot3::Quaternion(0.854230, 0.190253, 0.283162, -0.392318 );
  Point3 p1 = Point3(1.001367, 0.015390, 0.004948);
  expectedValues.insert(1, Pose3(R1, p1));

  EXPECT(assert_equal(expectedValues,*actualValues,1e-5));

  Matrix Info = Matrix(6,6);
  for (int i = 0; i < 6; i++){
    for (int j = i; j < 6; j++){
      if(i==j)
        Info(i, j) = 10000;
      else{
        Info(i, j) = i+1; // arbitrary nonzero number
        Info(j, i) = i+1;
      }
    }
  }
  noiseModel::Gaussian::shared_ptr model = noiseModel::Gaussian::Covariance(Info.inverse());
  NonlinearFactorGraph expectedGraph;
  Point3 p01 = Point3(1.001367, 0.015390, 0.004948);
  Rot3 R01 = Rot3::Quaternion(0.854230, 0.190253, 0.283162, -0.392318 );
  expectedGraph.emplace_shared<BetweenFactor<Pose3> >(0, 1, Pose3(R01,p01), model);

  EXPECT(assert_equal(expectedGraph,*actualGraph,1e-2));
}

/* ************************************************************************* */
TEST(dataSet, readG2oCheckDeterminants) {
  const string g2oFile = findExampleDataFile("toyExample.g2o");

  // Check determinants in factors
  auto factors = parseFactors<Pose3>(g2oFile);
  EXPECT_LONGS_EQUAL(6, factors.size());
  for (const auto& factor : factors) {
    const Rot3 R = factor->measured().rotation();
    EXPECT_DOUBLES_EQUAL(1.0, R.matrix().determinant(), 1e-9);
  }

  // Check determinants in initial values
  const map<size_t, Pose3> poses = parseVariables<Pose3>(g2oFile);
  EXPECT_LONGS_EQUAL(5, poses.size());
  for (const auto& key_value : poses) {
    const Rot3 R = key_value.second.rotation();
    EXPECT_DOUBLES_EQUAL(1.0, R.matrix().determinant(), 1e-9);
  }
  const map<size_t, Point3> landmarks = parseVariables<Point3>(g2oFile);
  EXPECT_LONGS_EQUAL(0, landmarks.size());
}

/* ************************************************************************* */
TEST(dataSet, readG2oLandmarks) {
  const string g2oFile = findExampleDataFile("example_with_vertices.g2o");

  // Check number of poses and landmarks. Should be 8 each.
  const map<size_t, Pose3> poses = parseVariables<Pose3>(g2oFile);
  EXPECT_LONGS_EQUAL(8, poses.size());
  const map<size_t, Point3> landmarks = parseVariables<Point3>(g2oFile);
  EXPECT_LONGS_EQUAL(8, landmarks.size());

  auto graphAndValues = load3D(g2oFile);
  EXPECT(graphAndValues.second->exists(L(0)));
}

/* ************************************************************************* */
static NonlinearFactorGraph expectedGraph(const SharedNoiseModel& model) {
  NonlinearFactorGraph g;
  using Factor = BetweenFactor<Pose2>;
  g.emplace_shared<Factor>(0, 1, Pose2(1.030390, 0.011350, -0.081596), model);
  g.emplace_shared<Factor>(1, 2, Pose2(1.013900, -0.058639, -0.220291), model);
  g.emplace_shared<Factor>(2, 3, Pose2(1.027650, -0.007456, -0.043627), model);
  g.emplace_shared<Factor>(3, 4, Pose2(-0.012016, 1.004360, 1.560229), model);
  g.emplace_shared<Factor>(4, 5, Pose2(1.016030, 0.014565, -0.030930), model);
  g.emplace_shared<Factor>(5, 6, Pose2(1.023890, 0.006808, -0.007452), model);
  g.emplace_shared<Factor>(6, 7, Pose2(0.957734, 0.003159, 0.082836), model);
  g.emplace_shared<Factor>(7, 8, Pose2(-1.023820, -0.013668, -3.084560), model);
  g.emplace_shared<Factor>(8, 9, Pose2(1.023440, 0.013984, -0.127624), model);
  g.emplace_shared<Factor>(9, 10, Pose2(1.003350, 0.022250, -0.195918), model);
  g.emplace_shared<Factor>(5, 9, Pose2(0.033943, 0.032439, 3.073637), model);
  g.emplace_shared<Factor>(3, 10, Pose2(0.044020, 0.988477, -1.553511), model);
  return g;
}

/* ************************************************************************* */
TEST(dataSet, readG2o) {
  const string g2oFile = findExampleDataFile("pose2example");
  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  boost::tie(actualGraph, actualValues) = readG2o(g2oFile);

  auto model = noiseModel::Diagonal::Precisions(
      Vector3(44.721360, 44.721360, 30.901699));
  EXPECT(assert_equal(expectedGraph(model), *actualGraph, 1e-5));

  Values expectedValues;
  expectedValues.insert(0, Pose2(0.000000, 0.000000, 0.000000));
  expectedValues.insert(1, Pose2(1.030390, 0.011350, -0.081596));
  expectedValues.insert(2, Pose2(2.036137, -0.129733, -0.301887));
  expectedValues.insert(3, Pose2(3.015097, -0.442395, -0.345514));
  expectedValues.insert(4, Pose2(3.343949, 0.506678, 1.214715));
  expectedValues.insert(5, Pose2(3.684491, 1.464049, 1.183785));
  expectedValues.insert(6, Pose2(4.064626, 2.414783, 1.176333));
  expectedValues.insert(7, Pose2(4.429778, 3.300180, 1.259169));
  expectedValues.insert(8, Pose2(4.128877, 2.321481, -1.825391));
  expectedValues.insert(9, Pose2(3.884653, 1.327509, -1.953016));
  expectedValues.insert(10, Pose2(3.531067, 0.388263, -2.148934));
  EXPECT(assert_equal(expectedValues, *actualValues, 1e-5));
}

/* ************************************************************************* */
TEST(dataSet, readG2oHuber) {
  const string g2oFile = findExampleDataFile("pose2example");
  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  bool is3D = false;
  boost::tie(actualGraph, actualValues) =
      readG2o(g2oFile, is3D, KernelFunctionTypeHUBER);

  auto baseModel = noiseModel::Diagonal::Precisions(
      Vector3(44.721360, 44.721360, 30.901699));
  auto model = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345), baseModel);

  EXPECT(assert_equal(expectedGraph(model), *actualGraph, 1e-5));
}

/* ************************************************************************* */
TEST(dataSet, readG2oTukey) {
  const string g2oFile = findExampleDataFile("pose2example");
  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  bool is3D = false;
  boost::tie(actualGraph, actualValues) =
      readG2o(g2oFile, is3D, KernelFunctionTypeTUKEY);

  auto baseModel = noiseModel::Diagonal::Precisions(
      Vector3(44.721360, 44.721360, 30.901699));
  auto model = noiseModel::Robust::Create(
      noiseModel::mEstimator::Tukey::Create(4.6851), baseModel);

  EXPECT(assert_equal(expectedGraph(model), *actualGraph, 1e-5));
}

/* ************************************************************************* */
TEST( dataSet, writeG2o)
{
  const string g2oFile = findExampleDataFile("pose2example");
  NonlinearFactorGraph::shared_ptr expectedGraph;
  Values::shared_ptr expectedValues;
  boost::tie(expectedGraph, expectedValues) = readG2o(g2oFile);

  const string filenameToWrite = createRewrittenFileName(g2oFile);
  writeG2o(*expectedGraph, *expectedValues, filenameToWrite);

  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  boost::tie(actualGraph, actualValues) = readG2o(filenameToWrite);
  EXPECT(assert_equal(*expectedValues,*actualValues,1e-5));
  EXPECT(assert_equal(*expectedGraph,*actualGraph,1e-5));
}

/* ************************************************************************* */
TEST( dataSet, writeG2o3D)
{
  const string g2oFile = findExampleDataFile("pose3example");
  NonlinearFactorGraph::shared_ptr expectedGraph;
  Values::shared_ptr expectedValues;
  bool is3D = true;
  boost::tie(expectedGraph, expectedValues) = readG2o(g2oFile, is3D);

  const string filenameToWrite = createRewrittenFileName(g2oFile);
  writeG2o(*expectedGraph, *expectedValues, filenameToWrite);

  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  boost::tie(actualGraph, actualValues) = readG2o(filenameToWrite, is3D);
  EXPECT(assert_equal(*expectedValues,*actualValues,1e-4));
  EXPECT(assert_equal(*expectedGraph,*actualGraph,1e-4));
}

/* ************************************************************************* */
TEST( dataSet, writeG2o3DNonDiagonalNoise)
{
  const string g2oFile = findExampleDataFile("pose3example-offdiagonal");
  NonlinearFactorGraph::shared_ptr expectedGraph;
  Values::shared_ptr expectedValues;
  bool is3D = true;
  boost::tie(expectedGraph, expectedValues) = readG2o(g2oFile, is3D);

  const string filenameToWrite = createRewrittenFileName(g2oFile);
  writeG2o(*expectedGraph, *expectedValues, filenameToWrite);

  NonlinearFactorGraph::shared_ptr actualGraph;
  Values::shared_ptr actualValues;
  boost::tie(actualGraph, actualValues) = readG2o(filenameToWrite, is3D);
  EXPECT(assert_equal(*expectedValues,*actualValues,1e-4));
  EXPECT(assert_equal(*expectedGraph,*actualGraph,1e-4));
}

/* ************************************************************************* */
TEST( dataSet, readBAL_Dubrovnik)
{
  ///< The structure where we will save the SfM data
  const string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData mydata;
  CHECK(readBAL(filename, mydata));

  // Check number of things
  EXPECT_LONGS_EQUAL(3,mydata.number_cameras());
  EXPECT_LONGS_EQUAL(7,mydata.number_tracks());
  const SfmTrack& track0 = mydata.tracks[0];
  EXPECT_LONGS_EQUAL(3,track0.number_measurements());

  // Check projection of a given point
  EXPECT_LONGS_EQUAL(0,track0.measurements[0].first);
  const SfmCamera& camera0 = mydata.cameras[0];
  Point2 expected = camera0.project(track0.p), actual = track0.measurements[0].second;
  EXPECT(assert_equal(expected,actual,12));
}

/* ************************************************************************* */
TEST( dataSet, openGL2gtsam)
{
  Vector3 rotVec(0.2, 0.7, 1.1);
  Rot3 R = Rot3::Expmap(rotVec);
  Point3 t = Point3(0.0,0.0,0.0);
  Pose3 poseGTSAM = Pose3(R,t);

  Pose3 expected = openGL2gtsam(R, t.x(), t.y(), t.z());

  Point3 r1 = R.r1(), r2 = R.r2(), r3 = R.r3(); //columns!
  Rot3 cRw(
      r1.x(),  r2.x(),  r3.x(),
     -r1.y(), -r2.y(), -r3.y(),
      -r1.z(), -r2.z(), -r3.z());
  Rot3 wRc = cRw.inverse();
  Pose3 actual = Pose3(wRc,t);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( dataSet, gtsam2openGL)
{
  Vector3 rotVec(0.2, 0.7, 1.1);
  Rot3 R = Rot3::Expmap(rotVec);
  Point3 t = Point3(1.0,20.0,10.0);
  Pose3 actual = Pose3(R,t);
  Pose3 poseGTSAM = openGL2gtsam(R, t.x(), t.y(), t.z());

  Pose3 expected = gtsam2openGL(poseGTSAM);
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( dataSet, writeBAL_Dubrovnik)
{
  ///< Read a file using the unit tested readBAL
  const string filenameToRead = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData readData;
  readBAL(filenameToRead, readData);

  // Write readData to file filenameToWrite
  const string filenameToWrite = createRewrittenFileName(filenameToRead);
  CHECK(writeBAL(filenameToWrite, readData));

  // Read what we wrote
  SfmData writtenData;
  CHECK(readBAL(filenameToWrite, writtenData));

  // Check that what we read is the same as what we wrote
  EXPECT_LONGS_EQUAL(readData.number_cameras(),writtenData.number_cameras());
  EXPECT_LONGS_EQUAL(readData.number_tracks(),writtenData.number_tracks());

  for (size_t i = 0; i < readData.number_cameras(); i++){
    PinholeCamera<Cal3Bundler> expectedCamera = writtenData.cameras[i];
    PinholeCamera<Cal3Bundler> actualCamera = readData.cameras[i];
    EXPECT(assert_equal(expectedCamera,actualCamera));
  }

  for (size_t j = 0; j < readData.number_tracks(); j++){
    // check point
    SfmTrack expectedTrack  = writtenData.tracks[j];
    SfmTrack actualTrack = readData.tracks[j];
    Point3 expectedPoint = expectedTrack.p;
    Point3 actualPoint = actualTrack.p;
    EXPECT(assert_equal(expectedPoint,actualPoint));

    // check rgb
    Point3 expectedRGB = Point3( expectedTrack.r,  expectedTrack.g, expectedTrack.b );
    Point3 actualRGB = Point3(  actualTrack.r,  actualTrack.g, actualTrack.b);
    EXPECT(assert_equal(expectedRGB,actualRGB));

    // check measurements
    for (size_t k = 0; k < actualTrack.number_measurements(); k++){
      EXPECT_LONGS_EQUAL(expectedTrack.measurements[k].first,actualTrack.measurements[k].first);
      EXPECT(assert_equal(expectedTrack.measurements[k].second,actualTrack.measurements[k].second));
    }
  }
}


/* ************************************************************************* */
TEST( dataSet, writeBALfromValues_Dubrovnik){

  ///< Read a file using the unit tested readBAL
  const string filenameToRead = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData readData;
  readBAL(filenameToRead, readData);

  Pose3 poseChange = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.3,0.1,0.3));

  Values value;
  for(size_t i=0; i < readData.number_cameras(); i++){ // for each camera
    Pose3 pose = poseChange.compose(readData.cameras[i].pose());
    value.insert(X(i), pose);
  }
  for(size_t j=0; j < readData.number_tracks(); j++){ // for each point
    Point3 point = poseChange.transformFrom( readData.tracks[j].p );
    value.insert(P(j), point);
  }

  // Write values and readData to a file
  const string filenameToWrite = createRewrittenFileName(filenameToRead);
  writeBALfromValues(filenameToWrite, readData, value);

  // Read the file we wrote
  SfmData writtenData;
  readBAL(filenameToWrite, writtenData);

  // Check that the reprojection errors are the same and the poses are correct
  // Check number of things
  EXPECT_LONGS_EQUAL(3,writtenData.number_cameras());
  EXPECT_LONGS_EQUAL(7,writtenData.number_tracks());
  const SfmTrack& track0 = writtenData.tracks[0];
  EXPECT_LONGS_EQUAL(3,track0.number_measurements());

  // Check projection of a given point
  EXPECT_LONGS_EQUAL(0,track0.measurements[0].first);
  const SfmCamera& camera0 = writtenData.cameras[0];
  Point2 expected = camera0.project(track0.p), actual = track0.measurements[0].second;
  EXPECT(assert_equal(expected,actual,12));

  Pose3 expectedPose = camera0.pose();
  Pose3 actualPose = value.at<Pose3>(X(0));
  EXPECT(assert_equal(expectedPose,actualPose, 1e-7));

  Point3 expectedPoint = track0.p;
  Point3 actualPoint = value.at<Point3>(P(0));
  EXPECT(assert_equal(expectedPoint,actualPoint, 1e-6));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
