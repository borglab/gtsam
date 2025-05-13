/**
 * @file    testEasySLAMGraph.cpp
 * @brief   Unit tests for EasySLAMGraph class
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/Pose3.h>
#include <time.h>

#include "EasySLAMConfig.h"
#include "Marker.h"
#include "easyExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(EasySLAMGraph, error) {
  EasySLAMGraph fg = createExampleGraph();  // small EasySLAM Graph
  FGConfig config = createExampleConfig();  // ground truth config
  double actual = fg.error(config);
  double expected = 0;
  DOUBLES_EQUAL(expected, actual, 0.0000000000001);
}

/* ************************************************************************* */
TEST(EasySLAMGraph, load_easy_and_error) {
  int NUM_OF_FRAMES = 2;
  string path = "data/EasyExample";
  EasySLAMGraph nfg(path, NUM_OF_FRAMES);
  EasySLAMConfig Easyfgc(path, NUM_OF_FRAMES);
  // Easyfgc.print();
  FGConfig fgc = Easyfgc.getFGConfig();
  double actual = nfg.error(fgc);
  double expected = 0.0;
  // printf("%f %f\n", expected, actual);
  DOUBLES_EQUAL(expected, actual, 0.0000000000000001);
}

/**************************************************************/
TEST(EasySLAMGraph, easy_optimize) {
  int NUM_OF_FRAMES = 2;
  string path = "data/EasyExample";
  EasySLAMGraph nfg(path, NUM_OF_FRAMES);
  EasySLAMConfig Easyfgc;

  // marker 1 is reference marker, does not play

  // correct
  EasySLAMConfig Easyexpected;
  Pose3 camera1_expected(Matrix_(3, 3, 1., 0., 0., 0., -1., 0., 0., 0., -1.),
                         Point3(0, 0, 500));

  Pose3 camera2_expected(Matrix_(3, 3, 0., 1., 0., 1., 0., 0., 0., 0., -1.),
                         Point3(0, 0, 500));

  Easyexpected.addRobotPose(0, camera1_expected);
  Easyexpected.addRobotPose(1, camera2_expected);
  Pose3 mymarker_expected(
      Matrix_(3, 3, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
      Point3(0.0, 0.0, -500.0));
  Easyexpected.addMarkerPose(2, mymarker_expected);

  // start from incorrect
  Pose3 camera1_initial(Matrix_(3, 3, 1., 0., 0., 0., -1., 0., 0., 0., -1.),
                        Point3(0, 0, 400));

  Pose3 camera2_initial(
      rodriguez(0.2, 0.2, 0.2) *
          Rot3(Matrix_(3, 3, -1., 0., 0., 0., 1., 0., 0., 0., -1.)),
      Point3(0, 0, 500));

  Easyfgc.addRobotPose(0, camera1_initial);
  Easyfgc.addRobotPose(1, camera2_initial);
  Pose3 mymarker_initial(Matrix_(3, 3, sqrt(3.0) / 2.0, 0.0, 0.5, 0.0, 1.0, 0.0,
                                 -0.5, 0.0, sqrt(3.0) / 2.0),
                         Point3(10.0, 30.0, -511.0));
  Easyfgc.addMarkerPose(2, mymarker_initial);

  // optimize in place
  EasyVectorConfig fgc = Easyfgc.getFGConfig();
  Ordering ord = nfg.getOrdering(fgc);
  nfg.optimize(fgc, ord, 1e-20, 0.01, 0);

  double markerSize = 160.0;
  Cal3_S2 K(625.0, 625.0, 0.0, 0.0, 0.0);
  Pose3 rPose(fgc.get("x1"));
  Pose3 mPose;
  // nfg.print();
  // rPose.print();
  SimpleCamera camera(K, rPose);
  Marker markerObj(mPose, markerSize);
  // Point2 point = hGetP(camera, mPose, markerSize, 1);
  Point2 point = hGetP(camera, markerObj, 1);
  // point.print();

  FGConfig expectedConfig = Easyexpected.getFGConfig();
  CHECK(fgc.equals(expectedConfig, 1e-5));
}

/* ************************************************************************* */
TEST(EasySLAMGraph, load_and_error) {
  int NUM_OF_FRAMES = 1;
  string path = "data/CanonS3IS/2008_11_11_Alireza_B/Marker_Locations";
  EasySLAMGraph nfg(path, NUM_OF_FRAMES);
  EasySLAMConfig Easyfgc(path, NUM_OF_FRAMES);
  FGConfig fgc = Easyfgc.getFGConfig();
  // fgc.print();
  double actual = nfg.error(fgc);
  double expected = 1000;
  // printf("%f %f\n", actual, expected);
  CHECK(actual < expected);
}

/* ************************************************************************* */
TEST(EasySLAMGraph, build_the_factor_graph) {
  // load from 08/11/11 experiment
  string path = "data/CanonA560/2008_10_01_BorgLab/C2";
  int nrFrames = 11;
  EasySLAMGraph actual(path, nrFrames);
  // actual.print();
  // cout << "***********************************************************" <<
  // endl; actual.printOrdering(); cout <<
  // "***********************************************************" << endl;

  Cal3_S2 K;
  K.load(path);
  double markerSize = 84.0;

  // compare factor 4
  ARRobotMarker markerMeasurement(0, 4, 400.665, 594.952, 392.129, 478.886,
                                  514.821, 400.059, 512.507, 515.127,
                                  markerSize, K);
  CameraMarkerFactor factor(markerMeasurement, 1.0);
  CameraMarkerFactor::shared_ptr cmf =
      boost::static_pointer_cast<CameraMarkerFactor>(actual[1]);
  // cmf->print();
  //  CHECK(factor.equals(*cmf));
}

/* ************************************************************************* */
// linearize and eliminate one node
TEST(EasySLAMGraph, linearize_eliminate) {
  EasySLAMGraph fg = createExampleGraph();       // small EasySLAM Graph
  FGConfig config = createExampleConfig();       // ground truth config
  LinearFactorGraph lfg = fg.linearize(config);  // linearize
  // fg.print();
  lfg.eliminate_one("x0");
  lfg.eliminate_one("x1");
  // lfg.eliminate_one("m1");
  lfg.eliminate_one("m2");

  // after eliminating all, we have one remaining factor with error
  LinearFactorGraph expected;
  Vector b(14, 0.0);
  LinearFactor::shared_ptr lf(new LinearFactor(b));
  expected.push_back(lf);

  CHECK(lfg.equals(expected));
}

/* ************************************************************************* */
// linearize and build Bayes net
TEST(EasySLAMGraph, linearize_eliminate2) {
  EasySLAMGraph fg = createExampleGraph();       // small EasySLAM Graph
  FGConfig config = createExampleConfig();       // ground truth config
  LinearFactorGraph lfg = fg.linearize(config);  // linearize

  // Create an ordering in which to eliminate the graph
  Ordering ord;
  ord.push_back("x0");
  ord.push_back("x1");
  // ord.push_back("m1");
  ord.push_back("m2");

  // cout << "test3" << endl;
  ChordalBayesNet::shared_ptr bn = lfg.eliminate(ord);
  // bn.print();
  // unit test needed
}

/* ************************************************************************* */
TEST(EasySLAMGraph, iterate) {
  EasySLAMGraph fg = createExampleGraph();  // small EasySLAM Graph
  FGConfig config = createExampleConfig();  // ground truth config

  // fg.print();
  //  Create an ordering in which to eliminate the graph
  Ordering ord;
  ord.push_back("x0");
  ord.push_back("x1");
  // ord.push_back("m1");
  ord.push_back("m2");
  // iterate
  // printf("hello\n");
  FGConfig actual = fg.iterate(config, ord);
  // actual.print();
  //  Expected delta configuration is zero
  FGConfig expected;
  Vector dx0(6, 0.);
  expected.insert("x0", dx0);
  Vector dx1(6, 0.);
  expected.insert("x1", dx1);
  Vector dm2(6, 0.);
  expected.insert("m2", dm2);

  CHECK(actual.equals(expected, 0.1));
}

/* ************************************************************************* */
TEST(EasySLAMGraph, dump) {
  EasySLAMGraph fg = createExampleGraph();  // small EasySLAM Graph
  fg.dump(string("1.txt"));
  EasySLAMGraph newfg;
  newfg.load_dumped(string("1.txt"));
  CHECK(fg.equals(newfg));
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  TestRegistry::runAllTests(tr);
  return 0;
}
/* ************************************************************************* */
