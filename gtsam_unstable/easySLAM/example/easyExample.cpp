/**
 * @file    easyExample.cpp
 * @brief   Create examples for unit testing
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#include <gtsam/Pose3.h>

#include "EasySLAMGraph.h"

using namespace gtsam;

typedef CameraMarkerFactor::shared_ptr shared_ptr;
typedef CameraMarkerFactor0::shared_ptr shared_ptr0;

/* ************************************************************************* */
shared_ptr0 createExampleFactor1() {
  Cal3_S2 K(625, 625, 0, 0, 0);
  double markerSize = 160.0;
  double u = 100, v = 100;  // measurements, in pixels
  double sigma = 2;         // standard deviation, in pixels
  ARRobotMarker markerMeasurement1(0, 1, -u, +v, -u, -v, +u, -v, +u, +v,
                                   markerSize, K);
  shared_ptr0 p(new CameraMarkerFactor0(markerMeasurement1, sigma));
  return p;
}

/* ************************************************************************* */
shared_ptr0 createExampleFactor2() {
  Cal3_S2 K(625, 625, 0, 0, 0);
  double markerSize = 160.0;
  double u = 100, v = 100;  // measurements, in pixels
  double sigma = 2;         // standard deviation, in pixels
  ARRobotMarker markerMeasurement2(1, 1, -u, -v, +u, -v, +u, +v, -u, +v,
                                   markerSize, K);
  shared_ptr0 p(new CameraMarkerFactor0(markerMeasurement2, sigma));
  return p;
}

/* ************************************************************************* */
shared_ptr createExampleFactor3() {
  Cal3_S2 K(625, 625, 0, 0, 0);
  double markerSize = 160.0;
  double u = 50, v = 50;  // measurements, in pixels
  double sigma = 2;       // standard deviation, in pixels
  ARRobotMarker markerMeasurement1(0, 2, -u, +v, -u, -v, +u, -v, +u, +v,
                                   markerSize, K);
  shared_ptr p(new CameraMarkerFactor(markerMeasurement1, sigma));
  return p;
}

/* ************************************************************************* */
shared_ptr createExampleFactor4() {
  Cal3_S2 K(625, 625, 0, 0, 0);
  double markerSize = 160.0;
  double u = 50, v = 50;  // measurements, in pixels
  double sigma = 2;       // standard deviation, in pixels
  ARRobotMarker markerMeasurement2(1, 2, -u, -v, +u, -v, +u, +v, -u, +v,
                                   markerSize, K);
  shared_ptr p(new CameraMarkerFactor(markerMeasurement2, sigma));
  return p;
}

/* ************************************************************************* */
EasySLAMGraph createExampleGraph() {
  // create graph and two measurement factors
  EasySLAMGraph graph;
  // graph.getOrdering().print();
  graph.push_back(createExampleFactor1());
  graph.push_back(createExampleFactor2());
  graph.push_back(createExampleFactor3());
  graph.push_back(createExampleFactor4());

  // create prior on marker
  /*Vector mu(6,0.);
  boost::shared_ptr<NonlinearFactor1> f(new NonlinearFactor1(mu, 0.00000000001,
  hPose, "m1", DhPose)); graph.push_back(f);*/
  return graph;
}

/* ************************************************************************* */

Pose3 marker1_local;

Pose3 marker2_local(eye(3, 3), Point3(0, 0, -500));

Pose3 camera1_local(Matrix_(3, 3, 1., 0., 0., 0., -1., 0., 0., 0., -1.),
                    Point3(0, 0, 500));

Pose3 camera2_local(Matrix_(3, 3, 0., 1., 0., 1., 0., 0., 0., 0., -1.),
                    Point3(0, 0, 500));

Pose3 marker1() { return marker1_local; }
Pose3 marker2() { return marker2_local; }
Pose3 camera1() { return camera1_local; }
Pose3 camera2() { return camera2_local; }

/* ************************************************************************* */
FGConfig createExampleConfig() {
  FGConfig config;
  // config.insert("m1",marker1_local.vector());
  config.insert("m2", marker2_local.vector());
  config.insert("x0", camera1_local.vector());
  config.insert("x1", camera2_local.vector());
  return config;
}
/* ************************************************************************* */

/**
 * @return 12D vector
 */
Vector prior_pose(const Vector& x) { return x; }

/**
 * @return derivative of pose identity function, which is a 6x6 identity matrix
 */
Matrix Dprior_pose(const Vector& x) {
  double H[] = {0, 0, 0,  0, 0, 0, 0,  0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0,

                0, 0, -1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0,  0, 0, 0, 0,

                0, 1, 0,  0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,

                0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 1, 0, 0, 0,  0, 0, 0, 1};

  return Matrix_(12, 6, H);
}

/* ************************************************************************* */

boost::shared_ptr<NonlinearFactor1> createPosePrior(const Pose3& pose,
                                                    double sigma,
                                                    const std::string& key1) {
  // pose.print();
  Vector z = pose.vector();

  // construct the factor
  boost::shared_ptr<NonlinearFactor1> ret(
      new NonlinearFactor1(z, sigma, prior_pose, key1, Dprior_pose));

  return ret;
}
