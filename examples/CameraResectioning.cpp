/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file		CameraResectioning.cpp
 * @brief   An example of gtsam for solving the camera resectioning problem
 * @author	Duy-Nguyen Ta
 * @date	  Aug 23, 2011
 */

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimization.h>

using namespace gtsam;

/**
 * Unary factor for the pose.
 */
class ResectioningFactor: public NonlinearFactor1<Pose3> {
  typedef NonlinearFactor1<Pose3> Base;

  shared_ptrK K_; // camera's intrinsic parameters
  Point3 P_; // 3D point on the calibration rig
  Point2 p_; // 2D measurement of the 3D point

public:
  ResectioningFactor(const SharedNoiseModel& model, const Symbol& key,
      const shared_ptrK& calib, const Point2& p, const Point3& P) :
    Base(model, key), K_(calib), P_(P), p_(p) {
  }

  virtual ~ResectioningFactor() {}

  virtual Vector evaluateError(const Pose3& X, boost::optional<Matrix&> H = boost::none) const {
    SimpleCamera camera(*K_, X);
    Point2 reprojectionError(camera.project(P_, H) - p_);
    return reprojectionError.vector();
  }
};

/*******************************************************************************/
/**
 * Camera: f = 1.0, Image: 100x100, center: 50.0, 50.0
 * Pose (ground truth): (Xw, -Yw, -Zw, [0,0,2.0]')
 * Known landmarks:
 *    3D Points: (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
 * Perfect measurements:
 *    2D Point:  (55,45)   (45,45)    (45,55)     (55,55)
 */
int main(int argc, char* argv[]) {
  /* read camera intrinsic parameters */
  shared_ptrK calib(new Cal3_S2(1.0, 1.0, 0, 50.0, 50.0));

  /* create keys for variables */
  // we have only 1 variable to solve: the camera pose
  Symbol X('x',1);

  /* 1. create graph */
  NonlinearFactorGraph graph;

  /* 2. add factors to the graph */
  // add measurement factors
  SharedDiagonal measurementNoise = sharedSigmas(Vector_(2, 0.5, 0.5));
  boost::shared_ptr<ResectioningFactor> factor;
  factor = boost::shared_ptr<ResectioningFactor>(new ResectioningFactor(
      measurementNoise, X, calib, Point2(55.0, 45.0), Point3(10.0, 10.0, 0.0)));
  graph.push_back(factor);
  factor = boost::shared_ptr<ResectioningFactor>(new ResectioningFactor(
      measurementNoise, X, calib, Point2(45.0, 45.0), Point3(-10.0, 10.0, 0.0)));
  graph.push_back(factor);
  factor = boost::shared_ptr<ResectioningFactor>(new ResectioningFactor(
      measurementNoise, X, calib, Point2(45.0, 55.0), Point3(-10.0, -10.0, 0.0)));
  graph.push_back(factor);
  factor = boost::shared_ptr<ResectioningFactor>(new ResectioningFactor(
      measurementNoise, X, calib, Point2(55.0, 55.0), Point3(10.0, -10.0, 0.0)));
  graph.push_back(factor);

  /* 3. Create an initial estimate for the camera pose */
  Values initial;
  initial.insert(X, Pose3(Rot3(1.,0.,0.,
                               0.,-1.,0.,
                               0.,0.,-1.), Point3(0.,0.,2.0)));

  /* 4. Optimize the graph using Levenberg-Marquardt*/
  Values result = optimize<NonlinearFactorGraph> (graph, initial);
  result.print("Final result: ");

  return 0;
}
