/*
 * @file		CameraResectioning.cpp
 * @brief   An example of gtsam for solving the camera resectioning problem
 * @author	Duy-Nguyen Ta
 * @created	Aug 23, 2011
 */
#include <gtsam/nonlinear/Key.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/LieValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimization-inl.h>

using namespace gtsam;

typedef TypedSymbol<Pose3, 'x'> PoseKey;
typedef LieValues<PoseKey> PoseValues;

/**
 * Unary factor for the pose.
 */
class ResectioningFactor: public NonlinearFactor1<PoseValues, PoseKey> {
  typedef NonlinearFactor1<PoseValues, PoseKey> Base;

  shared_ptrK K_; // camera's intrinsic parameters
  Point3 P_; // 3D point on the calibration rig
  Point2 p_; // 2D measurement of the 3D point

public:
  ResectioningFactor(const SharedGaussian& model, const PoseKey& key,
      const shared_ptrK& calib, const Point2& p, const Point3& P) :
    Base(model, key), K_(calib), P_(P), p_(p) {
  }

  virtual Vector evaluateError(const Pose3& X, boost::optional<Matrix&> H =
      boost::none) const {
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
  PoseKey X(1);

  /* 1. create graph */
  NonlinearFactorGraph<PoseValues> graph;

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
  PoseValues initial;
  initial.insert(X, Pose3(Rot3(1.,0.,0.,
                               0.,-1.,0.,
                               0.,0.,-1.), Point3(0.,0.,2.0)));

  /* 4. Optimize the graph using Levenberg-Marquardt*/
  PoseValues result = optimize<NonlinearFactorGraph<PoseValues> , PoseValues> (
      graph, initial);
  result.print("Final result: ");

  return 0;
}
