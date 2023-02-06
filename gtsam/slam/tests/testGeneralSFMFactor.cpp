/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGeneralSFMFactor.cpp
 * @date Dec 27, 2010
 * @author nikai
 * @brief unit tests for GeneralSFMFactor
 */

#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Testable.h>

#include <memory>
#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <vector>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

typedef PinholeCamera<Cal3_S2> GeneralCamera;
typedef GeneralSFMFactor<GeneralCamera, Point3> Projection;
typedef NonlinearEquality<GeneralCamera> CameraConstraint;
typedef NonlinearEquality<Point3> Point3Constraint;

class Graph: public NonlinearFactorGraph {
public:
  void addMeasurement(int i, int j, const Point2& z,
      const SharedNoiseModel& model) {
    push_back(std::make_shared<Projection>(z, model, X(i), L(j)));
  }

  void addCameraConstraint(int j, const GeneralCamera& p) {
    std::shared_ptr<CameraConstraint> factor(new CameraConstraint(X(j), p));
    push_back(factor);
  }

  void addPoint3Constraint(int j, const Point3& p) {
    std::shared_ptr<Point3Constraint> factor(new Point3Constraint(L(j), p));
    push_back(factor);
  }

};

static double getGaussian() {
  double S, V1, V2;
  // Use Box-Muller method to create gauss noise from uniform noise
  do {
    double U1 = rand() / (double) (RAND_MAX);
    double U2 = rand() / (double) (RAND_MAX);
    V1 = 2 * U1 - 1; /* V1=[-1,1] */
    V2 = 2 * U2 - 1; /* V2=[-1,1] */
    S = V1 * V1 + V2 * V2;
  } while (S >= 1);
  return sqrt(-2.f * (double) log(S) / S) * V1;
}

static const SharedNoiseModel sigma1(noiseModel::Unit::Create(2));
static const double baseline = 5.;

/* ************************************************************************* */
static vector<Point3> genPoint3() {
  const double z = 5;
  vector<Point3> landmarks;
  landmarks.push_back(Point3(-1., -1., z));
  landmarks.push_back(Point3(-1., 1., z));
  landmarks.push_back(Point3(1., 1., z));
  landmarks.push_back(Point3(1., -1., z));
  landmarks.push_back(Point3(-1.5, -1.5, 1.5 * z));
  landmarks.push_back(Point3(-1.5, 1.5, 1.5 * z));
  landmarks.push_back(Point3(1.5, 1.5, 1.5 * z));
  landmarks.push_back(Point3(1.5, -1.5, 1.5 * z));
  landmarks.push_back(Point3(-2., -2., 2 * z));
  landmarks.push_back(Point3(-2., 2., 2 * z));
  landmarks.push_back(Point3(2., 2., 2 * z));
  landmarks.push_back(Point3(2., -2., 2 * z));
  return landmarks;
}

static vector<GeneralCamera> genCameraDefaultCalibration() {
  vector<GeneralCamera> X;
  X.push_back(GeneralCamera(Pose3(Rot3(), Point3(-baseline / 2., 0., 0.))));
  X.push_back(GeneralCamera(Pose3(Rot3(), Point3(baseline / 2., 0., 0.))));
  return X;
}

static vector<GeneralCamera> genCameraVariableCalibration() {
  const Cal3_S2 K(640, 480, 0.1, 320, 240);
  vector<GeneralCamera> X;
  X.push_back(GeneralCamera(Pose3(Rot3(), Point3(-baseline / 2., 0., 0.)), K));
  X.push_back(GeneralCamera(Pose3(Rot3(), Point3(baseline / 2., 0., 0.)), K));
  return X;
}

static std::shared_ptr<Ordering> getOrdering(
    const vector<GeneralCamera>& cameras, const vector<Point3>& landmarks) {
  std::shared_ptr<Ordering> ordering(new Ordering);
  for (size_t i = 0; i < landmarks.size(); ++i)
    ordering->push_back(L(i));
  for (size_t i = 0; i < cameras.size(); ++i)
    ordering->push_back(X(i));
  return ordering;
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, equals ) {
  // Create two identical factors and make sure they're equal
  Point2 z(323., 240.);
  const Symbol cameraFrameNumber('x', 1), landmarkNumber('l', 1);
  const SharedNoiseModel sigma(noiseModel::Unit::Create(1));
  std::shared_ptr<Projection> factor1(
      new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

  std::shared_ptr<Projection> factor2(
      new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

  EXPECT(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, error ) {
  Point2 z(3., 0.);
  const SharedNoiseModel sigma(noiseModel::Unit::Create(2));
  Projection factor(z, sigma, X(1), L(1));
  // For the following configuration, the factor predicts 320,240
  Values values;
  Rot3 R;
  Point3 t1(0, 0, -6);
  Pose3 x1(R, t1);
  values.insert(X(1), GeneralCamera(x1));
  Point3 l1(0,0,0);
  values.insert(L(1), l1);
  EXPECT(
      assert_equal(((Vector ) Vector2(-3., 0.)),
          factor.unwhitenedError(values)));
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_defaultK ) {

  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraDefaultCalibration();

  // add measurement with noise
  Graph graph;
  for (size_t j = 0; j < cameras.size(); ++j) {
    for (size_t i = 0; i < landmarks.size(); ++i) {
      Point2 pt = cameras[j].project(landmarks[i]);
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = cameras.size() * landmarks.size();

  // add initial
  const double noise = baseline * 0.1;
  Values values;
  for (size_t i = 0; i < cameras.size(); ++i)
    values.insert(X(i), cameras[i]);

  for (size_t i = 0; i < landmarks.size(); ++i) {
    Point3 pt(landmarks[i].x() + noise * getGaussian(),
        landmarks[i].y() + noise * getGaussian(),
        landmarks[i].z() + noise * getGaussian());
    values.insert(L(i), pt);
  }

  graph.addCameraConstraint(0, cameras[0]);

  // Create an ordering of the variables
  Ordering ordering = *getOrdering(cameras, landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * 1e-5 * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_SingleMeasurementError ) {
  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();
  // add measurement with noise
  Graph graph;
  for (size_t j = 0; j < cameras.size(); ++j) {
    for (size_t i = 0; i < landmarks.size(); ++i) {
      Point2 pt = cameras[j].project(landmarks[i]);
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = cameras.size() * landmarks.size();

  // add initial
  const double noise = baseline * 0.1;
  Values values;
  for (size_t i = 0; i < cameras.size(); ++i)
    values.insert(X(i), cameras[i]);

  // add noise only to the first landmark
  for (size_t i = 0; i < landmarks.size(); ++i) {
    if (i == 0) {
      Point3 pt(landmarks[i].x() + noise * getGaussian(),
          landmarks[i].y() + noise * getGaussian(),
          landmarks[i].z() + noise * getGaussian());
      values.insert(L(i), pt);
    } else {
      values.insert(L(i), landmarks[i]);
    }
  }

  graph.addCameraConstraint(0, cameras[0]);
  const double reproj_error = 1e-5;

  Ordering ordering = *getOrdering(cameras, landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_FixCameras ) {

  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();

  // add measurement with noise
  const double noise = baseline * 0.1;
  Graph graph;
  for (size_t i = 0; i < cameras.size(); ++i) {
    for (size_t j = 0; j < landmarks.size(); ++j) {
      Point2 z = cameras[i].project(landmarks[j]);
      graph.addMeasurement(i, j, z, sigma1);
    }
  }

  const size_t nMeasurements = landmarks.size() * cameras.size();

  Values values;
  for (size_t i = 0; i < cameras.size(); ++i)
    values.insert(X(i), cameras[i]);

  for (size_t j = 0; j < landmarks.size(); ++j) {
    Point3 pt(landmarks[j].x() + noise * getGaussian(),
        landmarks[j].y() + noise * getGaussian(),
        landmarks[j].z() + noise * getGaussian());
    values.insert(L(j), pt);
  }

  for (size_t i = 0; i < cameras.size(); ++i)
    graph.addCameraConstraint(i, cameras[i]);

  const double reproj_error = 1e-5;

  Ordering ordering = *getOrdering(cameras, landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_FixLandmarks ) {

  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();

  // add measurement with noise
  Graph graph;
  for (size_t j = 0; j < cameras.size(); ++j) {
    for (size_t i = 0; i < landmarks.size(); ++i) {
      Point2 pt = cameras[j].project(landmarks[i]);
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = landmarks.size() * cameras.size();

  Values values;
  for (size_t i = 0; i < cameras.size(); ++i) {
    const double rot_noise = 1e-5, trans_noise = 1e-3, focal_noise = 1,
        skew_noise = 1e-5;
    if (i == 0) {
      values.insert(X(i), cameras[i]);
    } else {

      Vector delta = (Vector(11) << rot_noise, rot_noise, rot_noise, // rotation
      trans_noise, trans_noise, trans_noise, // translation
      focal_noise, focal_noise, // f_x, f_y
      skew_noise, // s
      trans_noise, trans_noise // ux, uy
          ).finished();
      values.insert(X(i), cameras[i].retract(delta));
    }
  }

  for (size_t i = 0; i < landmarks.size(); ++i) {
    values.insert(L(i), landmarks[i]);
  }

  // fix X0 and all landmarks, allow only the cameras[1] to move
  graph.addCameraConstraint(0, cameras[0]);
  for (size_t i = 0; i < landmarks.size(); ++i)
    graph.addPoint3Constraint(i, landmarks[i]);

  const double reproj_error = 1e-5;

  Ordering ordering = *getOrdering(cameras, landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_BA ) {
  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();

  // add measurement with noise
  Graph graph;
  for (size_t j = 0; j < cameras.size(); ++j) {
    for (size_t i = 0; i < landmarks.size(); ++i) {
      Point2 pt = cameras[j].project(landmarks[i]);
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = cameras.size() * landmarks.size();

  // add initial
  const double noise = baseline * 0.1;
  Values values;
  for (size_t i = 0; i < cameras.size(); ++i)
    values.insert(X(i), cameras[i]);

  // add noise only to the first landmark
  for (size_t i = 0; i < landmarks.size(); ++i) {
    Point3 pt(landmarks[i].x() + noise * getGaussian(),
        landmarks[i].y() + noise * getGaussian(),
        landmarks[i].z() + noise * getGaussian());
    values.insert(L(i), pt);
  }

  // Constrain position of system with the first camera constrained to the origin
  graph.addCameraConstraint(0, cameras[0]);

  // Constrain the scale of the problem with a soft range factor of 1m between the cameras
  graph.emplace_shared<
      RangeFactor<GeneralCamera, GeneralCamera> >(X(0), X(1), 2.,
          noiseModel::Isotropic::Sigma(1, 10.));

  const double reproj_error = 1e-5;

  Ordering ordering = *getOrdering(cameras, landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST(GeneralSFMFactor, GeneralCameraPoseRange) {
  // Tests range factor between a GeneralCamera and a Pose3
  Graph graph;
  graph.addCameraConstraint(0, GeneralCamera());
  graph.emplace_shared<
      RangeFactor<GeneralCamera, Pose3> >(X(0), X(1), 2.,
          noiseModel::Isotropic::Sigma(1, 1.));
  graph.addPrior(X(1), Pose3(Rot3(), Point3(1., 0., 0.)),
          noiseModel::Isotropic::Sigma(6, 1.));

  Values init;
  init.insert(X(0), GeneralCamera());
  init.insert(X(1), Pose3(Rot3(), Point3(1., 1., 1.)));

  // The optimal value between the 2m range factor and 1m prior is 1.5m
  Values expected;
  expected.insert(X(0), GeneralCamera());
  expected.insert(X(1), Pose3(Rot3(), Point3(1.5, 0., 0.)));

  LevenbergMarquardtParams params;
  params.absoluteErrorTol = 1e-9;
  params.relativeErrorTol = 1e-9;
  Values actual = LevenbergMarquardtOptimizer(graph, init, params).optimize();

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST(GeneralSFMFactor, CalibratedCameraPoseRange) {
  // Tests range factor between a CalibratedCamera and a Pose3
  NonlinearFactorGraph graph;
  graph.addPrior(X(0), CalibratedCamera(),
          noiseModel::Isotropic::Sigma(6, 1.));
  graph.emplace_shared<
      RangeFactor<CalibratedCamera, Pose3> >(X(0), X(1), 2.,
          noiseModel::Isotropic::Sigma(1, 1.));
  graph.addPrior(X(1), Pose3(Rot3(), Point3(1., 0., 0.)),
          noiseModel::Isotropic::Sigma(6, 1.));

  Values init;
  init.insert(X(0), CalibratedCamera());
  init.insert(X(1), Pose3(Rot3(), Point3(1., 1., 1.)));

  Values expected;
  expected.insert(X(0),
      CalibratedCamera(Pose3(Rot3(), Point3(-0.333333333333, 0, 0))));
  expected.insert(X(1), Pose3(Rot3(), Point3(1.333333333333, 0, 0)));

  LevenbergMarquardtParams params;
  params.absoluteErrorTol = 1e-9;
  params.relativeErrorTol = 1e-9;
  Values actual = LevenbergMarquardtOptimizer(graph, init, params).optimize();

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
// Frank created these tests after switching to a custom LinearizedFactor
TEST(GeneralSFMFactor, BinaryJacobianFactor) {
  Point2 measurement(3., -1.);

  // Create Values
  Values values;
  Rot3 R;
  Point3 t1(0, 0, -6);
  Pose3 x1(R, t1);
  values.insert(X(1), GeneralCamera(x1));
  Point3 l1(0,0,0);
  values.insert(L(1), l1);

  vector<SharedNoiseModel> models;
  {
    // Create various noise-models to test all cases
    using namespace noiseModel;
    Rot2 R = Rot2::fromAngle(0.3);
    Matrix2 cov = R.matrix() * R.matrix().transpose();
    models = {SharedNoiseModel(), Unit::Create(2), Isotropic::Sigma(2, 0.5),
              Constrained::All(2), Gaussian::Covariance(cov)};
  }

  // Now loop over all these noise models
  for(SharedNoiseModel model: models) {
    Projection factor(measurement, model, X(1), L(1));

    // Test linearize
    GaussianFactor::shared_ptr expected = //
        factor.NoiseModelFactor::linearize(values);
    GaussianFactor::shared_ptr actual = factor.linearize(values);
    EXPECT(assert_equal(*expected, *actual, 1e-9));

    // Test methods that rely on updateHessian
    if (model && !model->isConstrained()) {
      // Construct HessianFactor from single JacobianFactor
      HessianFactor expectedHessian(*expected), actualHessian(*actual);
      EXPECT(assert_equal(expectedHessian, actualHessian, 1e-9));

      // Convert back
      JacobianFactor actualJacobian(actualHessian);
      // Note we do not expect the actualJacobian to match *expected
      // Just that they have the same information on the variable.
      EXPECT(
          assert_equal(expected->augmentedInformation(),
              actualJacobian.augmentedInformation(), 1e-9));

      // Construct from GaussianFactorGraph
      GaussianFactorGraph gfg1 {expected}, gfg2 {actual};
      HessianFactor hessian1(gfg1), hessian2(gfg2);
      EXPECT(assert_equal(hessian1, hessian2, 1e-9));
    }
  }
}

/* ************************************************************************* */
// Do a thorough test of BinaryJacobianFactor
TEST( GeneralSFMFactor, BinaryJacobianFactor2 ) {

  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();

  Values values;
  for (size_t i = 0; i < cameras.size(); ++i)
    values.insert(X(i), cameras[i]);
  for (size_t j = 0; j < landmarks.size(); ++j)
    values.insert(L(j), landmarks[j]);

  for (size_t i = 0; i < cameras.size(); ++i) {
    for (size_t j = 0; j < landmarks.size(); ++j) {
      Point2 z = cameras[i].project(landmarks[j]);
      Projection::shared_ptr nonlinear = //
          std::make_shared<Projection>(z, sigma1, X(i), L(j));
      GaussianFactor::shared_ptr factor = nonlinear->linearize(values);
      HessianFactor hessian(*factor);
      JacobianFactor jacobian(hessian);
      EXPECT(
          assert_equal(factor->augmentedInformation(),
              jacobian.augmentedInformation(), 1e-9));
    }
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
