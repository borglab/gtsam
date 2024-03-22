/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDoglegOptimizer.cpp
 * @brief   Unit tests for DoglegOptimizer
 * @author  Richard Roberts
 * @author  Frank dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include <tests/smallExample.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizerImpl.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include "examples/SFMdata.h"

#include <functional>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;

/* ************************************************************************* */
TEST(DoglegOptimizer, ComputeBlend) {
  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn.emplace_shared<GaussianConditional>(
      0, Vector2(1.0,2.0), (Matrix(2, 2) << 3.0,4.0,0.0,6.0).finished(),
      3, (Matrix(2, 2) << 7.0,8.0,9.0,10.0).finished(),
      4, (Matrix(2, 2) << 11.0,12.0,13.0,14.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      1, Vector2(15.0,16.0), (Matrix(2, 2) << 17.0,18.0,0.0,20.0).finished(),
      2, (Matrix(2, 2) << 21.0,22.0,23.0,24.0).finished(),
      4, (Matrix(2, 2) << 25.0,26.0,27.0,28.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      2, Vector2(29.0,30.0), (Matrix(2, 2) << 31.0,32.0,0.0,34.0).finished(),
      3, (Matrix(2, 2) << 35.0,36.0,37.0,38.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      3, Vector2(39.0,40.0), (Matrix(2, 2) << 41.0,42.0,0.0,44.0).finished(),
      4, (Matrix(2, 2) << 45.0,46.0,47.0,48.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      4, Vector2(49.0,50.0), (Matrix(2, 2) << 51.0,52.0,0.0,54.0).finished());

  // Compute steepest descent point
  VectorValues xu = gbn.optimizeGradientSearch();

  // Compute Newton's method point
  VectorValues xn = gbn.optimize();

  // The Newton's method point should be more "adventurous", i.e. larger, than the steepest descent point
  EXPECT(xu.vector().norm() < xn.vector().norm());

  // Compute blend
  double Delta = 1.5;
  VectorValues xb = DoglegOptimizerImpl::ComputeBlend(Delta, xu, xn);
  DOUBLES_EQUAL(Delta, xb.vector().norm(), 1e-10);
}

/* ************************************************************************* */
TEST(DoglegOptimizer, ComputeDoglegPoint) {
  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn.emplace_shared<GaussianConditional>(
      0, Vector2(1.0,2.0), (Matrix(2, 2) << 3.0,4.0,0.0,6.0).finished(),
      3, (Matrix(2, 2) << 7.0,8.0,9.0,10.0).finished(),
      4, (Matrix(2, 2) << 11.0,12.0,13.0,14.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      1, Vector2(15.0,16.0), (Matrix(2, 2) << 17.0,18.0,0.0,20.0).finished(),
      2, (Matrix(2, 2) << 21.0,22.0,23.0,24.0).finished(),
      4, (Matrix(2, 2) << 25.0,26.0,27.0,28.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      2, Vector2(29.0,30.0), (Matrix(2, 2) << 31.0,32.0,0.0,34.0).finished(),
      3, (Matrix(2, 2) << 35.0,36.0,37.0,38.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      3, Vector2(39.0,40.0), (Matrix(2, 2) << 41.0,42.0,0.0,44.0).finished(),
      4, (Matrix(2, 2) << 45.0,46.0,47.0,48.0).finished());
  gbn.emplace_shared<GaussianConditional>(
      4, Vector2(49.0,50.0), (Matrix(2, 2) << 51.0,52.0,0.0,54.0).finished());

  // Compute dogleg point for different deltas

  double Delta1 = 0.5;  // Less than steepest descent
  VectorValues actual1 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta1, gbn.optimizeGradientSearch(), gbn.optimize());
  DOUBLES_EQUAL(Delta1, actual1.vector().norm(), 1e-5);

  double Delta2 = 1.5;  // Between steepest descent and Newton's method
  VectorValues expected2 = DoglegOptimizerImpl::ComputeBlend(Delta2, gbn.optimizeGradientSearch(), gbn.optimize());
  VectorValues actual2 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta2, gbn.optimizeGradientSearch(), gbn.optimize());
  DOUBLES_EQUAL(Delta2, actual2.vector().norm(), 1e-5);
  EXPECT(assert_equal(expected2, actual2));

  double Delta3 = 5.0;  // Larger than Newton's method point
  VectorValues expected3 = gbn.optimize();
  VectorValues actual3 = DoglegOptimizerImpl::ComputeDoglegPoint(Delta3, gbn.optimizeGradientSearch(), gbn.optimize());
  EXPECT(assert_equal(expected3, actual3));
}

/* ************************************************************************* */
TEST(DoglegOptimizer, Iterate) {
  // really non-linear factor graph
  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  // config far from minimum
  Point2 x0(3,0);
  Values config;
  config.insert(X(1), x0);

  double Delta = 1.0;
  for(size_t it=0; it<10; ++it) {
    auto linearized = fg.linearize(config);
    
    // Iterate assumes that linear error = nonlinear error at the linearization point, and this should be true
    double nonlinearError = fg.error(config);
    double linearError = linearized->error(config.zeroVectors());
    DOUBLES_EQUAL(nonlinearError, linearError, 1e-5);
    
    auto gbn = linearized->eliminateSequential();
    VectorValues dx_u = gbn->optimizeGradientSearch();
    VectorValues dx_n = gbn->optimize();
    DoglegOptimizerImpl::IterationResult result = DoglegOptimizerImpl::Iterate(
        Delta, DoglegOptimizerImpl::SEARCH_EACH_ITERATION, dx_u, dx_n, *gbn, fg,
        config, fg.error(config));
    Delta = result.delta;
    EXPECT(result.f_error < fg.error(config)); // Check that error decreases
    
    Values newConfig(config.retract(result.dx_d));
    config = newConfig;
    DOUBLES_EQUAL(fg.error(config), result.f_error, 1e-5); // Check that error is correctly filled in
  }
}

/* ************************************************************************* */
TEST(DoglegOptimizer, Constraint) {
  // Create a pose-graph graph with a constraint on the first pose
  NonlinearFactorGraph graph;
  const Pose2 origin(0, 0, 0), pose2(2, 0, 0);
  graph.emplace_shared<NonlinearEquality<Pose2> >(1, origin);
  auto model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, pose2, model);

  // Create feasible initial estimate
  Values initial;
  initial.insert(1, origin); // feasible !
  initial.insert(2, Pose2(2.3, 0.1, -0.2));

  // Optimize the initial values using DoglegOptimizer
  DoglegParams params;
  params.setVerbosityDL("VERBOSITY");
  DoglegOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();

  // Check result
  EXPECT(assert_equal(pose2, result.at<Pose2>(2)));

  // Create infeasible initial estimate
  Values infeasible;
  infeasible.insert(1, Pose2(0.1, 0, 0)); // infeasible !
  infeasible.insert(2, Pose2(2.3, 0.1, -0.2));

  // Try optimizing with infeasible initial estimate
  DoglegOptimizer optimizer2(graph, infeasible, params);

#ifdef GTSAM_USE_TBB
  CHECK_EXCEPTION(optimizer2.optimize(), std::exception);
#else
  CHECK_EXCEPTION(optimizer2.optimize(), std::invalid_argument);
#endif
}

/* ************************************************************************* */
TEST(DogLegOptimizer, VariableUpdate) {
  // Make the typename short so it looks much cleaner
  typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

  // create a typedef to the camera type
  typedef PinholePose<Cal3_S2> Camera;
  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  noiseModel::Isotropic::shared_ptr measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  // Create the set of ground-truth landmarks and poses
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph graph;

  ISAM2DoglegParams doglegparams = ISAM2DoglegParams();
  doglegparams.verbose = false;
  ISAM2Params isam2_params;
  isam2_params.evaluateNonlinearError = true;
  isam2_params.relinearizeThreshold = 0.0;
  isam2_params.enableRelinearization = true;
  isam2_params.optimizationParams = doglegparams;
  isam2_params.relinearizeSkip = 1;
  ISAM2 isam2(isam2_params);

  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  unordered_map<int, SmartFactor::shared_ptr> smart_factors;
  for (size_t j = 0; j < points.size(); ++j) {
    // every landmark represent a single landmark, we use shared pointer to init
    // the factor, and then insert measurements.
    SmartFactor::shared_ptr smartfactor(new SmartFactor(measurementNoise, K));

    for (size_t i = 0; i < poses.size(); ++i) {
      // generate the 2D measurement
      Camera camera(poses[i], K);
      Point2 measurement = camera.project(points[j]);

      // call add() function to add measurement into a single factor, here we
      // need to add:
      //    1. the 2D measurement
      //    2. the corresponding camera's key
      //    3. camera noise model
      //    4. camera calibration

      // add only first 3 measurements and update the later measurements
      // incrementally
      if (i < 3) smartfactor->add(measurement, i);
    }

    // insert the smart factor in the graph
    smart_factors[j] = smartfactor;
    graph.push_back(smartfactor);
  }

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  noiseModel::Diagonal::shared_ptr noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
  graph.emplace_shared<PriorFactor<Pose3> >(0, poses[0], noise);

  // Because the structure-from-motion problem has a scale ambiguity, the
  // problem is still under-constrained. Here we add a prior on the second pose
  // x1, so this will fix the scale by indicating the distance between x0 and
  // x1. Because these two are fixed, the rest of the poses will be also be
  // fixed.
  graph.emplace_shared<PriorFactor<Pose3> >(1, poses[1],
                                            noise);  // add directly to graph

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
  for (size_t i = 0; i < 3; ++i)
    initialEstimate.insert(i, poses[i].compose(delta));
  // initialEstimate.print("Initial Estimates:\n");

  // Optimize the graph and print results
  isam2.update(graph, initialEstimate);
  Values result = isam2.calculateEstimate();
  // result.print("Results:\n");

  // we add new measurements from this pose
  size_t pose_idx = 3;

  // Now update existing smart factors with new observations
  for (size_t j = 0; j < points.size(); ++j) {
    SmartFactor::shared_ptr smartfactor = smart_factors[j];

    // add the 4th measurement
    Camera camera(poses[pose_idx], K);
    Point2 measurement = camera.project(points[j]);
    smartfactor->add(measurement, pose_idx);
  }

  graph.resize(0);
  initialEstimate.clear();

  // update initial estimate for the new pose
  initialEstimate.insert(pose_idx, poses[pose_idx].compose(delta));

  // this should break the system
  isam2.update(graph, initialEstimate);
  result = isam2.calculateEstimate();
  EXPECT(std::find(result.keys().begin(), result.keys().end(), pose_idx) !=
         result.keys().end());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
