/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMExample_SmartFactorPCG.cpp
 * @brief   Version of SFMExample_SmartFactor that uses Preconditioned Conjugate Gradient
 * @author  Frank Dellaert
 */

// For an explanation of these headers, see SFMExample_SmartFactor.cpp
#include "SFMdata.h"
#include <gtsam/slam/SmartProjectionPoseFactor.h>

// These extra headers allow us a LM outer loop with PCG linear solver (inner loop)
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/PCGSolver.h>

using namespace std;
using namespace gtsam;

// Make the typename short so it looks much cleaner
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

// create a typedef to the camera type
typedef PinholePose<Cal3_S2> Camera;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  auto measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  // Create the set of ground-truth landmarks and poses
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Simulated measurements from each camera pose, adding them to the factor graph
  for (size_t j = 0; j < points.size(); ++j) {
    // every landmark represent a single landmark, we use shared pointer to init
    // the factor, and then insert measurements.
    SmartFactor::shared_ptr smartfactor(new SmartFactor(measurementNoise, K));

    for (size_t i = 0; i < poses.size(); ++i) {
      // generate the 2D measurement
      Camera camera(poses[i], K);
      Point2 measurement = camera.project(points[j]);

      // call add() function to add measurement into a single factor
      smartfactor->add(measurement, i);
    }

    // insert the smart factor in the graph
    graph.push_back(smartfactor);
  }

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  auto noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  graph.addPrior(0, poses[0], noise);

  // Fix the scale ambiguity by adding a prior
  graph.addPrior(1, poses[0], noise);

  // Create the initial estimate to the solution
  Values initialEstimate;
  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(i, poses[i].compose(delta));

  // We will use LM in the outer optimization loop, but by specifying
  // "Iterative" below We indicate that an iterative linear solver should be
  // used. In addition, the *type* of the iterativeParams decides on the type of
  // iterative solver, in this case the SPCG (subgraph PCG)
  LevenbergMarquardtParams parameters;
  parameters.linearSolverType = NonlinearOptimizerParams::Iterative;
  parameters.absoluteErrorTol = 1e-10;
  parameters.relativeErrorTol = 1e-10;
  parameters.maxIterations = 500;
  PCGSolverParameters::shared_ptr pcg =
      boost::make_shared<PCGSolverParameters>();
  pcg->preconditioner_ =
      boost::make_shared<BlockJacobiPreconditionerParameters>();
  // Following is crucial:
  pcg->setEpsilon_abs(1e-10);
  pcg->setEpsilon_rel(1e-10);
  parameters.iterativeParams = pcg;

  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);
  Values result = optimizer.optimize();

  // Display result as in SFMExample_SmartFactor.run
  result.print("Final results:\n");
  Values landmark_result;
  for (size_t j = 0; j < points.size(); ++j) {
    auto smart = boost::dynamic_pointer_cast<SmartFactor>(graph[j]);
    if (smart) {
      std::optional<Point3> point = smart->point(result);
      if (point)  // ignore if std::optional return nullptr
        landmark_result.insert(j, *point);
    }
  }

  landmark_result.print("Landmark results:\n");
  cout << "final error: " << graph.error(result) << endl;
  cout << "number of iterations: " << optimizer.iterations() << endl;

  return 0;
}
/* ************************************************************************* */

