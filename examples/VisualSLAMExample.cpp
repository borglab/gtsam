/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualSLAMExample.cpp
 * @brief   A visualSLAM example for the structure-from-motion problem on a simulated dataset
 * @author  Duy-Nguyen Ta
 */

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include "VisualSLAMData.h"

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

	VisualSLAMExampleData data = VisualSLAMExampleData::generate();

  /* 1. Create graph *///using the 2D measurements (features) and the calibration data
  visualSLAM::Graph graph;

  /* 2. Add factors to the graph */
  // 2a. Measurement factors
  for (size_t i=0; i<data.poses.size(); ++i) {
  	for (size_t j=0; j<data.points.size(); ++j)
  		graph.addMeasurement(data.z[i][j], data.noiseZ, X(i), L(j), data.sK);
  }
  // 2b. Prior factor for the first pose and point to constraint the system
  graph.addPosePrior(X(0), data.poses[0], data.noiseX);
  graph.addPointPrior(L(0), data.points[0], data.noiseL);

  /* 3. Initial estimates for variable nodes, simulated by Gaussian noises */
  Values initial;
  for (size_t i=0; i<data.poses.size(); ++i)
  	initial.insert(X(i), data.poses[i]/* *Pose3::Expmap(data.noiseX->sample())*/); // you can add noise if you want
  for (size_t j=0; j<data.points.size(); ++j)
  	initial.insert(L(j), data.points[j] /*+ Point3(data.noiseL->sample())*/); // you can add noise if you want
  initial.print("Intial Estimates: ");

  /* 4. Optimize the graph and print results */
  visualSLAM::Values result = GaussNewtonOptimizer(graph, initial).optimize();
//  visualSLAM::Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final results: ");

  return 0;
}
/* ************************************************************************* */

