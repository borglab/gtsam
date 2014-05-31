/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeVirtual.cpp
 * @brief   Time the overhead of using virtual destructors and methods
 * @author  Richard Roberts
 * @date    Dec 3, 2010
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/lago.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>

#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[]) {

  size_t trials = 1000;

  // read graph
  NonlinearFactorGraph g;
  Values initial;
  string inputFile = findExampleDataFile("noisyToyGraph");
  readG2o(inputFile, g, initial);

  // Add prior on the pose having index (key) = 0
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Sigmas(Vector3(0, 0, 0));
  g.add(PriorFactor<Pose2>(0, Pose2(), priorModel));

  // LAGO
  for (size_t i = 0; i < trials; i++) {
    {
      gttic_(lago);

      gttic_(init);
      Values lagoInitial = lago::initialize(g);
      gttoc_(init);

      gttic_(refine);
      GaussNewtonOptimizer optimizer(g, lagoInitial);
      Values result = optimizer.optimize();
      gttoc_(refine);
    }

    {
      gttic_(optimize);
      GaussNewtonOptimizer optimizer(g, initial);
      Values result = optimizer.optimize();
    }

    tictoc_finishedIteration_();
  }

  tictoc_print_();

  return 0;
}
