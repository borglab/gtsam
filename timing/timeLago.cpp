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
#include <gtsam/slam/lago.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/base/timing.h>

#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[]) {

  size_t trials = 1;

  // read graph
  Values::shared_ptr solution;
  NonlinearFactorGraph::shared_ptr g;
  string inputFile = findExampleDataFile("w10000");
  SharedDiagonal model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 5.0 * M_PI / 180.0).finished());
  boost::tie(g, solution) = load2D(inputFile, model);

  // add noise to create initial estimate
  Values initial;
  Sampler sampler(42u);
  Values::ConstFiltered<Pose2> poses = solution->filter<Pose2>();
  SharedDiagonal noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.5, 0.5, 15.0 * M_PI / 180.0).finished());
  for(const Values::ConstFiltered<Pose2>::KeyValuePair& it: poses)
    initial.insert(it.key, it.value.retract(sampler.sampleNewModel(noise)));

  // Add prior on the pose having index (key) = 0
  noiseModel::Diagonal::shared_ptr priorModel = //
      noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));
  g->addPrior(0, Pose2(), priorModel);

  // LAGO
  for (size_t i = 0; i < trials; i++) {
    {
      gttic_(lago);

      gttic_(init);
      Values lagoInitial = lago::initialize(*g);
      gttoc_(init);

      gttic_(refine);
      GaussNewtonOptimizer optimizer(*g, lagoInitial);
      Values result = optimizer.optimize();
      gttoc_(refine);
    }

    {
      gttic_(optimize);
      GaussNewtonOptimizer optimizer(*g, initial);
      Values result = optimizer.optimize();
    }

    tictoc_finishedIteration_();
  }

  tictoc_print_();

  return 0;
}
