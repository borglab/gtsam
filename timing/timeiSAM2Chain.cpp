/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    timeiSAM2Chain.cpp
 * @brief   Times each iteration of a long chain in iSAM2, to measure asymptotic performance
 * @author  Richard Roberts
 */

#include <gtsam/base/timing.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>

#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/export.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

typedef Pose2 Pose;

typedef NoiseModelFactorN<Pose> NM1;
typedef NoiseModelFactorN<Pose,Pose> NM2;
noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(3);

int main(int argc, char *argv[]) {

  const size_t steps = 50000;

  cout << "Playing forward " << steps << " time steps..." << endl;

  ISAM2 isam2;

  // Times
  vector<double> times(steps);

  for(size_t step=0; step < steps; ++step) {

    Values newVariables;
    NonlinearFactorGraph newFactors;

    // Collect measurements and new variables for the current step
    gttic_(Create_measurements);
    if(step == 0) {
      // Add prior
      newFactors.addPrior(0, Pose(), noiseModel::Unit::Create(3));
      newVariables.insert(0, Pose());
    } else {
      Vector eta = Vector::Random(3) * 0.1;
      Pose2 between = Pose().retract(eta);
      // Add between
      newFactors.add(BetweenFactor<Pose>(step-1, step, between, model));
      newVariables.insert(step, isam2.calculateEstimate<Pose>(step-1) * between);
    }

    gttoc_(Create_measurements);

    // Update iSAM2
    gttic_(Update_ISAM2);
    isam2.update(newFactors, newVariables);
    gttoc_(Update_ISAM2);

    tictoc_finishedIteration_();

    tictoc_getNode(node, Update_ISAM2);
    times[step] = node->time();

    if(step % 1000 == 0) {
      cout << "Step " << step << endl;
      tictoc_print_();
    }
  }

  tictoc_print_();

  // Write times
  ofstream timesFile("times.txt");
  for(double t: times) {
    timesFile << t << "\n"; }

  return 0;
}
