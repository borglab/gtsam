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
 * @author  Chris Beall
 */

#include <gtsam_unstable/nonlinear/ConcurrentIncrementalFilter.h>

#include <gtsam/base/timing.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>

#include <fstream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

typedef Pose2 Pose;

typedef NoiseModelFactor1<Pose> NM1;
typedef NoiseModelFactor2<Pose,Pose> NM2;
noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(Pose::Dim());

int main(int argc, char *argv[]) {

  const size_t steps = 5000000;

  cout << "Playing forward " << steps << " time steps..." << endl;


  // Create a Concurrent Incremental Filter
  ISAM2Params parameters;
  parameters.findUnusedFactorSlots = true;
  ConcurrentIncrementalFilter filter(parameters);

  // Times
  vector<double> times(steps);


  for(size_t step=0; step < steps; ++step) {

    Values newVariables;
    NonlinearFactorGraph newFactors;

    // Collect measurements and new variables for the current step
//    gttic_(Create_measurements);
    if(step == 0) {
      // Add prior
      newFactors.add(PriorFactor<Pose>(0, Pose(), noiseModel::Unit::Create(Pose::Dim())));
      newVariables.insert(0, Pose());
    } else {
      Vector eta = Vector::Random(Pose::Dim()) * 0.1;
      Pose2 between = Pose().retract(eta);
      // Add between
      newFactors.add(BetweenFactor<Pose>(step-1, step, between, model));
      newVariables.insert(step, filter.getISAM2().calculateEstimate<Pose>(step-1) * between);
    }

//    gttoc_(Create_measurements);

    FastList<Key> keysToMove;

    if (step >= 10) {
      keysToMove.push_back(step-10);
    }

    // Update iSAM2
//    gttic_(Update_ConcurrentFilter);
    std::clock_t start = std::clock();
    filter.update(newFactors, newVariables, keysToMove);
    double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
//    gttoc_(Update_ConcurrentFilter);

    filter.postsync();

//    tictoc_finishedIteration_();

//    tictoc_getNode(node, Update_ConcurrentFilter);
//    times[step] = node->time();
    times[step] = duration;

    if(step % 1000 == 0) {
      cout << "Step " << step << endl;
//      tictoc_print_();
    }
  }

//  tictoc_print_();

  // Write times
  ofstream timesFile("times.txt");
  BOOST_FOREACH(double t, times) {
    timesFile << t << "\n"; }

  return 0;
}
