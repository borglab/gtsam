/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeFrobeniusFactor.cpp
 * @brief   time FrobeniusFactor with BAL file
 * @author  Frank Dellaert
 * @date    June 6, 2015
 */

#include <gtsam/base/timing.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;

static SharedNoiseModel gNoiseModel = noiseModel::Unit::Create(2);

int main(int argc, char* argv[]) {
  // primitive argument parsing:
  if (argc > 3) {
    throw runtime_error("Usage: timeFrobeniusFactor [g2oFile]");
  }

  string g2oFile;
  try {
    if (argc > 1)
      g2oFile = argv[argc - 1];
    else
      g2oFile =
          "/Users/dellaert/git/2019c-notes-shonanrotationaveraging/matlabCode/"
          "datasets/randomTorus3D.g2o";
    // g2oFile = findExampleDataFile("sphere_smallnoise.graph");
  } catch (const exception& e) {
    cerr << e.what() << '\n';
    exit(1);
  }

  // Read G2O file
  const auto factors = parse3DFactors(g2oFile);
  const auto poses = parse3DPoses(g2oFile);

  // Build graph
  NonlinearFactorGraph graph;
  // graph.add(NonlinearEquality<SO4>(0, SO4()));
  auto priorModel = noiseModel::Isotropic::Sigma(6, 10000);
  graph.add(PriorFactor<SO4>(0, SO4(), priorModel));
  for (const auto& factor : factors) {
    const auto& keys = factor->keys();
    const auto& Tij = factor->measured();
    const auto& model = factor->noiseModel();
    graph.emplace_shared<FrobeniusWormholeFactor>(
        keys[0], keys[1], SO3(Tij.rotation().matrix()), model);
  }

  boost::mt19937 rng(42);

  // Set parameters to be similar to ceres
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setLinearSolverType("MULTIFRONTAL_QR");
  // params.setVerbosityLM("SUMMARY");
  // params.linearSolverType = LevenbergMarquardtParams::Iterative;
  // auto pcg = boost::make_shared<PCGSolverParameters>();
  // pcg->preconditioner_ =
  // boost::make_shared<SubgraphPreconditionerParameters>();
  // boost::make_shared<BlockJacobiPreconditionerParameters>();
  // params.iterativeParams = pcg;

  // Optimize
  for (size_t i = 0; i < 100; i++) {
    gttic_(optimize);
    Values initial;
    initial.insert(0, SO4());
    for (size_t j = 1; j < poses.size(); j++) {
      initial.insert(j, SO4::Random(rng));
    }
    LevenbergMarquardtOptimizer lm(graph, initial, params);
    Values result = lm.optimize();
    cout << "cost = " << graph.error(result) << endl;
  }

  tictoc_finishedIteration_();
  tictoc_print_();

  return 0;
}
