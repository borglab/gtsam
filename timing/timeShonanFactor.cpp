/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeShonanFactor.cpp
 * @brief   time ShonanFactor with BAL file
 * @author  Frank Dellaert
 * @date    2019
 */

#include <gtsam/base/timing.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/sfm/ShonanFactor.h>

#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;

static SharedNoiseModel gNoiseModel = noiseModel::Unit::Create(2);

int main(int argc, char* argv[]) {
  // primitive argument parsing:
  if (argc > 3) {
    throw runtime_error("Usage: timeShonanFactor [g2oFile]");
  }

  string g2oFile;
  try {
    if (argc > 1)
      g2oFile = argv[argc - 1];
    else
      g2oFile = findExampleDataFile("sphere_smallnoise.graph");
  } catch (const exception& e) {
    cerr << e.what() << '\n';
    exit(1);
  }

  // Read G2O file
  const auto measurements = parseMeasurements<Rot3>(g2oFile);
  const auto poses = parseVariables<Pose3>(g2oFile);

  // Build graph
  NonlinearFactorGraph graph;
  // graph.add(NonlinearEquality<SOn>(0, SOn::Identity(4)));
  auto priorModel = noiseModel::Isotropic::Sigma(6, 10000);
  graph.add(PriorFactor<SOn>(0, SOn::Identity(4), priorModel));
  auto G = std::make_shared<Matrix>(SOn::VectorizedGenerators(4));
  for (const auto &m : measurements) {
    const auto &keys = m.keys();
    const Rot3 &Rij = m.measured();
    const auto &model = m.noiseModel();
    graph.emplace_shared<ShonanFactor3>(
        keys[0], keys[1], Rij, 4, model, G);
  }

  std::mt19937 rng(42);

  // Set parameters to be similar to ceres
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.setLinearSolverType("MULTIFRONTAL_QR");
  // params.setVerbosityLM("SUMMARY");
  // params.linearSolverType = LevenbergMarquardtParams::Iterative;
  // auto pcg = std::make_shared<PCGSolverParameters>();
  // pcg->preconditioner =
  // std::make_shared<SubgraphPreconditionerParameters>();
  // std::make_shared<BlockJacobiPreconditionerParameters>();
  // params.iterativeParams = pcg;

  // Optimize
  for (size_t i = 0; i < 100; i++) {
    gttic_(optimize);
    Values initial;
    initial.insert(0, SOn::Identity(4));
    for (size_t j = 1; j < poses.size(); j++) {
      initial.insert(j, SOn::Random(rng, 4));
    }
    LevenbergMarquardtOptimizer lm(graph, initial, params);
    Values result = lm.optimize();
    cout << "cost = " << graph.error(result) << endl;
  }

  tictoc_finishedIteration_();
  tictoc_print_();

  return 0;
}
