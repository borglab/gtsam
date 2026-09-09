/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSfmBalBenchmark.cpp
 * @brief Tests for shared BAL smart-factor and PCG timing support.
 * @author Frank Dellaert (using 5.6 Sol)
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/slam/dataset.h>

#include <cstddef>

#include "internal/SfmBalBenchmark.h"
#include "internal/SfmPcgBenchmark.h"

using namespace gtsam;
namespace bal = gtsam::timing::bal;

/* ************************************************************************* */
namespace smart_graph_tests {

// Verifies smart BAL graphs omit underconstrained tracks and point variables.
TEST(SfmBalBenchmark, BuildsCameraOnlySmartGraph) {
  const SfmData data =
      bal::loadDataset(findExampleDataFile("dubrovnik-3-7-pre"));
  bal::BalBenchmarkConfig config;
  const NonlinearFactorGraph graph = bal::buildSmartSfmGraph(data, config);
  const Values initial = bal::buildSmartSfmInitial(data);
  const Ordering ordering = bal::createCameraOrdering(data);

  size_t validTrackCount = 0;
  for (const SfmTrack& track : data.tracks) {
    if (track.measurements.size() >= 2) ++validTrackCount;
  }
  LONGS_EQUAL(validTrackCount, graph.size());
  LONGS_EQUAL(data.numberCameras(), initial.size());
  LONGS_EQUAL(data.numberCameras(), ordering.size());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    CHECK(initial.exists(symbol_shorthand::C(i)));
    CHECK(ordering[i] == symbol_shorthand::C(i));
  }
}

}  // namespace smart_graph_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace smart_pcg_tests {

bal::PcgOptimizationResult optimize(const SfmData& data,
                                    LinearizationMode mode) {
  bal::BalBenchmarkConfig config;
  NonlinearFactorGraph graph = bal::buildSmartSfmGraph(
      data, config, SmartProjectionParams(mode));
  // This tiny dataset is underconstrained even beyond the similarity gauge.
  // Soft priors on every camera's pose and calibration make the comparison
  // observable, so roundoff does not send the two modes along different paths.
  const auto cameraPriorNoise = noiseModel::Unit::Create(9);
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    graph.addPrior<bal::Camera>(symbol_shorthand::C(i), data.cameras[i],
                               cameraPriorNoise);
  }
  const Values initial = bal::buildSmartSfmInitial(data);
  const Ordering ordering = bal::createCameraOrdering(data);
  LevenbergMarquardtParams parameters =
      bal::makeLevenbergMarquardtParams(config, &ordering, "SILENT");
  parameters.linearSolverType = NonlinearOptimizerParams::Iterative;
  return bal::runParallelPcgOptimization(graph, initial, parameters);
}

// Verifies explicit-Hessian and implicit-Schur smart PCG reach matching errors.
TEST(SfmBalBenchmark, SmartPcgLinearizationsAgree) {
  const SfmData data =
      bal::loadDataset(findExampleDataFile("dubrovnik-3-7-pre"));
  const bal::PcgOptimizationResult hessian = optimize(data, HESSIAN);
  const bal::PcgOptimizationResult implicit = optimize(data, IMPLICIT_SCHUR);

  CHECK(hessian.finalError < hessian.initialError);
  CHECK(implicit.finalError < implicit.initialError);
  DOUBLES_EQUAL(hessian.initialError, implicit.initialError, 1e-9);
  DOUBLES_EQUAL(hessian.finalError, implicit.finalError, 1e-6);
  CHECK(hessian.linearSolves > 0);
  CHECK(implicit.linearSolves > 0);
  LONGS_EQUAL(0, hessian.nonConvergedLinearSolves);
  LONGS_EQUAL(0, implicit.nonConvergedLinearSolves);
}

}  // namespace smart_pcg_tests
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
