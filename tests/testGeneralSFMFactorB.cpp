/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGeneralSFMFactorB.cpp
 * @author Frank Dellaert
 * @brief test general SFM class, with nonlinear optimization and BAL files
 */

#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <CppUnitLite/Failure.h>
#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestRegistry.h>
#include <CppUnitLite/TestResult.h>
#include <stddef.h>
#include <stdexcept>
#include <string>

using namespace std;
using namespace gtsam;

typedef GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3> sfmFactor;
using symbol_shorthand::P;

/* ************************************************************************* */
TEST(PinholeCamera, BAL) {
  string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData db = SfmData::FromBalFile(filename);

  SharedNoiseModel unit2 = noiseModel::Unit::Create(2);
  NonlinearFactorGraph graph;

  for (size_t j = 0; j < db.numberTracks(); j++) {
    for (const SfmMeasurement& m: db.tracks[j].measurements)
      graph.emplace_shared<sfmFactor>(m.second, unit2, m.first, P(j));
  }

  Values initial = initialCamerasAndPointsEstimate(db);

  LevenbergMarquardtOptimizer lm(graph, initial);

  Values actual = lm.optimize();
  double actualError = graph.error(actual);
  EXPECT_DOUBLES_EQUAL(0.0199833, actualError, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
