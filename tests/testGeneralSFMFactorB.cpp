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
#include <gtsam/slam/expressions.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
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
  ExpressionFactorGraph graph;

  for (size_t j = 0; j < db.numberTracks(); j++) {
    for (const SfmMeasurement& m: db.tracks[j].measurements)
      graph.emplace_shared<sfmFactor>(m.second, unit2, m.first, P(j));
  }

  // Fix exactly the seven-dimensional similarity gauge without constraining
  // the cameras' estimated calibration parameters.
  Expression<SfmCamera> camera0(0);
  Pose3_ pose0(&SfmCamera::getPose, camera0);
  graph.addExpressionFactor(pose0, db.cameras[0].pose(),
                            noiseModel::Constrained::All(6));
  graph.emplace_shared<RangeFactor<SfmCamera>>(
      0, 1, db.cameras[0].range(db.cameras[1]),
      noiseModel::Constrained::All(1));

  Values initial = initialCamerasAndPointsEstimate(db);

  LevenbergMarquardtOptimizer lm(graph, initial);

  Values actual = lm.optimize();
  double actualError = graph.error(actual);
  EXPECT_DOUBLES_EQUAL(0.020029, actualError, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
