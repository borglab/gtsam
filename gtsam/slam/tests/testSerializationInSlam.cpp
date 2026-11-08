/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSerializationSlam.cpp
 *  @brief all serialization tests in this directory
 *  @author Frank Dellaert
 *  @date   February 2022
 */

#include "smartFactorScenarios.h"
#include "PinholeFactor.h"

#include <gtsam/slam/SmartProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/base/std_optional_serialization.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

namespace {
static const double rankTol = 1.0;
static const double sigma = 0.1;
static SharedIsotropic model(noiseModel::Isotropic::Sigma(2, sigma));
}  // namespace

/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(noiseModel::Gaussian, "gtsam_noiseModel_Gaussian")
BOOST_CLASS_EXPORT_GUID(noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(noiseModel::Isotropic, "gtsam_noiseModel_Isotropic")
BOOST_CLASS_EXPORT_GUID(SharedNoiseModel, "gtsam_SharedNoiseModel")
BOOST_CLASS_EXPORT_GUID(SharedDiagonal, "gtsam_SharedDiagonal")

/* ************************************************************************* */
TEST(SmartFactorBase, serialize) {
  using namespace serializationTestHelpers;
  PinholeFactor factor(unit2);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
TEST(SerializationSlam, SmartProjectionFactor) {
  using namespace vanilla;
  using namespace serializationTestHelpers;
  SmartFactor factor(unit2);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
TEST(SerializationSlam, SmartProjectionPoseFactor) {
  using namespace vanillaPose;
  using namespace serializationTestHelpers;
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  SmartFactor factor(model, sharedK, params);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

TEST(SerializationSlam, SmartProjectionPoseFactor2) {
  using namespace vanillaPose;
  using namespace serializationTestHelpers;
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  Pose3 bts;
  SmartFactor factor(model, sharedK, bts, params);

  // insert some measurments
  KeyVector key_view;
  Point2Vector meas_view;
  key_view.push_back(Symbol('x', 1));
  meas_view.push_back(Point2(10, 10));
  factor.add(meas_view, key_view);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

// A factor with a fixed camera round-trips with that camera.
TEST(SerializationSlam, SmartProjectionPoseFactorFixedCamera) {
  using namespace vanillaPose;
  using namespace serializationTestHelpers;
  const Symbol x1('x', 1), x2('x', 2), x3('x', 3);
  SmartFactor factor(model, sharedK);
  factor.add(Camera(level_pose, sharedK).project(landmark1), x1);
  factor.add(Camera(pose_right, sharedK).project(landmark1), x2);
  factor.add(Camera(pose_above, sharedK).project(landmark1), x3);
  Values values;
  values.insert(x1, level_pose);
  values.insert(x2, pose_right);
  values.insert(x3, pose_above);
  const SmartFactor fixed = *factor.fixCamera(x1, values);

  EXPECT(equalsObj(fixed));
  EXPECT(equalsXML(fixed));
  EXPECT(equalsBinary(fixed));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
