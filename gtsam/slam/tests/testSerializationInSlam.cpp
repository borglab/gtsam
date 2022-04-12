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

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/map.hpp>
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

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
