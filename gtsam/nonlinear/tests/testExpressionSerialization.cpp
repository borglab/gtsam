/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExpressionSerialization.cpp
 * @brief Unit test for (de)serialization of expression factors
 * @author Jose Luis Blanco Claraco
 * @date July 25, 2020
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serializationTestHelpers.h>

/* -------------------------- Create GUIDs ----------------------------------*/
// clang-format off
// Export all classes derived from Value
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam::noiseModel::Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam::SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam::noiseModel::Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam::noiseModel::Unit");

GTSAM_VALUE_EXPORT(gtsam::Point3);
GTSAM_VALUE_EXPORT(gtsam::Rot3);

BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Point3>, "gtsam::ExpressionFactor<gtsam::Point3>");
// clang-format on
/* --------------------------   End GUIDs  ----------------------------------*/

static gtsam::Point3 pt0(1.0, 2.0, 3.0);
static gtsam::Point3 pt1(3.0, 4.0, 5.0);

TEST(Serialization, ExpressionFactor) {
  using namespace gtsam::serializationTestHelpers; // equalsXXX()
  using gtsam::symbol_shorthand::R;

  const auto noise = gtsam::noiseModel::Gaussian::Information(gtsam::I_3x3);

  const gtsam::Rot3_ r0 = gtsam::Rot3_(R(0));
  const gtsam::Point3_ ptRel = gtsam::rotate(r0, gtsam::Point3_(pt0));

  const auto f = gtsam::ExpressionFactor<gtsam::Point3>(noise, pt1, ptRel);

  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
