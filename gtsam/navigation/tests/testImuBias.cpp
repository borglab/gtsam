/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInertialNavFactor.cpp
 * @brief   Unit test for the InertialNavFactor
 * @author  Vadim Indelman, Stephen Williams
 */

#include <gtsam/navigation/ImuBias.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;


/* ************************************************************************* */
TEST( ImuBias, Constructor)
{
  Vector bias_acc((Vector(3) << 0.1,0.2,0.4));
  Vector bias_gyro((Vector(3) << -0.2, 0.5, 0.03));

  // Default Constructor
  gtsam::imuBias::ConstantBias bias1;

  // Acc + Gyro Constructor
  gtsam::imuBias::ConstantBias bias2(bias_acc, bias_gyro);

  // Copy Constructor
  gtsam::imuBias::ConstantBias bias3(bias2);
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
