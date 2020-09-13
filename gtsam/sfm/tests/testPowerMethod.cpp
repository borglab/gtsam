/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * testPowerMethod.cpp
 *
 * @file   testPowerMethod.cpp
 * @date   Sept 2020
 * @author Jing Wu
 * @brief  Check eigenvalue and eigenvector computed by power method
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/slam/PowerMethod.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(PowerMethod, initialize) {
  gtsam::Sparse A;
  MatrixProdFunctor(A, 1, A.rows());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
