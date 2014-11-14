/**
 * @file   testPriorFactor.cpp
 * @brief  Test PriorFactor
 * @author Frank Dellaert
 * @date   Nov 4, 2014
 */

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/LieScalar.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

// Constructor
TEST(PriorFactor, Constructor) {
  SharedNoiseModel model;
  PriorFactor<LieScalar> factor(1, LieScalar(1.0), model);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
