/**********************************************************
 * unit tests for EasyVectorConfig
 **********************************************************/
#include <CppUnitLite/TestHarness.h>

#include <fstream>
#include <iostream>

#include "EasySLAMConfig.h"

using namespace std;
using namespace gtsam;

TEST(EasyVectorConfig, plus) {
  EasyVectorConfig base;
  Rot3 tempRot = rodriguez(0.5, 0.1, -0.2);
  Point3 tempPoint(10, 20, -30);
  Pose3 pb(tempRot, tempPoint);
  base.insert("x0", pb.vector());

  EasyVectorConfig delta;
  Vector tempDelta(6);
  tempDelta(0) = -0.5;
  tempDelta(1) = -0.1;
  tempDelta(2) = 0.2;
  Matrix alaki = inverse(pb.matrix());
  tempDelta(3) = alaki(0, 3);
  tempDelta(4) = alaki(1, 3);
  tempDelta(5) = alaki(2, 3);
  delta.insert("x0", tempDelta);

  base += delta;
  //  delta.print("delta");
  //   base.print("base");

  Vector expected(12);
  expected(0) = 1;
  expected(4) = 1;
  expected(8) = 1;
  CHECK(assert_equal(base["x0"], expected,
                     0.0001));  // currently fails - translation in wrong place
}
/* ************************************************************************* */

// already the load function has been tested in testARRobotMarker
int main() {
  TestResult tr;
  TestRegistry::runAllTests(tr);
  return 0;
}
/* ************************************************************************* */
