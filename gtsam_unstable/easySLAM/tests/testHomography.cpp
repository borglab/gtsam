/**********************************************************
Written by Alireza Fathi, 17th of Nov 2008
**********************************************************/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/Cal3_S2.h>
#include <time.h>

#include "homography.h"
#include "utility.h"

using namespace std;
using namespace gtsam;

const Cal3_S2 K;

const double u = 80, v = 80;
const Matrix m = Matrix_(4, 2, 10 - u, 20 - v, 10 - u, 20 + v, 10 + u, 20 + v,
                         10 + u, 20 - v);

const double Msize = 40;
const Matrix M = Matrix_(4, 3, -Msize, -Msize, 1.0, -Msize, +Msize, 1.0, +Msize,
                         +Msize, 1.0, +Msize, -Msize, 1.0);

/* ************************************************************************* */
TEST(homography, getH) {
  Matrix Actual = getH(m, M);
  Matrix Expected =
      Matrix_(3, 3, 2.0, 0.0, 10.0, 0.0, 2.0, 20.0, 0.0, 0.0, 1.0);

  EQUALITY(Actual / Actual(2, 2), Expected);
}
/* ************************************************************************* */
/*TEST ( homography, efficientGetH ) {
  Matrix Actual = efficientGetH(m,M);
  Matrix Expected = Matrix_(3,3,
                              2.0, 0.0, 10.0,
                              0.0, 2.0, 20.0,
                              0.0, 0.0,  1.0);

  EQUALITY(Actual/Actual(2,2), Expected);
}
*/
/* ************************************************************************* */
TEST(homography, transformation_is_a_valid_Pose3) {
  // artificial example
  Matrix H = getH(m, M);
  Pose3 pose = getTransformationFromMarkerToCamera(H, K.matrix());
  Matrix R = pose.rotation().matrix();
  DOUBLES_EQUAL(1, det3(R), 0.05);
  Pose3 posep = pose.inverse();
  CHECK(assert_equal(posep.matrix(), inverse(pose.matrix())));

  // real example
  Matrix mm = Matrix_(4, 2, 2893.13, 1525.59, 2985.68, 1163.55, 3287.4, 1032.58,
                      3171.27, 1377.99);
  H = getH(mm, M);
  pose = getTransformationFromMarkerToCamera(H, K.matrix());
  R = pose.rotation().matrix();
  DOUBLES_EQUAL(1, det3(R), 0.05);
  DOUBLES_EQUAL(0, R(0, 0) * R(0, 1) + R(1, 0) * R(1, 1) + R(2, 0) * R(2, 1),
                0.05);
  posep = pose.inverse();
  CHECK(assert_equal(posep.matrix(), inverse(pose.matrix())));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  TestRegistry::runAllTests(tr);
  return 0;
}
/* ************************************************************************* */
