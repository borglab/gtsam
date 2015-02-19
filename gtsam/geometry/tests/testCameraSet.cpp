/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testCameraSet.cpp
 *  @brief  Unit tests for testCameraSet Class
 *  @author Frank Dellaert
 *  @date   Feb 19, 2015
 */

#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/Pose3.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
class PinholeSet: public CameraSet<PinholeCamera<Cal3Bundler> > {
};

TEST(CameraSet, Pinhole) {
  PinholeSet f;
}

/* ************************************************************************* */
#include <gtsam/geometry/StereoCamera.h>
class StereoSet: public CameraSet<StereoCamera> {
};

TEST(CameraSet, Stereo) {
  StereoSet f;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

