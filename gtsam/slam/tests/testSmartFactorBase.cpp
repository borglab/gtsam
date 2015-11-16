/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testSmartFactorBase.cpp
 *  @brief  Unit tests for testSmartFactorBase Class
 *  @author Frank Dellaert
 *  @date   Feb 2015
 */

#include <gtsam/slam/SmartFactorBase.h>
#include <gtsam/geometry/Pose3.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static SharedNoiseModel unit2(noiseModel::Unit::Create(2));
static SharedNoiseModel unit3(noiseModel::Unit::Create(3));

/* ************************************************************************* */
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>
class PinholeFactor: public SmartFactorBase<PinholeCamera<Cal3Bundler> > {
public:
  typedef SmartFactorBase<PinholeCamera<Cal3Bundler> > Base;
  PinholeFactor(const SharedNoiseModel& sharedNoiseModel): Base(sharedNoiseModel) {
  }
  virtual double error(const Values& values) const {
    return 0.0;
  }
  virtual boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const {
    return boost::shared_ptr<GaussianFactor>(new JacobianFactor());
  }
};

TEST(SmartFactorBase, Pinhole) {
  PinholeFactor f= PinholeFactor(unit2);
  f.add(Point2(), 1);
  f.add(Point2(), 2);
  EXPECT_LONGS_EQUAL(2 * 2, f.dim());
}

/* ************************************************************************* */
#include <gtsam/geometry/StereoCamera.h>
class StereoFactor: public SmartFactorBase<StereoCamera> {
public:
  typedef SmartFactorBase<StereoCamera> Base;
  StereoFactor(const SharedNoiseModel& sharedNoiseModel): Base(sharedNoiseModel) {
  }
  virtual double error(const Values& values) const {
    return 0.0;
  }
  virtual boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const {
    return boost::shared_ptr<GaussianFactor>(new JacobianFactor());
  }
};

TEST(SmartFactorBase, Stereo) {
  StereoFactor f(unit3);
  f.add(StereoPoint2(), 1);
  f.add(StereoPoint2(), 2);
  EXPECT_LONGS_EQUAL(2 * 3, f.dim());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

