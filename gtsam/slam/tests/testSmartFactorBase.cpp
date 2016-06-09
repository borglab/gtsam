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
#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static SharedNoiseModel unit2(noiseModel::Unit::Create(2));
static SharedNoiseModel unit3(noiseModel::Unit::Create(3));

/* ************************************************************************* */
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>

namespace gtsam {

class PinholeFactor: public SmartFactorBase<PinholeCamera<Cal3Bundler> > {
public:
  typedef SmartFactorBase<PinholeCamera<Cal3Bundler> > Base;
  PinholeFactor() {}
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

/// traits
template<>
struct traits<PinholeFactor> : public Testable<PinholeFactor> {};
}

TEST(SmartFactorBase, Pinhole) {
  PinholeFactor f= PinholeFactor(unit2);
  f.add(Point2(0,0), 1);
  f.add(Point2(0,0), 2);
  EXPECT_LONGS_EQUAL(2 * 2, f.dim());
}

/* ************************************************************************* */
#include <gtsam/geometry/StereoCamera.h>

namespace gtsam {

class StereoFactor: public SmartFactorBase<StereoCamera> {
public:
  typedef SmartFactorBase<StereoCamera> Base;
  StereoFactor() {}
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

/// traits
template<>
struct traits<StereoFactor> : public Testable<StereoFactor> {};
}

TEST(SmartFactorBase, Stereo) {
  StereoFactor f(unit3);
  f.add(StereoPoint2(), 1);
  f.add(StereoPoint2(), 2);
  EXPECT_LONGS_EQUAL(2 * 3, f.dim());
}

/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

TEST(SmartFactorBase, serialize) {
  using namespace gtsam::serializationTestHelpers;
  PinholeFactor factor(unit2);

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

