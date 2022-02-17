/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationSLAM.cpp
 * @brief test serialization
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <CppUnitLite/TestHarness.h>

#include <tests/smallExample.h>

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/StereoFactor.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/serializationTestHelpers.h>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/export.hpp>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

// Creating as many permutations of factors as possible
typedef PriorFactor<Point2>                 PriorFactorPoint2;
typedef PriorFactor<StereoPoint2>           PriorFactorStereoPoint2;
typedef PriorFactor<Point3>                 PriorFactorPoint3;
typedef PriorFactor<Rot2>                   PriorFactorRot2;
typedef PriorFactor<Rot3>                   PriorFactorRot3;
typedef PriorFactor<Pose2>                  PriorFactorPose2;
typedef PriorFactor<Pose3>                  PriorFactorPose3;
typedef PriorFactor<Cal3_S2>                PriorFactorCal3_S2;
typedef PriorFactor<Cal3DS2>                PriorFactorCal3DS2;
typedef PriorFactor<CalibratedCamera>       PriorFactorCalibratedCamera;
typedef PriorFactor<PinholeCameraCal3_S2>   PriorFactorPinholeCameraCal3_S2;
typedef PriorFactor<StereoCamera>           PriorFactorStereoCamera;

typedef BetweenFactor<Point2>          BetweenFactorPoint2;
typedef BetweenFactor<Point3>          BetweenFactorPoint3;
typedef BetweenFactor<Rot2>            BetweenFactorRot2;
typedef BetweenFactor<Rot3>            BetweenFactorRot3;
typedef BetweenFactor<Pose2>           BetweenFactorPose2;
typedef BetweenFactor<Pose3>           BetweenFactorPose3;

typedef NonlinearEquality<Point2>                 NonlinearEqualityPoint2;
typedef NonlinearEquality<StereoPoint2>           NonlinearEqualityStereoPoint2;
typedef NonlinearEquality<Point3>                 NonlinearEqualityPoint3;
typedef NonlinearEquality<Rot2>                   NonlinearEqualityRot2;
typedef NonlinearEquality<Rot3>                   NonlinearEqualityRot3;
typedef NonlinearEquality<Pose2>                  NonlinearEqualityPose2;
typedef NonlinearEquality<Pose3>                  NonlinearEqualityPose3;
typedef NonlinearEquality<Cal3_S2>                NonlinearEqualityCal3_S2;
typedef NonlinearEquality<Cal3DS2>                NonlinearEqualityCal3DS2;
typedef NonlinearEquality<CalibratedCamera>       NonlinearEqualityCalibratedCamera;
typedef NonlinearEquality<PinholeCameraCal3_S2>   NonlinearEqualityPinholeCameraCal3_S2;
typedef NonlinearEquality<StereoCamera>           NonlinearEqualityStereoCamera;

typedef RangeFactor<Pose2, Point2>                              RangeFactor2D;
typedef RangeFactor<Pose3, Point3>                              RangeFactor3D;
typedef RangeFactor<Pose2, Pose2>                               RangeFactorPose2;
typedef RangeFactor<Pose3, Pose3>                               RangeFactorPose3;
typedef RangeFactor<CalibratedCamera, Point3>                   RangeFactorCalibratedCameraPoint;
typedef RangeFactor<PinholeCameraCal3_S2, Point3>               RangeFactorPinholeCameraCal3_S2Point;
typedef RangeFactor<CalibratedCamera, CalibratedCamera>         RangeFactorCalibratedCamera;
typedef RangeFactor<PinholeCameraCal3_S2, PinholeCameraCal3_S2> RangeFactorPinholeCameraCal3_S2;

typedef BearingRangeFactor<Pose2, Point2>  BearingRangeFactor2D;
typedef BearingRangeFactor<Pose3, Point3>  BearingRangeFactor3D;

typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> GenericProjectionFactorCal3_S2;
typedef GenericProjectionFactor<Pose3, Point3, Cal3DS2> GenericProjectionFactorCal3DS2;

typedef gtsam::GeneralSFMFactor<gtsam::PinholeCameraCal3_S2, gtsam::Point3> GeneralSFMFactorCal3_S2;
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCameraCal3DS2, gtsam::Point3> GeneralSFMFactorCal3DS2;

typedef gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> GeneralSFMFactor2Cal3_S2;

typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> GenericStereoFactor3D;


// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;


/* Create GUIDs for Noisemodels */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust")

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base , "gtsam_noiseModel_mEstimator_Base")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null , "gtsam_noiseModel_mEstimator_Null")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair , "gtsam_noiseModel_mEstimator_Fair")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsam_noiseModel_mEstimator_Huber")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey")

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel")
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal")

/* Create GUIDs for geometry */
/* ************************************************************************* */
GTSAM_VALUE_EXPORT(gtsam::Point2)
GTSAM_VALUE_EXPORT(gtsam::StereoPoint2)
GTSAM_VALUE_EXPORT(gtsam::Point3)
GTSAM_VALUE_EXPORT(gtsam::Rot2)
GTSAM_VALUE_EXPORT(gtsam::Rot3)
GTSAM_VALUE_EXPORT(gtsam::Pose2)
GTSAM_VALUE_EXPORT(gtsam::Pose3)
GTSAM_VALUE_EXPORT(gtsam::Cal3_S2)
GTSAM_VALUE_EXPORT(gtsam::Cal3DS2)
GTSAM_VALUE_EXPORT(gtsam::Cal3_S2Stereo)
GTSAM_VALUE_EXPORT(gtsam::CalibratedCamera)
GTSAM_VALUE_EXPORT(gtsam::PinholeCameraCal3_S2)
GTSAM_VALUE_EXPORT(gtsam::StereoCamera)

/* Create GUIDs for factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor")

BOOST_CLASS_EXPORT_GUID(PriorFactorPoint2, "gtsam::PriorFactorPoint2")
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoPoint2, "gtsam::PriorFactorStereoPoint2")
BOOST_CLASS_EXPORT_GUID(PriorFactorPoint3, "gtsam::PriorFactorPoint3")
BOOST_CLASS_EXPORT_GUID(PriorFactorRot2, "gtsam::PriorFactorRot2")
BOOST_CLASS_EXPORT_GUID(PriorFactorRot3, "gtsam::PriorFactorRot3")
BOOST_CLASS_EXPORT_GUID(PriorFactorPose2, "gtsam::PriorFactorPose2")
BOOST_CLASS_EXPORT_GUID(PriorFactorPose3, "gtsam::PriorFactorPose3")
BOOST_CLASS_EXPORT_GUID(PriorFactorCal3_S2, "gtsam::PriorFactorCal3_S2")
BOOST_CLASS_EXPORT_GUID(PriorFactorCal3DS2, "gtsam::PriorFactorCal3DS2")
BOOST_CLASS_EXPORT_GUID(PriorFactorCalibratedCamera, "gtsam::PriorFactorCalibratedCamera")
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoCamera, "gtsam::PriorFactorStereoCamera")

BOOST_CLASS_EXPORT_GUID(BetweenFactorPoint2, "gtsam::BetweenFactorPoint2")
BOOST_CLASS_EXPORT_GUID(BetweenFactorPoint3, "gtsam::BetweenFactorPoint3")
BOOST_CLASS_EXPORT_GUID(BetweenFactorRot2, "gtsam::BetweenFactorRot2")
BOOST_CLASS_EXPORT_GUID(BetweenFactorRot3, "gtsam::BetweenFactorRot3")
BOOST_CLASS_EXPORT_GUID(BetweenFactorPose2, "gtsam::BetweenFactorPose2")
BOOST_CLASS_EXPORT_GUID(BetweenFactorPose3, "gtsam::BetweenFactorPose3")

BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint2, "gtsam::NonlinearEqualityPoint2")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoPoint2, "gtsam::NonlinearEqualityStereoPoint2")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint3, "gtsam::NonlinearEqualityPoint3")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot2, "gtsam::NonlinearEqualityRot2")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot3, "gtsam::NonlinearEqualityRot3")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose2, "gtsam::NonlinearEqualityPose2")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose3, "gtsam::NonlinearEqualityPose3")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3_S2, "gtsam::NonlinearEqualityCal3_S2")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3DS2, "gtsam::NonlinearEqualityCal3DS2")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCalibratedCamera, "gtsam::NonlinearEqualityCalibratedCamera")
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoCamera, "gtsam::NonlinearEqualityStereoCamera")

BOOST_CLASS_EXPORT_GUID(RangeFactor2D, "gtsam::RangeFactor2D")
BOOST_CLASS_EXPORT_GUID(RangeFactor3D, "gtsam::RangeFactor3D")
BOOST_CLASS_EXPORT_GUID(RangeFactorPose2, "gtsam::RangeFactorPose2")
BOOST_CLASS_EXPORT_GUID(RangeFactorPose3, "gtsam::RangeFactorPose3")
BOOST_CLASS_EXPORT_GUID(RangeFactorCalibratedCameraPoint, "gtsam::RangeFactorCalibratedCameraPoint")
BOOST_CLASS_EXPORT_GUID(RangeFactorPinholeCameraCal3_S2Point, "gtsam::RangeFactorPinholeCameraCal3_S2Point")
BOOST_CLASS_EXPORT_GUID(RangeFactorCalibratedCamera, "gtsam::RangeFactorCalibratedCamera")
BOOST_CLASS_EXPORT_GUID(RangeFactorPinholeCameraCal3_S2, "gtsam::RangeFactorPinholeCameraCal3_S2")

BOOST_CLASS_EXPORT_GUID(BearingRangeFactor2D, "gtsam::BearingRangeFactor2D")

BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3_S2, "gtsam::GenericProjectionFactorCal3_S2")
BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3DS2, "gtsam::GenericProjectionFactorCal3DS2")

BOOST_CLASS_EXPORT_GUID(GeneralSFMFactorCal3_S2, "gtsam::GeneralSFMFactorCal3_S2")
BOOST_CLASS_EXPORT_GUID(GeneralSFMFactorCal3DS2, "gtsam::GeneralSFMFactorCal3DS2")

BOOST_CLASS_EXPORT_GUID(GeneralSFMFactor2Cal3_S2, "gtsam::GeneralSFMFactor2Cal3_S2")

BOOST_CLASS_EXPORT_GUID(GenericStereoFactor3D, "gtsam::GenericStereoFactor3D")


/* ************************************************************************* */
TEST (testSerializationSLAM, smallExample_linear) {
  using namespace example;

  Ordering ordering; ordering += X(1),X(2),L(1);
  EXPECT(equalsObj(ordering));
  EXPECT(equalsXML(ordering));
  EXPECT(equalsBinary(ordering));

  GaussianFactorGraph fg = createGaussianFactorGraph();
  EXPECT(equalsObj(fg));
  EXPECT(equalsXML(fg));
  EXPECT(equalsBinary(fg));

  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  EXPECT(equalsObj(cbn));
  EXPECT(equalsXML(cbn));
  EXPECT(equalsBinary(cbn));
}

/* ************************************************************************* */
TEST (testSerializationSLAM, gaussianISAM) {
  using namespace example;
  GaussianFactorGraph smoother = createSmoother(7);
  GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal();
  GaussianISAM isam(bayesTree);

  EXPECT(equalsObj(isam));
  EXPECT(equalsXML(isam));
  EXPECT(equalsBinary(isam));
}

/* ************************************************************************* */
/* Create GUIDs for factors in simulated2D */
BOOST_CLASS_EXPORT_GUID(simulated2D::Prior,       "gtsam::simulated2D::Prior"      )
BOOST_CLASS_EXPORT_GUID(simulated2D::Odometry,    "gtsam::simulated2D::Odometry"   )
BOOST_CLASS_EXPORT_GUID(simulated2D::Measurement, "gtsam::simulated2D::Measurement")

/* ************************************************************************* */
TEST (testSerializationSLAM, smallExample_nonlinear) {
  using namespace example;
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  Values c1 = createValues();
  EXPECT(equalsObj(nfg));
  EXPECT(equalsXML(nfg));
  EXPECT(equalsBinary(nfg));

  EXPECT(equalsObj(c1));
  EXPECT(equalsXML(c1));
  EXPECT(equalsBinary(c1));
}

/* ************************************************************************* */
TEST (testSerializationSLAM, factors) {

  Point2 point2(1.0, 2.0);
  StereoPoint2 stereoPoint2(1.0, 2.0, 3.0);
  Point3 point3(1.0, 2.0, 3.0);
  Rot2 rot2(1.0);
  Rot3 rot3(Rot3::RzRyRx(1.0, 2.0, 3.0));
  Pose2 pose2(rot2, point2);
  Pose3 pose3(rot3, point3);
  Cal3_S2 cal3_s2(1.0, 2.0, 3.0, 4.0, 5.0);
  Cal3DS2 cal3ds2(1.0, 2.0, 3.0, 4.0, 5.0,6.0, 7.0, 8.0, 9.0);
  Cal3_S2Stereo cal3_s2stereo(1.0, 2.0, 3.0, 4.0, 5.0, 1.0);
  CalibratedCamera calibratedCamera(pose3);
  PinholeCamera<Cal3_S2> simpleCamera(pose3, cal3_s2);
  StereoCamera stereoCamera(pose3, boost::make_shared<Cal3_S2Stereo>(cal3_s2stereo));


  Symbol  a01('a',1),  a02('a',2),  a03('a',3),  a04('a',4),  a05('a',5),
          a06('a',6),  a07('a',7),  a08('a',8),  a09('a',9),  a10('a',10),
          a11('a',11), a12('a',12), a13('a',13), a14('a',14), a15('a',15);
  Symbol  b01('b',1),  b02('b',2),  b03('b',3),  b04('b',4),  b05('b',5),
          b06('b',6),  b07('b',7),  b08('b',8),  b09('b',9),  b10('b',10),
          b11('b',11), b12('b',12), b13('b',13), b14('b',14), b15('b',15);

  Values values;
  values.insert(a03, point2);
  values.insert(a04, stereoPoint2);
  values.insert(a05, point3);
  values.insert(a06, rot2);
  values.insert(a07, rot3);
  values.insert(a08, pose2);
  values.insert(a09, pose3);
  values.insert(a10, cal3_s2);
  values.insert(a11, cal3ds2);
  values.insert(a12, calibratedCamera);
  values.insert(a13, simpleCamera);
  values.insert(a14, stereoCamera);


  SharedNoiseModel model1 = noiseModel::Isotropic::Sigma(1, 0.3);
  SharedNoiseModel model2 = noiseModel::Isotropic::Sigma(2, 0.3);
  SharedNoiseModel model3 = noiseModel::Isotropic::Sigma(3, 0.3);
  SharedNoiseModel model4 = noiseModel::Isotropic::Sigma(4, 0.3);
  SharedNoiseModel model5 = noiseModel::Isotropic::Sigma(5, 0.3);
  SharedNoiseModel model6 = noiseModel::Isotropic::Sigma(6, 0.3);
  SharedNoiseModel model9 = noiseModel::Isotropic::Sigma(9, 0.3);
  SharedNoiseModel model11 = noiseModel::Isotropic::Sigma(11, 0.3);

  SharedNoiseModel robust1 = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(10.0, noiseModel::mEstimator::Huber::Scalar),
      noiseModel::Unit::Create(2));

  EXPECT(equalsDereferenced(robust1));
  EXPECT(equalsDereferencedXML(robust1));
  EXPECT(equalsDereferencedBinary(robust1));

  PriorFactorPoint2 priorFactorPoint2(a03, point2, model2);
  PriorFactorStereoPoint2 priorFactorStereoPoint2(a04, stereoPoint2, model3);
  PriorFactorPoint3 priorFactorPoint3(a05, point3, model3);
  PriorFactorRot2 priorFactorRot2(a06, rot2, model1);
  PriorFactorRot3 priorFactorRot3(a07, rot3, model3);
  PriorFactorPose2 priorFactorPose2(a08, pose2, model3);
  PriorFactorPose3 priorFactorPose3(a09, pose3, model6);
  PriorFactorCal3_S2 priorFactorCal3_S2(a10, cal3_s2, model5);
  PriorFactorCal3DS2 priorFactorCal3DS2(a11, cal3ds2, model9);
  PriorFactorCalibratedCamera priorFactorCalibratedCamera(a12, calibratedCamera, model6);
  PriorFactorStereoCamera priorFactorStereoCamera(a14, stereoCamera, model11);

  BetweenFactorPoint2 betweenFactorPoint2(a03, b03, point2, model2);
  BetweenFactorPoint3 betweenFactorPoint3(a05, b05, point3, model3);
  BetweenFactorRot2 betweenFactorRot2(a06, b06, rot2, model1);
  BetweenFactorRot3 betweenFactorRot3(a07, b07, rot3, model3);
  BetweenFactorPose2 betweenFactorPose2(a08, b08, pose2, model3);
  BetweenFactorPose3 betweenFactorPose3(a09, b09, pose3, model6);

  NonlinearEqualityPoint2 nonlinearEqualityPoint2(a03, point2);
  NonlinearEqualityStereoPoint2 nonlinearEqualityStereoPoint2(a04, stereoPoint2);
  NonlinearEqualityPoint3 nonlinearEqualityPoint3(a05, point3);
  NonlinearEqualityRot2 nonlinearEqualityRot2(a06, rot2);
  NonlinearEqualityRot3 nonlinearEqualityRot3(a07, rot3);
  NonlinearEqualityPose2 nonlinearEqualityPose2(a08, pose2);
  NonlinearEqualityPose3 nonlinearEqualityPose3(a09, pose3);
  NonlinearEqualityCal3_S2 nonlinearEqualityCal3_S2(a10, cal3_s2);
  NonlinearEqualityCal3DS2 nonlinearEqualityCal3DS2(a11, cal3ds2);
  NonlinearEqualityCalibratedCamera nonlinearEqualityCalibratedCamera(a12, calibratedCamera);
  NonlinearEqualityStereoCamera nonlinearEqualityStereoCamera(a14, stereoCamera);

  RangeFactor2D rangeFactor2D(a08, a03, 2.0, model1);
  RangeFactor3D rangeFactor3D(a09, a05, 2.0, model1);
  RangeFactorPose2 rangeFactorPose2(a08, b08, 2.0, model1);
  RangeFactorPose3 rangeFactorPose3(a09, b09, 2.0, model1);
  RangeFactorCalibratedCameraPoint rangeFactorCalibratedCameraPoint(a12, a05, 2.0, model1);
  RangeFactorPinholeCameraCal3_S2Point rangeFactorPinholeCameraCal3_S2Point(a13, a05, 2.0, model1);
  RangeFactorCalibratedCamera rangeFactorCalibratedCamera(a12, b12, 2.0, model1);
  RangeFactorPinholeCameraCal3_S2 rangeFactorPinholeCameraCal3_S2(a13, b13, 2.0, model1);

  BearingRangeFactor2D bearingRangeFactor2D(a08, a03, rot2, 2.0, model2);

  GenericProjectionFactorCal3_S2 genericProjectionFactorCal3_S2(point2, model2, a09, a05, boost::make_shared<Cal3_S2>(cal3_s2));
  GenericProjectionFactorCal3DS2 genericProjectionFactorCal3DS2(point2, model2, a09, a05, boost::make_shared<Cal3DS2>(cal3ds2));

  GeneralSFMFactorCal3_S2 generalSFMFactorCal3_S2(point2, model2, a13, a05);

  GeneralSFMFactor2Cal3_S2 generalSFMFactor2Cal3_S2(point2, model2, a09, a05, a10);

  GenericStereoFactor3D genericStereoFactor3D(stereoPoint2, model3, a09, a05, boost::make_shared<Cal3_S2Stereo>(cal3_s2stereo));


  NonlinearFactorGraph graph;
  graph += priorFactorPoint2;
  graph += priorFactorStereoPoint2;
  graph += priorFactorPoint3;
  graph += priorFactorRot2;
  graph += priorFactorRot3;
  graph += priorFactorPose2;
  graph += priorFactorPose3;
  graph += priorFactorCal3_S2;
  graph += priorFactorCal3DS2;
  graph += priorFactorCalibratedCamera;
  graph += priorFactorStereoCamera;

  graph += betweenFactorPoint2;
  graph += betweenFactorPoint3;
  graph += betweenFactorRot2;
  graph += betweenFactorRot3;
  graph += betweenFactorPose2;
  graph += betweenFactorPose3;

  graph += nonlinearEqualityPoint2;
  graph += nonlinearEqualityStereoPoint2;
  graph += nonlinearEqualityPoint3;
  graph += nonlinearEqualityRot2;
  graph += nonlinearEqualityRot3;
  graph += nonlinearEqualityPose2;
  graph += nonlinearEqualityPose3;
  graph += nonlinearEqualityCal3_S2;
  graph += nonlinearEqualityCal3DS2;
  graph += nonlinearEqualityCalibratedCamera;
  graph += nonlinearEqualityStereoCamera;

  graph += rangeFactor2D;
  graph += rangeFactor3D;
  graph += rangeFactorPose2;
  graph += rangeFactorPose3;
  graph += rangeFactorCalibratedCameraPoint;
  graph += rangeFactorPinholeCameraCal3_S2Point;
  graph += rangeFactorCalibratedCamera;
  graph += rangeFactorPinholeCameraCal3_S2;

  graph += bearingRangeFactor2D;

  graph += genericProjectionFactorCal3_S2;
  graph += genericProjectionFactorCal3DS2;

  graph += generalSFMFactorCal3_S2;

  graph += generalSFMFactor2Cal3_S2;

  graph += genericStereoFactor3D;


  // text
  EXPECT(equalsObj<Symbol>(a01));
  EXPECT(equalsObj<Symbol>(b02));
  EXPECT(equalsObj<Values>(values));
  EXPECT(equalsObj<NonlinearFactorGraph>(graph));

  EXPECT(equalsObj<PriorFactorPoint2>(priorFactorPoint2));
  EXPECT(equalsObj<PriorFactorStereoPoint2>(priorFactorStereoPoint2));
  EXPECT(equalsObj<PriorFactorPoint3>(priorFactorPoint3));
  EXPECT(equalsObj<PriorFactorRot2>(priorFactorRot2));
  EXPECT(equalsObj<PriorFactorRot3>(priorFactorRot3));
  EXPECT(equalsObj<PriorFactorPose2>(priorFactorPose2));
  EXPECT(equalsObj<PriorFactorPose3>(priorFactorPose3));
  EXPECT(equalsObj<PriorFactorCal3_S2>(priorFactorCal3_S2));
  EXPECT(equalsObj<PriorFactorCal3DS2>(priorFactorCal3DS2));
  EXPECT(equalsObj<PriorFactorCalibratedCamera>(priorFactorCalibratedCamera));
  EXPECT(equalsObj<PriorFactorStereoCamera>(priorFactorStereoCamera));

  EXPECT(equalsObj<BetweenFactorPoint2>(betweenFactorPoint2));
  EXPECT(equalsObj<BetweenFactorPoint3>(betweenFactorPoint3));
  EXPECT(equalsObj<BetweenFactorRot2>(betweenFactorRot2));
  EXPECT(equalsObj<BetweenFactorRot3>(betweenFactorRot3));
  EXPECT(equalsObj<BetweenFactorPose2>(betweenFactorPose2));
  EXPECT(equalsObj<BetweenFactorPose3>(betweenFactorPose3));

  EXPECT(equalsObj<NonlinearEqualityPoint2>(nonlinearEqualityPoint2));
  EXPECT(equalsObj<NonlinearEqualityStereoPoint2>(nonlinearEqualityStereoPoint2));
  EXPECT(equalsObj<NonlinearEqualityPoint3>(nonlinearEqualityPoint3));
  EXPECT(equalsObj<NonlinearEqualityRot2>(nonlinearEqualityRot2));
  EXPECT(equalsObj<NonlinearEqualityRot3>(nonlinearEqualityRot3));
  EXPECT(equalsObj<NonlinearEqualityPose2>(nonlinearEqualityPose2));
  EXPECT(equalsObj<NonlinearEqualityPose3>(nonlinearEqualityPose3));
  EXPECT(equalsObj<NonlinearEqualityCal3_S2>(nonlinearEqualityCal3_S2));
  EXPECT(equalsObj<NonlinearEqualityCal3DS2>(nonlinearEqualityCal3DS2));
  EXPECT(equalsObj<NonlinearEqualityCalibratedCamera>(nonlinearEqualityCalibratedCamera));
  EXPECT(equalsObj<NonlinearEqualityStereoCamera>(nonlinearEqualityStereoCamera));

  EXPECT(equalsObj<RangeFactor2D>(rangeFactor2D));
  EXPECT(equalsObj<RangeFactor3D>(rangeFactor3D));
  EXPECT(equalsObj<RangeFactorPose2>(rangeFactorPose2));
  EXPECT(equalsObj<RangeFactorPose3>(rangeFactorPose3));
  EXPECT(equalsObj<RangeFactorCalibratedCameraPoint>(rangeFactorCalibratedCameraPoint));
  EXPECT(equalsObj<RangeFactorPinholeCameraCal3_S2Point>(rangeFactorPinholeCameraCal3_S2Point));
  EXPECT(equalsObj<RangeFactorCalibratedCamera>(rangeFactorCalibratedCamera));
  EXPECT(equalsObj<RangeFactorPinholeCameraCal3_S2>(rangeFactorPinholeCameraCal3_S2));

  EXPECT(equalsObj<BearingRangeFactor2D>(bearingRangeFactor2D));

  EXPECT(equalsObj<GenericProjectionFactorCal3_S2>(genericProjectionFactorCal3_S2));
  EXPECT(equalsObj<GenericProjectionFactorCal3DS2>(genericProjectionFactorCal3DS2));

  EXPECT(equalsObj<GeneralSFMFactorCal3_S2>(generalSFMFactorCal3_S2));

  EXPECT(equalsObj<GeneralSFMFactor2Cal3_S2>(generalSFMFactor2Cal3_S2));

  EXPECT(equalsObj<GenericStereoFactor3D>(genericStereoFactor3D));


  // xml
  EXPECT(equalsXML<Symbol>(a01));
  EXPECT(equalsXML<Symbol>(b02));
  EXPECT(equalsXML<Values>(values));
  EXPECT(equalsXML<NonlinearFactorGraph>(graph));

  EXPECT(equalsXML<PriorFactorPoint2>(priorFactorPoint2));
  EXPECT(equalsXML<PriorFactorStereoPoint2>(priorFactorStereoPoint2));
  EXPECT(equalsXML<PriorFactorPoint3>(priorFactorPoint3));
  EXPECT(equalsXML<PriorFactorRot2>(priorFactorRot2));
  EXPECT(equalsXML<PriorFactorRot3>(priorFactorRot3));
  EXPECT(equalsXML<PriorFactorPose2>(priorFactorPose2));
  EXPECT(equalsXML<PriorFactorPose3>(priorFactorPose3));
  EXPECT(equalsXML<PriorFactorCal3_S2>(priorFactorCal3_S2));
  EXPECT(equalsXML<PriorFactorCal3DS2>(priorFactorCal3DS2));
  EXPECT(equalsXML<PriorFactorCalibratedCamera>(priorFactorCalibratedCamera));
  EXPECT(equalsXML<PriorFactorStereoCamera>(priorFactorStereoCamera));

  EXPECT(equalsXML<BetweenFactorPoint2>(betweenFactorPoint2));
  EXPECT(equalsXML<BetweenFactorPoint3>(betweenFactorPoint3));
  EXPECT(equalsXML<BetweenFactorRot2>(betweenFactorRot2));
  EXPECT(equalsXML<BetweenFactorRot3>(betweenFactorRot3));
  EXPECT(equalsXML<BetweenFactorPose2>(betweenFactorPose2));
  EXPECT(equalsXML<BetweenFactorPose3>(betweenFactorPose3));

  EXPECT(equalsXML<NonlinearEqualityPoint2>(nonlinearEqualityPoint2));
  EXPECT(equalsXML<NonlinearEqualityStereoPoint2>(nonlinearEqualityStereoPoint2));
  EXPECT(equalsXML<NonlinearEqualityPoint3>(nonlinearEqualityPoint3));
  EXPECT(equalsXML<NonlinearEqualityRot2>(nonlinearEqualityRot2));
  EXPECT(equalsXML<NonlinearEqualityRot3>(nonlinearEqualityRot3));
  EXPECT(equalsXML<NonlinearEqualityPose2>(nonlinearEqualityPose2));
  EXPECT(equalsXML<NonlinearEqualityPose3>(nonlinearEqualityPose3));
  EXPECT(equalsXML<NonlinearEqualityCal3_S2>(nonlinearEqualityCal3_S2));
  EXPECT(equalsXML<NonlinearEqualityCal3DS2>(nonlinearEqualityCal3DS2));
  EXPECT(equalsXML<NonlinearEqualityCalibratedCamera>(nonlinearEqualityCalibratedCamera));
  EXPECT(equalsXML<NonlinearEqualityStereoCamera>(nonlinearEqualityStereoCamera));

  EXPECT(equalsXML<RangeFactor2D>(rangeFactor2D));
  EXPECT(equalsXML<RangeFactor3D>(rangeFactor3D));
  EXPECT(equalsXML<RangeFactorPose2>(rangeFactorPose2));
  EXPECT(equalsXML<RangeFactorPose3>(rangeFactorPose3));
  EXPECT(equalsXML<RangeFactorCalibratedCameraPoint>(rangeFactorCalibratedCameraPoint));
  EXPECT(equalsXML<RangeFactorPinholeCameraCal3_S2Point>(rangeFactorPinholeCameraCal3_S2Point));
  EXPECT(equalsXML<RangeFactorCalibratedCamera>(rangeFactorCalibratedCamera));
  EXPECT(equalsXML<RangeFactorPinholeCameraCal3_S2>(rangeFactorPinholeCameraCal3_S2));

  EXPECT(equalsXML<BearingRangeFactor2D>(bearingRangeFactor2D));

  EXPECT(equalsXML<GenericProjectionFactorCal3_S2>(genericProjectionFactorCal3_S2));
  EXPECT(equalsXML<GenericProjectionFactorCal3DS2>(genericProjectionFactorCal3DS2));

  EXPECT(equalsXML<GeneralSFMFactorCal3_S2>(generalSFMFactorCal3_S2));

  EXPECT(equalsXML<GeneralSFMFactor2Cal3_S2>(generalSFMFactor2Cal3_S2));

  EXPECT(equalsXML<GenericStereoFactor3D>(genericStereoFactor3D));


  // binary
  EXPECT(equalsBinary<Symbol>(a01));
  EXPECT(equalsBinary<Symbol>(b02));
  EXPECT(equalsBinary<Values>(values));
  EXPECT(equalsBinary<NonlinearFactorGraph>(graph));

  EXPECT(equalsBinary<PriorFactorPoint2>(priorFactorPoint2));
  EXPECT(equalsBinary<PriorFactorStereoPoint2>(priorFactorStereoPoint2));
  EXPECT(equalsBinary<PriorFactorPoint3>(priorFactorPoint3));
  EXPECT(equalsBinary<PriorFactorRot2>(priorFactorRot2));
  EXPECT(equalsBinary<PriorFactorRot3>(priorFactorRot3));
  EXPECT(equalsBinary<PriorFactorPose2>(priorFactorPose2));
  EXPECT(equalsBinary<PriorFactorPose3>(priorFactorPose3));
  EXPECT(equalsBinary<PriorFactorCal3_S2>(priorFactorCal3_S2));
  EXPECT(equalsBinary<PriorFactorCal3DS2>(priorFactorCal3DS2));
  EXPECT(equalsBinary<PriorFactorCalibratedCamera>(priorFactorCalibratedCamera));
  EXPECT(equalsBinary<PriorFactorStereoCamera>(priorFactorStereoCamera));

  EXPECT(equalsBinary<BetweenFactorPoint2>(betweenFactorPoint2));
  EXPECT(equalsBinary<BetweenFactorPoint3>(betweenFactorPoint3));
  EXPECT(equalsBinary<BetweenFactorRot2>(betweenFactorRot2));
  EXPECT(equalsBinary<BetweenFactorRot3>(betweenFactorRot3));
  EXPECT(equalsBinary<BetweenFactorPose2>(betweenFactorPose2));
  EXPECT(equalsBinary<BetweenFactorPose3>(betweenFactorPose3));

  EXPECT(equalsBinary<NonlinearEqualityPoint2>(nonlinearEqualityPoint2));
  EXPECT(equalsBinary<NonlinearEqualityStereoPoint2>(nonlinearEqualityStereoPoint2));
  EXPECT(equalsBinary<NonlinearEqualityPoint3>(nonlinearEqualityPoint3));
  EXPECT(equalsBinary<NonlinearEqualityRot2>(nonlinearEqualityRot2));
  EXPECT(equalsBinary<NonlinearEqualityRot3>(nonlinearEqualityRot3));
  EXPECT(equalsBinary<NonlinearEqualityPose2>(nonlinearEqualityPose2));
  EXPECT(equalsBinary<NonlinearEqualityPose3>(nonlinearEqualityPose3));
  EXPECT(equalsBinary<NonlinearEqualityCal3_S2>(nonlinearEqualityCal3_S2));
  EXPECT(equalsBinary<NonlinearEqualityCal3DS2>(nonlinearEqualityCal3DS2));
  EXPECT(equalsBinary<NonlinearEqualityCalibratedCamera>(nonlinearEqualityCalibratedCamera));
  EXPECT(equalsBinary<NonlinearEqualityStereoCamera>(nonlinearEqualityStereoCamera));

  EXPECT(equalsBinary<RangeFactor2D>(rangeFactor2D));
  EXPECT(equalsBinary<RangeFactor3D>(rangeFactor3D));
  EXPECT(equalsBinary<RangeFactorPose2>(rangeFactorPose2));
  EXPECT(equalsBinary<RangeFactorPose3>(rangeFactorPose3));
  EXPECT(equalsBinary<RangeFactorCalibratedCameraPoint>(rangeFactorCalibratedCameraPoint));
  EXPECT(equalsBinary<RangeFactorPinholeCameraCal3_S2Point>(rangeFactorPinholeCameraCal3_S2Point));
  EXPECT(equalsBinary<RangeFactorCalibratedCamera>(rangeFactorCalibratedCamera));
  EXPECT(equalsBinary<RangeFactorPinholeCameraCal3_S2>(rangeFactorPinholeCameraCal3_S2));

  EXPECT(equalsBinary<BearingRangeFactor2D>(bearingRangeFactor2D));

  EXPECT(equalsBinary<GenericProjectionFactorCal3_S2>(genericProjectionFactorCal3_S2));
  EXPECT(equalsBinary<GenericProjectionFactorCal3DS2>(genericProjectionFactorCal3DS2));

  EXPECT(equalsBinary<GeneralSFMFactorCal3_S2>(generalSFMFactorCal3_S2));

  EXPECT(equalsBinary<GeneralSFMFactor2Cal3_S2>(generalSFMFactor2Cal3_S2));

  EXPECT(equalsBinary<GenericStereoFactor3D>(genericStereoFactor3D));
}

/* ************************************************************************* */
// Read from XML file
namespace {
static GaussianFactorGraph read(const string& name) {
  auto inputFile = findExampleDataFile(name);
  ifstream is(inputFile);
  if (!is.is_open()) throw runtime_error("Cannot find file " + inputFile);
  boost::archive::xml_iarchive in_archive(is);
  GaussianFactorGraph Ab;
  in_archive >> boost::serialization::make_nvp("graph", Ab);
  return Ab;
}
}  // namespace

/* ************************************************************************* */
// Read from XML file
TEST(SubgraphSolver, Solves) {
  using gtsam::example::planarGraph;

  // Create preconditioner
  SubgraphPreconditioner system;

  // We test on three different graphs
  const auto Ab1 = planarGraph(3).first;
  const auto Ab2 = read("toy3D");
  const auto Ab3 = read("randomGrid3D");

  // For all graphs, test solve and solveTranspose
  for (const auto& Ab : {Ab1, Ab2, Ab3}) {
    // Call build, a non-const method needed to make solve work :-(
    KeyInfo keyInfo(Ab);
    std::map<Key, Vector> lambda;
    system.build(Ab, keyInfo, lambda);

    // Create a perturbed (non-zero) RHS
    const auto xbar = system.Rc1().optimize();  // merely for use in zero below
    auto values_y = VectorValues::Zero(xbar);
    auto it = values_y.begin();
    it->second.setConstant(100);
    ++it;
    it->second.setConstant(-100);

    // Solve the VectorValues way
    auto values_x = system.Rc1().backSubstitute(values_y);

    // Solve the matrix way, this really just checks BN::backSubstitute
    // This only works with Rc1 ordering, not with keyInfo !
    // TODO(frank): why does this not work with an arbitrary ordering?
    const auto ord = system.Rc1().ordering();
    const Matrix R1 = system.Rc1().matrix(ord).first;
    auto ord_y = values_y.vector(ord);
    auto vector_x = R1.inverse() * ord_y;
    EXPECT(assert_equal(vector_x, values_x.vector(ord)));

    // Test that 'solve' does implement x = R^{-1} y
    // We do this by asserting it gives same answer as backSubstitute
    // Only works with keyInfo ordering:
    const auto ordering = keyInfo.ordering();
    auto vector_y = values_y.vector(ordering);
    const size_t N = R1.cols();
    Vector solve_x = Vector::Zero(N);
    system.solve(vector_y, solve_x);
    EXPECT(assert_equal(values_x.vector(ordering), solve_x));

    // Test that transposeSolve does implement x = R^{-T} y
    // We do this by asserting it gives same answer as backSubstituteTranspose
    auto values_x2 = system.Rc1().backSubstituteTranspose(values_y);
    Vector solveT_x = Vector::Zero(N);
    system.transposeSolve(vector_y, solveT_x);
    EXPECT(assert_equal(values_x2.vector(ordering), solveT_x));
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
