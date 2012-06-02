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

#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/visualSLAM.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

/* Create GUIDs for factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor");

/* ************************************************************************* */
// Export Noisemodels
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

/* ************************************************************************* */
TEST (Serialization, smallExample_linear) {
  using namespace example;

  Ordering ordering; ordering += Symbol('x',1),Symbol('x',2),Symbol('l',1);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  EXPECT(equalsObj(ordering));
  EXPECT(equalsXML(ordering));

  EXPECT(equalsObj(fg));
  EXPECT(equalsXML(fg));

  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  EXPECT(equalsObj(cbn));
  EXPECT(equalsXML(cbn));
}

/* ************************************************************************* */
TEST (Serialization, gaussianISAM) {
  using namespace example;
  Ordering ordering;
  GaussianFactorGraph smoother;
  boost::tie(smoother, ordering) = createSmoother(7);
  BayesTree<GaussianConditional> bayesTree = *GaussianMultifrontalSolver(smoother).eliminate();
  GaussianISAM isam(bayesTree);

  EXPECT(equalsObj(isam));
  EXPECT(equalsXML(isam));
}

/* ************************************************************************* */
/* Create GUIDs for factors in simulated2D */
BOOST_CLASS_EXPORT_GUID(simulated2D::Prior,       "gtsam::simulated2D::Prior"      )
BOOST_CLASS_EXPORT_GUID(simulated2D::Odometry,    "gtsam::simulated2D::Odometry"   )
BOOST_CLASS_EXPORT_GUID(simulated2D::Measurement, "gtsam::simulated2D::Measurement")
BOOST_CLASS_EXPORT(gtsam::Point2)

/* ************************************************************************* */
TEST (Serialization, smallExample) {
  using namespace example;
  Graph nfg = createNonlinearFactorGraph();
  Values c1 = createValues();
  EXPECT(equalsObj(nfg));
  EXPECT(equalsXML(nfg));

  EXPECT(equalsObj(c1));
  EXPECT(equalsXML(c1));
}

/* ************************************************************************* */
/* Create GUIDs for factors */
BOOST_CLASS_EXPORT_GUID(planarSLAM::Prior,       "gtsam::planarSLAM::Prior");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Bearing,     "gtsam::planarSLAM::Bearing");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Range,       "gtsam::planarSLAM::Range");
BOOST_CLASS_EXPORT_GUID(planarSLAM::BearingRange,"gtsam::planarSLAM::BearingRange");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Odometry,    "gtsam::planarSLAM::Odometry");
BOOST_CLASS_EXPORT_GUID(planarSLAM::Constraint,  "gtsam::planarSLAM::Constraint");

BOOST_CLASS_EXPORT(gtsam::Pose2)

/* ************************************************************************* */
TEST (Serialization, planar_system) {
  using namespace planarSLAM;
  planarSLAM::Values values;
	Symbol i2('x',2), i3('x',3), i4('x',4), i9('x',9), j3('l',3), j5('l',5), j9('l',9);
  values.insert(j3, Point2(1.0, 2.0));
  values.insert(i4, Pose2(1.0, 2.0, 0.3));

  SharedNoiseModel model1 = noiseModel::Isotropic::Sigma(1, 0.3);
  SharedNoiseModel model2 = noiseModel::Isotropic::Sigma(2, 0.3);
  SharedNoiseModel model3 = noiseModel::Isotropic::Sigma(3, 0.3);

  Prior prior(i3, Pose2(0.1,-0.3, 0.2), model1);
  Bearing bearing(i3, j5, Rot2::fromDegrees(0.5), model1);
  Range range(i2, j9, 7.0, model1);
  BearingRange bearingRange(i2, j3, Rot2::fromDegrees(0.6), 2.0, model2);
  Odometry odometry(i2, i3, Pose2(1.0, 2.0, 0.3), model3);
  Constraint constraint(i9, Pose2(2.0,-1.0, 0.2));

  Graph graph;
  graph.add(prior);
  graph.add(bearing);
  graph.add(range);
  graph.add(bearingRange);
  graph.add(odometry);
  graph.add(constraint);

  // text
  EXPECT(equalsObj<Symbol>(i2));
  EXPECT(equalsObj<Symbol>(j3));
  EXPECT(equalsObj<planarSLAM::Values>(values));
  EXPECT(equalsObj<Prior>(prior));
  EXPECT(equalsObj<Bearing>(bearing));
  EXPECT(equalsObj<BearingRange>(bearingRange));
  EXPECT(equalsObj<Range>(range));
  EXPECT(equalsObj<Odometry>(odometry));
  EXPECT(equalsObj<Constraint>(constraint));
  EXPECT(equalsObj<Graph>(graph));

  // xml
  EXPECT(equalsXML<Symbol>(i2));
  EXPECT(equalsXML<Symbol>(j3));
  EXPECT(equalsXML<planarSLAM::Values>(values));
  EXPECT(equalsXML<Prior>(prior));
  EXPECT(equalsXML<Bearing>(bearing));
  EXPECT(equalsXML<BearingRange>(bearingRange));
  EXPECT(equalsXML<Range>(range));
  EXPECT(equalsXML<Odometry>(odometry));
  EXPECT(equalsXML<Constraint>(constraint));
  EXPECT(equalsXML<Graph>(graph));
}

/* ************************************************************************* */
/* Create GUIDs for factors */
BOOST_CLASS_EXPORT_GUID(visualSLAM::PoseConstraint,  "gtsam::visualSLAM::PoseConstraint");
BOOST_CLASS_EXPORT_GUID(visualSLAM::PointConstraint, "gtsam::visualSLAM::PointConstraint");
BOOST_CLASS_EXPORT_GUID(visualSLAM::PosePrior,       "gtsam::visualSLAM::PosePrior");
BOOST_CLASS_EXPORT_GUID(visualSLAM::PointPrior,      "gtsam::visualSLAM::PointPrior");
BOOST_CLASS_EXPORT_GUID(visualSLAM::ProjectionFactor,"gtsam::visualSLAM::ProjectionFactor");
BOOST_CLASS_EXPORT_GUID(visualSLAM::StereoFactor,    "gtsam::visualSLAM::StereoFactor");

BOOST_CLASS_EXPORT(gtsam::Pose3)
BOOST_CLASS_EXPORT(gtsam::Point3)

Point3 pt3(1.0, 2.0, 3.0);
Rot3 rt3 = Rot3::RzRyRx(1.0, 3.0, 2.0);
Pose3 pose3(rt3, pt3);
Cal3_S2 cal1(1.0, 2.0, 0.3, 0.1, 0.5);

/* ************************************************************************* */
TEST (Serialization, visual_system) {
  using namespace visualSLAM;
  Values values;
  Symbol x1('x',1), x2('x',2);
  Symbol l1('l',1), l2('l',2);
  Pose3 pose1 = pose3, pose2 = pose3.inverse();
  Point3 pt1(1.0, 2.0, 3.0), pt2(4.0, 5.0, 6.0);
  values.insert(x1, pose1);
  values.insert(l1, pt1);
  SharedNoiseModel model2 = noiseModel::Isotropic::Sigma(2, 0.3);
  SharedNoiseModel model3 = noiseModel::Isotropic::Sigma(3, 0.3);
  SharedNoiseModel model6 = noiseModel::Isotropic::Sigma(6, 0.3);
  boost::shared_ptr<Cal3_S2> K(new Cal3_S2(cal1));

  Graph graph;
  graph.addMeasurement(Point2(1.0, 2.0), model2, x1, l1, K);
  graph.addPointConstraint(l1, pt1);
  graph.addPointPrior(l1, pt2, model3);
  graph.addPoseConstraint(x1, pose1);
  graph.addPosePrior(x1, pose2, model6);

  EXPECT(equalsObj(values));
  EXPECT(equalsObj(graph));

  EXPECT(equalsXML(values));
  EXPECT(equalsXML(graph));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
