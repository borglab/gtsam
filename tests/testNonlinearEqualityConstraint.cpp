/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testNonlinearEqualityConstraint.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/slam/simulated2DConstraints.h>
#include <gtsam/slam/visualSLAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>

namespace eq2D = gtsam::simulated2D::equality_constraints;

using namespace std;
using namespace gtsam;

static const double tol = 1e-5;

SharedDiagonal hard_model = noiseModel::Constrained::All(2);
SharedDiagonal soft_model = noiseModel::Isotropic::Sigma(2, 1.0);

typedef NonlinearFactorGraph<simulated2D::Values> Graph;
typedef boost::shared_ptr<Graph> shared_graph;
typedef boost::shared_ptr<simulated2D::Values> shared_values;
typedef NonlinearOptimizer<Graph, simulated2D::Values> Optimizer;

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, unary_basics ) {
	Point2 pt(1.0, 2.0);
	simulated2D::PoseKey key(1);
	double mu = 1000.0;
	eq2D::UnaryEqualityConstraint constraint(pt, key, mu);

	simulated2D::Values config1;
	config1.insert(key, pt);
	EXPECT(constraint.active(config1));
	EXPECT(assert_equal(zero(2), constraint.evaluateError(pt), tol));
	EXPECT(assert_equal(zero(2), constraint.unwhitenedError(config1), tol));
	EXPECT_DOUBLES_EQUAL(0.0, constraint.error(config1), tol);

	simulated2D::Values config2;
	Point2 ptBad1(2.0, 2.0);
	config2.insert(key, ptBad1);
	EXPECT(constraint.active(config2));
	EXPECT(assert_equal(Vector_(2, 1.0, 0.0), constraint.evaluateError(ptBad1), tol));
	EXPECT(assert_equal(Vector_(2, 1.0, 0.0), constraint.unwhitenedError(config2), tol));
	EXPECT_DOUBLES_EQUAL(1000.0, constraint.error(config2), tol);
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, unary_linearization ) {
	Point2 pt(1.0, 2.0);
	simulated2D::PoseKey key(1);
	double mu = 1000.0;
	Ordering ordering;
	ordering += key;
	eq2D::UnaryEqualityConstraint constraint(pt, key, mu);

	simulated2D::Values config1;
	config1.insert(key, pt);
	GaussianFactor::shared_ptr actual1 = constraint.linearize(config1, ordering);
	GaussianFactor::shared_ptr expected1(new JacobianFactor(ordering[key], eye(2,2), zero(2), hard_model));
	EXPECT(assert_equal(*expected1, *actual1, tol));

	simulated2D::Values config2;
	Point2 ptBad(2.0, 2.0);
	config2.insert(key, ptBad);
	GaussianFactor::shared_ptr actual2 = constraint.linearize(config2, ordering);
	GaussianFactor::shared_ptr expected2(new JacobianFactor(ordering[key], eye(2,2), Vector_(2,-1.0,0.0), hard_model));
	EXPECT(assert_equal(*expected2, *actual2, tol));
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, unary_simple_optimization ) {
	// create a single-node graph with a soft and hard constraint to
	// ensure that the hard constraint overrides the soft constraint
	Point2 truth_pt(1.0, 2.0);
	simulated2D::PoseKey key(1);
	double mu = 1000.0;
	eq2D::UnaryEqualityConstraint::shared_ptr constraint(
			new eq2D::UnaryEqualityConstraint(truth_pt, key, mu));

	Point2 badPt(100.0, -200.0);
	simulated2D::Prior::shared_ptr factor(
			new simulated2D::Prior(badPt, soft_model, key));

	shared_graph graph(new Graph());
	graph->push_back(constraint);
	graph->push_back(factor);

	shared_values initValues(new simulated2D::Values());
	initValues->insert(key, badPt);

	Optimizer::shared_values actual = Optimizer::optimizeLM(graph, initValues);
	simulated2D::Values expected;
	expected.insert(key, truth_pt);
	CHECK(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, odo_basics ) {
	Point2 x1(1.0, 2.0), x2(2.0, 3.0), odom(1.0, 1.0);
	simulated2D::PoseKey key1(1), key2(2);
	double mu = 1000.0;
	eq2D::OdoEqualityConstraint constraint(odom, key1, key2, mu);

	simulated2D::Values config1;
	config1.insert(key1, x1);
	config1.insert(key2, x2);
	EXPECT(constraint.active(config1));
	EXPECT(assert_equal(zero(2), constraint.evaluateError(x1, x2), tol));
	EXPECT(assert_equal(zero(2), constraint.unwhitenedError(config1), tol));
	EXPECT_DOUBLES_EQUAL(0.0, constraint.error(config1), tol);

	simulated2D::Values config2;
	Point2 x1bad(2.0, 2.0);
	Point2 x2bad(2.0, 2.0);
	config2.insert(key1, x1bad);
	config2.insert(key2, x2bad);
	EXPECT(constraint.active(config2));
	EXPECT(assert_equal(Vector_(2, -1.0, -1.0), constraint.evaluateError(x1bad, x2bad), tol));
	EXPECT(assert_equal(Vector_(2, -1.0, -1.0), constraint.unwhitenedError(config2), tol));
	EXPECT_DOUBLES_EQUAL(2000.0, constraint.error(config2), tol);
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, odo_linearization ) {
	Point2 x1(1.0, 2.0), x2(2.0, 3.0), odom(1.0, 1.0);
	simulated2D::PoseKey key1(1), key2(2);
	double mu = 1000.0;
	Ordering ordering;
	ordering += key1, key2;
	eq2D::OdoEqualityConstraint constraint(odom, key1, key2, mu);

	simulated2D::Values config1;
	config1.insert(key1, x1);
	config1.insert(key2, x2);
	GaussianFactor::shared_ptr actual1 = constraint.linearize(config1, ordering);
	GaussianFactor::shared_ptr expected1(
			new JacobianFactor(ordering[key1], -eye(2,2), ordering[key2],
					eye(2,2), zero(2), hard_model));
	EXPECT(assert_equal(*expected1, *actual1, tol));

	simulated2D::Values config2;
	Point2 x1bad(2.0, 2.0);
	Point2 x2bad(2.0, 2.0);
	config2.insert(key1, x1bad);
	config2.insert(key2, x2bad);
	GaussianFactor::shared_ptr actual2 = constraint.linearize(config2, ordering);
	GaussianFactor::shared_ptr expected2(
			new JacobianFactor(ordering[key1], -eye(2,2), ordering[key2],
					eye(2,2), Vector_(2, 1.0, 1.0), hard_model));
	EXPECT(assert_equal(*expected2, *actual2, tol));
}

/* ************************************************************************* */
TEST( testNonlinearEqualityConstraint, odo_simple_optimize ) {
	// create a two-node graph, connected by an odometry constraint, with
	// a hard prior on one variable, and a conflicting soft prior
	// on the other variable - the constraints should override the soft constraint
	Point2 truth_pt1(1.0, 2.0), truth_pt2(3.0, 2.0);
	simulated2D::PoseKey key1(1), key2(2);

	// hard prior on x1
	eq2D::UnaryEqualityConstraint::shared_ptr constraint1(
			new eq2D::UnaryEqualityConstraint(truth_pt1, key1));

	// soft prior on x2
	Point2 badPt(100.0, -200.0);
	simulated2D::Prior::shared_ptr factor(
			new simulated2D::Prior(badPt, soft_model, key2));

	// odometry constraint
	eq2D::OdoEqualityConstraint::shared_ptr constraint2(
			new eq2D::OdoEqualityConstraint(
					truth_pt1.between(truth_pt2), key1, key2));

	shared_graph graph(new Graph());
	graph->push_back(constraint1);
	graph->push_back(constraint2);
	graph->push_back(factor);

	shared_values initValues(new simulated2D::Values());
	initValues->insert(key1, Point2());
	initValues->insert(key2, badPt);

	Optimizer::shared_values actual = Optimizer::optimizeLM(graph, initValues);
	simulated2D::Values expected;
	expected.insert(key1, truth_pt1);
	expected.insert(key2, truth_pt2);
	CHECK(assert_equal(expected, *actual, tol));
}

/* ********************************************************************* */
TEST (testNonlinearEqualityConstraint, two_pose ) {
	/*
	 * Determining a ground truth linear system
	 * with two poses seeing one landmark, with each pose
	 * constrained to a particular value
	 */

	shared_graph graph(new Graph());

	simulated2D::PoseKey x1(1), x2(2);
	simulated2D::PointKey l1(1), l2(2);
	Point2 pt_x1(1.0, 1.0),
		   pt_x2(5.0, 6.0);
	graph->add(eq2D::UnaryEqualityConstraint(pt_x1, x1));
	graph->add(eq2D::UnaryEqualityConstraint(pt_x2, x2));

	Point2 z1(0.0, 5.0);
	SharedNoiseModel sigma(noiseModel::Isotropic::Sigma(2, 0.1));
	graph->add(simulated2D::Measurement(z1, sigma, x1,l1));

	Point2 z2(-4.0, 0.0);
	graph->add(simulated2D::Measurement(z2, sigma, x2,l2));

	graph->add(eq2D::PointEqualityConstraint(l1, l2));

	shared_values initialEstimate(new simulated2D::Values());
	initialEstimate->insert(x1, pt_x1);
	initialEstimate->insert(x2, Point2());
	initialEstimate->insert(l1, Point2(1.0, 6.0)); // ground truth
	initialEstimate->insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame

	Optimizer::shared_values actual = Optimizer::optimizeLM(graph, initialEstimate);

	simulated2D::Values expected;
	expected.insert(x1, pt_x1);
	expected.insert(l1, Point2(1.0, 6.0));
	expected.insert(l2, Point2(1.0, 6.0));
	expected.insert(x2, Point2(5.0, 6.0));
	CHECK(assert_equal(expected, *actual, 1e-5));
}

/* ********************************************************************* */
TEST (testNonlinearEqualityConstraint, map_warp ) {
	// get a graph
	shared_graph graph(new Graph());

	// keys
	simulated2D::PoseKey x1(1), x2(2);
	simulated2D::PointKey l1(1), l2(2);

	// constant constraint on x1
	Point2 pose1(1.0, 1.0);
	graph->add(eq2D::UnaryEqualityConstraint(pose1, x1));

	SharedDiagonal sigma = noiseModel::Isotropic::Sigma(1,0.1);

	// measurement from x1 to l1
	Point2 z1(0.0, 5.0);
	graph->add(simulated2D::Measurement(z1, sigma, x1, l1));

	// measurement from x2 to l2
	Point2 z2(-4.0, 0.0);
	graph->add(simulated2D::Measurement(z2, sigma, x2, l2));

	// equality constraint between l1 and l2
	graph->add(eq2D::PointEqualityConstraint(l1, l2));

	// create an initial estimate
	shared_values initialEstimate(new simulated2D::Values());
	initialEstimate->insert(x1, Point2( 1.0, 1.0));
	initialEstimate->insert(l1, Point2( 1.0, 6.0));
	initialEstimate->insert(l2, Point2(-4.0, 0.0)); // starting with a separate reference frame
	initialEstimate->insert(x2, Point2( 0.0, 0.0)); // other pose starts at origin

	// optimize
	Optimizer::shared_values actual = Optimizer::optimizeLM(graph, initialEstimate);

	simulated2D::Values expected;
	expected.insert(x1, Point2(1.0, 1.0));
	expected.insert(l1, Point2(1.0, 6.0));
	expected.insert(l2, Point2(1.0, 6.0));
	expected.insert(x2, Point2(5.0, 6.0));
	CHECK(assert_equal(expected, *actual, tol));
}

// make a realistic calibration matrix
double fov = 60; // degrees
size_t w=640,h=480;
Cal3_S2 K(fov,w,h);
boost::shared_ptr<Cal3_S2> shK(new Cal3_S2(K));

// typedefs for visual SLAM example
typedef visualSLAM::Values VValues;
typedef boost::shared_ptr<VValues> shared_vconfig;
typedef visualSLAM::Graph VGraph;
typedef NonlinearOptimizer<VGraph,VValues> VOptimizer;

// factors for visual slam
typedef NonlinearEquality2<VValues, visualSLAM::PointKey> Point3Equality;

/* ********************************************************************* */
TEST (testNonlinearEqualityConstraint, stereo_constrained ) {

	// create initial estimates
	Rot3 faceDownY(Matrix_(3,3,
			1.0, 0.0, 0.0,
			0.0, 0.0, 1.0,
			0.0, -1.0, 0.0));
	Pose3 pose1(faceDownY, Point3()); // origin, left camera
	SimpleCamera camera1(K, pose1);
	Pose3 pose2(faceDownY, Point3(2.0, 0.0, 0.0)); // 2 units to the left
	SimpleCamera camera2(K, pose2);
	Point3 landmark(1.0, 5.0, 0.0); //centered between the cameras, 5 units away

	// keys
	visualSLAM::PoseKey x1(1), x2(2);
	visualSLAM::PointKey l1(1), l2(2);

	// create graph
	VGraph::shared_graph graph(new VGraph());

	// create equality constraints for poses
	graph->addPoseConstraint(1, camera1.pose());
	graph->addPoseConstraint(2, camera2.pose());

	// create  factors
	SharedDiagonal vmodel = noiseModel::Unit::Create(3);
	graph->addMeasurement(camera1.project(landmark), vmodel, 1, 1, shK);
	graph->addMeasurement(camera2.project(landmark), vmodel, 2, 2, shK);

	// add equality constraint
	graph->add(Point3Equality(l1, l2));

	// create initial data
	Point3 landmark1(0.5, 5.0, 0.0);
	Point3 landmark2(1.5, 5.0, 0.0);

	shared_vconfig initValues(new VValues());
	initValues->insert(x1, pose1);
	initValues->insert(x2, pose2);
	initValues->insert(l1, landmark1);
	initValues->insert(l2, landmark2);

	// optimize
	VOptimizer::shared_values actual = VOptimizer::optimizeLM(graph, initValues);

	// create config
	VValues truthValues;
	truthValues.insert(x1, camera1.pose());
	truthValues.insert(x2, camera2.pose());
	truthValues.insert(l1, landmark);
	truthValues.insert(l2, landmark);

	// check if correct
	CHECK(assert_equal(truthValues, *actual, 1e-5));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


