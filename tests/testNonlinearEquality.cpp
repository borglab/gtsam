/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testNonlinearEquality.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/simulated2DConstraints.h>
#include <gtsam/slam/visualSLAM.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>

using namespace std;
using namespace gtsam;

namespace eq2D = gtsam::simulated2D::equality_constraints;

static const double tol = 1e-5;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef PriorFactor<PoseKey> PosePrior;
typedef NonlinearEquality<PoseKey> PoseNLE;
typedef boost::shared_ptr<PoseNLE> shared_poseNLE;

typedef NonlinearFactorGraph PoseGraph;
typedef NonlinearOptimizer<PoseGraph> PoseOptimizer;

PoseKey key(1);

/* ************************************************************************* */
TEST ( NonlinearEquality, linearization ) {
	Pose2 value = Pose2(2.1, 1.0, 2.0);
	DynamicValues linearize;
	linearize.insert(key, value);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check linearize
	SharedDiagonal constraintModel = noiseModel::Constrained::All(3);
	JacobianFactor expLF(0, eye(3), zero(3), constraintModel);
	GaussianFactor::shared_ptr actualLF = nle->linearize(linearize, *linearize.orderingArbitrary());
	EXPECT(assert_equal(*actualLF, (const GaussianFactor&)expLF));
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_pose ) {

	PoseKey key(1);
	Pose2 value;
	DynamicValues config;
	config.insert(key, value);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	GaussianFactor::shared_ptr actualLF = nle->linearize(config, *config.orderingArbitrary());
	EXPECT(true);
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_fail ) {
	Pose2 value = Pose2(2.1, 1.0, 2.0);
	Pose2 wrong = Pose2(2.1, 3.0, 4.0);
	DynamicValues bad_linearize;
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check linearize to ensure that it fails for bad linearization points
	CHECK_EXCEPTION(nle->linearize(bad_linearize, *bad_linearize.orderingArbitrary()), std::invalid_argument);
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_fail_pose ) {

	PoseKey key(1);
	Pose2 value(2.0, 1.0, 2.0),
		  wrong(2.0, 3.0, 4.0);
	DynamicValues bad_linearize;
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check linearize to ensure that it fails for bad linearization points
	CHECK_EXCEPTION(nle->linearize(bad_linearize, *bad_linearize.orderingArbitrary()), std::invalid_argument);
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_fail_pose_origin ) {

	PoseKey key(1);
	Pose2 value,
		  wrong(2.0, 3.0, 4.0);
	DynamicValues bad_linearize;
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check linearize to ensure that it fails for bad linearization points
	CHECK_EXCEPTION(nle->linearize(bad_linearize, *bad_linearize.orderingArbitrary()), std::invalid_argument);
}

/* ************************************************************************* */
TEST ( NonlinearEquality, error ) {
	Pose2 value = Pose2(2.1, 1.0, 2.0);
	Pose2 wrong = Pose2(2.1, 3.0, 4.0);
	DynamicValues feasible, bad_linearize;
	feasible.insert(key, value);
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check error function outputs
	Vector actual = nle->unwhitenedError(feasible);
	EXPECT(assert_equal(actual, zero(3)));

	actual = nle->unwhitenedError(bad_linearize);
	EXPECT(assert_equal(actual, repeat(3, std::numeric_limits<double>::infinity())));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, equals ) {
	Pose2 value1 = Pose2(2.1, 1.0, 2.0);
	Pose2 value2 = Pose2(2.1, 3.0, 4.0);

	// create some constraints to compare
	shared_poseNLE nle1(new PoseNLE(key, value1));
	shared_poseNLE nle2(new PoseNLE(key, value1));
	shared_poseNLE nle3(new PoseNLE(key, value2));

	// verify
	EXPECT(nle1->equals(*nle2));  // basic equality = true
	EXPECT(nle2->equals(*nle1));  // test symmetry of equals()
	EXPECT(!nle1->equals(*nle3)); // test config
}

/* ************************************************************************* */
TEST ( NonlinearEquality, allow_error_pose ) {
	PoseKey key1(1);
	Pose2 feasible1(1.0, 2.0, 3.0);
	double error_gain = 500.0;
	PoseNLE nle(key1, feasible1, error_gain);

	// the unwhitened error should provide logmap to the feasible state
	Pose2 badPoint1(0.0, 2.0, 3.0);
	Vector actVec = nle.evaluateError(badPoint1);
	Vector expVec = Vector_(3, -0.989992, -0.14112, 0.0);
	EXPECT(assert_equal(expVec, actVec, 1e-5));

	// the actual error should have a gain on it
	DynamicValues config;
	config.insert(key1, badPoint1);
	double actError = nle.error(config);
	DOUBLES_EQUAL(500.0, actError, 1e-9);

	// check linearization
	GaussianFactor::shared_ptr actLinFactor = nle.linearize(config, *config.orderingArbitrary());
	Matrix A1 = eye(3,3);
	Vector b = expVec;
	SharedDiagonal model = noiseModel::Constrained::All(3);
	GaussianFactor::shared_ptr expLinFactor(new JacobianFactor(0, A1, b, model));
	EXPECT(assert_equal(*expLinFactor, *actLinFactor, 1e-5));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, allow_error_optimize ) {
	PoseKey key1(1);
	Pose2 feasible1(1.0, 2.0, 3.0);
	double error_gain = 500.0;
	PoseNLE nle(key1, feasible1, error_gain);

	// add to a graph
	boost::shared_ptr<PoseGraph> graph(new PoseGraph());
	graph->add(nle);

	// initialize away from the ideal
	Pose2 initPose(0.0, 2.0, 3.0);
	boost::shared_ptr<DynamicValues> init(new DynamicValues());
	init->insert(key1, initPose);

	// optimize
	boost::shared_ptr<Ordering> ord(new Ordering());
	ord->push_back(key1);
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-5, 1e-5);
	PoseOptimizer optimizer(graph, init, ord, params);
	PoseOptimizer result = optimizer.levenbergMarquardt();

	// verify
	DynamicValues expected;
	expected.insert(key1, feasible1);
	EXPECT(assert_equal(expected, *result.values()));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, allow_error_optimize_with_factors ) {

	// create a hard constraint
	PoseKey key1(1);
	Pose2 feasible1(1.0, 2.0, 3.0);

	// initialize away from the ideal
	boost::shared_ptr<DynamicValues> init(new DynamicValues());
	Pose2 initPose(0.0, 2.0, 3.0);
	init->insert(key1, initPose);

	double error_gain = 500.0;
	PoseNLE nle(key1, feasible1, error_gain);

	// create a soft prior that conflicts
	PosePrior prior(key1, initPose, noiseModel::Isotropic::Sigma(3, 0.1));

	// add to a graph
	boost::shared_ptr<PoseGraph> graph(new PoseGraph());
	graph->add(nle);
	graph->add(prior);

	// optimize
	boost::shared_ptr<Ordering> ord(new Ordering());
	ord->push_back(key1);
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-5, 1e-5);
	PoseOptimizer optimizer(graph, init, ord, params);
	PoseOptimizer result = optimizer.levenbergMarquardt();

	// verify
	DynamicValues expected;
	expected.insert(key1, feasible1);
	EXPECT(assert_equal(expected, *result.values()));
}

/* ************************************************************************* */
SharedDiagonal hard_model = noiseModel::Constrained::All(2);
SharedDiagonal soft_model = noiseModel::Isotropic::Sigma(2, 1.0);

typedef NonlinearFactorGraph Graph;
typedef boost::shared_ptr<Graph> shared_graph;
typedef boost::shared_ptr<DynamicValues> shared_values;
typedef NonlinearOptimizer<Graph> Optimizer;

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
	EXPECT_DOUBLES_EQUAL(500.0, constraint.error(config2), tol);
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
	double mu = 10.0;
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

	// verify error values
	EXPECT(constraint->active(*initValues));

	DynamicValues expected;
	expected.insert(key, truth_pt);
	EXPECT(constraint->active(expected));
	EXPECT_DOUBLES_EQUAL(0.0, constraint->error(expected), tol);

	Optimizer::shared_values actual = Optimizer::optimizeLM(graph, initValues);
	EXPECT(assert_equal(expected, *actual, tol));
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
	EXPECT_DOUBLES_EQUAL(1000.0, constraint.error(config2), tol);
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
	DynamicValues expected;
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

	DynamicValues expected;
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

	DynamicValues expected;
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
typedef boost::shared_ptr<DynamicValues> shared_vconfig;
typedef visualSLAM::Graph VGraph;
typedef NonlinearOptimizer<VGraph> VOptimizer;

// factors for visual slam
typedef NonlinearEquality2<visualSLAM::PointKey> Point3Equality;

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

	shared_vconfig initValues(new DynamicValues());
	initValues->insert(x1, pose1);
	initValues->insert(x2, pose2);
	initValues->insert(l1, landmark1);
	initValues->insert(l2, landmark2);

	// optimize
	VOptimizer::shared_values actual = VOptimizer::optimizeLM(graph, initValues);

	// create config
	DynamicValues truthValues;
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
