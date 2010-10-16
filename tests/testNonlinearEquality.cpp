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

#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>

#include <gtsam/nonlinear/LieValues-inl.h>

using namespace std;
using namespace gtsam;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef LieValues<PoseKey> PoseValues;
typedef PriorFactor<PoseValues, PoseKey> PosePrior;
typedef NonlinearEquality<PoseValues, PoseKey> PoseNLE;
typedef boost::shared_ptr<PoseNLE> shared_poseNLE;

typedef NonlinearFactorGraph<PoseValues> PoseGraph;
typedef NonlinearOptimizer<PoseGraph,PoseValues> PoseOptimizer;

PoseKey key(1);

/* ************************************************************************* */
TEST ( NonlinearEquality, linearization ) {
	Pose2 value = Pose2(2.1, 1.0, 2.0);
	PoseValues linearize;
	linearize.insert(key, value);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check linearize
	SharedDiagonal constraintModel = noiseModel::Constrained::All(3);
	GaussianFactor expLF(0, eye(3), zero(3), constraintModel);
	GaussianFactor::shared_ptr actualLF = nle->linearize(linearize, *linearize.orderingArbitrary());
	EXPECT(assert_equal(*actualLF, expLF));
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_pose ) {

	PoseKey key(1);
	Pose2 value;
	PoseValues config;
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
	PoseValues bad_linearize;
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
	PoseValues bad_linearize;
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
	PoseValues bad_linearize;
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
	PoseValues feasible, bad_linearize;
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
	PoseValues config;
	config.insert(key1, badPoint1);
	double actError = nle.error(config);
	DOUBLES_EQUAL(500.0, actError, 1e-9);

	// check linearization
	GaussianFactor::shared_ptr actLinFactor = nle.linearize(config, *config.orderingArbitrary());
	Matrix A1 = eye(3,3);
	Vector b = expVec;
	SharedDiagonal model = noiseModel::Constrained::All(3);
	GaussianFactor::shared_ptr expLinFactor(new GaussianFactor(0, A1, b, model));
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
	boost::shared_ptr<PoseValues> init(new PoseValues());
	init->insert(key1, initPose);

	// optimize
	boost::shared_ptr<Ordering> ord(new Ordering());
	ord->push_back(key1);
	PoseOptimizer::shared_solver solver(new PoseOptimizer::solver(ord));
	PoseOptimizer optimizer(graph, init, solver);
	double relThresh = 1e-5, absThresh = 1e-5;
	PoseOptimizer result = optimizer.levenbergMarquardt(relThresh, absThresh, PoseOptimizer::Parameters::SILENT);

	// verify
	PoseValues expected;
	expected.insert(key1, feasible1);
	EXPECT(assert_equal(expected, *result.config()));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, allow_error_optimize_with_factors ) {

	// create a hard constraint
	PoseKey key1(1);
	Pose2 feasible1(1.0, 2.0, 3.0);

	// initialize away from the ideal
	boost::shared_ptr<PoseValues> init(new PoseValues());
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
	PoseOptimizer::shared_solver solver(new PoseOptimizer::solver(ord));
	PoseOptimizer optimizer(graph, init, solver);
	double relThresh = 1e-5, absThresh = 1e-5;
	PoseOptimizer result = optimizer.levenbergMarquardt(relThresh, absThresh, PoseOptimizer::Parameters::SILENT);

	// verify
	PoseValues expected;
	expected.insert(key1, feasible1);
	EXPECT(assert_equal(expected, *result.config()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
