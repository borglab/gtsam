/*
 * @file testNonlinearEquality.cpp
 * @author Alex Cunningham
 */

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/VectorConfig.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>

#include <gtsam/nonlinear/LieConfig-inl.h>

using namespace std;
using namespace gtsam;

typedef NonlinearEquality<VectorConfig,string,Vector> NLE;
typedef boost::shared_ptr<NLE> shared_nle;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef LieConfig<PoseKey, Pose2> PoseConfig;
typedef PriorFactor<PoseConfig, PoseKey, Pose2> PosePrior;
typedef NonlinearEquality<PoseConfig, PoseKey, Pose2> PoseNLE;
typedef boost::shared_ptr<PoseNLE> shared_poseNLE;

typedef NonlinearFactorGraph<PoseConfig> PoseGraph;
typedef NonlinearOptimizer<PoseGraph,PoseConfig> PoseOptimizer;

bool vector_compare(const Vector& a, const Vector& b) {
	return equal_with_abs_tol(a, b, 1e-5);
}

/* ************************************************************************* */
TEST ( NonlinearEquality, linearization ) {
	Symbol key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	VectorConfig linearize;
	linearize.insert(key, value);

	// create a nonlinear equality constraint
	shared_nle nle(new NLE(key, value,vector_compare));

	// check linearize
	SharedDiagonal constraintModel = noiseModel::Constrained::All(2);
	GaussianFactor expLF(key, eye(2), zero(2), constraintModel);
	GaussianFactor::shared_ptr actualLF = nle->linearize(linearize);
	CHECK(assert_equal(*actualLF, expLF));
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_pose ) {

	PoseKey key(1);
	Pose2 value;
	PoseConfig config;
	config.insert(key, value);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	GaussianFactor::shared_ptr actualLF = nle->linearize(config);
	CHECK(true);
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_fail ) {
  Symbol key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	Vector wrong = Vector_(2, 3.0, 4.0);
	VectorConfig bad_linearize;
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_nle nle(new NLE(key, value,vector_compare));

	// check linearize to ensure that it fails for bad linearization points
	CHECK_EXCEPTION(nle->linearize(bad_linearize), std::invalid_argument);
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_fail_pose ) {

	PoseKey key(1);
	Pose2 value(2.0, 1.0, 2.0),
		  wrong(2.0, 3.0, 4.0);
	PoseConfig bad_linearize;
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check linearize to ensure that it fails for bad linearization points
	CHECK_EXCEPTION(nle->linearize(bad_linearize), std::invalid_argument);
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_fail_pose_origin ) {

	PoseKey key(1);
	Pose2 value,
		  wrong(2.0, 3.0, 4.0);
	PoseConfig bad_linearize;
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_poseNLE nle(new PoseNLE(key, value));

	// check linearize to ensure that it fails for bad linearization points
	CHECK_EXCEPTION(nle->linearize(bad_linearize), std::invalid_argument);
}

/* ************************************************************************* */
TEST ( NonlinearEquality, error ) {
  Symbol key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	Vector wrong = Vector_(2, 3.0, 4.0);
	VectorConfig feasible, bad_linearize;
	feasible.insert(key, value);
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_nle nle(new NLE(key, value,vector_compare));

	// check error function outputs
	Vector actual = nle->unwhitenedError(feasible);
	CHECK(assert_equal(actual, zero(2)));

	actual = nle->unwhitenedError(bad_linearize);
	CHECK(assert_equal(actual, repeat(2, std::numeric_limits<double>::infinity())));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, equals ) {
	string key1 = "x";
	Vector value1 = Vector_(2, 1.0, 2.0);
	Vector value2 = Vector_(2, 3.0, 4.0);

	// create some constraints to compare
	shared_nle nle1(new NLE(key1, value1,vector_compare));
	shared_nle nle2(new NLE(key1, value1,vector_compare));
	shared_nle nle3(new NLE(key1, value2,vector_compare));

	// verify
	CHECK(nle1->equals(*nle2));  // basic equality = true
	CHECK(nle2->equals(*nle1));  // test symmetry of equals()
	CHECK(!nle1->equals(*nle3)); // test config
}

/* ************************************************************************* */
TEST ( NonlinearEquality, allow_error_vector ) {
	Symbol key1 = "x";
	Vector feasible1 = Vector_(3, 1.0, 2.0, 3.0);
	double error_gain = 500.0;
	NLE nle(key1, feasible1, error_gain,vector_compare);

	// the unwhitened error should provide logmap to the feasible state
	Vector badPoint1 = Vector_(3, 0.0, 2.0, 3.0);
	Vector actVec = nle.evaluateError(badPoint1);
	Vector expVec = Vector_(3, 1.0, 0.0, 0.0);
	CHECK(assert_equal(expVec, actVec));

	// the actual error should have a gain on it
	VectorConfig config;
	config.insert(key1, badPoint1);
	double actError = nle.error(config);
	DOUBLES_EQUAL(500.0, actError, 1e-9);

	// check linearization
	GaussianFactor::shared_ptr actLinFactor = nle.linearize(config);
	Matrix A1 = eye(3,3);
	Vector b = expVec;
	SharedDiagonal model = noiseModel::Constrained::All(3);
	GaussianFactor::shared_ptr expLinFactor(new GaussianFactor(key1, A1, b, model));
	CHECK(assert_equal(*expLinFactor, *actLinFactor));
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
	CHECK(assert_equal(expVec, actVec, 1e-5));

	// the actual error should have a gain on it
	PoseConfig config;
	config.insert(key1, badPoint1);
	double actError = nle.error(config);
	DOUBLES_EQUAL(500.0, actError, 1e-9);

	// check linearization
	GaussianFactor::shared_ptr actLinFactor = nle.linearize(config);
	Matrix A1 = eye(3,3);
	Vector b = expVec;
	SharedDiagonal model = noiseModel::Constrained::All(3);
	GaussianFactor::shared_ptr expLinFactor(new GaussianFactor(key1, A1, b, model));
	CHECK(assert_equal(*expLinFactor, *actLinFactor, 1e-5));
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
	boost::shared_ptr<PoseConfig> init(new PoseConfig());
	init->insert(key1, initPose);

	// optimize
	boost::shared_ptr<Ordering> ord(new Ordering());
	ord->push_back(key1);
	PoseOptimizer::shared_solver solver(new PoseOptimizer::solver(ord));
	PoseOptimizer optimizer(graph, init, solver);
	double relThresh = 1e-5, absThresh = 1e-5;
	PoseOptimizer result = optimizer.levenbergMarquardt(relThresh, absThresh, PoseOptimizer::SILENT);

	// verify
	PoseConfig expected;
	expected.insert(key1, feasible1);
	CHECK(assert_equal(expected, *result.config()));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, allow_error_optimize_with_factors ) {

	// create a hard constraint
	PoseKey key1(1);
	Pose2 feasible1(1.0, 2.0, 3.0);

	// initialize away from the ideal
	boost::shared_ptr<PoseConfig> init(new PoseConfig());
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
	PoseOptimizer result = optimizer.levenbergMarquardt(relThresh, absThresh, PoseOptimizer::SILENT);

	// verify
	PoseConfig expected;
	expected.insert(key1, feasible1);
	CHECK(assert_equal(expected, *result.config()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
