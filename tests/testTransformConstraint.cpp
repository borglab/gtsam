/*
 * @file testTransformConstraint.cpp
 * @author Alex Cunningham
 */

#include <iostream>

#include <boost/bind.hpp>

#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

#include <gtsam/slam/TransformConstraint.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

// implementations
#include <gtsam/nonlinear/LieConfig-inl.h>
#include <gtsam/nonlinear/TupleConfig-inl.h>
#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>

using namespace std;
using namespace gtsam;
using namespace boost;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef TypedSymbol<Point2, 'l'> PointKey;
typedef TypedSymbol<Pose2, 'T'> TransformKey;

typedef LieConfig<PoseKey> PoseConfig;
typedef LieConfig<PointKey> PointConfig;
typedef LieConfig<TransformKey> TransformConfig;

typedef TupleConfig3< PoseConfig, PointConfig, TransformConfig > DDFConfig;
typedef NonlinearFactorGraph<DDFConfig> DDFGraph;
typedef NonlinearOptimizer<DDFGraph, DDFConfig> Optimizer;

typedef NonlinearEquality<DDFConfig, PoseKey> PoseConstraint;
typedef NonlinearEquality<DDFConfig, PointKey> PointConstraint;
typedef NonlinearEquality<DDFConfig, TransformKey> TransformPriorConstraint;

typedef TransformConstraint<DDFConfig, PointKey, TransformKey> PointTransformConstraint;

PointKey lA1(1), lA2(2), lB1(3);
TransformKey t1(1);

/* ************************************************************************* */
TEST( TransformConstraint, equals ) {
	PointTransformConstraint c1(lB1, t1, lA1),
			   c2(lB1, t1, lA1),
			   c3(lB1, t1, lA2);

	CHECK(assert_equal(c1, c1));
	CHECK(assert_equal(c1, c2));
	CHECK(!c1.equals(c3));
}

/* ************************************************************************* */
LieVector evaluateError_(const PointTransformConstraint& c,
		const Point2& global, const Pose2& trans, const Point2& local) {
	return LieVector(c.evaluateError(global, trans, local));
}
TEST( TransformConstraint, jacobians ) {

	// from examples below
	Point2 local(2.0, 3.0), global(-1.0, 2.0);
	Pose2 trans(1.5, 2.5, 0.3);

	PointTransformConstraint tc(lA1, t1, lB1);
	Matrix actualDT, actualDL, actualDF;
	tc.evaluateError(global, trans, local, actualDF, actualDT, actualDL);

	Matrix numericalDT, numericalDL, numericalDF;
	numericalDF = numericalDerivative31<LieVector,Point2,Pose2,Point2>(
			boost::bind(evaluateError_, tc, _1, _2, _3),
			global, trans, local, 1e-5);
	numericalDT = numericalDerivative32<LieVector,Point2,Pose2,Point2>(
			boost::bind(evaluateError_, tc, _1, _2, _3),
			global, trans, local, 1e-5);
	numericalDL = numericalDerivative33<LieVector,Point2,Pose2,Point2>(
			boost::bind(evaluateError_, tc, _1, _2, _3),
			global, trans, local, 1e-5);

	CHECK(assert_equal(numericalDF, actualDF));
	CHECK(assert_equal(numericalDL, actualDL));
	CHECK(assert_equal(numericalDT, actualDT));
}

/* ************************************************************************* */
TEST( TransformConstraint, jacobians_zero ) {

	// get values that are ideal
	Pose2 trans(2.0, 3.0, 0.0);
	Point2 global(5.0, 6.0);
	Point2 local = trans.transform_from(global);

	PointTransformConstraint tc(lA1, t1, lB1);
	Vector actCost = tc.evaluateError(global, trans, local),
		   expCost = zero(2);
	CHECK(assert_equal(expCost, actCost, 1e-5));

	Matrix actualDT, actualDL, actualDF;
	tc.evaluateError(global, trans, local, actualDF, actualDT, actualDL);

	Matrix numericalDT, numericalDL, numericalDF;
	numericalDF = numericalDerivative31<LieVector,Point2,Pose2,Point2>(
			boost::bind(evaluateError_, tc, _1, _2, _3),
			global, trans, local, 1e-5);
	numericalDT = numericalDerivative32<LieVector,Point2,Pose2,Point2>(
			boost::bind(evaluateError_, tc, _1, _2, _3),
			global, trans, local, 1e-5);
	numericalDL = numericalDerivative33<LieVector,Point2,Pose2,Point2>(
			boost::bind(evaluateError_, tc, _1, _2, _3),
			global, trans, local, 1e-5);

	CHECK(assert_equal(numericalDF, actualDF));
	CHECK(assert_equal(numericalDL, actualDL));
	CHECK(assert_equal(numericalDT, actualDT));
}

/* ************************************************************************* */
TEST( TransformConstraint, converge_trans ) {

	// initial points
	Point2 local1(2.0, 2.0), local2(4.0, 5.0),
			global1(-1.0, 5.0), global2(2.0, 3.0);
	Pose2 transIdeal(7.0, 3.0, M_PI/2);

	// verify direction
	CHECK(assert_equal(local1, transIdeal.transform_from(global1)));
	CHECK(assert_equal(local2, transIdeal.transform_from(global2)));

	// choose transform
//	Pose2 trans = transIdeal; // ideal - works
//	Pose2 trans = transIdeal * Pose2(0.1, 1.0, 0.00);  // translation - works
//	Pose2 trans = transIdeal * Pose2(10.1, 1.0, 0.00);  // large translation - works
//	Pose2 trans = transIdeal * Pose2(0.0, 0.0, 0.1);   // small rotation - works
	Pose2 trans = transIdeal * Pose2(-200.0, 100.0, 1.3); // combined - works
//	Pose2 trans = transIdeal * Pose2(-200.0, 100.0, 2.0); // beyond pi/2 - fails

	// keys
	PointKey localK1(1), localK2(2),
			 globalK1(3), globalK2(4);
	TransformKey transK(1);

	DDFGraph graph;

	graph.add(PointTransformConstraint(globalK1, transK, localK1));
	graph.add(PointTransformConstraint(globalK2, transK, localK2));

	// hard constraints on points
	double error_gain = 1000.0;
	graph.add(PointConstraint(localK1, local1, error_gain));
	graph.add(PointConstraint(localK2, local2, error_gain));
	graph.add(PointConstraint(globalK1, global1, error_gain));
	graph.add(PointConstraint(globalK2, global2, error_gain));

	// create initial estimate
	DDFConfig init;
	init.insert(localK1, local1);
	init.insert(localK2, local2);
	init.insert(globalK1, global1);
	init.insert(globalK2, global2);
	init.insert(transK, trans);

	// optimize
	Optimizer::shared_config actual = Optimizer::optimizeLM(graph, init);

	DDFConfig expected;
	expected.insert(localK1, local1);
	expected.insert(localK2, local2);
	expected.insert(globalK1, global1);
	expected.insert(globalK2, global2);
	expected.insert(transK, transIdeal);

	CHECK(assert_equal(expected, *actual, 1e-4));
}

/* ************************************************************************* */
TEST( TransformConstraint, converge_local ) {

	// initial points
	Point2 global(-1.0, 2.0);
//	Pose2 trans(1.5, 2.5, 0.3); // original
//	Pose2 trans(1.5, 2.5, 1.0); // larger rotation
	Pose2 trans(1.5, 2.5, 3.1); // significant rotation

	Point2 idealLocal = trans.transform_from(global);

	// perturb the initial estimate
//	Point2 local = idealLocal; // Ideal case - works
//	Point2 local = idealLocal + Point2(1.0, 0.0); // works
	Point2 local = idealLocal + Point2(-10.0, 10.0); // works


	// keys
	PointKey localK(1), globalK(2);
	TransformKey transK(1);

	DDFGraph graph;
	double error_gain = 1000.0;
	graph.add(PointTransformConstraint(globalK, transK, localK));
	graph.add(PointConstraint(globalK, global, error_gain));
	graph.add(TransformPriorConstraint(transK, trans, error_gain));

	// create initial estimate
	DDFConfig init;
	init.insert(localK, local);
	init.insert(globalK, global);
	init.insert(transK, trans);

	// optimize
	Optimizer::shared_config actual = Optimizer::optimizeLM(graph, init);

	CHECK(assert_equal(idealLocal, actual->at(localK), 1e-5));
}

/* ************************************************************************* */
TEST( TransformConstraint, converge_global ) {

	// initial points
	Point2 local(2.0, 3.0);
//	Pose2 trans(1.5, 2.5, 0.3); // original
//	Pose2 trans(1.5, 2.5, 1.0); // larger rotation
	Pose2 trans(1.5, 2.5, 3.1); // significant rotation

	Point2 idealForeign = trans.inverse().transform_from(local);

	// perturb the initial estimate
//	Point2 global = idealForeign; // Ideal - works
//	Point2 global = idealForeign + Point2(1.0, 0.0); // simple - works
	Point2 global = idealForeign + Point2(10.0, -10.0); // larger - works

	// keys
	PointKey localK(1), globalK(2);
	TransformKey transK(1);

	DDFGraph graph;
	double error_gain = 1000.0;
	graph.add(PointTransformConstraint(globalK, transK, localK));
	graph.add(PointConstraint(localK, local, error_gain));
	graph.add(TransformPriorConstraint(transK, trans, error_gain));

	// create initial estimate
	DDFConfig init;
	init.insert(localK, local);
	init.insert(globalK, global);
	init.insert(transK, trans);

	// optimize
	Optimizer::shared_config actual = Optimizer::optimizeLM(graph, init);

	// verify
	CHECK(assert_equal(idealForeign, actual->at(globalK), 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


