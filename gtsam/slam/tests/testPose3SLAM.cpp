/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testpose3SLAM.cpp
 *  @author Frank Dellaert
 *  @author Viorela Ila
 **/

#include <gtsam/slam/pose3SLAM.h>
#include <gtsam/slam/PartialPriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost;
using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;

// common measurement covariance
static noiseModel::Gaussian::shared_ptr covariance(noiseModel::Unit::Create(6));

const double tol=1e-5;

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose3Graph, optimizeCircle) {

	// Create a hexagon of poses
	double radius = 10;
	Values hexagon = pose3SLAM::Values::Circle(6,radius);
  Pose3 gT0 = hexagon.at<Pose3>(0), gT1 = hexagon.at<Pose3>(1);

	// create a Pose graph with one equality constraint and one measurement
  pose3SLAM::Graph fg;
  fg.addHardConstraint(0, gT0);
  Pose3 _0T1 = gT0.between(gT1); // inv(gT0)*gT1
  double theta = M_PI/3.0;
  CHECK(assert_equal(Pose3(Rot3::yaw(-theta),Point3(radius*sin(theta),-radius*cos(theta),0)),_0T1));
  fg.addConstraint(0,1, _0T1, covariance);
  fg.addConstraint(1,2, _0T1, covariance);
  fg.addConstraint(2,3, _0T1, covariance);
  fg.addConstraint(3,4, _0T1, covariance);
  fg.addConstraint(4,5, _0T1, covariance);
  fg.addConstraint(5,0, _0T1, covariance);

  // Create initial config
  Values initial;
  initial.insert(0, gT0);
  initial.insert(1, hexagon.at<Pose3>(1).retract(Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));
  initial.insert(2, hexagon.at<Pose3>(2).retract(Vector_(6, 0.1,-0.1, 0.1, 0.1,-0.1, 0.1)));
  initial.insert(3, hexagon.at<Pose3>(3).retract(Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));
  initial.insert(4, hexagon.at<Pose3>(4).retract(Vector_(6, 0.1,-0.1, 0.1, 0.1,-0.1, 0.1)));
  initial.insert(5, hexagon.at<Pose3>(5).retract(Vector_(6,-0.1, 0.1,-0.1,-0.1, 0.1,-0.1)));

  // Choose an ordering and optimize
  Ordering ordering;
  ordering += 0,1,2,3,4,5;

  Values actual = LevenbergMarquardtOptimizer(fg, initial, ordering).optimize();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual,1e-4));

  // Check loop closure
  CHECK(assert_equal(_0T1, actual.at<Pose3>(5).between(actual.at<Pose3>(0)),1e-5));
}

/* ************************************************************************* */
TEST(Pose3Graph, partial_prior_height) {
	typedef PartialPriorFactor<Pose3> Partial;

	// reference: Pose3 Expmap - (0-2: Rot3) (3-5: Point3)

	// height prior - single element interface
	double exp_height = 5.0;
	SharedDiagonal model = noiseModel::Unit::Create(1);
	Pose3 init(Rot3(), Point3(1.0, 2.0, 3.0)), expected(Rot3(), Point3(1.0, 2.0, exp_height));
	Partial height(1, 5, exp_height, model);
	Matrix actA;
	EXPECT(assert_equal(Vector_(1,-2.0), height.evaluateError(init, actA), tol));
	Matrix expA = Matrix_(1, 6, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	EXPECT(assert_equal(expA, actA));

	pose3SLAM::Graph graph;
	graph.add(height);

	Values values;
	values.insert(1, init);

	// linearization
	EXPECT_DOUBLES_EQUAL(2.0, height.error(values), tol);

	Values actual = LevenbergMarquardtOptimizer(graph, values).optimize();
	EXPECT(assert_equal(expected, actual.at<Pose3>(1), tol));
	EXPECT_DOUBLES_EQUAL(0.0, graph.error(actual), tol);
}

/* ************************************************************************* */
TEST( Pose3Factor, error )
{
	// Create example
	Pose3 t1; // origin
	Pose3 t2(Rot3::rodriguez(0.1,0.2,0.3),Point3(0,1,0));
	Pose3 z(Rot3::rodriguez(0.2,0.2,0.3),Point3(0,1.1,0));;

	// Create factor
	SharedNoiseModel I6(noiseModel::Unit::Create(6));
	pose3SLAM::Constraint factor(1, 2, z, I6);

	// Create config
	Values x;
	x.insert(1,t1);
	x.insert(2,t2);

	// Get error h(x)-z -> localCoordinates(z,h(x)) = localCoordinates(z,between(t1,t2))
	Vector actual = factor.unwhitenedError(x);
	Vector expected = z.localCoordinates(t1.between(t2));
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(Pose3Graph, partial_prior_xy) {
	typedef PartialPriorFactor<Pose3> Partial;

	// XY prior - full mask interface
	Vector exp_xy = Vector_(2, 3.0, 4.0);
	SharedDiagonal model = noiseModel::Unit::Create(2);
	Pose3 init(Rot3(), Point3(1.0,-2.0, 3.0)), expected(Rot3(), Point3(3.0, 4.0, 3.0));
	vector<size_t> mask; mask += 3, 4;
	Partial priorXY(1, mask, exp_xy, model);
	Matrix actA;
	EXPECT(assert_equal(Vector_(2,-2.0,-6.0), priorXY.evaluateError(init, actA), tol));
	Matrix expA = Matrix_(2, 6,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	EXPECT(assert_equal(expA, actA));

	pose3SLAM::Graph graph;
	graph.add(priorXY);

	Values values;
	values.insert(1, init);

	Values actual = LevenbergMarquardtOptimizer(graph, values).optimize();
	EXPECT(assert_equal(expected, actual.at<Pose3>(1), tol));
	EXPECT_DOUBLES_EQUAL(0.0, graph.error(actual), tol);
}

// The world coordinate system has z pointing up, y north, x east
// The vehicle has X forward, Y right, Z down
Rot3 R1(Point3( 0, 1, 0), Point3( 1, 0, 0), Point3(0, 0, -1));
Rot3 R2(Point3(-1, 0, 0), Point3( 0, 1, 0), Point3(0, 0, -1));
Rot3 R3(Point3( 0,-1, 0), Point3(-1, 0, 0), Point3(0, 0, -1));
Rot3 R4(Point3( 1, 0, 0), Point3( 0,-1, 0), Point3(0, 0, -1));

/* ************************************************************************* */
TEST( Values, pose3Circle )
{
	// expected is 4 poses tangent to circle with radius 1m
	Values expected;
	expected.insert(0, Pose3(R1, Point3( 1, 0, 0)));
	expected.insert(1, Pose3(R2, Point3( 0, 1, 0)));
	expected.insert(2, Pose3(R3, Point3(-1, 0, 0)));
	expected.insert(3, Pose3(R4, Point3( 0,-1, 0)));

	Values actual = pose3SLAM::Values::Circle(4,1.0);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Values, expmap )
{
	Values expected;
	expected.insert(0, Pose3(R1, Point3( 1.0, 0.1, 0)));
	expected.insert(1, Pose3(R2, Point3(-0.1, 1.0, 0)));
	expected.insert(2, Pose3(R3, Point3(-1.0,-0.1, 0)));
	expected.insert(3, Pose3(R4, Point3( 0.1,-1.0, 0)));

	Ordering ordering(*expected.orderingArbitrary());
	VectorValues delta(expected.dims(ordering));
	delta.vector() = Vector_(24,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0);
	Values actual = pose3SLAM::Values::Circle(4,1.0).retract(delta, ordering);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
