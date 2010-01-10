/**
 *  @file  testPose2Factor.cpp
 *  @brief Unit tests for Pose2Factor Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Vector.h"
#include "numericalDerivative.h"
#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"
#include "Pose2Factor.h"
#include "Ordering.h"
#include "Pose2Graph.h"

using namespace std;
using namespace gtsam;

// Common measurement covariance
static double sx=0.5, sy=0.5,st=0.1;
static Matrix covariance = Matrix_(3,3,
		sx*sx, 0.0, 0.0,
		0.0, sy*sy, 0.0,
		0.0, 0.0, st*st
		);

/* ************************************************************************* */
// Very simple test establishing Ax-b \approx h(x)-z
TEST( Pose2Factor, error )
{
	// Choose a linearization point
	Pose2 p1; // robot at origin
	Pose2 p2(1, 0, 0); // robot at (1,0)
	Pose2Config x0;
	x0.insert("p1", p1);
	x0.insert("p2", p2);

	// Create factor
	Pose2 z = between(p1,p2);
	Pose2Factor factor("p1", "p2", z, covariance);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> linear = factor.linearize(x0);

	// Check error at x0 = zero !
	VectorConfig delta;
	delta.insert("p1", zero(3));
	delta.insert("p2", zero(3));
	Vector error_at_zero = Vector_(3,0.0,0.0,0.0);
	CHECK(assert_equal(error_at_zero,factor.error_vector(x0)));
	CHECK(assert_equal(-error_at_zero,linear->error_vector(delta)));

	// Check error after increasing p2
	VectorConfig plus = delta + VectorConfig("p2", Vector_(3, 0.1, 0.0, 0.0));
	Pose2Config x1 = expmap(x0, plus);
	Vector error_at_plus = Vector_(3,-0.1/sx,0.0,0.0);
	CHECK(assert_equal(error_at_plus,factor.error_vector(x1)));
	CHECK(assert_equal(-error_at_plus,linear->error_vector(plus)));
}

/* ************************************************************************* */
// common Pose2Factor for tests below
static Pose2 measured(2,2,M_PI_2);
static Pose2Factor factor("p1","p2",measured, covariance);

/* ************************************************************************* */
TEST( Pose2Factor, rhs )
{
	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4.1) looking at negative (ground truth is at -1,4)
	Pose2Config x0;
	x0.insert("p1",p1);
	x0.insert("p2",p2);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> linear = factor.linearize(x0);

	// Check RHS
	Pose2 hx0 = between(p1,p2);
	CHECK(assert_equal(Pose2(2.1, 2.1, M_PI_2),hx0));
	Vector expected_b = Vector_(3, -0.1/sx, 0.1/sy, 0.0);
	CHECK(assert_equal(expected_b,factor.error_vector(x0)));
	CHECK(assert_equal(expected_b,linear->get_b()));
}

/* ************************************************************************* */
// The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
// Hence i.e., b = approximates z-h(x0) = error_vector(x0)
Vector h(const Pose2& p1,const Pose2& p2) {
	Pose2Config x;
	x.insert("p1",p1);
	x.insert("p2",p2);
	return -factor.error_vector(x);
}

/* ************************************************************************* */
TEST( Pose2Factor, linearize )
{
	// Choose a linearization point at ground truth
	Pose2 p1(1,2,M_PI_2);
	Pose2 p2(-1,4,M_PI);
	Pose2Config x0;
	x0.insert("p1",p1);
	x0.insert("p2",p2);

	// expected linearization
	Matrix square_root_inverse_covariance = Matrix_(3,3,
	    2.0, 0.0, 0.0,
	    0.0, 2.0, 0.0,
	    0.0, 0.0, 10.0
	);
	Matrix expectedH1 = square_root_inverse_covariance*Matrix_(3,3,
	    0.0,-1.0,-2.0,
	    1.0, 0.0,-2.0,
	    0.0, 0.0,-1.0
	);
	Matrix expectedH2 = square_root_inverse_covariance*Matrix_(3,3,
	    1.0, 0.0, 0.0,
	    0.0, 1.0, 0.0,
	    0.0, 0.0, 1.0
	);
	Vector expected_b = Vector_(3, 0.0, 0.0, 0.0);

	// expected linear factor
	GaussianFactor expected("p1", expectedH1, "p2", expectedH2, expected_b, 1.0);

	// Actual linearization
	boost::shared_ptr<GaussianFactor> actual = factor.linearize(x0);
	CHECK(assert_equal(expected,*actual));

	// Numerical do not work out because BetweenFactor is approximate ?
	Matrix numericalH1 = numericalDerivative21(h, p1, p2, 1e-5);
	CHECK(assert_equal(expectedH1,numericalH1));
	Matrix numericalH2 = numericalDerivative22(h, p1, p2, 1e-5);
	CHECK(assert_equal(expectedH2,numericalH2));
}

/* ************************************************************************* */
bool poseCompare(const std::string& key,
    const gtsam::Pose2Config& feasible,
    const gtsam::Pose2Config& input) {
  return feasible.get(key).equals(input.get(key));
}

/* ************************************************************************* */
TEST(Pose2Factor, optimize) {

	// create a Pose graph with one equality constraint and one measurement
  Pose2Graph fg;
  Pose2Config feasible;
  feasible.insert("p0", Pose2(0,0,0));
  fg.push_back(Pose2Graph::sharedFactor(
      new NonlinearEquality<Pose2Config>("p0", feasible, dim(Pose2()), poseCompare)));
  fg.add("p0", "p1", Pose2(1,2,M_PI_2), covariance);

  // Create initial config
  boost::shared_ptr<Pose2Config> initial(new Pose2Config());
  initial->insert("p0", Pose2(0,0,0));
  initial->insert("p1", Pose2(0,0,0));

  // Choose an ordering and optimize
  Ordering ordering;
  ordering += "p0","p1";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Config> Optimizer;
  Optimizer optimizer(fg, ordering, initial);
  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
  //Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  optimizer = optimizer.levenbergMarquardt(1e-15, 1e-15, verbosity);

  // Check with expected config
  Pose2Config expected;
  expected.insert("p0", Pose2(0,0,0));
  expected.insert("p1", Pose2(1,2,M_PI_2));
  CHECK(assert_equal(expected, *optimizer.config()));

}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST(Pose2Factor, optimizeCircle) {

	// Create a hexagon of poses
	Pose2Config hexagon = pose2Circle(6,1.0,'p');
  Pose2 p0 = hexagon["p0"], p1 = hexagon["p1"];

	// create a Pose graph with one equality constraint and one measurement
  Pose2Graph fg;
  Pose2Config feasible;
  feasible.insert("p0", p0);
  fg.push_back(Pose2Graph::sharedFactor(
      new NonlinearEquality<Pose2Config>("p0", feasible, dim(Pose2()), poseCompare)));
  Pose2 delta = between(p0,p1);
  fg.add("p0", "p1", delta, covariance);
  fg.add("p1", "p2", delta, covariance);
  fg.add("p2", "p3", delta, covariance);
  fg.add("p3", "p4", delta, covariance);
  fg.add("p4", "p5", delta, covariance);
  fg.add("p5", "p0", delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose2Config> initial(new Pose2Config());
  initial->insert("p0", p0);
  initial->insert("p1", expmap(hexagon["p1"],Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert("p2", expmap(hexagon["p2"],Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert("p3", expmap(hexagon["p3"],Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert("p4", expmap(hexagon["p4"],Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert("p5", expmap(hexagon["p5"],Vector_(3,-0.1, 0.1,-0.1)));

  // Choose an ordering and optimize
  Ordering ordering;
  ordering += "p0","p1","p2","p3","p4","p5";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Config> Optimizer;
  Optimizer optimizer(fg, ordering, initial);
//  Optimizer::verbosityLevel verbosity = Optimizer::SILENT;
  Optimizer::verbosityLevel verbosity = Optimizer::ERROR;
  optimizer = optimizer.levenbergMarquardt(1e-15, 1e-15, verbosity);

  Pose2Config actual = *optimizer.config();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual));

  // Check loop closure
  CHECK(assert_equal(delta,between(actual["p5"],actual["p0"])));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
