/**
 * @file testControlPoint.cpp
 * @brief A single point in time on a trajectory
 * @author Alex Cunningham
 */

#include <set>
#include <vector>
#include <boost/foreach.hpp>
#include <CppUnitLite/TestHarness.h>
#include <ControlPoint.h>

using namespace std;
using namespace gtsam;

TEST ( ControlPoint, constructors ) {

	// make a default point, stationary at time zero at the origin
	ControlPoint pt1;
	CHECK(assert_equal(pt1.pos(), Pose2()));
	CHECK(assert_equal(pt1.vel(), Pose2()));
	CHECK(pt1.time() < 1e-9); // check for zero time

	// make a point in same place to test constructors
	ControlPoint pt2(Pose2(), Pose2(), 0.0);
	CHECK(assert_equal(pt2.pos(), Pose2()));
	CHECK(assert_equal(pt2.vel(), Pose2()));
	CHECK(pt2.time() < 1e-9); // check for zero time

	// check equality
	CHECK(assert_equal(pt2, pt1));

	// make a specific point
	Pose2 pos(1.0, 2.0, 3.0);
	Pose2 vel(0.1, 0.2, 0.3);
	double time = 0.5;
	ControlPoint pt3(pos, vel, time);
	CHECK(assert_equal(pt3.pos(), pos));
	CHECK(assert_equal(pt3.vel(), vel));
	CHECK(fabs(pt3.time()-time) < 1e-9);

	// use manual constructor
	double posx=1.0, posy=2.0, posr=3.0;
	double velx=0.1, vely=0.2, velr=0.3;
	ControlPoint pt4(posx, posy, posr, velx, vely, velr, time);
	CHECK(assert_equal(pt3, pt4));
}

TEST ( ControlPoint, exmap ) {
	// add a delta to an existing point
	Pose2 pos(1.0, 2.0, 3.0);
	Pose2 vel(0.1, 0.2, 0.3);
	double time = 0.5;
	ControlPoint pt(pos, vel, time);

	// ensure that zero vector doesn't change the point
	ControlPoint act1 = pt.exmap(zero(7));
	CHECK(assert_equal(pt, act1));

	// add a real delta
	Vector delta1 = Vector_(7, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7);
	ControlPoint act2 = pt.exmap(delta1);
	Pose2 pos_exp(1.1, 2.2, 3.3);
	Pose2 vel_exp(0.5, 0.7, 0.9);
	double time_exp = 1.2;
	ControlPoint pt_exp(pos_exp, vel_exp, time_exp);
	CHECK(assert_equal(act2, pt_exp));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
