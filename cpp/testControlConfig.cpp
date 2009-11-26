/**
 * @file testControlConfig.cpp
 * @brief Test for configuration using 2D control inputs on a PV model
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <ControlConfig.h>
#include <ControlPoint.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( ControlConfig, basic ) {

	// create a config
	ControlConfig config;

	// add a robot to the config
	string r1 = "R1", r2 = "R2";
	config.addAgent(r1);
	config.addAgent(r2);

	// add some states for each of the robots
	ControlPoint s1,
				 s2(Pose2(1.0, 1.0, 1.0), Pose2(), 2.0),
				 s3(Pose2(1.0, 1.0, 1.0), Pose2(), 3.0);
	config.addPoint(r1, s1);
	config.addPoint(r1, s2);
	config.addPoint(r1, s3);

	// get the path back out again
	ControlConfig::path act1 = config.getPath(r1);
	CHECK(act1.size() == 3);
	CHECK(assert_equal(act1.at(0), s1));
	CHECK(assert_equal(act1.at(1), s2));
	CHECK(assert_equal(act1.at(2), s3));

	// check size
	CHECK(config.size() == 2);
}

/* ************************************************************************* */
TEST ( ControlConfig, equals ) {
	ControlConfig cfg1, cfg2, cfg3;
	cfg1.addAgent("r1");
	cfg2.addAgent("r1");
	cfg3.addAgent("r2");

	CHECK(assert_equal(cfg1, cfg2));
	CHECK(!cfg1.equals(cfg3));

	ControlPoint s1, s2(Pose2(1.0, 1.0, 1.0), Pose2(), 2.0);
	cfg1.addPoint("r1", s1);
	cfg2.addPoint("r1", s2);

	CHECK(!cfg1.equals(cfg2));
}

/* ************************************************************************* */
TEST ( ControlConfig, exmap ) {
	// create a config with two agents and some trajectories
	ControlConfig config;
	ControlPoint s1,
				 s2(Pose2(1.0, 1.0, 1.0), Pose2(), 1.0),
				 s3(Pose2(2.0, 2.0, 2.0), Pose2(0.1, 0.2, 0.3), 2.0),
				 s4(Pose2(1.0, 2.0, 1.0), Pose2(), 0.0),
				 s5(Pose2(3.0, 4.0, 3.0), Pose2(0.4, 0.5, 0.6), 1.5);


	config.addAgent("r1");
	config.addPoint("r1", s1);
	config.addPoint("r1", s2);
	config.addPoint("r1", s3);
	config.addAgent("r2");
	config.addPoint("r2", s4);
	config.addPoint("r2", s5);

	// create a delta config
	VectorConfig delta;
	Vector d1 = repeat(7, 0.1);
	Vector d2 = repeat(7, 0.2);
	Vector d3 = repeat(7, 0.3);
	Vector dother = repeat(7, 100.0);
	delta.insert("r1_0", d1);
	delta.insert("r1_1", d2);
	delta.insert("r1_2", d3);
	delta.insert("r2_0", d1);
	delta.insert("r2_1", d2);
	delta.insert("penguin", dother);


	ControlConfig actual = config.exmap(delta);

	// Verify
	ControlConfig expected;
	expected.addAgent("r1");
	expected.addPoint("r1", s1.exmap(d1));
	expected.addPoint("r1", s2.exmap(d2));
	expected.addPoint("r1", s3.exmap(d3));
	expected.addAgent("r2");
	expected.addPoint("r2", s4.exmap(d1));
	expected.addPoint("r2", s5.exmap(d2));

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
