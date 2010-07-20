/*
 * testTupleConfig.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: richard
 */

#include <CppUnitLite/TestHarness.h>
#include <stdexcept>

#define GTSAM_MAGIC_KEY

#include <Pose2.h>
#include <Point2.h>
#include <Pose3.h>
#include <Point3.h>

#include "Vector.h"
#include "Key.h"
#include "VectorConfig.h"
#include "TupleConfig-inl.h"

using namespace gtsam;
using namespace std;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef TypedSymbol<Point2, 'l'> PointKey;
typedef LieConfig<PoseKey, Pose2> PoseConfig;
typedef LieConfig<PointKey, Point2> PointConfig;
typedef TupleConfig2<PoseConfig, PointConfig> Config;

/* ************************************************************************* */
TEST( TupleConfig, constructors )
{
	Pose2 x1(1,2,3), x2(6,7,8);
	Point2 l1(4,5), l2(9,10);

	Config::Config1 cfg1;
	cfg1.insert(PoseKey(1), x1);
	cfg1.insert(PoseKey(2), x2);
	Config::Config2 cfg2;
	cfg2.insert(PointKey(1), l1);
	cfg2.insert(PointKey(2), l2);

	Config actual(cfg1, cfg2), expected;
	expected.insert(PoseKey(1), x1);
	expected.insert(PoseKey(2), x2);
	expected.insert(PointKey(1), l1);
	expected.insert(PointKey(2), l2);

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( TupleConfig, insert_equals1 )
{
	Pose2 x1(1,2,3), x2(6,7,8);
	Point2 l1(4,5), l2(9,10);

  Config expected;
  expected.insert(PoseKey(1), x1);
  expected.insert(PoseKey(2), x2);
  expected.insert(PointKey(1), l1);
  expected.insert(PointKey(2), l2);

  Config actual;
  actual.insert(PoseKey(1), x1);
  actual.insert(PoseKey(2), x2);
  actual.insert(PointKey(1), l1);
  actual.insert(PointKey(2), l2);

  CHECK(assert_equal(expected,actual));
}

TEST( TupleConfig, insert_equals2 )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config config1;
  config1.insert(PoseKey(1), x1);
  config1.insert(PoseKey(2), x2);
  config1.insert(PointKey(1), l1);
  config1.insert(PointKey(2), l2);

  Config config2;
  config2.insert(PoseKey(1), x1);
  config2.insert(PoseKey(2), x2);
  config2.insert(PointKey(1), l1);

  CHECK(!config1.equals(config2));

  config2.insert(2, Point2(9,11));

  CHECK(!config1.equals(config2));
}

///* ************************************************************************* */
TEST( TupleConfig, insert_duplicate )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config config1;
  config1.insert(1, x1); // 3
  config1.insert(2, x2); // 6
  config1.insert(1, l1); // 8
  config1.insert(2, l2); // 10
  config1.insert(2, l1); // still 10 !!!!

  CHECK(assert_equal(l2, config1[PointKey(2)]));
  LONGS_EQUAL(4,config1.size());
  LONGS_EQUAL(10,config1.dim());
}

/* ************************************************************************* */
TEST( TupleConfig, size_dim )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config config1;
  config1.insert(PoseKey(1), x1);
  config1.insert(PoseKey(2), x2);
  config1.insert(PointKey(1), l1);
  config1.insert(PointKey(2), l2);

  CHECK(config1.size() == 4);
  CHECK(config1.dim() == 10);
}

/* ************************************************************************* */
TEST(TupleConfig, at)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config config1;
  config1.insert(PoseKey(1), x1);
  config1.insert(PoseKey(2), x2);
  config1.insert(PointKey(1), l1);
  config1.insert(PointKey(2), l2);

  CHECK(assert_equal(x1, config1[PoseKey(1)]));
  CHECK(assert_equal(x2, config1[PoseKey(2)]));
  CHECK(assert_equal(l1, config1[PointKey(1)]));
  CHECK(assert_equal(l2, config1[PointKey(2)]));

  CHECK_EXCEPTION(config1[PoseKey(3)], std::invalid_argument);
  CHECK_EXCEPTION(config1[PointKey(3)], std::invalid_argument);
}

/* ************************************************************************* */
TEST(TupleConfig, zero_expmap_logmap)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config config1;
  config1.insert(PoseKey(1), x1);
  config1.insert(PoseKey(2), x2);
  config1.insert(PointKey(1), l1);
  config1.insert(PointKey(2), l2);

  VectorConfig expected_zero;
  expected_zero.insert("x1", zero(3));
  expected_zero.insert("x2", zero(3));
  expected_zero.insert("l1", zero(2));
  expected_zero.insert("l2", zero(2));

  CHECK(assert_equal(expected_zero, config1.zero()));

  VectorConfig delta;
  delta.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
  delta.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
  delta.insert("l1", Vector_(2, 1.0, 1.1));
  delta.insert("l2", Vector_(2, 1.3, 1.4));

  Config expected;
  expected.insert(PoseKey(1), expmap(x1, Vector_(3, 1.0, 1.1, 1.2)));
  expected.insert(PoseKey(2), expmap(x2, Vector_(3, 1.3, 1.4, 1.5)));
  expected.insert(PointKey(1), Point2(5.0, 6.1));
  expected.insert(PointKey(2), Point2(10.3, 11.4));

  Config actual = expmap(config1, delta);
  CHECK(assert_equal(expected, actual));

  // Check log
  VectorConfig expected_log = delta;
  VectorConfig actual_log = logmap(config1,actual);
  CHECK(assert_equal(expected_log, actual_log));
}

/* ************************************************************************* */

// some key types
typedef TypedSymbol<Vector, 'L'> LamKey;
typedef TypedSymbol<Pose3, 'a'> Pose3Key;
typedef TypedSymbol<Point3, 'b'> Point3Key;
typedef TypedSymbol<Point3, 'c'> Point3Key2;

// some config types
typedef LieConfig<PoseKey, Pose2> PoseConfig;
typedef LieConfig<PointKey, Point2> PointConfig;
typedef LieConfig<LamKey, Vector> LamConfig;
typedef LieConfig<Pose3Key, Pose3> Pose3Config;
typedef LieConfig<Point3Key, Point3> Point3Config;
typedef LieConfig<Point3Key2, Point3> Point3Config2;

// some TupleConfig types
typedef TupleConfig<PoseConfig, TupleConfigEnd<PointConfig> > ConfigA;
typedef TupleConfig<PoseConfig, TupleConfig<PointConfig, TupleConfigEnd<LamConfig> > > ConfigB;

/* ************************************************************************* */
TEST(TupleConfig, basic_functions) {
	// create some tuple configs
	ConfigA configA;
	ConfigB configB;

	PoseKey x1(1);
	PointKey l1(1);
	LamKey L1(1);
	Pose2 pose1(1.0, 2.0, 0.3);
	Point2 point1(2.0, 3.0);
	Vector lam1 = Vector_(1, 2.3);

	// Insert
	configA.insert(x1, pose1);
	configA.insert(l1, point1);

	configB.insert(x1, pose1);
	configB.insert(l1, point1);
	configB.insert(L1, lam1);

	// bracket operator
	CHECK(assert_equal(configA[x1], pose1));
	CHECK(assert_equal(configA[l1], point1));
	CHECK(assert_equal(configB[x1], pose1));
	CHECK(assert_equal(configB[l1], point1));
	CHECK(assert_equal(configB[L1], lam1));

	// exists
	CHECK(configA.exists(x1));
	CHECK(configA.exists(l1));
	CHECK(configB.exists(x1));
	CHECK(configB.exists(l1));
	CHECK(configB.exists(L1));

	// at
	CHECK(assert_equal(configA.at(x1), pose1));
	CHECK(assert_equal(configA.at(l1), point1));
	CHECK(assert_equal(configB.at(x1), pose1));
	CHECK(assert_equal(configB.at(l1), point1));
	CHECK(assert_equal(configB.at(L1), lam1));

	// size
	CHECK(configA.size() == 2);
	CHECK(configB.size() == 3);

	// dim
	CHECK(configA.dim() == 5);
	CHECK(configB.dim() == 6);

	// erase
	configA.erase(x1);
	CHECK(!configA.exists(x1));
	CHECK(configA.size() == 1);
	configA.erase(l1);
	CHECK(!configA.exists(l1));
	CHECK(configA.size() == 0);
	configB.erase(L1);
	CHECK(!configB.exists(L1));
	CHECK(configB.size() == 2);
}

/* ************************************************************************* */
TEST(TupleConfig, insert_config) {
	ConfigB config1, config2, expected;

	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	LamKey L1(1), L2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);
	Vector lam1 = Vector_(1, 2.3), lam2 = Vector_(1, 4.5);

	config1.insert(x1, pose1);
	config1.insert(l1, point1);
	config1.insert(L1, lam1);

	config2.insert(x2, pose2);
	config2.insert(l2, point2);
	config2.insert(L2, lam2);

	config1.insert(config2);

	expected.insert(x1, pose1);
	expected.insert(l1, point1);
	expected.insert(L1, lam1);
	expected.insert(x2, pose2);
	expected.insert(l2, point2);
	expected.insert(L2, lam2);

	CHECK(assert_equal(expected, config1));
}

/* ************************************************************************* */
TEST( TupleConfig, update_element )
{
	TupleConfig2<PoseConfig, PointConfig> cfg;
	Pose2 x1(2.0, 1.0, 2.0), x2(3.0, 4.0, 5.0);
	Point2 l1(1.0, 2.0), l2(3.0, 4.0);
	PoseKey xk(1);
	PointKey lk(1);

	cfg.insert(xk, x1);
	CHECK(cfg.size() == 1);
	CHECK(assert_equal(x1, cfg.at(xk)));

	cfg.update(xk, x2);
	CHECK(cfg.size() == 1);
	CHECK(assert_equal(x2, cfg.at(xk)));

	cfg.insert(lk, l1);
	CHECK(cfg.size() == 2);
	CHECK(assert_equal(l1, cfg.at(lk)));

	cfg.update(lk, l2);
	CHECK(cfg.size() == 2);
	CHECK(assert_equal(l2, cfg.at(lk)));
}

/* ************************************************************************* */
TEST( TupleConfig, equals )
{
	Pose2 x1(1,2,3), x2(6,7,8), x2_alt(5,6,7);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	ConfigA config1, config2, config3, config4, config5;

	config1.insert(x1k, x1);
	config1.insert(x2k, x2);
	config1.insert(l1k, l1);
	config1.insert(l2k, l2);

	config2.insert(x1k, x1);
	config2.insert(x2k, x2);
	config2.insert(l1k, l1);
	config2.insert(l2k, l2);

	config3.insert(x2k, x2);
	config3.insert(l1k, l1);

	config4.insert(x1k, x1);
	config4.insert(x2k, x2_alt);
	config4.insert(l1k, l1);
	config4.insert(l2k, l2);

	ConfigA config6(config1);

	CHECK(assert_equal(config1,config2));
	CHECK(assert_equal(config1,config1));
	CHECK(!config1.equals(config3));
	CHECK(!config1.equals(config4));
	CHECK(!config1.equals(config5));
	CHECK(assert_equal(config1, config6));
}

/* ************************************************************************* */
TEST(TupleConfig, expmap)
{
	Pose2 x1(1,2,3), x2(6,7,8);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	ConfigA config1;
	config1.insert(x1k, x1);
	config1.insert(x2k, x2);
	config1.insert(l1k, l1);
	config1.insert(l2k, l2);

	VectorConfig delta;
	delta.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
	delta.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
	delta.insert("l1", Vector_(2, 1.0, 1.1));
	delta.insert("l2", Vector_(2, 1.3, 1.4));

	ConfigA expected;
	expected.insert(x1k, expmap(x1, Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(x2k, expmap(x2, Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(l1k, Point2(5.0, 6.1));
	expected.insert(l2k, Point2(10.3, 11.4));

	CHECK(assert_equal(expected, expmap(config1, delta)));
	CHECK(assert_equal(delta, logmap(config1, expected)));
}

/* ************************************************************************* */
TEST(TupleConfig, expmap_typedefs)
{
	Pose2 x1(1,2,3), x2(6,7,8);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	TupleConfig2<PoseConfig, PointConfig> config1, expected, actual;
	config1.insert(x1k, x1);
	config1.insert(x2k, x2);
	config1.insert(l1k, l1);
	config1.insert(l2k, l2);

	VectorConfig delta;
	delta.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
	delta.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
	delta.insert("l1", Vector_(2, 1.0, 1.1));
	delta.insert("l2", Vector_(2, 1.3, 1.4));

	expected.insert(x1k, expmap(x1, Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(x2k, expmap(x2, Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(l1k, Point2(5.0, 6.1));
	expected.insert(l2k, Point2(10.3, 11.4));

	CHECK(assert_equal(expected, expmap(config1, delta)));
	//CHECK(assert_equal(delta, logmap(config1, expected)));
}

/* ************************************************************************* */
TEST(TupleConfig, typedefs)
{
	TupleConfig2<PoseConfig, PointConfig> config1;
	TupleConfig3<PoseConfig, PointConfig, LamConfig> config2;
	TupleConfig4<PoseConfig, PointConfig, LamConfig, Point3Config> config3;
	TupleConfig5<PoseConfig, PointConfig, LamConfig, Point3Config, Pose3Config> config4;
	TupleConfig6<PoseConfig, PointConfig, LamConfig, Point3Config, Pose3Config, Point3Config2> config5;
}

/* ************************************************************************* */
TEST( TupleConfig, pairconfig_style )
{
	PoseKey x1(1);
	PointKey l1(1);
	LamKey L1(1);
	Pose2 pose1(1.0, 2.0, 0.3);
	Point2 point1(2.0, 3.0);
	Vector lam1 = Vector_(1, 2.3);

	PoseConfig config1; config1.insert(x1, pose1);
	PointConfig config2; config2.insert(l1, point1);
	LamConfig config3; config3.insert(L1, lam1);

	// Constructor
	TupleConfig3<PoseConfig, PointConfig, LamConfig> config(config1, config2, config3);

	// access
	CHECK(assert_equal(config1, config.first()));
	CHECK(assert_equal(config2, config.second()));
	CHECK(assert_equal(config3, config.third()));
}

/* ************************************************************************* */
TEST(TupleConfig, insert_config_typedef) {

	TupleConfig4<PoseConfig, PointConfig, LamConfig, Point3Config> config1, config2, expected;

	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	LamKey L1(1), L2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);
	Vector lam1 = Vector_(1, 2.3), lam2 = Vector_(1, 4.5);

	config1.insert(x1, pose1);
	config1.insert(l1, point1);
	config1.insert(L1, lam1);

	config2.insert(x2, pose2);
	config2.insert(l2, point2);
	config2.insert(L2, lam2);

	config1.insert(config2);

	expected.insert(x1, pose1);
	expected.insert(l1, point1);
	expected.insert(L1, lam1);
	expected.insert(x2, pose2);
	expected.insert(l2, point2);
	expected.insert(L2, lam2);

	CHECK(assert_equal(expected, config1));
}

/* ************************************************************************* */
TEST(TupleConfig, partial_insert) {
	TupleConfig3<PoseConfig, PointConfig, LamConfig> init, expected;

	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	LamKey L1(1), L2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);
	Vector lam1 = Vector_(1, 2.3), lam2 = Vector_(1, 4.5);

	init.insert(x1, pose1);
	init.insert(l1, point1);
	init.insert(L1, lam1);

	PoseConfig cfg1;
	cfg1.insert(x2, pose2);

	init.insertSub(cfg1);

	expected.insert(x1, pose1);
	expected.insert(l1, point1);
	expected.insert(L1, lam1);
	expected.insert(x2, pose2);

	CHECK(assert_equal(expected, init));
}

/* ************************************************************************* */
TEST(TupleConfig, update) {
	TupleConfig3<PoseConfig, PointConfig, LamConfig> init, superset, expected;

	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);

	init.insert(x1, pose1);
	init.insert(l1, point1);


	Pose2 pose1_(1.0, 2.0, 0.4);
	Point2 point1_(2.0, 4.0);
	superset.insert(x1, pose1_);
	superset.insert(l1, point1_);
	superset.insert(x2, pose2);
	superset.insert(l2, point2);
	init.update(superset);

	expected.insert(x1, pose1_);
	expected.insert(l1, point1_);

	CHECK(assert_equal(expected, init));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
