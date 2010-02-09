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
typedef PairConfig<PoseKey, Pose2, PointKey, Point2> Config;

/* ************************************************************************* */
TEST( PairConfig, insert_equals1 )
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

/* ************************************************************************* */
TEST( PairConfig, insert_equals2 )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(PoseKey(1), x1);
  cfg1.insert(PoseKey(2), x2);
  cfg1.insert(PointKey(1), l1);
  cfg1.insert(PointKey(2), l2);

  Config cfg2;
  cfg2.insert(PoseKey(1), x1);
  cfg2.insert(PoseKey(2), x2);
  cfg2.insert(PointKey(1), l1);

  CHECK(!cfg1.equals(cfg2));

  cfg2.insert(2, Point2(9,11));

  CHECK(!cfg1.equals(cfg2));
}

///* ************************************************************************* */
//TEST( PairConfig, insert_duplicate )
//{
//  Pose2 x1(1,2,3), x2(6,7,8);
//  Point2 l1(4,5), l2(9,10);
//
//  Config cfg1;
//  cfg1.insert(1, x1);
//  cfg1.insert(2, x2);
//  cfg1.insert(1, l1);
//  cfg1.insert(2, l2);
//  cfg1.insert(2, l1);
//
//  CHECK(assert_equal(l2, cfg1[PointKey(2)]));
//  CHECK(cfg1.size() == 4);
//  CHECK(cfg1.dim() == 10);
//}


/* ************************************************************************* */
TEST( PairConfig, size_dim )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(PoseKey(1), x1);
  cfg1.insert(PoseKey(2), x2);
  cfg1.insert(PointKey(1), l1);
  cfg1.insert(PointKey(2), l2);

  CHECK(cfg1.size() == 4);
  CHECK(cfg1.dim() == 10);
}

/* ************************************************************************* */
TEST(PairConfig, at)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(PoseKey(1), x1);
  cfg1.insert(PoseKey(2), x2);
  cfg1.insert(PointKey(1), l1);
  cfg1.insert(PointKey(2), l2);

  CHECK(assert_equal(x1, cfg1[PoseKey(1)]));
  CHECK(assert_equal(x2, cfg1[PoseKey(2)]));
  CHECK(assert_equal(l1, cfg1[PointKey(1)]));
  CHECK(assert_equal(l2, cfg1[PointKey(2)]));

  bool caught = false;
  try {
    cfg1[PoseKey(3)];
  } catch(invalid_argument e) {
    caught = true;
  }
  CHECK(caught);

  caught = false;
  try {
    cfg1[PointKey(3)];
  } catch(invalid_argument e) {
    caught = true;
  }
  CHECK(caught);
}

/* ************************************************************************* */
TEST(PairConfig, expmap)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Config cfg1;
  cfg1.insert(PoseKey(1), x1);
  cfg1.insert(PoseKey(2), x2);
  cfg1.insert(PointKey(1), l1);
  cfg1.insert(PointKey(2), l2);

  VectorConfig increment;
  increment.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
  increment.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
  increment.insert("l1", Vector_(2, 1.0, 1.1));
  increment.insert("l2", Vector_(2, 1.3, 1.4));

  Config expected;
  expected.insert(PoseKey(1), expmap(x1, Vector_(3, 1.0, 1.1, 1.2)));
  expected.insert(PoseKey(2), expmap(x2, Vector_(3, 1.3, 1.4, 1.5)));
  expected.insert(PointKey(1), Point2(5.0, 6.1));
  expected.insert(PointKey(2), Point2(10.3, 11.4));

  Config actual = expmap(cfg1, increment);
  CHECK(assert_equal(expected, actual));
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
TEST( TupleConfig, equals )
{
	Pose2 x1(1,2,3), x2(6,7,8), x2_alt(5,6,7);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	ConfigA cfg1, cfg2, cfg3, cfg4, cfg5;

	cfg1.insert(x1k, x1);
	cfg1.insert(x2k, x2);
	cfg1.insert(l1k, l1);
	cfg1.insert(l2k, l2);

	cfg2.insert(x1k, x1);
	cfg2.insert(x2k, x2);
	cfg2.insert(l1k, l1);
	cfg2.insert(l2k, l2);

	cfg3.insert(x2k, x2);
	cfg3.insert(l1k, l1);

	cfg4.insert(x1k, x1);
	cfg4.insert(x2k, x2_alt);
	cfg4.insert(l1k, l1);
	cfg4.insert(l2k, l2);

	ConfigA cfg6(cfg1);

	CHECK(assert_equal(cfg1,cfg2));
	CHECK(assert_equal(cfg1,cfg1));
	CHECK(!cfg1.equals(cfg3));
	CHECK(!cfg1.equals(cfg4));
	CHECK(!cfg1.equals(cfg5));
	CHECK(assert_equal(cfg1, cfg6));
}

/* ************************************************************************* */
TEST(TupleConfig, expmap)
{
	Pose2 x1(1,2,3), x2(6,7,8);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	ConfigA cfg1;
	cfg1.insert(x1k, x1);
	cfg1.insert(x2k, x2);
	cfg1.insert(l1k, l1);
	cfg1.insert(l2k, l2);

	VectorConfig increment;
	increment.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
	increment.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
	increment.insert("l1", Vector_(2, 1.0, 1.1));
	increment.insert("l2", Vector_(2, 1.3, 1.4));

	ConfigA expected;
	expected.insert(x1k, expmap(x1, Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(x2k, expmap(x2, Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(l1k, Point2(5.0, 6.1));
	expected.insert(l2k, Point2(10.3, 11.4));

	CHECK(assert_equal(expected, expmap(cfg1, increment)));
	CHECK(assert_equal(increment, logmap(cfg1, expected)));
}

/* ************************************************************************* */
TEST(TupleConfig, expmap_typedefs)
{
	Pose2 x1(1,2,3), x2(6,7,8);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	TupleConfig2<PoseConfig, PointConfig> cfg1, expected, actual;
	cfg1.insert(x1k, x1);
	cfg1.insert(x2k, x2);
	cfg1.insert(l1k, l1);
	cfg1.insert(l2k, l2);

	VectorConfig increment;
	increment.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
	increment.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
	increment.insert("l1", Vector_(2, 1.0, 1.1));
	increment.insert("l2", Vector_(2, 1.3, 1.4));

	expected.insert(x1k, expmap(x1, Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(x2k, expmap(x2, Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(l1k, Point2(5.0, 6.1));
	expected.insert(l2k, Point2(10.3, 11.4));

	CHECK(assert_equal(expected, expmap(cfg1, increment)));
	//CHECK(assert_equal(increment, logmap(cfg1, expected)));
}

/* ************************************************************************* */
TEST(TupleConfig, typedefs)
{
	TupleConfig2<PoseConfig, PointConfig> cfg1;
	TupleConfig3<PoseConfig, PointConfig, LamConfig> cfg2;
	TupleConfig4<PoseConfig, PointConfig, LamConfig, Point3Config> cfg3;
	TupleConfig5<PoseConfig, PointConfig, LamConfig, Point3Config, Pose3Config> cfg4;
	TupleConfig6<PoseConfig, PointConfig, LamConfig, Point3Config, Pose3Config, Point3Config2> cfg5;
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

	PoseConfig cfg1; cfg1.insert(x1, pose1);
	PointConfig cfg2; cfg2.insert(l1, point1);
	LamConfig cfg3; cfg3.insert(L1, lam1);

	// Constructor
	TupleConfig3<PoseConfig, PointConfig, LamConfig> config(cfg1, cfg2, cfg3);

	// access
	CHECK(assert_equal(cfg1, config.first()));
	CHECK(assert_equal(cfg2, config.second()));
	CHECK(assert_equal(cfg3, config.third()));
}

/* ************************************************************************* */
#include "NonlinearFactorGraph-inl.h"
TEST( TupleConfig, graphs_and_factors )
{
	typedef TupleConfig3<PoseConfig, PointConfig, LamConfig> ConfigC;
	typedef NonlinearFactorGraph<ConfigC> GraphC;
	typedef NonlinearFactor1<ConfigC, PoseKey, Pose2> FactorC;

	// test creation
	GraphC graph;
	ConfigC config;

}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
