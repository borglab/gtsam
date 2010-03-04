/*
 * testLieConfig.cpp
 *
 *  Created on: Jan 5, 2010
 *      Author: richard
 */

#include <CppUnitLite/TestHarness.h>
#include <stdexcept>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#define GTSAM_MAGIC_KEY

#include <Pose2.h>

#include "LieConfig-inl.h"
#include "Vector.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST( LieConfig, equals1 )
{
  LieConfig<string,Vector> expected;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  expected.insert("a",v);
  LieConfig<string,Vector> actual;
  actual.insert("a",v);
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( LieConfig, equals2 )
{
  LieConfig<string,Vector> cfg1, cfg2;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 5.0, 6.0, 8.0);
  cfg1.insert("x", v1);
  cfg2.insert("x", v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( LieConfig, equals_nan )
{
  LieConfig<string,Vector> cfg1, cfg2;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 0.0/0.0, 0.0/0.0, 0.0/0.0);
  cfg1.insert("x", v1);
  cfg2.insert("x", v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( LieConfig, insert_config )
{
  LieConfig<string,Vector> cfg1, cfg2, expected;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 8.0, 9.0, 1.0);
  Vector v3 = Vector_(3, 2.0, 4.0, 3.0);
  Vector v4 = Vector_(3, 8.0, 3.0, 7.0);
  cfg1.insert("x1", v1);
  cfg1.insert("x2", v2);
  cfg2.insert("x2", v3);
  cfg2.insert("x3", v4);

  cfg1.insert(cfg2);

  expected.insert("x1", v1);
  expected.insert("x2", v2);
  expected.insert("x2", v3);
  expected.insert("x3", v4);

  CHECK(assert_equal(cfg1, expected));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_a)
{
  LieConfig<string,Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));

  VectorConfig increment;
  increment.insert("v1", Vector_(3, 1.0, 1.1, 1.2));
  increment.insert("v2", Vector_(3, 1.3, 1.4, 1.5));

  LieConfig<string,Vector> expected;
  expected.insert("v1", Vector_(3, 2.0, 3.1, 4.2));
  expected.insert("v2", Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_b)
{
  LieConfig<string,Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));

  VectorConfig increment;
  increment.insert("v2", Vector_(3, 1.3, 1.4, 1.5));

  LieConfig<string,Vector> expected;
  expected.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  expected.insert("v2", Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_c)
{
  LieConfig<string,Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));

  Vector increment = Vector_(6,
      1.0, 1.1, 1.2,
      1.3, 1.4, 1.5);

  LieConfig<string,Vector> expected;
  expected.insert("v1", Vector_(3, 2.0, 3.1, 4.2));
  expected.insert("v2", Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_d)
{
  LieConfig<string,Vector> config0;
  config0.insert("v1", Vector_(3, 1.0, 2.0, 3.0));
  config0.insert("v2", Vector_(3, 5.0, 6.0, 7.0));
  //config0.print("config0");
  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));

  LieConfig<string,Pose2> poseconfig;
  poseconfig.insert("p1", Pose2(1,2,3));
  poseconfig.insert("p2", Pose2(0.3, 0.4, 0.5));
  //poseconfig.print("poseconfig");
  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));
}

/* ************************************************************************* */
TEST(LieConfig, extract_keys)
{
	typedef TypedSymbol<Pose2, 'x'> PoseKey;
	LieConfig<PoseKey, Pose2> config;

	config.insert(PoseKey(1), Pose2());
	config.insert(PoseKey(2), Pose2());
	config.insert(PoseKey(4), Pose2());
	config.insert(PoseKey(5), Pose2());

	list<PoseKey> expected, actual;
	expected += PoseKey(1), PoseKey(2), PoseKey(4), PoseKey(5);
	actual = config.keys();

	CHECK(actual.size() == expected.size());
	list<PoseKey>::const_iterator itAct = actual.begin(), itExp = expected.begin();
	for (; itAct != actual.end() && itExp != expected.end(); ++itAct, ++itExp) {
		CHECK(assert_equal(*itExp, *itAct));
	}
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
