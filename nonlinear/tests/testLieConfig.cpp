/*
 * testLieConfig.cpp
 *
 *  Created on: Jan 5, 2010
 *      Author: richard
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <stdexcept>
#include <limits>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/nonlinear/LieConfig-inl.h>
#include <gtsam/base/LieVector.h>

using namespace gtsam;
using namespace std;
static double inf = std::numeric_limits<double>::infinity();

typedef TypedSymbol<LieVector, 'v'> VecKey;
typedef LieConfig<VecKey> Config;

VecKey key1(1), key2(2), key3(3), key4(4);

/* ************************************************************************* */
TEST( LieConfig, equals1 )
{
  Config expected;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  expected.insert(key1,v);
  Config actual;
  actual.insert(key1,v);
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( LieConfig, equals2 )
{
  Config cfg1, cfg2;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 5.0, 6.0, 8.0);
  cfg1.insert(key1, v1);
  cfg2.insert(key1, v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( LieConfig, equals_nan )
{
  Config cfg1, cfg2;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, inf, inf, inf);
  cfg1.insert(key1, v1);
  cfg2.insert(key1, v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( LieConfig, insert_config )
{
  Config cfg1, cfg2, expected;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 8.0, 9.0, 1.0);
  Vector v3 = Vector_(3, 2.0, 4.0, 3.0);
  Vector v4 = Vector_(3, 8.0, 3.0, 7.0);
  cfg1.insert(key1, v1);
  cfg1.insert(key2, v2);
  cfg2.insert(key2, v3);
  cfg2.insert(key3, v4);

  cfg1.insert(cfg2);

  expected.insert(key1, v1);
  expected.insert(key2, v2);
  expected.insert(key2, v3);
  expected.insert(key3, v4);

  CHECK(assert_equal(cfg1, expected));
}

/* ************************************************************************* */
TEST( LieConfig, update_element )
{
  Config cfg;
  Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
  Vector v2 = Vector_(3, 8.0, 9.0, 1.0);

  cfg.insert(key1, v1);
  CHECK(cfg.size() == 1);
  CHECK(assert_equal(v1, cfg.at(key1)));

  cfg.update(key1, v2);
  CHECK(cfg.size() == 1);
  CHECK(assert_equal(v2, cfg.at(key1)));
}

/* ************************************************************************* */
TEST(LieConfig, dim_zero)
{
  Config config0;
  config0.insert(key1, Vector_(2, 2.0, 3.0));
  config0.insert(key2, Vector_(3, 5.0, 6.0, 7.0));
  LONGS_EQUAL(5,config0.dim());

  VectorConfig expected;
  expected.insert(key1, zero(2));
  expected.insert(key2, zero(3));
  CHECK(assert_equal(expected, config0.zero()));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_a)
{
  Config config0;
  config0.insert(key1, Vector_(3, 1.0, 2.0, 3.0));
  config0.insert(key2, Vector_(3, 5.0, 6.0, 7.0));

  VectorConfig increment;
  increment.insert(key1, Vector_(3, 1.0, 1.1, 1.2));
  increment.insert(key2, Vector_(3, 1.3, 1.4, 1.5));

  Config expected;
  expected.insert(key1, Vector_(3, 2.0, 3.1, 4.2));
  expected.insert(key2, Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_b)
{
  Config config0;
  config0.insert(key1, Vector_(3, 1.0, 2.0, 3.0));
  config0.insert(key2, Vector_(3, 5.0, 6.0, 7.0));

  VectorConfig increment;
  increment.insert(key2, Vector_(3, 1.3, 1.4, 1.5));

  Config expected;
  expected.insert(key1, Vector_(3, 1.0, 2.0, 3.0));
  expected.insert(key2, Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
TEST(LieConfig, expmap_c)
{
  Config config0;
  config0.insert(key1, Vector_(3, 1.0, 2.0, 3.0));
  config0.insert(key2, Vector_(3, 5.0, 6.0, 7.0));

  Vector increment = Vector_(6,
      1.0, 1.1, 1.2,
      1.3, 1.4, 1.5);

  Config expected;
  expected.insert(key1, Vector_(3, 2.0, 3.1, 4.2));
  expected.insert(key2, Vector_(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, expmap(config0, increment)));
}

/* ************************************************************************* */
/*TEST(LieConfig, expmap_d)
{
  Config config0;
  config0.insert(key1, Vector_(3, 1.0, 2.0, 3.0));
  config0.insert(key2, Vector_(3, 5.0, 6.0, 7.0));
  //config0.print("config0");
  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));

  LieConfig<string,Pose2> poseconfig;
  poseconfig.insert("p1", Pose2(1,2,3));
  poseconfig.insert("p2", Pose2(0.3, 0.4, 0.5));
  //poseconfig.print("poseconfig");
  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));
}*/

/* ************************************************************************* */
/*TEST(LieConfig, extract_keys)
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
}*/

/* ************************************************************************* */
TEST(LieConfig, exists_)
{
	Config config0;
	config0.insert(key1, Vector_(1, 1.));
	config0.insert(key2, Vector_(1, 2.));

	boost::optional<LieVector> v = config0.exists_(key1);
	CHECK(assert_equal(Vector_(1, 1.),*v));
}

/* ************************************************************************* */
TEST(LieConfig, update)
{
	Config config0;
	config0.insert(key1, Vector_(1, 1.));
	config0.insert(key2, Vector_(1, 2.));

	Config superset;
	superset.insert(key1, Vector_(1, -1.));
	superset.insert(key2, Vector_(1, -2.));
	superset.insert(key3, Vector_(1, -3.));
	config0.update(superset);

	Config expected;
	expected.insert(key1, Vector_(1, -1.));
	expected.insert(key2, Vector_(1, -2.));
	CHECK(assert_equal(expected,config0));
}

/* ************************************************************************* */
TEST(LieConfig, dummy_initialization)
{
	typedef TypedSymbol<LieVector, 'z'> Key;
	typedef LieConfig<Key> Config1;

	Config1 init1;
	init1.insert(Key(1), Vector_(2, 1.0, 2.0));
	init1.insert(Key(2), Vector_(2, 4.0, 3.0));

	Config init2;
	init2.insert(key1, Vector_(2, 1.0, 2.0));
	init2.insert(key2, Vector_(2, 4.0, 3.0));

	Config1 actual1(init2);
	Config actual2(init1);

	EXPECT(actual1.empty());
	EXPECT(actual2.empty());
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
