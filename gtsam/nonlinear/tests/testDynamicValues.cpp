/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testDynamicValues.cpp
 * @author Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>
#include <stdexcept>
#include <limits>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/base/Testable.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/DynamicValues.h>

using namespace gtsam;
using namespace std;
static double inf = std::numeric_limits<double>::infinity();

typedef TypedSymbol<LieVector, 'v'> VecKey;

VecKey key1(1), key2(2), key3(3), key4(4);

/* ************************************************************************* */
TEST( DynamicValues, equals1 )
{
  DynamicValues expected;
  LieVector v(3, 5.0, 6.0, 7.0);
  expected.insert(key1,v);
  DynamicValues actual;
  actual.insert(key1,v);
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( DynamicValues, equals2 )
{
  DynamicValues cfg1, cfg2;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, 5.0, 6.0, 8.0);
  cfg1.insert(key1, v1);
  cfg2.insert(key1, v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( DynamicValues, equals_nan )
{
  DynamicValues cfg1, cfg2;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, inf, inf, inf);
  cfg1.insert(key1, v1);
  cfg2.insert(key1, v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( DynamicValues, insert_good )
{
  DynamicValues cfg1, cfg2, expected;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, 8.0, 9.0, 1.0);
  LieVector v3(3, 2.0, 4.0, 3.0);
  LieVector v4(3, 8.0, 3.0, 7.0);
  cfg1.insert(key1, v1);
  cfg1.insert(key2, v2);
  cfg2.insert(key3, v4);

  cfg1.insert(cfg2);

  expected.insert(key1, v1);
  expected.insert(key2, v2);
  expected.insert(key3, v4);

  CHECK(assert_equal(expected, cfg1));
}

/* ************************************************************************* */
TEST( DynamicValues, insert_bad )
{
  DynamicValues cfg1, cfg2;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, 8.0, 9.0, 1.0);
  LieVector v3(3, 2.0, 4.0, 3.0);
  LieVector v4(3, 8.0, 3.0, 7.0);
  cfg1.insert(key1, v1);
  cfg1.insert(key2, v2);
  cfg2.insert(key2, v3);
  cfg2.insert(key3, v4);

  CHECK_EXCEPTION(cfg1.insert(cfg2), DynamicValuesKeyAlreadyExists);
}

/* ************************************************************************* */
TEST( DynamicValues, update_element )
{
  DynamicValues cfg;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, 8.0, 9.0, 1.0);

  cfg.insert(key1, v1);
  CHECK(cfg.size() == 1);
  CHECK(assert_equal(v1, cfg.at(key1)));

  cfg.update(key1, v2);
  CHECK(cfg.size() == 1);
  CHECK(assert_equal(v2, cfg.at(key1)));
}

///* ************************************************************************* */
//TEST(DynamicValues, dim_zero)
//{
//  DynamicValues config0;
//  config0.insert(key1, LieVector(2, 2.0, 3.0));
//  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));
//  LONGS_EQUAL(5, config0.dim());
//
//  VectorValues expected;
//  expected.insert(key1, zero(2));
//  expected.insert(key2, zero(3));
//  CHECK(assert_equal(expected, config0.zero()));
//}

/* ************************************************************************* */
TEST(DynamicValues, expmap_a)
{
  DynamicValues config0;
  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));

  Ordering ordering(*config0.orderingArbitrary());
  VectorValues increment(config0.dims(ordering));
  increment[ordering[key1]] = Vector_(3, 1.0, 1.1, 1.2);
  increment[ordering[key2]] = Vector_(3, 1.3, 1.4, 1.5);

  DynamicValues expected;
  expected.insert(key1, LieVector(3, 2.0, 3.1, 4.2));
  expected.insert(key2, LieVector(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, config0.retract(increment, ordering)));
}

/* ************************************************************************* */
TEST(DynamicValues, expmap_b)
{
  DynamicValues config0;
  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));

  Ordering ordering(*config0.orderingArbitrary());
  VectorValues increment(config0.dims(ordering));
  increment[ordering[key2]] = LieVector(3, 1.3, 1.4, 1.5);

  DynamicValues expected;
  expected.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  expected.insert(key2, LieVector(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, config0.retract(increment, ordering)));
}

/* ************************************************************************* */
//TEST(DynamicValues, expmap_c)
//{
//  DynamicValues config0;
//  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
//  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));
//
//  Vector increment = LieVector(6,
//      1.0, 1.1, 1.2,
//      1.3, 1.4, 1.5);
//
//  DynamicValues expected;
//  expected.insert(key1, LieVector(3, 2.0, 3.1, 4.2));
//  expected.insert(key2, LieVector(3, 6.3, 7.4, 8.5));
//
//  CHECK(assert_equal(expected, config0.retract(increment)));
//}

/* ************************************************************************* */
TEST(DynamicValues, expmap_d)
{
  DynamicValues config0;
  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));
  //config0.print("config0");
  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));

  typedef TypedSymbol<Pose2, 'p'> PoseKey;
  DynamicValues poseconfig;
  poseconfig.insert(PoseKey(1), Pose2(1,2,3));
  poseconfig.insert(PoseKey(2), Pose2(0.3, 0.4, 0.5));

  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));
}

/* ************************************************************************* */
TEST(DynamicValues, extract_keys)
{
	typedef TypedSymbol<Pose2, 'x'> PoseKey;
	DynamicValues config;

	config.insert(PoseKey(1), Pose2());
	config.insert(PoseKey(2), Pose2());
	config.insert(PoseKey(4), Pose2());
	config.insert(PoseKey(5), Pose2());

	FastList<Symbol> expected, actual;
	expected += PoseKey(1), PoseKey(2), PoseKey(4), PoseKey(5);
	actual = config.keys();

	CHECK(actual.size() == expected.size());
	FastList<Symbol>::const_iterator itAct = actual.begin(), itExp = expected.begin();
	for (; itAct != actual.end() && itExp != expected.end(); ++itAct, ++itExp) {
		CHECK(assert_equal(*itExp, *itAct));
	}
}

/* ************************************************************************* */
TEST(DynamicValues, exists_)
{
	DynamicValues config0;
	config0.insert(key1, LieVector(Vector_(1, 1.)));
	config0.insert(key2, LieVector(Vector_(1, 2.)));

	boost::optional<const LieVector&> v = config0.exists(key1);
	CHECK(assert_equal(Vector_(1, 1.),*v));
}

/* ************************************************************************* */
TEST(DynamicValues, update)
{
	DynamicValues config0;
	config0.insert(key1, LieVector(1, 1.));
	config0.insert(key2, LieVector(1, 2.));

	DynamicValues superset;
	superset.insert(key1, LieVector(1, -1.));
	superset.insert(key2, LieVector(1, -2.));
	config0.update(superset);

	DynamicValues expected;
	expected.insert(key1, LieVector(1, -1.));
	expected.insert(key2, LieVector(1, -2.));
	CHECK(assert_equal(expected,config0));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
