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

#define GTSAM_MAGIC_KEY

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;
using namespace std;
static double inf = std::numeric_limits<double>::infinity();

Key key1(Symbol("v1")), key2(Symbol("v2")), key3(Symbol("v3")), key4(Symbol("v4"));

/* ************************************************************************* */
TEST( Values, equals1 )
{
  Values expected;
  LieVector v(3, 5.0, 6.0, 7.0);
  expected.insert(key1,v);
  Values actual;
  actual.insert(key1,v);
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Values, equals2 )
{
  Values cfg1, cfg2;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, 5.0, 6.0, 8.0);
  cfg1.insert(key1, v1);
  cfg2.insert(key1, v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( Values, equals_nan )
{
  Values cfg1, cfg2;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, inf, inf, inf);
  cfg1.insert(key1, v1);
  cfg2.insert(key1, v2);
  CHECK(!cfg1.equals(cfg2));
  CHECK(!cfg2.equals(cfg1));
}

/* ************************************************************************* */
TEST( Values, insert_good )
{
  Values cfg1, cfg2, expected;
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
TEST( Values, insert_bad )
{
  Values cfg1, cfg2;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, 8.0, 9.0, 1.0);
  LieVector v3(3, 2.0, 4.0, 3.0);
  LieVector v4(3, 8.0, 3.0, 7.0);
  cfg1.insert(key1, v1);
  cfg1.insert(key2, v2);
  cfg2.insert(key2, v3);
  cfg2.insert(key3, v4);

  CHECK_EXCEPTION(cfg1.insert(cfg2), ValuesKeyAlreadyExists);
}

/* ************************************************************************* */
TEST( Values, update_element )
{
  Values cfg;
  LieVector v1(3, 5.0, 6.0, 7.0);
  LieVector v2(3, 8.0, 9.0, 1.0);

  cfg.insert(key1, v1);
  CHECK(cfg.size() == 1);
  CHECK(assert_equal(v1, cfg.at<LieVector>(key1)));

  cfg.update(key1, v2);
  CHECK(cfg.size() == 1);
  CHECK(assert_equal(v2, cfg.at<LieVector>(key1)));
}

///* ************************************************************************* */
//TEST(Values, dim_zero)
//{
//  Values config0;
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
TEST(Values, expmap_a)
{
  Values config0;
  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));

  Ordering ordering(*config0.orderingArbitrary());
  VectorValues increment(config0.dims(ordering));
  increment[ordering[key1]] = Vector_(3, 1.0, 1.1, 1.2);
  increment[ordering[key2]] = Vector_(3, 1.3, 1.4, 1.5);

  Values expected;
  expected.insert(key1, LieVector(3, 2.0, 3.1, 4.2));
  expected.insert(key2, LieVector(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, config0.retract(increment, ordering)));
}

/* ************************************************************************* */
TEST(Values, expmap_b)
{
  Values config0;
  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));

  Ordering ordering(*config0.orderingArbitrary());
  VectorValues increment(VectorValues::Zero(config0.dims(ordering)));
  increment[ordering[key2]] = LieVector(3, 1.3, 1.4, 1.5);

  Values expected;
  expected.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  expected.insert(key2, LieVector(3, 6.3, 7.4, 8.5));

  CHECK(assert_equal(expected, config0.retract(increment, ordering)));
}

/* ************************************************************************* */
//TEST(Values, expmap_c)
//{
//  Values config0;
//  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
//  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));
//
//  Vector increment = LieVector(6,
//      1.0, 1.1, 1.2,
//      1.3, 1.4, 1.5);
//
//  Values expected;
//  expected.insert(key1, LieVector(3, 2.0, 3.1, 4.2));
//  expected.insert(key2, LieVector(3, 6.3, 7.4, 8.5));
//
//  CHECK(assert_equal(expected, config0.retract(increment)));
//}

/* ************************************************************************* */
TEST(Values, expmap_d)
{
  Values config0;
  config0.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
  config0.insert(key2, LieVector(3, 5.0, 6.0, 7.0));
  //config0.print("config0");
  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));

  Values poseconfig;
  poseconfig.insert(key1, Pose2(1,2,3));
  poseconfig.insert(key2, Pose2(0.3, 0.4, 0.5));

  CHECK(equal(config0, config0));
  CHECK(config0.equals(config0));
}

/* ************************************************************************* */
TEST(Values, extract_keys)
{
	Values config;

	config.insert(key1, Pose2());
	config.insert(key2, Pose2());
	config.insert(key3, Pose2());
	config.insert(key4, Pose2());

	FastList<Key> expected, actual;
	expected += key1, key2, key3, key4;
	actual = config.keys();

	CHECK(actual.size() == expected.size());
	FastList<Key>::const_iterator itAct = actual.begin(), itExp = expected.begin();
	for (; itAct != actual.end() && itExp != expected.end(); ++itAct, ++itExp) {
		LONGS_EQUAL(*itExp, *itAct);
	}
}

/* ************************************************************************* */
TEST(Values, exists_)
{
	Values config0;
	config0.insert(key1, LieVector(Vector_(1, 1.)));
	config0.insert(key2, LieVector(Vector_(1, 2.)));

	boost::optional<const LieVector&> v = config0.exists<LieVector>(key1);
	CHECK(assert_equal(Vector_(1, 1.),*v));
}

/* ************************************************************************* */
TEST(Values, update)
{
	Values config0;
	config0.insert(key1, LieVector(1, 1.));
	config0.insert(key2, LieVector(1, 2.));

	Values superset;
	superset.insert(key1, LieVector(1, -1.));
	superset.insert(key2, LieVector(1, -2.));
	config0.update(superset);

	Values expected;
	expected.insert(key1, LieVector(1, -1.));
	expected.insert(key2, LieVector(1, -2.));
	CHECK(assert_equal(expected,config0));
}

/* ************************************************************************* */
TEST(Values, filter) {
  Values values;
  values.insert(0, Pose2());
  values.insert(1, Pose3());
  values.insert(2, Pose2());
  values.insert(3, Pose3());

  // Filter by key
  int i = 0;
  for(Values::filter_iterator it = values.beginFilterByKey(boost::bind(std::greater_equal<Key>(), _1, 2));
      it != values.endFilterByKey(boost::bind(std::greater_equal<Key>(), _1, 2)); ++it, ++i) {
    if(i == 0) {
      LONGS_EQUAL(2, it->first);
      EXPECT(typeid(Pose2) == typeid(it->second));
    } else if(i == 1) {
      LONGS_EQUAL(3, it->first);
      EXPECT(typeid(Pose3) == typeid(it->second));
    } else {
      EXPECT(false);
    }
  }
  LONGS_EQUAL(2, i);

  // Filter by type
  i = 0;
  for(Values::type_filter_iterator<Pose3>::type it = values.beginFilterByType<Pose3>();
      it != values.endFilterByType<Pose3>(); ++it, ++i) {
    if(i == 0) {
      LONGS_EQUAL(1, it->first);
      EXPECT(assert_equal(Pose3(), it->second));
    } else if(i == 1) {
      LONGS_EQUAL(3, it->first);
      EXPECT(assert_equal(Pose3(), it->second));
    } else {
      EXPECT(false);
    }
  }
  LONGS_EQUAL(2, i);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
