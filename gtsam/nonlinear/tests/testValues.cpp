/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testValues.cpp
 * @author Richard Roberts
 */

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/LieVector.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;
#include <stdexcept>
#include <limits>

using namespace gtsam;
using namespace std;
static double inf = std::numeric_limits<double>::infinity();

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

const Symbol key1('v',1), key2('v',2), key3('v',3), key4('v',4);


class TestValueData {
public:
	static int ConstructorCount;
	static int DestructorCount;
	TestValueData(const TestValueData& other) { cout << "Copy constructor" << endl; ++ ConstructorCount; }
	TestValueData() { cout << "Default constructor" << endl; ++ ConstructorCount; }
	~TestValueData() { cout << "Destructor" << endl; ++ DestructorCount; }
};
int TestValueData::ConstructorCount = 0;
int TestValueData::DestructorCount = 0;
class TestValue : public DerivedValue<TestValue> {
	TestValueData data_;
public:
	virtual void print(const std::string& str = "") const {}
	bool equals(const TestValue& other, double tol = 1e-9) const { return true; }
	virtual size_t dim() const { return 0; }
	TestValue retract(const Vector&) const { return TestValue(); }
	Vector localCoordinates(const TestValue&) const { return Vector(); }
};

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
TEST(Values, localCoordinates)
{
	Values valuesA;
	valuesA.insert(key1, LieVector(3, 1.0, 2.0, 3.0));
	valuesA.insert(key2, LieVector(3, 5.0, 6.0, 7.0));

	Ordering ordering = *valuesA.orderingArbitrary();

	VectorValues expDelta = valuesA.zeroVectors(ordering);
//	expDelta.at(ordering[key1]) = Vector_(3, 0.1, 0.2, 0.3);
//	expDelta.at(ordering[key2]) = Vector_(3, 0.4, 0.5, 0.6);

	Values valuesB = valuesA.retract(expDelta, ordering);

	EXPECT(assert_equal(expDelta, valuesA.localCoordinates(valuesB, ordering)));
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
		EXPECT(*itExp == *itAct);
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
	Pose2 pose0(1.0, 2.0, 0.3);
	Pose3 pose1(Pose2(0.1, 0.2, 0.3));
	Pose2 pose2(4.0, 5.0, 0.6);
	Pose3 pose3(Pose2(0.3, 0.7, 0.9));

  Values values;
  values.insert(0, pose0);
  values.insert(1, pose1);
  values.insert(2, pose2);
  values.insert(3, pose3);

  // Filter by key
  int i = 0;
  Values::Filtered<Value> filtered = values.filter(boost::bind(std::greater_equal<Key>(), _1, 2));
  EXPECT_LONGS_EQUAL(2, filtered.size());
  BOOST_FOREACH(const Values::Filtered<>::KeyValuePair& key_value, filtered) {
    if(i == 0) {
      LONGS_EQUAL(2, key_value.key);
      EXPECT(typeid(Pose2) == typeid(key_value.value));
      EXPECT(assert_equal(pose2, dynamic_cast<const Pose2&>(key_value.value)));
    } else if(i == 1) {
      LONGS_EQUAL(3, key_value.key);
      EXPECT(typeid(Pose3) == typeid(key_value.value));
      EXPECT(assert_equal(pose3, dynamic_cast<const Pose3&>(key_value.value)));
    } else {
      EXPECT(false);
    }
    ++ i;
  }
  EXPECT_LONGS_EQUAL(2, i);

  // construct a values with the view
  Values actualSubValues1(filtered);
  Values expectedSubValues1;
  expectedSubValues1.insert(2, pose2);
  expectedSubValues1.insert(3, pose3);
  EXPECT(assert_equal(expectedSubValues1, actualSubValues1));

  // Filter by type
  i = 0;
  Values::ConstFiltered<Pose3> pose_filtered = values.filter<Pose3>();
  EXPECT_LONGS_EQUAL(2, pose_filtered.size());
  BOOST_FOREACH(const Values::ConstFiltered<Pose3>::KeyValuePair& key_value, pose_filtered) {
    if(i == 0) {
    	EXPECT_LONGS_EQUAL(1, key_value.key);
      EXPECT(assert_equal(pose1, key_value.value));
    } else if(i == 1) {
    	EXPECT_LONGS_EQUAL(3, key_value.key);
      EXPECT(assert_equal(pose3, key_value.value));
    } else {
      EXPECT(false);
    }
    ++ i;
  }
  EXPECT_LONGS_EQUAL(2, i);

  // construct a values with the view
  Values actualSubValues2(pose_filtered);
  Values expectedSubValues2;
  expectedSubValues2.insert(1, pose1);
  expectedSubValues2.insert(3, pose3);
  EXPECT(assert_equal(expectedSubValues2, actualSubValues2));
}

/* ************************************************************************* */
TEST(Values, Symbol_filter) {
  Pose2 pose0(1.0, 2.0, 0.3);
  Pose3 pose1(Pose2(0.1, 0.2, 0.3));
  Pose2 pose2(4.0, 5.0, 0.6);
  Pose3 pose3(Pose2(0.3, 0.7, 0.9));

  Values values;
  values.insert(X(0), pose0);
  values.insert(Symbol('y',1), pose1);
  values.insert(X(2), pose2);
  values.insert(Symbol('y',3), pose3);

  int i = 0;
  BOOST_FOREACH(const Values::Filtered<Value>::KeyValuePair& key_value, values.filter(Symbol::ChrTest('y'))) {
    if(i == 0) {
      LONGS_EQUAL(Symbol('y',1), key_value.key);
      EXPECT(assert_equal(pose1, dynamic_cast<const Pose3&>(key_value.value)));
    } else if(i == 1) {
      LONGS_EQUAL(Symbol('y',3), key_value.key);
      EXPECT(assert_equal(pose3, dynamic_cast<const Pose3&>(key_value.value)));
    } else {
      EXPECT(false);
    }
    ++ i;
  }
  LONGS_EQUAL(2, i);
}

/* ************************************************************************* */
TEST(Values, Destructors) {
	// Check that Value destructors are called when Values container is deleted
	{
		Values values;
		{
			TestValue value1;
			TestValue value2;
			LONGS_EQUAL(2, TestValueData::ConstructorCount);
			LONGS_EQUAL(0, TestValueData::DestructorCount);
			values.insert(0, value1);
			values.insert(1, value2);
		}
		LONGS_EQUAL(4, TestValueData::ConstructorCount);
		LONGS_EQUAL(2, TestValueData::DestructorCount);
	}
	LONGS_EQUAL(4, TestValueData::ConstructorCount);
	LONGS_EQUAL(4, TestValueData::DestructorCount);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
