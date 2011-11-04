/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTupleValues.cpp
 * @author Richard Roberts
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <stdexcept>

#include <gtsam/base/TestableAssertions.h>

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/TupleValues-inl.h>

using namespace gtsam;
using namespace std;

static const double tol = 1e-5;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef TypedSymbol<Point2, 'l'> PointKey;
typedef LieValues<PoseKey> PoseValues;
typedef LieValues<PointKey> PointValues;
typedef TupleValues2<PoseValues, PointValues> Values;

/* ************************************************************************* */
TEST( TupleValues, constructors )
{
	Pose2 x1(1,2,3), x2(6,7,8);
	Point2 l1(4,5), l2(9,10);

	Values::Values1 cfg1;
	cfg1.insert(PoseKey(1), x1);
	cfg1.insert(PoseKey(2), x2);
	Values::Values2 cfg2;
	cfg2.insert(PointKey(1), l1);
	cfg2.insert(PointKey(2), l2);

	Values actual(cfg1, cfg2), expected;
	expected.insert(PoseKey(1), x1);
	expected.insert(PoseKey(2), x2);
	expected.insert(PointKey(1), l1);
	expected.insert(PointKey(2), l2);

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( TupleValues, insert_equals1 )
{
	Pose2 x1(1,2,3), x2(6,7,8);
	Point2 l1(4,5), l2(9,10);

  Values expected;
  expected.insert(PoseKey(1), x1);
  expected.insert(PoseKey(2), x2);
  expected.insert(PointKey(1), l1);
  expected.insert(PointKey(2), l2);

  Values actual;
  actual.insert(PoseKey(1), x1);
  actual.insert(PoseKey(2), x2);
  actual.insert(PointKey(1), l1);
  actual.insert(PointKey(2), l2);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( TupleValues, insert_equals2 )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Values values1;
  values1.insert(PoseKey(1), x1);
  values1.insert(PoseKey(2), x2);
  values1.insert(PointKey(1), l1);
  values1.insert(PointKey(2), l2);

  Values values2;
  values2.insert(PoseKey(1), x1);
  values2.insert(PoseKey(2), x2);
  values2.insert(PointKey(1), l1);

  CHECK(!values1.equals(values2));

  values2.insert(2, Point2(9,11));

  CHECK(!values1.equals(values2));
}

/* ************************************************************************* */
TEST( TupleValues, insert_duplicate )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Values values1;
  values1.insert(1, x1); // 3
  values1.insert(2, x2); // 6
  values1.insert(1, l1); // 8
  values1.insert(2, l2); // 10
  values1.insert(2, l1); // still 10 !!!!

  CHECK(assert_equal(l2, values1[PointKey(2)]));
  LONGS_EQUAL(4,values1.size());
  LONGS_EQUAL(10,values1.dim());
}

/* ************************************************************************* */
TEST( TupleValues, size_dim )
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Values values1;
  values1.insert(PoseKey(1), x1);
  values1.insert(PoseKey(2), x2);
  values1.insert(PointKey(1), l1);
  values1.insert(PointKey(2), l2);

  EXPECT(values1.size() == 4);
  EXPECT(values1.dim() == 10);
}

/* ************************************************************************* */
TEST(TupleValues, at)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Values values1;
  values1.insert(PoseKey(1), x1);
  values1.insert(PoseKey(2), x2);
  values1.insert(PointKey(1), l1);
  values1.insert(PointKey(2), l2);

  EXPECT(assert_equal(x1, values1[PoseKey(1)]));
  EXPECT(assert_equal(x2, values1[PoseKey(2)]));
  EXPECT(assert_equal(l1, values1[PointKey(1)]));
  EXPECT(assert_equal(l2, values1[PointKey(2)]));

  CHECK_EXCEPTION(values1[PoseKey(3)], std::invalid_argument);
  CHECK_EXCEPTION(values1[PointKey(3)], std::invalid_argument);
}

/* ************************************************************************* */
TEST(TupleValues, zero_expmap_logmap)
{
  Pose2 x1(1,2,3), x2(6,7,8);
  Point2 l1(4,5), l2(9,10);

  Values values1;
  values1.insert(PoseKey(1), x1);
  values1.insert(PoseKey(2), x2);
  values1.insert(PointKey(1), l1);
  values1.insert(PointKey(2), l2);

  Ordering o; o += "x1", "x2", "l1", "l2";
  VectorValues expected_zero(values1.dims(o));
  expected_zero[o["x1"]] = zero(3);
  expected_zero[o["x2"]] = zero(3);
  expected_zero[o["l1"]] = zero(2);
  expected_zero[o["l2"]] = zero(2);

  CHECK(assert_equal(expected_zero, values1.zero(o)));

  VectorValues delta(values1.dims(o));
  delta[o["x1"]] = Vector_(3, 1.0, 1.1, 1.2);
  delta[o["x2"]] = Vector_(3, 1.3, 1.4, 1.5);
  delta[o["l1"]] = Vector_(2, 1.0, 1.1);
  delta[o["l2"]] = Vector_(2, 1.3, 1.4);

  Values expected;
  expected.insert(PoseKey(1), x1.retract(Vector_(3, 1.0, 1.1, 1.2)));
  expected.insert(PoseKey(2), x2.retract(Vector_(3, 1.3, 1.4, 1.5)));
  expected.insert(PointKey(1), Point2(5.0, 6.1));
  expected.insert(PointKey(2), Point2(10.3, 11.4));

  Values actual = values1.retract(delta, o);
  CHECK(assert_equal(expected, actual));

  // Check log
  VectorValues expected_log = delta;
  VectorValues actual_log = values1.unretract(actual, o);
  CHECK(assert_equal(expected_log, actual_log));
}

/* ************************************************************************* */

// some key types
typedef TypedSymbol<LieVector, 'L'> LamKey;
typedef TypedSymbol<Pose3, 'a'> Pose3Key;
typedef TypedSymbol<Point3, 'b'> Point3Key;
typedef TypedSymbol<Point3, 'c'> Point3Key2;

// some values types
typedef LieValues<PoseKey> PoseValues;
typedef LieValues<PointKey> PointValues;
typedef LieValues<LamKey> LamValues;
typedef LieValues<Pose3Key> Pose3Values;
typedef LieValues<Point3Key> Point3Values;
typedef LieValues<Point3Key2> Point3Values2;

// some TupleValues types
typedef TupleValues<PoseValues, TupleValuesEnd<PointValues> > ValuesA;
typedef TupleValues<PoseValues, TupleValues<PointValues, TupleValuesEnd<LamValues> > > ValuesB;

typedef TupleValues1<PoseValues> TuplePoseValues;
typedef TupleValues1<PointValues> TuplePointValues;
typedef TupleValues2<PoseValues, PointValues> SimpleValues;

/* ************************************************************************* */
TEST(TupleValues, slicing) {
	PointKey l1(1), l2(2);
	Point2 l1_val(1.0, 2.0), l2_val(3.0, 4.0);
	PoseKey x1(1), x2(2);
	Pose2 x1_val(1.0, 2.0, 0.3), x2_val(3.0, 4.0, 0.4);

	PoseValues liePoseValues;
	liePoseValues.insert(x1, x1_val);
	liePoseValues.insert(x2, x2_val);

	PointValues liePointValues;
	liePointValues.insert(l1, l1_val);
	liePointValues.insert(l2, l2_val);

	// construct TupleValues1 from the base values
	TuplePoseValues tupPoseValues1(liePoseValues);
	EXPECT(assert_equal(liePoseValues, tupPoseValues1.first(), tol));

	TuplePointValues tupPointValues1(liePointValues);
	EXPECT(assert_equal(liePointValues, tupPointValues1.first(), tol));

//	// construct a TupleValues2 from a TupleValues1
//	SimpleValues pairValues1(tupPoseValues1);
//	EXPECT(assert_equal(liePoseValues, pairValues1.first(), tol));
//	EXPECT(pairValues1.second().empty());
//
//	SimpleValues pairValues2(tupPointValues1);
//	EXPECT(assert_equal(liePointValues, pairValues2.second(), tol));
//	EXPECT(pairValues1.first().empty());

}

/* ************************************************************************* */
TEST(TupleValues, basic_functions) {
	// create some tuple values
	ValuesA valuesA;
	ValuesB valuesB;

	PoseKey x1(1);
	PointKey l1(1);
	LamKey L1(1);
	Pose2 pose1(1.0, 2.0, 0.3);
	Point2 point1(2.0, 3.0);
	LieVector lam1 = LieVector(2.3);

	// Insert
	valuesA.insert(x1, pose1);
	valuesA.insert(l1, point1);

	valuesB.insert(x1, pose1);
	valuesB.insert(l1, point1);
	valuesB.insert(L1, lam1);

	// bracket operator
	EXPECT(assert_equal(valuesA[x1], pose1));
	EXPECT(assert_equal(valuesA[l1], point1));
	EXPECT(assert_equal(valuesB[x1], pose1));
	EXPECT(assert_equal(valuesB[l1], point1));
	EXPECT(assert_equal(valuesB[L1], lam1));

	// exists
	EXPECT(valuesA.exists(x1));
	EXPECT(valuesA.exists(l1));
	EXPECT(valuesB.exists(x1));
	EXPECT(valuesB.exists(l1));
	EXPECT(valuesB.exists(L1));

	// at
	EXPECT(assert_equal(valuesA.at(x1), pose1));
	EXPECT(assert_equal(valuesA.at(l1), point1));
	EXPECT(assert_equal(valuesB.at(x1), pose1));
	EXPECT(assert_equal(valuesB.at(l1), point1));
	EXPECT(assert_equal(valuesB.at(L1), lam1));

	// size
	EXPECT(valuesA.size() == 2);
	EXPECT(valuesB.size() == 3);

	// dim
	EXPECT(valuesA.dim() == 5);
	EXPECT(valuesB.dim() == 6);

	// erase
	valuesA.erase(x1);
	CHECK(!valuesA.exists(x1));
	CHECK(valuesA.size() == 1);
	valuesA.erase(l1);
	CHECK(!valuesA.exists(l1));
	CHECK(valuesA.size() == 0);
	valuesB.erase(L1);
	CHECK(!valuesB.exists(L1));
	CHECK(valuesB.size() == 2);

	// clear
	valuesA.clear();
	EXPECT(valuesA.size() == 0);
	valuesB.clear();
	EXPECT(valuesB.size() == 0);

	// empty
	EXPECT(valuesA.empty());
	EXPECT(valuesB.empty());
}

/* ************************************************************************* */
TEST(TupleValues, insert_values) {
	ValuesB values1, values2, expected;

	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	LamKey L1(1), L2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);
	LieVector lam1 = LieVector(2.3), lam2 = LieVector(4.5);

	values1.insert(x1, pose1);
	values1.insert(l1, point1);
	values1.insert(L1, lam1);

	values2.insert(x2, pose2);
	values2.insert(l2, point2);
	values2.insert(L2, lam2);

	values1.insert(values2);

	expected.insert(x1, pose1);
	expected.insert(l1, point1);
	expected.insert(L1, lam1);
	expected.insert(x2, pose2);
	expected.insert(l2, point2);
	expected.insert(L2, lam2);

	CHECK(assert_equal(expected, values1));
}

/* ************************************************************************* */
TEST( TupleValues, update_element )
{
	TupleValues2<PoseValues, PointValues> cfg;
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
TEST( TupleValues, equals )
{
	Pose2 x1(1,2,3), x2(6,7,8), x2_alt(5,6,7);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	ValuesA values1, values2, values3, values4, values5;

	values1.insert(x1k, x1);
	values1.insert(x2k, x2);
	values1.insert(l1k, l1);
	values1.insert(l2k, l2);

	values2.insert(x1k, x1);
	values2.insert(x2k, x2);
	values2.insert(l1k, l1);
	values2.insert(l2k, l2);

	values3.insert(x2k, x2);
	values3.insert(l1k, l1);

	values4.insert(x1k, x1);
	values4.insert(x2k, x2_alt);
	values4.insert(l1k, l1);
	values4.insert(l2k, l2);

	ValuesA values6(values1);

	EXPECT(assert_equal(values1,values2));
	EXPECT(assert_equal(values1,values1));
	EXPECT(assert_inequal(values1,values3));
	EXPECT(assert_inequal(values1,values4));
	EXPECT(assert_inequal(values1,values5));
	EXPECT(assert_equal(values1, values6));
}

/* ************************************************************************* */
TEST(TupleValues, expmap)
{
	Pose2 x1(1,2,3), x2(6,7,8);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

	Ordering o; o += "x1", "x2", "l1", "l2";

	ValuesA values1;
	values1.insert(x1k, x1);
	values1.insert(x2k, x2);
	values1.insert(l1k, l1);
	values1.insert(l2k, l2);

	VectorValues delta(values1.dims(o));
	delta[o["x1"]] = Vector_(3, 1.0, 1.1, 1.2);
	delta[o["x2"]] = Vector_(3, 1.3, 1.4, 1.5);
	delta[o["l1"]] = Vector_(2, 1.0, 1.1);
	delta[o["l2"]] = Vector_(2, 1.3, 1.4);

	ValuesA expected;
	expected.insert(x1k, x1.retract(Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(x2k, x2.retract(Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(l1k, Point2(5.0, 6.1));
	expected.insert(l2k, Point2(10.3, 11.4));

	CHECK(assert_equal(expected, values1.retract(delta, o)));
	CHECK(assert_equal(delta, values1.unretract(expected, o)));
}

/* ************************************************************************* */
TEST(TupleValues, expmap_typedefs)
{
	Pose2 x1(1,2,3), x2(6,7,8);
	PoseKey x1k(1), x2k(2);
	Point2 l1(4,5), l2(9,10);
	PointKey l1k(1), l2k(2);

  Ordering o; o += "x1", "x2", "l1", "l2";

	TupleValues2<PoseValues, PointValues> values1, expected, actual;
	values1.insert(x1k, x1);
	values1.insert(x2k, x2);
	values1.insert(l1k, l1);
	values1.insert(l2k, l2);

  VectorValues delta(values1.dims(o));
  delta[o["x1"]] = Vector_(3, 1.0, 1.1, 1.2);
  delta[o["x2"]] = Vector_(3, 1.3, 1.4, 1.5);
  delta[o["l1"]] = Vector_(2, 1.0, 1.1);
  delta[o["l2"]] = Vector_(2, 1.3, 1.4);

	expected.insert(x1k, x1.retract(Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(x2k, x2.retract(Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(l1k, Point2(5.0, 6.1));
	expected.insert(l2k, Point2(10.3, 11.4));

	CHECK(assert_equal(expected, TupleValues2<PoseValues, PointValues>(values1.retract(delta, o))));
	//CHECK(assert_equal(delta, values1.unretract(expected)));
}

/* ************************************************************************* */
TEST(TupleValues, typedefs)
{
	TupleValues2<PoseValues, PointValues> values1;
	TupleValues3<PoseValues, PointValues, LamValues> values2;
	TupleValues4<PoseValues, PointValues, LamValues, Point3Values> values3;
	TupleValues5<PoseValues, PointValues, LamValues, Point3Values, Pose3Values> values4;
	TupleValues6<PoseValues, PointValues, LamValues, Point3Values, Pose3Values, Point3Values2> values5;
}

/* ************************************************************************* */
TEST( TupleValues, pairvalues_style )
{
	PoseKey x1(1);
	PointKey l1(1);
	LamKey L1(1);
	Pose2 pose1(1.0, 2.0, 0.3);
	Point2 point1(2.0, 3.0);
	LieVector lam1 = LieVector(2.3);

	PoseValues values1; values1.insert(x1, pose1);
	PointValues values2; values2.insert(l1, point1);
	LamValues values3; values3.insert(L1, lam1);

	// Constructor
	TupleValues3<PoseValues, PointValues, LamValues> values(values1, values2, values3);

	// access
	CHECK(assert_equal(values1, values.first()));
	CHECK(assert_equal(values2, values.second()));
	CHECK(assert_equal(values3, values.third()));
}

/* ************************************************************************* */
TEST(TupleValues, insert_values_typedef) {

	TupleValues4<PoseValues, PointValues, LamValues, Point3Values> values1, values2, expected;

	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	LamKey L1(1), L2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);
	LieVector lam1 = LieVector(2.3), lam2 = LieVector(4.5);

	values1.insert(x1, pose1);
	values1.insert(l1, point1);
	values1.insert(L1, lam1);

	values2.insert(x2, pose2);
	values2.insert(l2, point2);
	values2.insert(L2, lam2);

	values1.insert(values2);

	expected.insert(x1, pose1);
	expected.insert(l1, point1);
	expected.insert(L1, lam1);
	expected.insert(x2, pose2);
	expected.insert(l2, point2);
	expected.insert(L2, lam2);

	CHECK(assert_equal(expected, values1));
}

/* ************************************************************************* */
TEST(TupleValues, partial_insert) {
	TupleValues3<PoseValues, PointValues, LamValues> init, expected;

	PoseKey x1(1), x2(2);
	PointKey l1(1), l2(2);
	LamKey L1(1), L2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);
	LieVector lam1 = LieVector(2.3), lam2 = LieVector(4.5);

	init.insert(x1, pose1);
	init.insert(l1, point1);
	init.insert(L1, lam1);

	PoseValues cfg1;
	cfg1.insert(x2, pose2);

	init.insertSub(cfg1);

	expected.insert(x1, pose1);
	expected.insert(l1, point1);
	expected.insert(L1, lam1);
	expected.insert(x2, pose2);

	CHECK(assert_equal(expected, init));
}

/* ************************************************************************* */
TEST(TupleValues, update) {
	TupleValues3<PoseValues, PointValues, LamValues> init, superset, expected;

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
TEST(TupleValues, arbitrary_ordering ) {
	TupleValues1<PoseValues> values;
	PoseKey x1(1), x2(2);
	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	values.insert(x1, pose1);
	values.insert(x2, pose2);
	Ordering::shared_ptr actual = values.orderingArbitrary();
	Ordering expected; expected += x1, x2;
	EXPECT(assert_equal(expected, *actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
