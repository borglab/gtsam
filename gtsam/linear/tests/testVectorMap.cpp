/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testVectorMap.cpp
 * @brief  Unit tests for Factor Graph Values
 * @author Carlos Nieto
 **/

/*STL/C++*/
#include <iostream>
#include <sstream>

#define GTSAM_MAGIC_KEY

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorMap.h>

using namespace std;
using namespace gtsam;

static const Index l1=0, x1=1, x2=2;
static const Index _a_=3, _x_=4, _y_=5, _g_=6, _p_=7;

/* ************************************************************************* */
VectorMap smallVectorMap() {
	VectorMap c;
	c.insert(l1, Vector_(2,  0.0, -1.0));
	c.insert(x1, Vector_(2,  0.0,  0.0));
	c.insert(x2, Vector_(2,  1.5,  0.0));
	return c;
}

/* ************************************************************************* */
TEST( VectorMap, equals1 )
 {
   VectorMap expected;
   Vector v = Vector_(3, 5.0, 6.0, 7.0);
   expected.insert(_a_,v);
   VectorMap actual;
   actual.insert(_a_,v);
   CHECK(assert_equal(expected,actual));
   CHECK(assert_equal(expected[_a_],actual.get(_a_)))
}

/* ************************************************************************* */
TEST( VectorMap, equals2 )
 {	 
   VectorMap cfg1, cfg2;
   Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
   Vector v2 = Vector_(3, 5.0, 6.0, 8.0);
   cfg1.insert(_x_, v1);
   cfg2.insert(_x_, v2);
   CHECK(!cfg1.equals(cfg2));
   CHECK(!cfg2.equals(cfg1));
 }

/* ************************************************************************* */
TEST( VectorMap, fullVector)
{
	VectorMap c = smallVectorMap();
	Vector actual = c.vector();
	Vector expected = Vector_(6, 0.0, -1.0, 0.0,  0.0, 1.5,  0.0);
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */

#include <limits>
double inf = std::numeric_limits<double>::infinity();

TEST( VectorMap, equals_nan )
 {
   VectorMap cfg1, cfg2;
   Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
   Vector v2 = Vector_(3, inf, inf, inf);
   cfg1.insert(_x_, v1);
   cfg2.insert(_x_, v2);
   CHECK(!cfg1.equals(cfg2));
   CHECK(!cfg2.equals(cfg1));
 }

/* ************************************************************************* */
TEST( VectorMap, contains)
{
  VectorMap fg;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  fg.insert(_a_, v);
  CHECK(fg.contains(_a_));
  CHECK(!fg.contains(_g_));
}

/* ************************************************************************* */
TEST( VectorMap, expmap)
{
	VectorMap c = smallVectorMap();
	Vector v = Vector_(6, 0.0,-1.0, 0.0, 0.0, 1.5, 0.0); // l1, x1, x2
  CHECK(assert_equal(expmap(c,c),expmap(c,v)));
}

/* ************************************************************************* */
TEST( VectorMap, plus)
{
  VectorMap c;
  Vector vx = Vector_(3, 5.0, 6.0, 7.0), vy = Vector_(2, 8.0, 9.0);
  c += VectorMap(_x_,vx);
  c += VectorMap(_y_,vy);

  VectorMap delta;
  Vector dx = Vector_(3, 1.0, 1.0, 1.0), dy = Vector_(2, -1.0, -1.0);
  delta.insert(_x_, dx).insert(_y_,dy);

  VectorMap expected;
  Vector wx = Vector_(3, 6.0, 7.0, 8.0), wy = Vector_(2, 7.0, 8.0);
  expected.insert(_x_, wx).insert(_y_,wy);

  // functional
  VectorMap actual = expmap(c,delta);
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( VectorMap, scale) {
	VectorMap cfg;
	cfg.insert(_x_, Vector_(2, 1.0, 2.0));
	cfg.insert(_y_, Vector_(2,-1.0,-2.0));

	VectorMap actual = cfg.scale(2.0);

	VectorMap expected;
	expected.insert(_x_, Vector_(2, 2.0, 4.0));
	expected.insert(_y_, Vector_(2,-2.0,-4.0));

	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( VectorMap, axpy) {
  VectorMap x,y,expected;
  x += VectorMap(_x_,Vector_(3, 1.0, 1.0, 1.0));
  x += VectorMap(_y_,Vector_(2, -1.0, -1.0));
  y += VectorMap(_x_,Vector_(3, 5.0, 6.0, 7.0));
  y += VectorMap(_y_,Vector_(2, 8.0, 9.0));
  expected += VectorMap(_x_,Vector_(3, 15.0, 16.0, 17.0));
  expected += VectorMap(_y_,Vector_(2, -2.0, -1.0));
  axpy(10,x,y);
  CHECK(assert_equal(expected,y));
}

/* ************************************************************************* */
TEST( VectorMap, scal) {
  VectorMap x,expected;
  x += VectorMap(_x_,Vector_(3, 1.0, 2.0, 3.0));
  x += VectorMap(_y_,Vector_(2, 4.0, 5.0));
  expected += VectorMap(_x_,Vector_(3, 10.0, 20.0, 30.0));
  expected += VectorMap(_y_,Vector_(2, 40.0, 50.0));
  scal(10,x);
  CHECK(assert_equal(expected,x));
}

/* ************************************************************************* */
TEST( VectorMap, update_with_large_delta) {
	// this test ensures that if the update for delta is larger than
	// the size of the config, it only updates existing variables
	VectorMap init, delta;
	init.insert(_x_, Vector_(2, 1.0, 2.0));
	init.insert(_y_, Vector_(2, 3.0, 4.0));
	delta.insert(_x_, Vector_(2, 0.1, 0.1));
	delta.insert(_y_, Vector_(2, 0.1, 0.1));
	delta.insert(_p_, Vector_(2, 0.1, 0.1));

	VectorMap actual = expmap(init,delta);
	VectorMap expected;
	expected.insert(_x_, Vector_(2, 1.1, 2.1));
	expected.insert(_y_, Vector_(2, 3.1, 4.1));

	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( VectorMap, dot) {
	VectorMap c = smallVectorMap();
	DOUBLES_EQUAL(3.25,dot(c,c),1e-9);
}

/* ************************************************************************* */
TEST( VectorMap, dim) {
	VectorMap c = smallVectorMap();
	LONGS_EQUAL(6,c.dim());
}

/* ************************************************************************* */
TEST( VectorMap, operators) {
	VectorMap c; c.insert(_x_, Vector_(2, 1.1, 2.2));
	VectorMap expected1; expected1.insert(_x_, Vector_(2, 2.2, 4.4));
	CHECK(assert_equal(expected1,c*2));
	CHECK(assert_equal(expected1,c+c));
	VectorMap expected2; expected2.insert(_x_, Vector_(2, 0.0, 0.0));
	CHECK(assert_equal(expected2,c-c));
}

/* ************************************************************************* */
TEST( VectorMap, getReference) {
	VectorMap c; c.insert(_x_, Vector_(2, 1.1, 2.2));
	Vector& cx = c[_x_];
	cx = cx*2.0;
	VectorMap expected; expected.insert(_x_, Vector_(2, 2.2, 4.4));
	CHECK(assert_equal(expected,c));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
