/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testVectorBTree.cpp
 * @brief  Unit tests for Factor Graph Values
 * @author Frank Dellaert
 **/

#include <iostream>
#include <sstream>
#include <limits>

#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign; // bring 'operator+=()' into scope

//#include TEST_AC_DEFINE

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/VectorBTree.h>

using namespace std;
using namespace gtsam;

static Symbol l1('l',1), x1('x',1), x2('x',2);
static double inf = std::numeric_limits<double>::infinity();

/* ************************************************************************* */
VectorBTree smallVectorBTree() {
	VectorBTree c;
	c.insert(l1, Vector_(2,  0.0, -1.0));
	c.insert(x1, Vector_(2,  0.0,  0.0));
	c.insert(x2, Vector_(2,  1.5,  0.0));
	return c;
}

/* ************************************************************************* */
TEST( VectorBTree, constructor_insert_get )
{
	VectorBTree expected;
	Vector v = Vector_(3, 5.0, 6.0, 7.0);
	expected.insert(x1, v);
	VectorBTree actual(x1, v);
	LONGS_EQUAL(1,actual.size())
	CHECK(assert_equal(expected,actual))
	CHECK(equal_with_abs_tol(v,actual[x1]))
	CHECK(equal_with_abs_tol(v,actual.get(x1)))
}

/* ************************************************************************* */
TEST( VectorBTree, dim) {
	VectorBTree c = smallVectorBTree();
	LONGS_EQUAL(6,c.dim());
}

/* ************************************************************************* */
TEST( VectorBTree, insertAdd) {
	VectorBTree expected;
	expected.insert(l1, Vector_(2,  0.0, -2.0));
	expected.insert(x1, Vector_(2,  0.0,  0.0));
	expected.insert(x2, Vector_(2,  3.0,  0.0));
	VectorBTree actual = smallVectorBTree();
	actual.insertAdd(actual);
	CHECK(assert_equal(expected,actual))
}

/* ************************************************************************* */
TEST( VectorBTree, zero) {
	VectorBTree expected;
	expected.insert(l1, Vector_(2,  0.0,  0.0));
	expected.insert(x1, Vector_(2,  0.0,  0.0));
	expected.insert(x2, Vector_(2,  0.0,  0.0));
	VectorBTree actual = smallVectorBTree();
	CHECK(assert_equal(expected,VectorBTree::zero(actual)));
	CHECK(assert_equal(expected,actual.zero()));
}

/* ************************************************************************* */
TEST( VectorBTree, insert_config) {
	VectorBTree expected = smallVectorBTree();
	VectorBTree actual, toAdd = smallVectorBTree();
	actual.insert(toAdd);
	CHECK(assert_equal(expected,actual))
}

/* ************************************************************************* */
TEST( VectorBTree, get_names) {
	VectorBTree c = smallVectorBTree();
  std::vector<Symbol> expected, actual = c.get_names();
  expected += l1, x1, x2;
	CHECK(expected==actual)
}

/* ************************************************************************* */
TEST( VectorBTree, const_iterator) {
	VectorBTree c = smallVectorBTree();
	VectorBTree::const_iterator it = c.begin();
	CHECK(assert_equal(l1,it->first));
	CHECK(assert_equal(Vector_(2, 0.0,-1.0),(it++)->second));
	CHECK(assert_equal(x1,it->first));
	CHECK(assert_equal(Vector_(2, 0.0, 0.0),(it++)->second));
	CHECK(assert_equal(x2,it->first));
	CHECK(assert_equal(Vector_(2, 1.5, 0.0),(it++)->second));
	CHECK(it==c.end());
}

/* ************************************************************************* */
TEST( VectorBTree, equals )
{
	VectorBTree cfg1;
	cfg1.insert(x1, Vector_(3, 5.0, 6.0, 7.0));
	CHECK(cfg1.equals(cfg1));
	CHECK(cfg1.compatible(cfg1));
	CHECK(cfg1.cloned(cfg1));

	VectorBTree cfg2;
	cfg2.insert(x1, Vector_(3, 5.0, 6.0, 7.0));
	CHECK(cfg1.equals(cfg2));
	CHECK(cfg1.compatible(cfg2));
	CHECK(!cfg1.cloned(cfg2));

	VectorBTree cfg3 = cfg1;
	CHECK(cfg1.equals(cfg3));
	CHECK(cfg1.compatible(cfg3));
	CHECK(cfg1.cloned(cfg3));

	VectorBTree cfg4;
	cfg4.insert(x1, Vector_(3, 5.0, 6.0, 8.0));
	CHECK(!cfg1.equals(cfg4));
	CHECK(cfg1.compatible(cfg4));
	CHECK(!cfg1.cloned(cfg4));
}

/* ************************************************************************* */
TEST( VectorBTree, equals_nan )
 {
   VectorBTree cfg1, cfg2;
   Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
   Vector v2 = Vector_(3, inf, inf, inf);
   cfg1.insert(x1, v1);
   cfg2.insert(x1, v2);
   CHECK(!cfg1.equals(cfg2));
   CHECK(!cfg2.equals(cfg1));
 }

/* ************************************************************************* */
TEST( VectorBTree, contains)
{
  VectorBTree fg;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  fg.insert(x1, v);
  CHECK(fg.contains(x1));
  CHECK(!fg.contains(x2));
}

/* ************************************************************************* */
TEST( VectorBTree, max) {
	VectorBTree c = smallVectorBTree();
	DOUBLES_EQUAL(1.5,c.max(),1e-9);
}

/* ************************************************************************* */
TEST( VectorBTree, scale) {
	VectorBTree cfg;
	cfg.insert(x1, Vector_(2, 1.0, 2.0));
	cfg.insert(x2, Vector_(2,-1.0,-2.0));

	VectorBTree actual = cfg.scale(2.0);

	VectorBTree expected;
	expected.insert(x1, Vector_(2, 2.0, 4.0));
	expected.insert(x2, Vector_(2,-2.0,-4.0));

	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( VectorBTree, plus)
{
  VectorBTree c;
  Vector vx = Vector_(3, 5.0, 6.0, 7.0), vy = Vector_(2, 8.0, 9.0);
  c.insert(x1,vx).insert(x2,vy);

  VectorBTree delta;
  Vector dx = Vector_(3, 1.0, 1.0, 1.0), dy = Vector_(2, -1.0, -1.0);
  delta.insert(x1, dx).insert(x2,dy);
  CHECK(delta.compatible(c));

  // operator +
  VectorBTree expected;
  Vector wx = Vector_(3, 6.0, 7.0, 8.0), wy = Vector_(2, 7.0, 8.0);
  expected.insert(x1, wx).insert(x2,wy);
  CHECK(assert_equal(expected,c+delta));

  // operator -
  VectorBTree expected2;
  Vector wx2 = Vector_(3, -5.0, -6.0, -7.0), wy2 = Vector_(2, -8.0, -9.0);
  expected2.insert(x1, wx2).insert(x2,wy2);
  CHECK(assert_equal(expected2,-c));

  // expmap
  VectorBTree actual = expmap(c,delta);
  CHECK(assert_equal(expected,actual));

  // in-place (although + already tests that, really)
  c += delta;
  CHECK(assert_equal(expected,c));
}

/* ************************************************************************* */
TEST( VectorBTree, operators) {
	VectorBTree c; c.insert(x1, Vector_(2, 1.1, 2.2));
	VectorBTree expected1; expected1.insert(x1, Vector_(2, 2.2, 4.4));
	CHECK(assert_equal(expected1,c*2));
	CHECK(assert_equal(expected1,c+c));
	VectorBTree expected2; expected2.insert(x1, Vector_(2, 0.0, 0.0));
	CHECK(assert_equal(expected2,c-c));
}

/* ************************************************************************* */
TEST( VectorBTree, dot) {
	VectorBTree c = smallVectorBTree();
	DOUBLES_EQUAL(3.25,dot(c,c),1e-9);
}

/* ************************************************************************* */
TEST( VectorBTree, expmap)
{
	VectorBTree c = smallVectorBTree();
	Vector v = Vector_(6, 0.0,-1.0, 0.0, 0.0, 1.5, 0.0); // l1, x1, x2
  CHECK(assert_equal(expmap(c,c),expmap(c,v)));
}

/* ************************************************************************* */
TEST( VectorBTree, scal) {
  VectorBTree x,expected;
  x.insert(x1,Vector_(3, 1.0, 2.0, 3.0));
  x.insert(x2,Vector_(2, 4.0, 5.0));
  expected.insert(x1,Vector_(3, 10.0, 20.0, 30.0));
  expected.insert(x2,Vector_(2, 40.0, 50.0));
  scal(10,x);
  CHECK(assert_equal(expected,x));
}

/* ************************************************************************* */
TEST( VectorBTree, axpy) {
  VectorBTree x;
  x.insert(x1,Vector_(3, 1.0, 1.0, 1.0));
  x.insert(x2,Vector_(2, -1.0, -1.0));

  // axpy will only work on cloned configs - enforced for speed
  VectorBTree y = VectorBTree::zero(x);
  y[x1] = Vector_(3, 5.0, 6.0, 7.0);
  y[x2] = Vector_(2, 8.0, 9.0);
//  axpy(10,x,y);
//
//  // Check result
//  VectorBTree expected;
//  expected.insert(x1,Vector_(3, 15.0, 16.0, 17.0));
//  expected.insert(x2,Vector_(2, -2.0, -1.0));
//  CHECK(assert_equal(expected,y));
}

/* ************************************************************************* */
TEST( VectorBTree, subVector) {
	VectorBTree c; c.insert(x1, Vector_(2, 1.1, 2.2));
	SubVector cx = c[x1];
	for (size_t i = 0; i < 2; i++)
		cx(i) = cx(i)*2.0;
	VectorBTree expected; expected.insert(x1, Vector_(2, 2.2, 4.4));
	CHECK(assert_equal(expected,c));
}

/* ************************************************************************* */
#ifdef HAVE_BOOST_SERIALIZATION
TEST( VectorBTree, serialize)
{
    //DEBUG:
    cout << "VectorBTree: Running Serialization Test" << endl;
    
    //create an VectorBTree
    VectorBTree fg = createValues();
    
    //serialize the config
    std::ostringstream in_archive_stream;
    boost::archive::text_oarchive in_archive(in_archive_stream);
    in_archive << fg;
    std::string serialized_fgc = in_archive_stream.str();
    
    //deserialize the config
    std::istringstream out_archive_stream(serialized_fgc);
    boost::archive::text_iarchive out_archive(out_archive_stream);
    VectorBTree output;
    out_archive >> output;
    
    //check for equality
    CHECK(fg.equals(output));
}
#endif //HAVE_BOOST_SERIALIZATION

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}

/* ************************************************************************* */
