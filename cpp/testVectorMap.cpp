/**
 * @file   testVectorMap.cpp
 * @brief  Unit tests for Factor Graph Configuration
 * @author Carlos Nieto
 **/

/*STL/C++*/
#include <iostream>
#include <sstream>

//#include TEST_AC_DEFINE

#ifdef HAVE_BOOST_SERIALIZATION
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#endif //HAVE_BOOST_SERIALIZATION

#define GTSAM_MAGIC_KEY

#include <CppUnitLite/TestHarness.h>
#include "Matrix.h"
#include "VectorMap.h"

using namespace std;
using namespace gtsam;

static Symbol l1('l',1), x1('x',1), x2('x',2);

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
   expected.insert("a",v);
   VectorMap actual;
   actual.insert("a",v);
   CHECK(assert_equal(expected,actual));
   CHECK(assert_equal(expected["a"],actual.get("a")))
}

/* ************************************************************************* */
TEST( VectorMap, equals2 )
 {	 
   VectorMap cfg1, cfg2;
   Vector v1 = Vector_(3, 5.0, 6.0, 7.0);
   Vector v2 = Vector_(3, 5.0, 6.0, 8.0);
   cfg1.insert("x", v1);
   cfg2.insert("x", v2);
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
   cfg1.insert("x", v1);
   cfg2.insert("x", v2);
   CHECK(!cfg1.equals(cfg2));
   CHECK(!cfg2.equals(cfg1));
 }

/* ************************************************************************* */
TEST( VectorMap, contains)
{
  VectorMap fg;
  Vector v = Vector_(3, 5.0, 6.0, 7.0);
  fg.insert("a", v);
  CHECK(fg.contains("a"));
  CHECK(!fg.contains("g"));
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
  c += VectorMap("x",vx);
  c += VectorMap("y",vy);

  VectorMap delta;
  Vector dx = Vector_(3, 1.0, 1.0, 1.0), dy = Vector_(2, -1.0, -1.0);
  delta.insert("x", dx).insert("y",dy);

  VectorMap expected;
  Vector wx = Vector_(3, 6.0, 7.0, 8.0), wy = Vector_(2, 7.0, 8.0);
  expected.insert("x", wx).insert("y",wy);

  // functional
  VectorMap actual = expmap(c,delta);
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( VectorMap, scale) {
	VectorMap cfg;
	cfg.insert("x", Vector_(2, 1.0, 2.0));
	cfg.insert("y", Vector_(2,-1.0,-2.0));

	VectorMap actual = cfg.scale(2.0);

	VectorMap expected;
	expected.insert("x", Vector_(2, 2.0, 4.0));
	expected.insert("y", Vector_(2,-2.0,-4.0));

	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( VectorMap, axpy) {
  VectorMap x,y,expected;
  x += VectorMap("x",Vector_(3, 1.0, 1.0, 1.0));
  x += VectorMap("y",Vector_(2, -1.0, -1.0));
  y += VectorMap("x",Vector_(3, 5.0, 6.0, 7.0));
  y += VectorMap("y",Vector_(2, 8.0, 9.0));
  expected += VectorMap("x",Vector_(3, 15.0, 16.0, 17.0));
  expected += VectorMap("y",Vector_(2, -2.0, -1.0));
  axpy(10,x,y);
  CHECK(assert_equal(expected,y));
}

/* ************************************************************************* */
TEST( VectorMap, scal) {
  VectorMap x,expected;
  x += VectorMap("x",Vector_(3, 1.0, 2.0, 3.0));
  x += VectorMap("y",Vector_(2, 4.0, 5.0));
  expected += VectorMap("x",Vector_(3, 10.0, 20.0, 30.0));
  expected += VectorMap("y",Vector_(2, 40.0, 50.0));
  scal(10,x);
  CHECK(assert_equal(expected,x));
}

/* ************************************************************************* */
TEST( VectorMap, update_with_large_delta) {
	// this test ensures that if the update for delta is larger than
	// the size of the config, it only updates existing variables
	VectorMap init, delta;
	init.insert("x", Vector_(2, 1.0, 2.0));
	init.insert("y", Vector_(2, 3.0, 4.0));
	delta.insert("x", Vector_(2, 0.1, 0.1));
	delta.insert("y", Vector_(2, 0.1, 0.1));
	delta.insert("p", Vector_(2, 0.1, 0.1));

	VectorMap actual = expmap(init,delta);
	VectorMap expected;
	expected.insert("x", Vector_(2, 1.1, 2.1));
	expected.insert("y", Vector_(2, 3.1, 4.1));

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
	VectorMap c; c.insert("x", Vector_(2, 1.1, 2.2));
	VectorMap expected1; expected1.insert("x", Vector_(2, 2.2, 4.4));
	CHECK(assert_equal(expected1,c*2));
	CHECK(assert_equal(expected1,c+c));
	VectorMap expected2; expected2.insert("x", Vector_(2, 0.0, 0.0));
	CHECK(assert_equal(expected2,c-c));
}

/* ************************************************************************* */
TEST( VectorMap, getReference) {
	VectorMap c; c.insert("x", Vector_(2, 1.1, 2.2));
	Vector& cx = c["x"];
	cx = cx*2.0;
	VectorMap expected; expected.insert("x", Vector_(2, 2.2, 4.4));
	CHECK(assert_equal(expected,c));
}

/* ************************************************************************* */
#ifdef HAVE_BOOST_SERIALIZATION
TEST( VectorMap, serialize)
{
    //DEBUG:
    cout << "VectorMap: Running Serialization Test" << endl;
    
    //create an VectorMap
    VectorMap fg = createConfig();
    
    //serialize the config
    std::ostringstream in_archive_stream;
    boost::archive::text_oarchive in_archive(in_archive_stream);
    in_archive << fg;
    std::string serialized_fgc = in_archive_stream.str();
    
    //deserialize the config
    std::istringstream out_archive_stream(serialized_fgc);
    boost::archive::text_iarchive out_archive(out_archive_stream);
    VectorMap output;
    out_archive >> output;
    
    //check for equality
    CHECK(fg.equals(output));
}
#endif //HAVE_BOOST_SERIALIZATION
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
