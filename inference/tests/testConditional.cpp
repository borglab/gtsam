/**
 * @file    testConditional.cpp
 * @brief   Unit tests for Conditional class
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/Factor-inl.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Conditional, empty )
{
  Conditional c0;
  LONGS_EQUAL(0,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( Conditional, noParents )
{
  Conditional c0(0);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( Conditional, oneParents )
{
  Conditional c0(0,1);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(1,c0.nrParents())
}

/* ************************************************************************* */
TEST( Conditional, twoParents )
{
  Conditional c0(0,1,2);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(2,c0.nrParents())
}

/* ************************************************************************* */
TEST( Conditional, threeParents )
{
  Conditional c0(0,1,2,3);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(3,c0.nrParents())
}

/* ************************************************************************* */
TEST( Conditional, fourParents )
{
	vector<Index> parents;
	parents += 1,2,3,4;
  Conditional c0(0,parents);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(4,c0.nrParents())
}

/* ************************************************************************* */
TEST( Conditional, FromRange )
{
	list<Index> keys;
	keys += 1,2,3,4,5;
  Conditional::shared_ptr c0 = Conditional::FromRange(keys.begin(),keys.end(),2);
  LONGS_EQUAL(2,c0->nrFrontals())
  LONGS_EQUAL(3,c0->nrParents())
}

/* ************************************************************************* */
TEST( Conditional, equals )
{
  Conditional c0(0, 1, 2), c1(0, 1, 2), c2(1, 2, 3), c3(3,4);
  CHECK(c0.equals(c1));
  CHECK(c1.equals(c0));
  CHECK(!c0.equals(c2));
  CHECK(!c2.equals(c0));
  CHECK(!c0.equals(c3));
  CHECK(!c3.equals(c0));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
