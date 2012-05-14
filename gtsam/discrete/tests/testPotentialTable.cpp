/*
 * @file		testPotentialTable.cpp
 * @brief		Develop recursive potential operations
 * @author	Frank Dellaert
 * @date	Mar 6, 2011
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/PotentialTable.h>

using namespace std;
using namespace gtsam;

/* ******************************************************************************** */
TEST( PotentialTable, Iterator)
{
	PotentialTable::Cardinalities cs;
	cs += 2, 3;
	PotentialTable::Iterator it(cs);
	LONGS_EQUAL(0,it[0]);
	LONGS_EQUAL(0,it[1]);
	++it;
	LONGS_EQUAL(0,it[0]);
	LONGS_EQUAL(1,it[1]);
	++it;
	LONGS_EQUAL(0,it[0]);
	LONGS_EQUAL(2,it[1]);
	++it;
	LONGS_EQUAL(1,it[0]);
	LONGS_EQUAL(0,it[1]);
	++it;
	LONGS_EQUAL(1,it[0]);
	LONGS_EQUAL(1,it[1]);
	++it;
	LONGS_EQUAL(1,it[0]);
	LONGS_EQUAL(2,it[1]);
	++it;
	LONGS_EQUAL(0,it[0]);
	LONGS_EQUAL(0,it[1]);
}

/* ******************************************************************************** */
#include <boost/unordered_map.hpp>

TEST( PotentialTable, unordered_map)
{
	boost::unordered_map<bool, int> x;
	x[false] = 7;
}

/* ******************************************************************************** */

struct Factor {
	vector<double> table_;
	vector<Index> keys_;
	bool operator==(const Factor& f) const {
		return table_ == f.table_ && keys_ == f.keys_;
	}
};

Factor operator*(const double& s, const Factor& f) {
	Factor r = f;
	BOOST_FOREACH(double& ri, r.table_)
					ri *= s;
	return r;
}

Factor operator*(const Factor& f, const double& s) {
	Factor r = f;
	BOOST_FOREACH(double& ri, r.table_)
					ri *= s;
	return r;
}

Factor operator*(const Factor& f1, const Factor& f2) {
	Factor r;

	// base case 1, both tables start with same key
	if (f1.keys_.front() == f2.keys_.front()) {
	}

	return r;
}

/* ******************************************************************************** */
// f(5)*f(5) = f0*f0 @ f1*f1
TEST( PotentialTable, baseCase1a)
{
	Factor f1, f2, expected;
	f1.table_ += 00, 01;
	f2.table_ += 20, 21;
	f1.keys_ += 5;
	f2.keys_ += 5;
	expected.table_ += 00 * 20, 01 * 21;
	expected.keys_ += 5;
	CHECK(f1*f2==expected)
}

/* ******************************************************************************** */
// f(0,1)*f(0) = f0(1)*f0 @ f1(1)*f1
TEST( PotentialTable, baseCase1b)
{
	Factor f1, f2, expected;
	f1.table_ += 00, 01, 10, 11;
	f2.table_ += 20, 21;
	f1.keys_ += 0, 1;
	f2.keys_ += 0;
	expected.table_ += 00 * 20, 00 * 21, 01 * 20, 01 * 21, 10 * 20, 10 * 21, 11
			* 20, 11 * 21;
	expected.keys_ += 0, 1, 2;
	CHECK(f1*f2==expected)
}

/* ******************************************************************************** */
// f0(1)*f(2) = f00*f(2) @ f01*f(2)
TEST( PotentialTable, baseCase2)
{
	Factor f1, f2, expected;
	f1.table_ += 00, 01;
	f2.table_ += 20, 21;
	f1.keys_ += 1;
	f2.keys_ += 2;
	expected.table_ += 00 * 20, 00 * 21, 01 * 20, 01 * 21;
	expected.keys_ += 1, 2;
	CHECK(f1*f2==expected)
}

/* ******************************************************************************** */
// f(0,1)*f(2) = f0(1)*f(2) @ f1(1)*f(2)
// f0(1)*f(2) = f00*f(2) @ f01*f(2)
TEST( PotentialTable, multiplication)
{
	Factor f1, f2, expected;
	f1.table_ += 00, 01, 10, 11;
	f2.table_ += 20, 21;
	f1.keys_ += 0, 1;
	f2.keys_ += 2;
	expected.table_ += 00 * 20, 00 * 21, 01 * 20, 01 * 21, 10 * 20, 10 * 21, 11
			* 20, 11 * 21;
	expected.keys_ += 0, 1, 2;
	CHECK(f1*f2==expected)
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
