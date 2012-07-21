/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file    testDecisionTreeFactor.cpp
 * @brief   unit tests for DiscreteConditional
 * @author  Duy-Nguyen Ta
 * @date Feb 14, 2011
 */

#include <boost/make_shared.hpp>
#include <boost/assign/std/map.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( DiscreteConditionalTest, constructors)
{
	DiscreteKey X(0, 2), Y(2, 3), Z(1, 2); // watch ordering !

	DiscreteConditional::shared_ptr expected1 = //
			boost::make_shared<DiscreteConditional>(X | Y = "1/1 2/3 1/4");
	EXPECT(expected1);
	DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
	DiscreteConditional actual1(1, f1);
	EXPECT(assert_equal(*expected1, actual1, 1e-9));

	DecisionTreeFactor f2(X & Y & Z,
			"0.2 0.5 0.3 0.6 0.4 0.7 0.25 0.55 0.35 0.65 0.45 0.75");
	DiscreteConditional actual2(1, f2);
	DecisionTreeFactor::shared_ptr actual2factor = actual2.toFactor();
//	EXPECT(assert_equal(f2, *actual2factor, 1e-9));
}

/* ************************************************************************* */
TEST( DiscreteConditionalTest, constructors_alt_interface)
{
	DiscreteKey X(0, 2), Y(2, 3), Z(1, 2); // watch ordering !

	Signature::Table table;
	Signature::Row r1, r2, r3;
	r1 += 1.0, 1.0; r2 += 2.0, 3.0; r3 += 1.0, 4.0;
	table += r1, r2, r3;
	DiscreteConditional::shared_ptr expected1 = //
			boost::make_shared<DiscreteConditional>(X | Y = table);
	EXPECT(expected1);
	DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
	DiscreteConditional actual1(1, f1);
	EXPECT(assert_equal(*expected1, actual1, 1e-9));

	DecisionTreeFactor f2(X & Y & Z,
			"0.2 0.5 0.3 0.6 0.4 0.7 0.25 0.55 0.35 0.65 0.45 0.75");
	DiscreteConditional actual2(1, f2);
	DecisionTreeFactor::shared_ptr actual2factor = actual2.toFactor();
//	EXPECT(assert_equal(f2, *actual2factor, 1e-9));
}

/* ************************************************************************* */
TEST( DiscreteConditionalTest, constructors2)
{
	// Declare keys and ordering
	DiscreteKey C(0,2), B(1,2);
	DecisionTreeFactor expected(C & B, "0.8 0.75 0.2 0.25");
	Signature signature((C | B) = "4/1 3/1");
	DiscreteConditional actual(signature);
	DecisionTreeFactor::shared_ptr actualFactor = actual.toFactor();
	EXPECT(assert_equal(expected, *actualFactor));
}

/* ************************************************************************* */
TEST( DiscreteConditionalTest, constructors3)
{
	// Declare keys and ordering
	DiscreteKey C(0,2), B(1,2), A(2,2);
	DecisionTreeFactor expected(C & B & A, "0.8 0.5 0.5 0.2 0.2 0.5 0.5 0.8");
	Signature signature((C | B, A) = "4/1 1/1 1/1 1/4");
	DiscreteConditional actual(signature);
	DecisionTreeFactor::shared_ptr actualFactor = actual.toFactor();
	EXPECT(assert_equal(expected, *actualFactor));
}

/* ************************************************************************* */
int main() {	TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

