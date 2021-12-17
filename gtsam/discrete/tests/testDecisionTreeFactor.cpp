/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDecisionTreeFactor.cpp
 *
 *  @date Feb 5, 2012
 *  @author Frank Dellaert
 *  @author Duy-Nguyen Ta
 */

#include <gtsam/discrete/Signature.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( DecisionTreeFactor, constructors)
{
  // Declare a bunch of keys
  DiscreteKey X(0,2), Y(1,3), Z(2,2);

  // Create factors
  DecisionTreeFactor f1(X, "2 8");
  DecisionTreeFactor f2(X & Y, "2 5 3 6 4 7");
  DecisionTreeFactor f3(X & Y & Z, "2 5 3 6 4 7 25 55 35 65 45 75");
  EXPECT_LONGS_EQUAL(1,f1.size());
  EXPECT_LONGS_EQUAL(2,f2.size());
  EXPECT_LONGS_EQUAL(3,f3.size());

  DiscreteValues values;
  values[0] = 1; // x
  values[1] = 2; // y
  values[2] = 1; // z
  EXPECT_DOUBLES_EQUAL(8, f1(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(7, f2(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(75, f3(values), 1e-9);
}

/* ************************************************************************* */
TEST_UNSAFE( DecisionTreeFactor, multiplication)
{
  DiscreteKey v0(0,2), v1(1,2), v2(2,2);

  DecisionTreeFactor f1(v0 & v1, "1 2 3 4");
  DecisionTreeFactor f2(v1 & v2, "5 6 7 8");

  DecisionTreeFactor expected(v0 & v1 & v2, "5 6 14 16 15 18 28 32");

  DecisionTreeFactor actual = f1 * f2;
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( DecisionTreeFactor, sum_max)
{
  DiscreteKey v0(0,3), v1(1,2);
  DecisionTreeFactor f1(v0 & v1, "1 2  3 4  5 6");

  DecisionTreeFactor expected(v1, "9 12");
  DecisionTreeFactor::shared_ptr actual = f1.sum(1);
  CHECK(assert_equal(expected, *actual, 1e-5));

  DecisionTreeFactor expected2(v1, "5 6");
  DecisionTreeFactor::shared_ptr actual2 = f1.max(1);
  CHECK(assert_equal(expected2, *actual2));

  DecisionTreeFactor f2(v1 & v0, "1 2  3 4  5 6");
  DecisionTreeFactor::shared_ptr actual22 = f2.sum(1);
}

/* ************************************************************************* */
TEST( DecisionTreeFactor, markdown)
{
  DiscreteKey v0(0,3), v1(1,2);
  DecisionTreeFactor f1(v0 & v1, "1 2  3 4  5 6");
  string expected =
      "|0(3)|1(2)|value|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0|1|\n"
      "|1|0|3|\n"
      "|2|0|5|\n"
      "|0|1|2|\n"
      "|1|1|4|\n"
      "|2|1|6|\n";
  string actual = f1._repr_markdown_();
  EXPECT(actual == expected);
  }

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

