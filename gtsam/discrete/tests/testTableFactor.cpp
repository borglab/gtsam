/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file testTableFactor.cpp
 *  @date May 12, 2022
 *  @author Yoonwoo Kim
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/discrete/DecisionTreeFactor.h>

#include <Eigen/Sparse>
#include <boost/assign/std/map.hpp>

using namespace boost::assign;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(TableFactor, constructors) {
  // Declare a bunch of keys
  DiscreteKey X(0, 2), Y(1, 3), Z(2, 2), A(3, 5);

  // Create factors
  TableFactor f_zeros(A, {0, 0, 0, 0, 1});
  TableFactor f1(X, {2, 8});
  TableFactor f2(X & Y, "2 5 3 6 4 7");
  TableFactor f3(X & Y & Z, "2 5 3 6 4 7 25 55 35 65 45 75");
  EXPECT_LONGS_EQUAL(1, f1.size());
  EXPECT_LONGS_EQUAL(2, f2.size());
  EXPECT_LONGS_EQUAL(3, f3.size());

  DiscreteValues values;
  values[0] = 1;  // x
  values[1] = 2;  // y
  values[2] = 1;  // z
  values[3] = 4;
  EXPECT_DOUBLES_EQUAL(1, f_zeros(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(8, f1(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(7, f2(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(75, f3(values), 1e-9);
}

/* ************************************************************************* */
TEST(TableFactor, lazy_cartesian_product) {
  DiscreteKey v0(0, 2), v1(1, 2), v2(2, 2), A(3, 5);

  TableFactor f1(v0 & v1, "1 2 3 4");
  double actual_v0 = f1.lazy_cp(v0.first, 2);
  EXPECT_DOUBLES_EQUAL(1, actual_v0, 1e-9);

  double actual_v1 = f1.lazy_cp(v1.first, 2);
  EXPECT_DOUBLES_EQUAL(0, actual_v1, 1e-9);

  TableFactor f_zeros(A, {0, 0, 0, 0, 1});
  double actual_zero = f_zeros.lazy_cp(A.first, 3);
  EXPECT_DOUBLES_EQUAL(3, actual_zero, 1e-9);
}

/* ************************************************************************* */
TEST(TableFactor, project) {
  DiscreteKey v0(0, 2), v1(1, 2), v2(2, 2), A(3, 5);

  TableFactor f1(v0 & v1, "1 2 3 4");
  // (v0 = 0)
  DiscreteValues assignment_1;
  assignment_1[v0.first] = 0;;
  std::vector<DiscreteValues> actual_vec1 = f1.project(assignment_1);
  DiscreteValues expected_1;
  expected_1[v0.first] = 0;
  expected_1[v1.first] = 0;
  DiscreteValues expected_2;
  expected_2[v0.first] = 0;
  expected_2[v1.first] = 1;
  std::vector<DiscreteValues> expected_vec1 = {expected_1, expected_2};
  EXPECT(expected_vec1 == actual_vec1);
  
  // (v0 = 1)
  DiscreteValues assignment_2;
  assignment_2[v0.first] = 1;
  std::vector<DiscreteValues> actual_vec2 = f1.project(assignment_2);
  DiscreteValues expected_3;
  expected_3[v0.first] = 1;
  expected_3[v1.first] = 0;
  DiscreteValues expected_4;
  expected_4[v0.first] = 1;
  expected_4[v1.first] = 1;
  std::vector<DiscreteValues> expected_vec2 = {expected_3, expected_4};
  EXPECT(expected_vec2 == actual_vec2);
  
  // (v1 = 0)
  DiscreteValues assignment_3;
  assignment_3[v1.first] = 0;
  std::vector<DiscreteValues> actual_vec3 = f1.project(assignment_3);
  DiscreteValues expected_5;
  expected_5[v0.first] = 0;
  expected_5[v1.first] = 0;
  DiscreteValues expected_6;
  expected_6[v0.first] = 1;
  expected_6[v1.first] = 0;
  std::vector<DiscreteValues> expected_vec3 = {expected_5, expected_6};
  EXPECT(expected_vec3 == actual_vec3);
  
  //  (v1 = 1)
  DiscreteValues assignment_4;
  assignment_4[v1.first] = 1;
  std::vector<DiscreteValues> actual_vec4 = f1.project(assignment_4);
  DiscreteValues expected_7;
  expected_7[v0.first] = 0;
  expected_7[v1.first] = 1;
  DiscreteValues expected_8;
  expected_8[v0.first] = 1;
  expected_8[v1.first] = 1;
  std::vector<DiscreteValues> expected_vec4 = {expected_7, expected_8};
  EXPECT(expected_vec4 == actual_vec4);
}

/* ************************************************************************* */
TEST(TableFactor, find_idx) {
  DiscreteKey v0(0, 2), v1(1, 3);
  TableFactor f1(v0 & v1, "1 2 3 4 5 6");

  DiscreteValues assignment;
  assignment[v0.first] = 1;
  assignment[v1.first] = 1;

  double actual_1 = f1.findIndex(assignment);
  EXPECT_LONGS_EQUAL(1 * 3 + 1, actual_1);

  assignment[v0.first] = 1;
  assignment[v1.first] = 0;

  double actual_2 = f1.findIndex(assignment);
  EXPECT_LONGS_EQUAL(1 * 3 + 0, actual_2);
}

/* ************************************************************************* */
TEST(TableFactor, multiplication) {
  DiscreteKey v0(0, 2), v1(1, 2), v2(2, 2);

  // Multiply with a DiscreteDistribution, i.e., Bayes Law!
  DiscreteDistribution prior(v1 % "1/3");
  TableFactor f1(v0 & v1, "1 2 3 4");
  DecisionTreeFactor expected(v0 & v1, "0.25 1.5 0.75 3");
  CHECK(assert_equal(expected, static_cast<DecisionTreeFactor>(prior) *
                                   f1.toDecisionTreeFactor()));
  CHECK(assert_equal(expected, f1 * prior));

  // Multiply two factors
  TableFactor f2(v1 & v2, "5 6 7 8");
  TableFactor actual = f1 * f2;
  TableFactor expected2(v0 & v1 & v2, "5 6 14 16 15 18 28 32");
  CHECK(assert_equal(expected2, actual));

  DiscreteKey A(0, 3), B(1, 2), C(2, 2);
  TableFactor f_zeros1(A & C, "0 0 0 2 0 3");
  TableFactor f_zeros2(B & C, "4 0 0 5");
  TableFactor actual_zeros = f_zeros1 * f_zeros2;
  TableFactor expected3(A & B & C, "0 0 0 0 0 0 0 10 0 0 0 15");
  CHECK(assert_equal(expected3, actual_zeros));
}

/* ************************************************************************* */
TEST(TableFactor, sum_max) {
  DiscreteKey v0(0, 3), v1(1, 2);
  TableFactor f1(v0 & v1, "1 2  3 4  5 6");

  TableFactor expected(v1, "9 12");
  TableFactor::shared_ptr actual = f1.sum(1);
  CHECK(assert_equal(expected, *actual, 1e-5));

  TableFactor expected2(v1, "5 6");
  TableFactor actual2 = f1.max(1);
  CHECK(assert_equal(expected2, actual2));

  TableFactor f2(v1 & v0, "1 2  3 4  5 6");
  TableFactor::shared_ptr actual22 = f2.sum(1);
}

/* ************************************************************************* */
TEST( DecisionTreeFactor, max_ordering)
{
  DiscreteKey v0(0,3), v1(1,2), v2(2, 2);
  TableFactor f1(v0 & v1 & v2, "1 2 3 10 5 6 7 11 9 4 8 12");
  Ordering ordering;
  ordering += Key(1);

  TableFactor actual = f1.max(ordering);
  TableFactor expected(v0 & v2, "3 10 7 11 9 12");
  CHECK(assert_equal(expected, actual));

  TableFactor f2(v0 & v1 & v2, "0 0 0 0 0 0 0 15 0 0 0 10");
  TableFactor actual_zeros = f2.max(ordering);
  TableFactor expected_zeros(v0 & v2, "0 0 0 15 0 10");
  CHECK(assert_equal(expected_zeros, actual_zeros));
}

/* ************************************************************************* */
TEST( DecisionTreeFactor, max_assignment)
{
  DiscreteKey v0(0,3), v1(1,2), v2(2, 2);
  TableFactor f1(v0 & v1 & v2, "1 2 3 10 5 6 7 11 9 4 8 12");
  DiscreteValues assignment = f1.maxAssignment();
  DiscreteValues expected;
  expected[v0.first] = 2;
  expected[v1.first] = 1;
  expected[v2.first] = 1;
  CHECK(assert_equal(expected, assignment));

  TableFactor f2(v0 & v1 & v2, "0 0 0 0 0 0 0 15 0 0 0 10");
  DiscreteValues assignment_zeros = f2.maxAssignment();
  DiscreteValues expected_zeros;
  expected_zeros[v0.first] = 1;
  expected_zeros[v1.first] = 1;
  expected_zeros[v2.first] = 1;
  CHECK(assert_equal(expected_zeros, assignment_zeros));
}

/* ************************************************************************* */
// Check enumerate yields the correct list of assignment/value pairs.
TEST(TableFactor, enumerate) {
  DiscreteKey A(12, 3), B(5, 2);
  TableFactor f(A & B, "1 2  3 4  5 6");
  auto actual = f.enumerate();
  std::vector<std::pair<DiscreteValues, double>> expected;
  DiscreteValues values;
  for (size_t a : {0, 1, 2}) {
    for (size_t b : {0, 1}) {
      values[12] = a;
      values[5] = b;
      expected.emplace_back(values, f(values));
    }
  }
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected.
TEST(TableFactor, markdown) {
  DiscreteKey A(12, 3), B(5, 2);
  TableFactor f(A & B, "1 2  3 4  5 6");
  string expected =
      "|A|B|value|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0|1|\n"
      "|0|1|2|\n"
      "|1|0|3|\n"
      "|1|1|4|\n"
      "|2|0|5|\n"
      "|2|1|6|\n";
  auto formatter = [](Key key) { return key == 12 ? "A" : "B"; };
  string actual = f.markdown(formatter);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation with a value formatter.
TEST(TableFactor, markdownWithValueFormatter) {
  DiscreteKey A(12, 3), B(5, 2);
  TableFactor f(A & B, "1 2  3 4  5 6");
  string expected =
      "|A|B|value|\n"
      "|:-:|:-:|:-:|\n"
      "|Zero|-|1|\n"
      "|Zero|+|2|\n"
      "|One|-|3|\n"
      "|One|+|4|\n"
      "|Two|-|5|\n"
      "|Two|+|6|\n";
  auto keyFormatter = [](Key key) { return key == 12 ? "A" : "B"; };
  TableFactor::Names names{{12, {"Zero", "One", "Two"}},
                                  {5, {"-", "+"}}};
  string actual = f.markdown(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check html representation with a value formatter.
TEST(TableFactor, htmlWithValueFormatter) {
  DiscreteKey A(12, 3), B(5, 2);
  TableFactor f(A & B, "1 2  3 4  5 6");
  string expected =
      "<div>\n"
      "<table class='TableFactor'>\n"
      "  <thead>\n"
      "    <tr><th>A</th><th>B</th><th>value</th></tr>\n"
      "  </thead>\n"
      "  <tbody>\n"
      "    <tr><th>Zero</th><th>-</th><td>1</td></tr>\n"
      "    <tr><th>Zero</th><th>+</th><td>2</td></tr>\n"
      "    <tr><th>One</th><th>-</th><td>3</td></tr>\n"
      "    <tr><th>One</th><th>+</th><td>4</td></tr>\n"
      "    <tr><th>Two</th><th>-</th><td>5</td></tr>\n"
      "    <tr><th>Two</th><th>+</th><td>6</td></tr>\n"
      "  </tbody>\n"
      "</table>\n"
      "</div>";
  auto keyFormatter = [](Key key) { return key == 12 ? "A" : "B"; };
  TableFactor::Names names{{12, {"Zero", "One", "Two"}},
                                  {5, {"-", "+"}}};
  string actual = f.html(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
