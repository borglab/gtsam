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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/Signature.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( DecisionTreeFactor, constructors)
{
  // Declare a bunch of keys
  DiscreteKey X(0,2), Y(1,3), Z(2,2);

  // Create factors
  DecisionTreeFactor f1(X, {2, 8});
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
TEST(DecisionTreeFactor, multiplication) {
  DiscreteKey v0(0, 2), v1(1, 2), v2(2, 2);

  // Multiply with a DiscreteDistribution, i.e., Bayes Law!
  DiscreteDistribution prior(v1 % "1/3");
  DecisionTreeFactor f1(v0 & v1, "1 2 3 4");
  DecisionTreeFactor expected(v0 & v1, "0.25 1.5 0.75 3");
  CHECK(assert_equal(expected, static_cast<DecisionTreeFactor>(prior) * f1));
  CHECK(assert_equal(expected, f1 * prior));

  // Multiply two factors
  DecisionTreeFactor f2(v1 & v2, "5 6 7 8");
  DecisionTreeFactor actual = f1 * f2;
  DecisionTreeFactor expected2(v0 & v1 & v2, "5 6 14 16 15 18 28 32");
  CHECK(assert_equal(expected2, actual));
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
// Check enumerate yields the correct list of assignment/value pairs.
TEST(DecisionTreeFactor, enumerate) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
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
// Check pruning of the decision tree works as expected.
TEST(DecisionTreeFactor, Prune) {
  DiscreteKey A(1, 2), B(2, 2), C(3, 2);
  DecisionTreeFactor f(A & B & C, "1 5 3 7 2 6 4 8");

  // Only keep the leaves with the top 5 values.
  size_t maxNrAssignments = 5;
  auto pruned5 = f.prune(maxNrAssignments);

  // Pruned leaves should be 0
  DecisionTreeFactor expected(A & B & C, "0 5 0 7 0 6 4 8");
  EXPECT(assert_equal(expected, pruned5));

  // Check for more extreme pruning where we only keep the top 2 leaves
  maxNrAssignments = 2;
  auto pruned2 = f.prune(maxNrAssignments);
  DecisionTreeFactor expected2(A & B & C, "0 0 0 7 0 0 0 8");
  EXPECT(assert_equal(expected2, pruned2));

  DiscreteKey D(4, 2);
  DecisionTreeFactor factor(
      D & C & B & A,
      "0.0 0.0 0.0 0.60658897 0.61241912 0.61241969 0.61247685 0.61247742 0.0 "
      "0.0 0.0 0.99995287 1.0 1.0 1.0 1.0");

  DecisionTreeFactor expected3(
      D & C & B & A,
      "0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 "
      "0.999952870000 1.0 1.0 1.0 1.0");
  maxNrAssignments = 5;
  auto pruned3 = factor.prune(maxNrAssignments);
  EXPECT(assert_equal(expected3, pruned3));
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, DotWithNames) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
  auto formatter = [](Key key) { return key == 12 ? "A" : "B"; };

  for (bool showZero:{true, false}) {
    string actual = f.dot(formatter, showZero);
    // pretty weak test, as ids are pointers and not stable across platforms.
    string expected = "digraph G {";
    EXPECT(actual.substr(0, 11) == expected);
  }
}

/* ************************************************************************* */
// Check markdown representation looks as expected.
TEST(DecisionTreeFactor, markdown) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
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
TEST(DecisionTreeFactor, markdownWithValueFormatter) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
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
  DecisionTreeFactor::Names names{{12, {"Zero", "One", "Two"}},
                                  {5, {"-", "+"}}};
  string actual = f.markdown(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check html representation with a value formatter.
TEST(DecisionTreeFactor, htmlWithValueFormatter) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
  string expected =
      "<div>\n"
      "<table class='DecisionTreeFactor'>\n"
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
  DecisionTreeFactor::Names names{{12, {"Zero", "One", "Two"}},
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
