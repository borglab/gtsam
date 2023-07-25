/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testTableFactor.cpp
 *
 *  @date Feb 15, 2023
 *  @author Yoonwoo Kim
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/discrete/TableFactor.h>

#include <chrono>
#include <random>

using namespace std;
using namespace gtsam;

vector<double> genArr(double dropout, size_t size) {
  random_device rd;
  mt19937 g(rd());
  vector<double> dropoutmask(size);  // Chance of 0

  uniform_int_distribution<> dist(1, 9);
  auto gen = [&dist, &g]() { return dist(g); };
  generate(dropoutmask.begin(), dropoutmask.end(), gen);

  fill_n(dropoutmask.begin(), dropoutmask.size() * (dropout), 0);
  shuffle(dropoutmask.begin(), dropoutmask.end(), g);

  return dropoutmask;
}

map<double, pair<chrono::microseconds, chrono::microseconds>> measureTime(
    DiscreteKeys keys1, DiscreteKeys keys2, size_t size) {
  vector<double> dropouts = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  map<double, pair<chrono::microseconds, chrono::microseconds>> measured_times;

  for (auto dropout : dropouts) {
    vector<double> arr1 = genArr(dropout, size);
    vector<double> arr2 = genArr(dropout, size);
    TableFactor f1(keys1, arr1);
    TableFactor f2(keys2, arr2);
    DecisionTreeFactor f1_dt(keys1, arr1);
    DecisionTreeFactor f2_dt(keys2, arr2);

    // measure time TableFactor
    auto tb_start = chrono::high_resolution_clock::now();
    TableFactor actual = f1 * f2;
    auto tb_end = chrono::high_resolution_clock::now();
    auto tb_time_diff =
        chrono::duration_cast<chrono::microseconds>(tb_end - tb_start);

    // measure time DT
    auto dt_start = chrono::high_resolution_clock::now();
    DecisionTreeFactor actual_dt = f1_dt * f2_dt;
    auto dt_end = chrono::high_resolution_clock::now();
    auto dt_time_diff =
        chrono::duration_cast<chrono::microseconds>(dt_end - dt_start);

    bool flag = true;
    for (auto assignmentVal : actual_dt.enumerate()) {
      flag = actual_dt(assignmentVal.first) != actual(assignmentVal.first);
      if (flag) {
        std::cout << "something is wrong: " << std::endl;
        assignmentVal.first.print();
        std::cout << "dt: " << actual_dt(assignmentVal.first) << std::endl;
        std::cout << "tb: " << actual(assignmentVal.first) << std::endl;
        break;
      }
    }
    if (flag) break;
    measured_times[dropout] = make_pair(tb_time_diff, dt_time_diff);
  }
  return measured_times;
}

void printTime(map<double, pair<chrono::microseconds, chrono::microseconds>>
                   measured_time) {
  for (auto&& kv : measured_time) {
    cout << "dropout: " << kv.first
         << " | TableFactor time: " << kv.second.first.count()
         << " | DecisionTreeFactor time: " << kv.second.second.count() << endl;
  }
}

/* ************************************************************************* */
// Check constructors for TableFactor.
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
  values[3] = 4;  // a
  EXPECT_DOUBLES_EQUAL(1, f_zeros(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(8, f1(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(7, f2(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(75, f3(values), 1e-9);

  // Assert that error = -log(value)
  EXPECT_DOUBLES_EQUAL(-log(f1(values)), f1.error(values), 1e-9);

  // Construct from DiscreteConditional
  DiscreteConditional conditional(X | Y = "1/1 2/3 1/4");
  TableFactor f4(conditional);
  // Manually constructed via inspection and comparison to DecisionTreeFactor
  TableFactor expected(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
  EXPECT(assert_equal(expected, f4));
}

/* ************************************************************************* */
// Check multiplication between two TableFactors.
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
// Benchmark which compares runtime of multiplication of two TableFactors
// and two DecisionTreeFactors given sparsity from dense to 90% sparsity.
// NOTE: Enable to run.
TEST_DISABLED(TableFactor, benchmark) {
  DiscreteKey A(0, 5), B(1, 2), C(2, 5), D(3, 2), E(4, 5), F(5, 2), G(6, 3),
      H(7, 2), I(8, 5), J(9, 7), K(10, 2), L(11, 3);

  // 100
  DiscreteKeys one_1 = {A, B, C, D};
  DiscreteKeys one_2 = {C, D, E, F};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_1 =
      measureTime(one_1, one_2, 100);
  printTime(time_map_1);
  // 200
  DiscreteKeys two_1 = {A, B, C, D, F};
  DiscreteKeys two_2 = {B, C, D, E, F};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_2 =
      measureTime(two_1, two_2, 200);
  printTime(time_map_2);
  // 300
  DiscreteKeys three_1 = {A, B, C, D, G};
  DiscreteKeys three_2 = {C, D, E, F, G};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_3 =
      measureTime(three_1, three_2, 300);
  printTime(time_map_3);
  // 400
  DiscreteKeys four_1 = {A, B, C, D, F, H};
  DiscreteKeys four_2 = {B, C, D, E, F, H};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_4 =
      measureTime(four_1, four_2, 400);
  printTime(time_map_4);
  // 500
  DiscreteKeys five_1 = {A, B, C, D, I};
  DiscreteKeys five_2 = {C, D, E, F, I};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_5 =
      measureTime(five_1, five_2, 500);
  printTime(time_map_5);
  // 600
  DiscreteKeys six_1 = {A, B, C, D, F, G};
  DiscreteKeys six_2 = {B, C, D, E, F, G};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_6 =
      measureTime(six_1, six_2, 600);
  printTime(time_map_6);
  // 700
  DiscreteKeys seven_1 = {A, B, C, D, J};
  DiscreteKeys seven_2 = {C, D, E, F, J};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_7 =
      measureTime(seven_1, seven_2, 700);
  printTime(time_map_7);
  // 800
  DiscreteKeys eight_1 = {A, B, C, D, F, H, K};
  DiscreteKeys eight_2 = {B, C, D, E, F, H, K};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_8 =
      measureTime(eight_1, eight_2, 800);
  printTime(time_map_8);
  // 900
  DiscreteKeys nine_1 = {A, B, C, D, G, L};
  DiscreteKeys nine_2 = {C, D, E, F, G, L};
  map<double, pair<chrono::microseconds, chrono::microseconds>> time_map_9 =
      measureTime(nine_1, nine_2, 900);
  printTime(time_map_9);
}

/* ************************************************************************* */
// Check sum and max over frontals.
TEST(TableFactor, sum_max) {
  DiscreteKey v0(0, 3), v1(1, 2);
  TableFactor f1(v0 & v1, "1 2  3 4  5 6");

  TableFactor expected(v1, "9 12");
  TableFactor::shared_ptr actual = f1.sum(1);
  CHECK(assert_equal(expected, *actual, 1e-5));

  TableFactor expected2(v1, "5 6");
  TableFactor::shared_ptr actual2 = f1.max(1);
  CHECK(assert_equal(expected2, *actual2));

  TableFactor f2(v1 & v0, "1 2  3 4  5 6");
  TableFactor::shared_ptr actual22 = f2.sum(1);
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
// Check pruning of the decision tree works as expected.
TEST(TableFactor, Prune) {
  DiscreteKey A(1, 2), B(2, 2), C(3, 2);
  TableFactor f(A & B & C, "1 5 3 7 2 6 4 8");

  // Only keep the leaves with the top 5 values.
  size_t maxNrAssignments = 5;
  auto pruned5 = f.prune(maxNrAssignments);

  // Pruned leaves should be 0
  TableFactor expected(A & B & C, "0 5 0 7 0 6 4 8");
  EXPECT(assert_equal(expected, pruned5));

  // Check for more extreme pruning where we only keep the top 2 leaves
  maxNrAssignments = 2;
  auto pruned2 = f.prune(maxNrAssignments);
  TableFactor expected2(A & B & C, "0 0 0 7 0 0 0 8");
  EXPECT(assert_equal(expected2, pruned2));

  DiscreteKey D(4, 2);
  TableFactor factor(
      D & C & B & A,
      "0.0 0.0 0.0 0.60658897 0.61241912 0.61241969 0.61247685 0.61247742 0.0 "
      "0.0 0.0 0.99995287 1.0 1.0 1.0 1.0");

  TableFactor expected3(D & C & B & A,
                        "0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 "
                        "0.999952870000 1.0 1.0 1.0 1.0");
  maxNrAssignments = 5;
  auto pruned3 = factor.prune(maxNrAssignments);
  EXPECT(assert_equal(expected3, pruned3));
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
  TableFactor::Names names{{12, {"Zero", "One", "Two"}}, {5, {"-", "+"}}};
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
  TableFactor::Names names{{12, {"Zero", "One", "Two"}}, {5, {"-", "+"}}};
  string actual = f.html(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
TEST(TableFactor, Unary) {
  // Declare a bunch of keys
  DiscreteKey X(0, 2), Y(1, 3);

  // Create factors
  TableFactor f(X & Y, "2 5 3 6 2 7");
  auto op = [](const double x) { return 2 * x; };
  auto g = f.apply(op);

  TableFactor expected(X & Y, "4 10 6 12 4 14");
  EXPECT(assert_equal(g, expected));

  auto sq_op = [](const double x) { return x * x; };
  auto g_sq = f.apply(sq_op);
  TableFactor expected_sq(X & Y, "4 25 9 36 4 49");
  EXPECT(assert_equal(g_sq, expected_sq));
}

/* ************************************************************************* */
TEST(TableFactor, UnaryAssignment) {
  // Declare a bunch of keys
  DiscreteKey X(0, 2), Y(1, 3);

  // Create factors
  TableFactor f(X & Y, "2 5 3 6 2 7");
  auto op = [](const Assignment<Key>& key, const double x) { return 2 * x; };
  auto g = f.apply(op);

  TableFactor expected(X & Y, "4 10 6 12 4 14");
  EXPECT(assert_equal(g, expected));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
