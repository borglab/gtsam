/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteValues.cpp
 *
 *  @date Jan, 2022
 *  @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/discrete/Signature.h>

#include <boost/assign/std/map.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

static const DiscreteValues kExample{{12, 1}, {5, 0}};

/* ************************************************************************* */
// Check insert
TEST(DiscreteValues, Insert) {
  EXPECT(assert_equal({{12, 1}, {5, 0}, {13, 2}},
                      DiscreteValues(kExample).insert({{13, 2}})));
}

/* ************************************************************************* */
// Check update.
TEST(DiscreteValues, Update) {
  EXPECT(assert_equal({{12, 2}, {5, 0}},
                      DiscreteValues(kExample).update({{12, 2}})));
}

/* ************************************************************************* */
// Check markdown representation with a value formatter.
TEST(DiscreteValues, markdownWithValueFormatter) {
  string expected =
      "|Variable|value|\n"
      "|:-:|:-:|\n"
      "|B|-|\n"
      "|A|One|\n";
  auto keyFormatter = [](Key key) { return key == 12 ? "A" : "B"; };
  DiscreteValues::Names names{{12, {"Zero", "One", "Two"}}, {5, {"-", "+"}}};
  string actual = kExample.markdown(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check html representation with a value formatter.
TEST(DiscreteValues, htmlWithValueFormatter) {
  string expected =
      "<div>\n"
      "<table class='DiscreteValues'>\n"
      "  <thead>\n"
      "    <tr><th>Variable</th><th>value</th></tr>\n"
      "  </thead>\n"
      "  <tbody>\n"
      "    <tr><th>B</th><td>-</td></tr>\n"
      "    <tr><th>A</th><td>One</td></tr>\n"
      "  </tbody>\n"
      "</table>\n"
      "</div>";
  auto keyFormatter = [](Key key) { return key == 12 ? "A" : "B"; };
  DiscreteValues::Names names{{12, {"Zero", "One", "Two"}}, {5, {"-", "+"}}};
  string actual = kExample.html(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
