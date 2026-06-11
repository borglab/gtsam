/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testQpsParser.cpp
 * @brief   Unit tests for constrained-module QPS parsing.
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QpsParser.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include <sstream>
#include <stdexcept>
#include <string>

using namespace gtsam;

/* ************************************************************************* */
namespace qps_parser_fixture {

const Key x1 = Symbol('X', 1);
const Key x2 = Symbol('X', 2);

Matrix Matrix1(double value) { return (Matrix(1, 1) << value).finished(); }

Vector Vector1D(double value) { return (Vector(1) << value).finished(); }

QpsParserResult ParseExample(const std::string& name) {
  return QpsParser::parseFile(findExampleDataFile(name));
}

const QpCost& OnlyCost(const QpProblem& problem) {
  const auto* cost = dynamic_cast<const QpCost*>(problem.costs().at(0).get());
  if (!cost) {
    throw std::runtime_error("expected QpCost");
  }
  return *cost;
}

// Verifies QPExample metadata, objective, and constraint counts.
TEST(QpsParser, QPExampleStructure) {
  const QpsParserResult result = ParseExample("QPExample.QPS");

  CHECK(result.name == "QP example");
  CHECK(result.objectiveName == "obj");
  CHECK_EQUAL(2, result.variableKeys.size());
  CHECK_EQUAL(x1, result.variableKeys.at("c1"));
  CHECK_EQUAL(x2, result.variableKeys.at("c2"));
  CHECK_EQUAL(1, result.problem.costs().size());
  CHECK_EQUAL(0, result.problem.eConstraints().size());
  CHECK_EQUAL(5, result.problem.iConstraints().size());

  const HessianFactor expected(x1, x2, 8.0 * Matrix1(1.0), 2.0 * Matrix1(1.0),
                               -1.5 * Vector1D(1.0), 10.0 * Matrix1(1.0),
                               2.0 * Vector1D(1.0), 8.0);
  CHECK(assert_equal(expected, OnlyCost(result.problem).hessianFactor(), 1e-7));
}

// Verifies parse errors include the supplied source name and line number.
TEST(QpsParser, MalformedReportsSourceAndLine) {
  std::istringstream input(
      "NAME bad\n"
      "ROWS\n"
      " N obj\n"
      "COLUMNS\n"
      " x obj not_a_number\n"
      "ENDATA\n");

  try {
    QpsParser::parse(input, "bad.qps");
    CHECK(false);
  } catch (const QpsParserException& exception) {
    const std::string message = exception.what();
    CHECK(message.find("bad.qps:5") != std::string::npos);
  }
}

}  // namespace qps_parser_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
