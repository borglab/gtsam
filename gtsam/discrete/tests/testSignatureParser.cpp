/**
 * Unit tests for the SimpleParser class.
 * @file testSimpleParser.cpp
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/SignatureParser.h>

using namespace gtsam;

/* ************************************************************************* */
// Simple test case
bool compareTables(const SignatureParser::Table& table1,
                   const SignatureParser::Table& table2) {
  if (table1.size() != table2.size()) {
    return false;
  }
  for (size_t i = 0; i < table1.size(); ++i) {
    if (table1[i].size() != table2[i].size()) {
      return false;
    }
    for (size_t j = 0; j < table1[i].size(); ++j) {
      if (table1[i][j] != table2[i][j]) {
        return false;
      }
    }
  }
  return true;
}

/* ************************************************************************* */
// Simple test case
TEST(SimpleParser, Simple) {
  SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 1}, {2, 3}, {1, 4}};
  bool ret = SignatureParser::parse("1/1 2/3 1/4", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compareTables(table, expectedTable));
}

/* ************************************************************************* */
// Test case with each row having 3 elements
TEST(SimpleParser, ThreeElements) {
  SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 1, 1}, {2, 3, 2}, {1, 4, 3}};
  bool ret = SignatureParser::parse("1/1/1 2/3/2 1/4/3", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compareTables(table, expectedTable));
}

/* ************************************************************************* */
// A test case to check if we can parse a signature with 'T' and 'F'
TEST(SimpleParser, TAndF) {
  SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 0}, {1, 0}, {1, 0}, {0, 1}};
  bool ret = SignatureParser::parse("F F F T", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compareTables(table, expectedTable));
}

/* ************************************************************************* */
// A test to parse {F F F 1}
TEST(SimpleParser, FFF1) {
  SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 0}, {1, 0}, {1, 0}};
  // should ignore the last 1
  bool ret = SignatureParser::parse("F F F 1", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compareTables(table, expectedTable));
}

/* ************************************************************************* */
// Expect false if the string is empty
TEST(SimpleParser, emptyString) {
  SignatureParser::Table table;
  bool ret = SignatureParser::parse("", table);
  EXPECT(!ret);
}

/* ************************************************************************* */
// Expect false if gibberish
TEST(SimpleParser, Gibberish) {
  SignatureParser::Table table;
  bool ret = SignatureParser::parse("sdf 22/3", table);
  EXPECT(!ret);
}

// If Gibberish is in the middle, it should still parse the rest
TEST(SimpleParser, GibberishInMiddle) {
  SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 1}, {2, 3}};
  bool ret = SignatureParser::parse("1/1 2/3 sdf 1/4", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compareTables(table, expectedTable));
}

/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
