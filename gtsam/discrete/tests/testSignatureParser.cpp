/**
 * Unit tests for the SimpleParser class.
 * @file testSimpleParser.cpp
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/SignatureParser.h>
#include <CppUnitLite/TestHarness.h>

bool compare_tables(const gtsam::SignatureParser::Table& table1,
                    const gtsam::SignatureParser::Table& table2) {
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

// Simple test case
TEST(SimpleParser, simple) {
  gtsam::SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 1}, {2, 3}, {1, 4}};
  bool ret = gtsam::SignatureParser::parse("1/1 2/3 1/4", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compare_tables(table, expectedTable));
}

// Test case with each row having 3 elements
TEST(SimpleParser, three_elements) {
  gtsam::SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 1, 1}, {2, 3, 2}, {1, 4, 3}};
  bool ret = gtsam::SignatureParser::parse("1/1/1 2/3/2 1/4/3", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compare_tables(table, expectedTable));
}

// A test case to check if we can parse a signarue with 'T' and 'F'
TEST(SimpleParser, TandF) {
  gtsam::SignatureParser::Table table, expectedTable;
  expectedTable = {{1,0}, {1,0}, {1,0}, {0,1}};
  bool ret = gtsam::SignatureParser::parse("F F F T", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compare_tables(table, expectedTable));
}

// A test to parse {F F F 1}
TEST(SimpleParser, FFF1) {
  gtsam::SignatureParser::Table table, expectedTable;
  expectedTable = {{1,0}, {1,0}, {1,0}};
  // should ignore the last 1
  bool ret = gtsam::SignatureParser::parse("F F F 1", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compare_tables(table, expectedTable));
}

// Expect false if the string is empty
TEST(SimpleParser, empty_string) {
  gtsam::SignatureParser::Table table;
  bool ret = gtsam::SignatureParser::parse("", table);
  EXPECT(!ret);
}


// Expect false if jibberish
TEST(SimpleParser, jibberish) {
  gtsam::SignatureParser::Table table;
  bool ret = gtsam::SignatureParser::parse("sdf 22/3", table);
  EXPECT(!ret);
}

// If jibberish is in the middle, it should still parse the rest
TEST(SimpleParser, jibberish_in_middle) {
  gtsam::SignatureParser::Table table, expectedTable;
  expectedTable = {{1, 1}, {2, 3}};
  bool ret = gtsam::SignatureParser::parse("1/1 2/3 sdf 1/4", table);
  EXPECT(ret);
  // compare the tables
  EXPECT(compare_tables(table, expectedTable));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
