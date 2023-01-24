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
  SignatureParser::Table expectedTable{{1, 1}, {2, 3}, {1, 4}};
  const auto table = SignatureParser::Parse("1/1 2/3 1/4");
  CHECK(table);
  // compare the tables
  EXPECT(compareTables(*table, expectedTable));
}

/* ************************************************************************* */
// Test case with each row having 3 elements
TEST(SimpleParser, ThreeElements) {
  SignatureParser::Table expectedTable{{1, 1, 1}, {2, 3, 2}, {1, 4, 3}};
  const auto table = SignatureParser::Parse("1/1/1 2/3/2 1/4/3");
  CHECK(table);
  // compare the tables
  EXPECT(compareTables(*table, expectedTable));
}

/* ************************************************************************* */
// A test case to check if we can parse a signature with 'T' and 'F'
TEST(SimpleParser, TAndF) {
  SignatureParser::Table expectedTable{{1, 0}, {1, 0}, {1, 0}, {0, 1}};
  const auto table = SignatureParser::Parse("F F F T");
  CHECK(table);
  // compare the tables
  EXPECT(compareTables(*table, expectedTable));
}

/* ************************************************************************* */
// A test to parse {F F F 1}
TEST(SimpleParser, FFF1) {
  SignatureParser::Table expectedTable{{1, 0}, {1, 0}, {1, 0}};
  const auto table = SignatureParser::Parse("F F F");
  CHECK(table);
  // compare the tables
  EXPECT(compareTables(*table, expectedTable));
}

/* ************************************************************************* */
// Expect false if the string is empty
TEST(SimpleParser, emptyString) {
  const auto table = SignatureParser::Parse("");
  EXPECT(!table);
}

/* ************************************************************************* */
// Expect false if gibberish
TEST(SimpleParser, Gibberish) {
  const auto table = SignatureParser::Parse("sdf 22/3");
  EXPECT(!table);
}

// If Gibberish is in the middle, it should not parse.
TEST(SimpleParser, GibberishInMiddle) {
  const auto table = SignatureParser::Parse("1/1 2/3 sdf 1/4");
  EXPECT(!table);
}

// A test with slash in the end
TEST(SimpleParser, SlashInEnd) {
  const auto table = SignatureParser::parse("1/1 2/");
  EXPECT(!table);
}

/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
