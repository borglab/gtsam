/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testLabeledSymbol.cpp
 * @author Alex Cunningham
 */

#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/list.hpp> // for operator +=

using namespace boost::assign;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(LabeledSymbol, KeyLabeledSymbolConversion ) {
  LabeledSymbol expected('x', 'A', 4);
  Key key(expected);
  LabeledSymbol actual(key);

  EXPECT(assert_equal(expected, actual))
}

/* ************************************************************************* */
TEST(LabeledSymbol, KeyLabeledSymbolEncoding ) {

  // Test encoding of LabeledSymbol <-> size_t <-> string
  // Encoding scheme:
  // Top 8 bits: variable type (255 possible values) - zero will not process
  // Next 8 high bits: variable type (255 possible values)
  // TODO: use fewer bits for type - only 4 or 5 should be necessary - will need more decoding

  if(sizeof(Key) == 8) {
    LabeledSymbol symbol(0x78, 0x41, 5);
    Key key = 0x7841000000000005;
    string str = "xA5";

    EXPECT_LONGS_EQUAL((long)key, (long)(Key)symbol);
    EXPECT_LONGS_EQUAL(5, symbol.index());
    EXPECT_LONGS_EQUAL(0x78, symbol.chr());
    EXPECT_LONGS_EQUAL(0x41, symbol.label());
    EXPECT(assert_equal(str, MultiRobotKeyFormatter(symbol)));
    EXPECT(assert_equal(symbol, LabeledSymbol(key)));
  } else if(sizeof(Key) == 4) {
    LabeledSymbol symbol(0x78, 0x41, 5);
    Key key = 0x78410005;
    string str = "xA5";

    EXPECT_LONGS_EQUAL((long)key, (long)(Key) symbol);
    EXPECT_LONGS_EQUAL(5, symbol.index());
    EXPECT_LONGS_EQUAL(0x78, symbol.chr());
    EXPECT_LONGS_EQUAL(0x41, symbol.label());
    EXPECT(assert_equal(str, MultiRobotKeyFormatter(symbol)));
    EXPECT(assert_equal(symbol, LabeledSymbol(key)));
  }
}

/* ************************************************************************* */
TEST(LabeledSymbol, ChrTest) {
  Key key = LabeledSymbol('c','A',3);
  EXPECT(LabeledSymbol::TypeTest('c')(key));
  EXPECT(!LabeledSymbol::TypeTest('d')(key));
  EXPECT(LabeledSymbol::LabelTest('A')(key));
  EXPECT(!LabeledSymbol::LabelTest('D')(key));
  EXPECT(LabeledSymbol::TypeLabelTest('c','A')(key));
  EXPECT(!LabeledSymbol::TypeLabelTest('c','D')(key));
}

/* ************************************************************************* */
// A custom (nonsensical) formatter.
string labeledSymbolMyFormatter(Key key) {
  return "special";
}

TEST(LabeledSymbol, Formatting) {
  LabeledSymbol symbol('c', 'A', 3);

  // use key_formatter with a function pointer
  stringstream ss2;
  ss2 << key_formatter(labeledSymbolMyFormatter) << symbol;
  EXPECT("special" == ss2.str());

  // use key_formatter with a function object.
  stringstream ss3;
  ss3 << key_formatter(MultiRobotKeyFormatter) << symbol;
  EXPECT("cA3" == ss3.str());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

