/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testKey.cpp
 * @author Alex Cunningham
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/Symbol.h>

using namespace std;
using namespace gtsam;

Key aKey = gtsam::symbol_shorthand::X(4);

/* ************************************************************************* */
TEST(Key, KeySymbolConversion) {
  Symbol original('j', 4);
  Key key(original);
  EXPECT(assert_equal(key, original.key()))
  Symbol actual(key);
  EXPECT(assert_equal(original, actual))
}

/* ************************************************************************* */
TEST(Key, KeySymbolEncoding) {

  // Test encoding of Symbol <-> size_t <-> string

  if(sizeof(Key) == 8) {
    Symbol symbol(0x61, 5);
    Key key = 0x6100000000000005;
    string str = "a5";

    EXPECT_LONGS_EQUAL(key, (Key)symbol);
    EXPECT(assert_equal(str, DefaultKeyFormatter(symbol)));
    EXPECT(assert_equal(symbol, Symbol(key)));
  } else if(sizeof(Key) == 4) {
    Symbol symbol(0x61, 5);
    Key key = 0x61000005;
    string str = "a5";

    EXPECT_LONGS_EQUAL(key, (Key)symbol);
    EXPECT(assert_equal(str, DefaultKeyFormatter(symbol)));
    EXPECT(assert_equal(symbol, Symbol(key)));
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

