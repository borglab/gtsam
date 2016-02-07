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

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;
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
template<int KeySize>
Key KeyTestValue();

template<>
Key KeyTestValue<8>()
{
  return 0x6100000000000005;
};

template<>
Key KeyTestValue<4>()
{
  return 0x61000005;
};

/* ************************************************************************* */
TEST(Key, KeySymbolEncoding) {

  // Test encoding of Symbol <-> size_t <-> string
  Symbol symbol(0x61, 5);
  Key key = KeyTestValue<sizeof(Key)>();
  string str = "a5";

  EXPECT_LONGS_EQUAL((long)key, (long)(Key)symbol);
  EXPECT(assert_equal(str, DefaultKeyFormatter(symbol)));
  EXPECT(assert_equal(symbol, Symbol(key)));
}

/* ************************************************************************* */
TEST(Key, ChrTest) {
  Key key = Symbol('c',3);
  EXPECT(Symbol::ChrTest('c')(key));
  EXPECT(!Symbol::ChrTest('d')(key));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

