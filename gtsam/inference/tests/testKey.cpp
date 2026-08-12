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

#include <sstream>

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
TEST(Key, SymbolGenerator) {
  const auto x1 = gtsam::symbol_shorthand::X(1);
  const auto v1 = gtsam::symbol_shorthand::V(1);
  const auto a1 = gtsam::symbol_shorthand::A(1);

  const auto Z = gtsam::SymbolGenerator('x');
  const auto DZ = gtsam::SymbolGenerator('v');
  const auto DDZ = gtsam::SymbolGenerator('a');

  const auto z1 = Z(1);
  const auto dz1 = DZ(1);
  const auto ddz1 = DDZ(1);

  EXPECT(assert_equal(x1, z1));
  EXPECT(assert_equal(v1, dz1));
  EXPECT(assert_equal(a1, ddz1));
}

/* ************************************************************************* */
TEST(Key, SymbolGeneratorConstexpr) {
  constexpr auto Z = gtsam::SymbolGenerator('x');
  EXPECT(assert_equal(Z.chr(), 'x'));
}

/* ************************************************************************* */
template<int KeySize>
Key KeyTestValue();

template <>
Key KeyTestValue<8>() {
  return 0x6100000000000005;
}

template <>
Key KeyTestValue<4>() {
  return 0x61000005;
}

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
  Symbol key('c', 3);
  EXPECT(Symbol::ChrTest('c')(key));
  EXPECT(!Symbol::ChrTest('d')(key));
}

/* ************************************************************************* */
// A custom (nonsensical) formatter.
string keyMyFormatter(Key key) {
  return "special";
}

TEST(Key, Formatting) {
  Symbol key('c', 3);
  EXPECT("c3" == DefaultKeyFormatter(key));

  // Try streaming keys, should be default-formatted.
  stringstream ss;
  ss << StreamedKey(key);
  EXPECT("c3" == ss.str());

  // use key_formatter with a function pointer
  stringstream ss2;
  ss2 << key_formatter(keyMyFormatter) << StreamedKey(key);
  EXPECT("special" == ss2.str());

  // use key_formatter with a function object.
  stringstream ss3;
  ss3 << key_formatter(DefaultKeyFormatter) << StreamedKey(key);
  EXPECT("c3" == ss3.str());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
