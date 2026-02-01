/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testSymbol.cpp
 * @author Frank Dellaert
 */

#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

#include <sstream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// A custom (nonsensical) formatter.
string symbolMyFormatter(Key key) {
  return "special";
}

TEST(Symbol, Formatting) {
  Symbol symbol('c', 3);

  // use key_formatter with a function pointer
  stringstream ss2;
  ss2 << key_formatter(symbolMyFormatter) << symbol;
  EXPECT("special" == ss2.str());

  // use key_formatter with a function object.
  stringstream ss3;
  ss3 << key_formatter(MultiRobotKeyFormatter) << symbol;
  EXPECT("c3" == ss3.str());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

