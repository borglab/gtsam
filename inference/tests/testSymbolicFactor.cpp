/**
 * @file    testSymbolicFactor.cpp
 * @brief   Unit tests for a symbolic Factor
 * @author  Frank Dellaert
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/Conditional.h>

#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST(SymbolicFactor, eliminate) {
  vector<varid_t> keys; keys += 2, 3, 4, 6, 7, 9, 10, 11;
  Factor actual(keys.begin(), keys.end());
  BayesNet<Conditional> fragment = *actual.eliminate(3);

  Factor expected(keys.begin()+3, keys.end());
  Conditional expected0(keys.begin(), keys.end(), 1);
  Conditional expected1(keys.begin()+1, keys.end(), 1);
  Conditional expected2(keys.begin()+2, keys.end(), 1);

  CHECK(assert_equal(fragment.size(), size_t(3)));
  CHECK(assert_equal(expected, actual));
  BayesNet<Conditional>::const_iterator fragmentCond = fragment.begin();
  CHECK(assert_equal(**fragmentCond++, expected0));
  CHECK(assert_equal(**fragmentCond++, expected1));
  CHECK(assert_equal(**fragmentCond++, expected2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
