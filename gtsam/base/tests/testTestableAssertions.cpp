/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTestableAssertions
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/deprecated/LieScalar.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtsam;

/* ************************************************************************* */
TEST( testTestableAssertions, optional ) {
  typedef boost::optional<LieScalar> OptionalScalar;
  LieScalar x(1.0);
  OptionalScalar ox(x), dummy = boost::none;
  EXPECT(assert_equal(ox, ox));
  EXPECT(assert_equal(x, ox));
  EXPECT(assert_equal(dummy, dummy));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
