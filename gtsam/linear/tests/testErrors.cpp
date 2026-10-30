/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testErrors.cpp
 * @date Feb 20, 2010
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/linear/Errors.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(Errors, arithmetic) {
  Errors e = {Vector2(1.0, 2.0), Vector3(3.0, 4.0, 5.0)};
  DOUBLES_EQUAL(1 + 4 + 9 + 16 + 25, dot(e, e), 1e-9);

  axpy(2.0, e, e);
  const Errors expected = {Vector2(3.0, 6.0), Vector3(9.0, 12.0, 15.0)};
  CHECK(assert_equal(expected, e));
}

/* ************************************************************************* */
namespace dot_product {

// Empty factor slots contribute zero without shifting the remaining blocks.
TEST(Errors, DotWithEmptyBlocks) {
  const Errors a{Vector2{1., 2.}, Vector{}, Vector1{3.}};
  const Errors b{Vector2{4., 5.}, Vector{}, Vector1{6.}};
  DOUBLES_EQUAL(32., dot(a, b), 1e-9);
  DOUBLES_EQUAL(0., dot(Errors{}, Errors{}), 0.);
}

// Unequal block counts throw in either argument order, including empty lists.
TEST(Errors, DotRejectsBlockCounts) {
  const Errors a{Vector1{1.}}, b{Vector1{1.}, Vector{}};
  CHECK_EXCEPTION(dot(a, Errors{}), std::invalid_argument);
  CHECK_EXCEPTION(dot(Errors{}, a), std::invalid_argument);
  CHECK_EXCEPTION(dot(a, b), std::invalid_argument);
  CHECK_EXCEPTION(dot(b, a), std::invalid_argument);
}

// Matching block counts still require equal dimensions in every vector pair.
TEST(Errors, DotRejectsBlockDimensions) {
  const Errors a{Vector1{1.}, Vector2{2., 3.}}, b{Vector1{4.}, Vector1{5.}};
  CHECK_EXCEPTION(dot(a, b), std::invalid_argument);
  CHECK_EXCEPTION(dot(b, a), std::invalid_argument);
  const Errors emptyBlock{Vector{}}, nonemptyBlock{Vector1{1.}};
  CHECK_EXCEPTION(dot(emptyBlock, nonemptyBlock), std::invalid_argument);
  CHECK_EXCEPTION(dot(nonemptyBlock, emptyBlock), std::invalid_argument);
}

}  // namespace dot_product
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
