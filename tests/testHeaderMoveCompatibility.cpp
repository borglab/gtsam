/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testHeaderMoveCompatibility.cpp
 * @brief Test the deprecated GTSAM 4.3 include paths for moved functionality.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/kruskal.h>
#include <gtsam/base/kruskal-inl.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/EssentialMatrixConstraint.h>
#include <gtsam/slam/EssentialMatrixFactor.h>
#include <gtsam/slam/AntiFactor.h>
#include <gtsam/slam/BoundingConstraint.h>

#include <type_traits>
#include <vector>

using namespace gtsam;

// Verifies that the deprecated headers expose the canonical declarations.
TEST(HeaderMoveCompatibility, DeprecatedHeaders) {
  static_assert(std::is_base_of_v<NonlinearFactor, AntiFactor>);
  static_assert(std::is_base_of_v<NonlinearFactor,
                                  EssentialMatrixConstraint>);
  static_assert(std::is_base_of_v<NonlinearFactor, EssentialMatrixFactor>);
  static_assert(std::is_base_of_v<NonlinearEqualityConstraint,
                                  NonlinearEquality<Point2>>);
  static_assert(std::is_base_of_v<NonlinearInequalityConstraint,
                                  BoundingConstraint1<Point2>>);

  const std::vector<size_t> order = utils::sortedIndices({2.0, 1.0});
  LONGS_EQUAL(1, order.front());
  LONGS_EQUAL(0, order.back());
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
