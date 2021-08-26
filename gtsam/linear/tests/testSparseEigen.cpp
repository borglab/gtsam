/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSparseMatrix.cpp
 * @author  Mandy Xie
 * @author  Fan Jiang
 * @author  Gerry Chen
 * @author  Frank Dellaert
 * @date    Jan, 2021
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SparseEigen.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(SparseEigen, sparseJacobianEigen) {
  GaussianFactorGraph gfg;
  SharedDiagonal model = noiseModel::Isotropic::Sigma(2, 0.5);
  const Key x123 = 0, x45 = 1;
  gfg.add(x123, (Matrix(2, 3) << 1, 2, 3, 5, 6, 7).finished(),
          Vector2(4, 8), model);
  gfg.add(x123, (Matrix(2, 3) << 9, 10, 0, 0, 0, 0).finished(),
          x45,  (Matrix(2, 2) << 11, 12, 14, 15.).finished(),
          Vector2(13, 16), model);

  // Sparse Matrix
  auto sparseResult = sparseJacobianEigen(gfg);
  EXPECT_LONGS_EQUAL(16, sparseResult.nonZeros());
  EXPECT(assert_equal(4, sparseResult.rows()));
  EXPECT(assert_equal(6, sparseResult.cols()));
  EXPECT(assert_equal(gfg.augmentedJacobian(), Matrix(sparseResult)));

  // Call sparseJacobian with optional ordering...
  auto ordering = Ordering(list_of(x45)(x123));

  // Eigen Sparse with optional ordering
  EXPECT(assert_equal(gfg.augmentedJacobian(ordering),
                      Matrix(sparseJacobianEigen(gfg, ordering))));

  // Check matrix dimensions when zero rows / cols
  gfg.add(x123, Matrix23::Zero(), Vector2::Zero(), model);  // zero row
  gfg.add(2, Matrix21::Zero(), Vector2::Zero(), model);     // zero col
  sparseResult = sparseJacobianEigen(gfg);
  EXPECT_LONGS_EQUAL(16, sparseResult.nonZeros());
  EXPECT(assert_equal(8, sparseResult.rows()));
  EXPECT(assert_equal(7, sparseResult.cols()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
