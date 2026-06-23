/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBatchJacobianFactor.cpp
 * @brief Unit tests for compact fixed-dimension batch Jacobian factors.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/BatchJacobianFactor.h>

#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace update_hessian_fixture {

using Factor = BatchJacobianFactor<2, 2, 3>;

const KeyVector keys{0, 1, 2};
const std::vector<size_t> keyDimensions{2, 2, 3};
const std::vector<size_t> augmentedDimensions{2, 2, 3, 1};

Factor createWeightedFactor() {
  auto model = noiseModel::Diagonal::Sigmas(
      (Vector(4) << 2.0, 3.0, 4.0, 5.0).finished());
  Factor factor(keys, keyDimensions, model);

  Matrix A0 = (Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished();
  Matrix A1 = (Matrix(2, 2) << -1.0, 0.5, 2.0, -0.25).finished();
  Matrix B0 = (Matrix(2, 3) << 0.5, 1.0, -1.5, 2.0, -0.5, 0.25).finished();
  Matrix B1 = (Matrix(2, 3) << 1.5, -2.0, 0.75, -1.0, 0.25, 2.5).finished();
  Vector b0 = (Vector(2) << 0.25, -0.75).finished();
  Vector b1 = (Vector(2) << 1.25, 0.5).finished();

  factor.addRow({0, 2}, {A0, B0}, b0);
  factor.addRow({1, 2}, {A1, B1}, b1);
  return factor;
}

Matrix assembledHessian(const SymmetricBlockMatrix& info) {
  return Matrix(info.selfadjointView());
}

// Verifies direct compact Hessian assembly matches dense compatibility assembly.
TEST(BatchJacobianFactor, UpdateHessian) {
  const Factor factor = createWeightedFactor();

  SymmetricBlockMatrix actual(augmentedDimensions), expected(augmentedDimensions);
  actual.setZero();
  expected.setZero();

  factor.updateHessian(keys, &actual);
  factor.toJacobianFactor().updateHessian(keys, &expected);

  EXPECT(assert_equal(assembledHessian(expected), assembledHessian(actual),
                      1e-12));
}

// Verifies partial-column Hessian assembly matches dense compatibility assembly.
TEST(BatchJacobianFactor, UpdateHessianColumnRanges) {
  const Factor factor = createWeightedFactor();

  SymmetricBlockMatrix actual(augmentedDimensions), expected(augmentedDimensions);
  actual.setZero();
  expected.setZero();

  for (DenseIndex column = 0; column < actual.nBlocks(); ++column) {
    factor.updateHessian(keys, &actual, column, column + 1);
    factor.toJacobianFactor().updateHessian(keys, &expected, column,
                                            column + 1);
  }

  EXPECT(assert_equal(assembledHessian(expected), assembledHessian(actual),
                      1e-12));
}

}  // namespace update_hessian_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

/* ************************************************************************* */
