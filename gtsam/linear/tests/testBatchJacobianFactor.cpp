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
 * @author Fan Jiang
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
  auto model = noiseModel::Diagonal::Sigmas(Vector{{2.0, 3.0, 4.0, 5.0}});
  Factor factor(keys, keyDimensions, model);

  Matrix A0{{1.0, 2.0}, {3.0, 4.0}};
  Matrix A1{{-1.0, 0.5}, {2.0, -0.25}};
  Matrix B0{{0.5, 1.0, -1.5}, {2.0, -0.5, 0.25}};
  Matrix B1{{1.5, -2.0, 0.75}, {-1.0, 0.25, 2.5}};
  Vector b0{{0.25, -0.75}};
  Vector b1{{1.25, 0.5}};

  factor.addRow({0, 2}, {A0, B0}, b0);
  factor.addRow({1, 2}, {A1, B1}, b1);
  return factor;
}

Factor createUnweightedFactor() {
  Factor factor(keys, keyDimensions);

  Matrix A0{{1.0, 2.0}, {3.0, 4.0}};
  Matrix A1{{-1.0, 0.5}, {2.0, -0.25}};
  Matrix B0{{0.5, 1.0, -1.5}, {2.0, -0.5, 0.25}};
  Matrix B1{{1.5, -2.0, 0.75}, {-1.0, 0.25, 2.5}};
  Vector b0{{0.25, -0.75}};
  Vector b1{{1.25, 0.5}};

  factor.addRow({0, 2}, {A0, B0}, b0);
  factor.addRow({1, 2}, {A1, B1}, b1);
  return factor;
}

Matrix assembledHessian(const SymmetricBlockMatrix& info) {
  return Matrix(info.selfadjointView());
}

// Verifies direct compact Hessian assembly matches dense compatibility
// assembly.
TEST(BatchJacobianFactor, UpdateHessian) {
  const Factor factor = createWeightedFactor();

  SymmetricBlockMatrix actual(augmentedDimensions),
      expected(augmentedDimensions);
  actual.setZero();
  expected.setZero();

  factor.updateHessian(keys, &actual);
  factor.toJacobianFactor().updateHessian(keys, &expected);

  EXPECT(assert_equal(assembledHessian(expected), assembledHessian(actual),
                      1e-12));
}

// Verifies unweighted compact Hessian assembly matches dense compatibility
// assembly, including the scalar RHS diagonal block.
TEST(BatchJacobianFactor, UpdateHessianUnweighted) {
  const Factor factor = createUnweightedFactor();

  SymmetricBlockMatrix actual(augmentedDimensions),
      expected(augmentedDimensions);
  actual.setZero();
  expected.setZero();

  factor.updateHessian(keys, &actual);
  factor.toJacobianFactor().updateHessian(keys, &expected);

  EXPECT(assert_equal(assembledHessian(expected), assembledHessian(actual),
                      1e-12));
}

// Verifies partial-column Hessian assembly matches dense compatibility
// assembly.
TEST(BatchJacobianFactor, UpdateHessianColumnRanges) {
  const Factor factor = createWeightedFactor();

  SymmetricBlockMatrix actual(augmentedDimensions),
      expected(augmentedDimensions);
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

// Verifies mapped-slot buffers are sized per row-group, not per scalar row.
TEST(BatchJacobianFactor, BuildMappedSlotsUsesRowGroupStride) {
  const Factor factor = createWeightedFactor();

  std::vector<DenseIndex> slotIndices = {0, 1, 2, 3};
  std::vector<DenseIndex> mappedSlots;
  factor.buildMappedSlots(slotIndices, mappedSlots);

  const size_t rowGroupCount = factor.rows() / 2;
  const size_t expectedSize = rowGroupCount * (2 + 1);
  EXPECT_LONGS_EQUAL((long)expectedSize, (long)mappedSlots.size());

  // First row maps slots for keys {0,2} and rhs.
  EXPECT_LONGS_EQUAL(slotIndices[0], mappedSlots[0]);
  EXPECT_LONGS_EQUAL(slotIndices[2], mappedSlots[1]);
  EXPECT_LONGS_EQUAL(slotIndices[3], mappedSlots[2]);

  // Second row maps slots for keys {1,2} and rhs.
  EXPECT_LONGS_EQUAL(slotIndices[1], mappedSlots[3]);
  EXPECT_LONGS_EQUAL(slotIndices[2], mappedSlots[4]);
  EXPECT_LONGS_EQUAL(slotIndices[3], mappedSlots[5]);
}

// Verifies the compact mapped-slot API updates identically to the dense path.
TEST(BatchJacobianFactor, UpdateHessianWithMappedSlotsMatchesToJacobian) {
  const Factor factor = createWeightedFactor();

  SymmetricBlockMatrix mapped(augmentedDimensions),
      expected(augmentedDimensions);
  mapped.setZero();
  expected.setZero();

  std::vector<DenseIndex> slotIndices = {0, 1, 2, 3};
  std::vector<DenseIndex> mappedSlots;
  factor.buildMappedSlots(slotIndices, mappedSlots);
  factor.updateHessianWithMappedSlots(mappedSlots, &mapped);

  factor.toJacobianFactor().updateHessian(keys, &expected);

  EXPECT(assert_equal(assembledHessian(expected), assembledHessian(mapped),
                      1e-12));
}

// Verifies slot permutation from factor keys to info blocks preserves assembly
// correctness for mapped and direct full-range paths.
TEST(BatchJacobianFactor, UpdateHessianWithPermutedSlotOrder) {
  const Factor factor = createWeightedFactor();

  const KeyVector permutedInfoKeys{1, 2, 0};
  const std::vector<size_t> permutedAugmentedDimensions{2, 3, 2, 1};
  std::vector<DenseIndex> slotIndices = {2, 0, 1, 3};

  SymmetricBlockMatrix permutedDirect(permutedAugmentedDimensions),
      permutedMapped(permutedAugmentedDimensions),
      expected(permutedAugmentedDimensions);
  permutedDirect.setZero();
  permutedMapped.setZero();
  expected.setZero();

  factor.updateHessian(slotIndices, &permutedDirect);

  std::vector<DenseIndex> mappedSlots;
  factor.buildMappedSlots(slotIndices, mappedSlots);
  factor.updateHessianWithMappedSlots(mappedSlots, &permutedMapped);

  factor.toJacobianFactor().updateHessian(permutedInfoKeys, &expected);

  EXPECT(assert_equal(assembledHessian(expected),
                      assembledHessian(permutedDirect), 1e-12));
  EXPECT(assert_equal(assembledHessian(expected),
                      assembledHessian(permutedMapped), 1e-12));
}

// Verifies mapped-slot buffers handle fixed-key slots as -1 and still match the
// safe dense-mapping path.
TEST(BatchJacobianFactor, UpdateHessianWithMappedSlotsSkipsNegativeSlots) {
  const Factor factor = createWeightedFactor();

  std::vector<DenseIndex> slotIndicesWithRhs = {-1, 1, 2, 3};
  std::vector<DenseIndex> mappedSlots;
  factor.buildMappedSlots(slotIndicesWithRhs, mappedSlots);

  EXPECT_LONGS_EQUAL(-1, mappedSlots[0]);
  EXPECT_LONGS_EQUAL(1, mappedSlots[3]);

  SymmetricBlockMatrix mapped(augmentedDimensions),
      expected(augmentedDimensions);
  mapped.setZero();
  expected.setZero();

  factor.updateHessianWithMappedSlots(mappedSlots, &mapped);
  factor.updateHessian(slotIndicesWithRhs, &expected);

  EXPECT(assert_equal(assembledHessian(expected), assembledHessian(mapped),
                      1e-12));
}

// Duplicate mapped slots violate the required slot mapping contract and are
// treated as an invariant violation in debug assertions.
TEST(BatchJacobianFactor, DuplicateMappedSlotsAreInvalid) {
  const Factor factor = createWeightedFactor();

  std::vector<DenseIndex> slotIndicesWithDuplicate = {0, 1, 0, 3};
  std::vector<DenseIndex> mappedSlots;
  factor.buildMappedSlots(slotIndicesWithDuplicate, mappedSlots);

  EXPECT_LONGS_EQUAL(0, mappedSlots[0]);
  EXPECT_LONGS_EQUAL(0, mappedSlots[1]);
  // Debug builds assert if this duplicate mapping reaches the mapped-slot
  // update path; we do not have a dedicated death-test helper in this suite.
}

// Verifies the flat Hessian-vector kernel matches the dense compatibility path.
TEST(BatchJacobianFactor, FlatMultiplyHessianAdd) {
  const Factor factor = createWeightedFactor();
  const std::vector<size_t> offsets{0, 2, 4};
  const Vector x =
      (Vector(7) << 0.2, -0.4, 0.7, 0.1, -0.3, 0.6, 0.9).finished();
  Vector actual = Vector::Zero(7);
  factor.multiplyHessianAdd(1.0, offsets, x.data(), actual.data());

  VectorValues valuesX;
  valuesX.emplace(0, x.segment<2>(0));
  valuesX.emplace(1, x.segment<2>(2));
  valuesX.emplace(2, x.segment<3>(4));
  VectorValues valuesY;
  valuesY.emplace(0, Vector2::Zero());
  valuesY.emplace(1, Vector2::Zero());
  valuesY.emplace(2, Vector3::Zero());
  factor.toJacobianFactor().multiplyHessianAdd(1.0, valuesX, valuesY);
  const Vector expected = valuesY.vector(keys);

  EXPECT(assert_equal(expected, actual, 1e-12));
}

// Verifies the flat zero-point gradient matches the dense compatibility path.
TEST(BatchJacobianFactor, FlatGradientAtZero) {
  const Factor factor = createWeightedFactor();
  const std::vector<size_t> offsets{0, 2, 4};
  Vector actual = Vector::Zero(7);
  factor.gradientAtZeroAdd(offsets, actual.data());
  const Vector expected =
      factor.toJacobianFactor().gradientAtZero().vector(keys);

  EXPECT(assert_equal(expected, actual, 1e-12));
}

// Verifies direct ordered block-diagonal accumulation avoids keyed maps without
// changing numerical results.
TEST(BatchJacobianFactor, FlatHessianBlockDiagonal) {
  const Factor factor = createWeightedFactor();
  std::vector<Matrix> actual{Matrix::Zero(2, 2), Matrix::Zero(2, 2),
                             Matrix::Zero(3, 3)};
  factor.hessianBlockDiagonalAdd({0, 1, 2}, &actual);
  const std::map<Key, Matrix> expected =
      factor.toJacobianFactor().hessianBlockDiagonal();

  for (size_t slot = 0; slot < actual.size(); ++slot) {
    EXPECT(assert_equal(expected.at(slot), actual[slot], 1e-12));
  }
}

}  // namespace update_hessian_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

/* ************************************************************************* */
