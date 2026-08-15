/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBinaryJacobianFactor.cpp
 * @brief Tests for fixed-size BinaryJacobianFactor operations.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace binary_jacobian_factor {

using Factor = BinaryJacobianFactor<2, 3, 2>;

constexpr Key kKey1 = 10;
constexpr Key kKey2 = 20;
const Matrix23 kA1{{1.0, 2.0, -1.0}, {0.5, -3.0, 4.0}};
const Matrix2 kA2{{2.0, -0.5}, {1.5, 3.0}};
const Vector2 kB{0.25, -2.0};

std::vector<SharedDiagonal> noiseModels() {
  return {SharedDiagonal(), noiseModel::Unit::Create(2),
          noiseModel::Isotropic::Sigma(2, 0.5),
          noiseModel::Diagonal::Sigmas(Vector2{0.5, 2.0}),
          noiseModel::Constrained::MixedSigmas(Vector2{0.0, 2.0})};
}

VectorValues values() {
  VectorValues result;
  result.insert(kKey1, Vector3{0.5, -1.0, 2.0});
  result.insert(kKey2, Vector2{-0.25, 1.5});
  return result;
}

// Verifies fixed-size delta error matches the generic implementation.
TEST(BinaryJacobianFactor, DeltaErrorMatchesGeneric) {
  const VectorValues testValues = values();
  for (const SharedDiagonal& model : noiseModels()) {
    const Factor actual(kKey1, kA1, kKey2, kA2, kB, model);
    const JacobianFactor expected(kKey1, kA1, kKey2, kA2, kB, model);
    const GaussianFactor& actualBase = actual;

    double expectedOld = 0.0, expectedNew = 0.0;
    double actualOld = 0.0, actualNew = 0.0;
    const double expectedDelta =
        expected.deltaError(testValues, &expectedOld, &expectedNew);
    const double actualDelta =
        actualBase.deltaError(testValues, &actualOld, &actualNew);

    DOUBLES_EQUAL(expectedOld, actualOld, 1e-12);
    DOUBLES_EQUAL(expectedNew, actualNew, 1e-12);
    DOUBLES_EQUAL(expectedDelta, actualDelta, 1e-12);
    DOUBLES_EQUAL(expected.deltaError(testValues),
                  actualBase.deltaError(testValues), 1e-12);
  }
}

// Verifies fixed-size diagonal insertion matches the generic implementation.
TEST(BinaryJacobianFactor, HessianDiagonalMatchesGeneric) {
  for (const SharedDiagonal& model : noiseModels()) {
    const Factor actual(kKey1, kA1, kKey2, kA2, kB, model);
    const JacobianFactor expected(kKey1, kA1, kKey2, kA2, kB, model);
    const GaussianFactor& actualBase = actual;

    VectorValues expectedDiagonal, actualDiagonal;
    expected.hessianDiagonalAdd(expectedDiagonal);
    actualBase.hessianDiagonalAdd(actualDiagonal);
    EXPECT(assert_equal(expectedDiagonal, actualDiagonal, 1e-12));
  }
}

// Verifies fixed-size diagonal updates accumulate into existing entries.
TEST(BinaryJacobianFactor, HessianDiagonalAccumulates) {
  for (const SharedDiagonal& model : noiseModels()) {
    const Factor actual(kKey1, kA1, kKey2, kA2, kB, model);
    const JacobianFactor expected(kKey1, kA1, kKey2, kA2, kB, model);
    const GaussianFactor& actualBase = actual;

    VectorValues expectedDiagonal, actualDiagonal;
    for (VectorValues* diagonal : {&expectedDiagonal, &actualDiagonal}) {
      diagonal->insert(kKey1, Vector3::Constant(2.0));
      diagonal->insert(kKey2, Vector2::Constant(-1.0));
      diagonal->insert(30, Vector1{7.0});
    }
    expected.hessianDiagonalAdd(expectedDiagonal);
    actualBase.hessianDiagonalAdd(actualDiagonal);
    EXPECT(assert_equal(expectedDiagonal, actualDiagonal, 1e-12));
  }
}

// Verifies ranged updates reproduce whole updates for either key ordering.
TEST(BinaryJacobianFactor, RangedUpdateMatchesWholeAndGeneric) {
  for (const SharedDiagonal& model : noiseModels()) {
    if (model && model->isConstrained()) continue;

    const Factor actual(kKey1, kA1, kKey2, kA2, kB, model);
    const JacobianFactor expected(kKey1, kA1, kKey2, kA2, kB, model);
    const GaussianFactor& actualBase = actual;
    for (const bool reverse : {false, true}) {
      const KeyVector infoKeys =
          reverse ? KeyVector{kKey2, kKey1} : KeyVector{kKey1, kKey2};
      const std::vector<size_t> dimensions =
          reverse ? std::vector<size_t>{2, 3, 1} : std::vector<size_t>{3, 2, 1};
      SymmetricBlockMatrix expectedInfo(dimensions), wholeInfo(dimensions),
          rangedInfo(dimensions);
      expectedInfo.setZero();
      wholeInfo.setZero();
      rangedInfo.setZero();

      expected.updateHessian(infoKeys, &expectedInfo);
      actualBase.updateHessian(infoKeys, &wholeInfo);
      for (DenseIndex column = 0; column < rangedInfo.nBlocks(); ++column) {
        actualBase.updateHessian(infoKeys, &rangedInfo, column, column + 1);
      }

      EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                          Matrix(wholeInfo.selfadjointView()), 1e-12));
      EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                          Matrix(rangedInfo.selfadjointView()), 1e-12));
    }
  }
}

// Verifies scalar variable blocks use the same whole and ranged Hessian update
// semantics as a generic JacobianFactor, including reversed physical slots.
TEST(BinaryJacobianFactor, ScalarBlocksMatchGeneric) {
  using ScalarFactor = BinaryJacobianFactor<3, 1, 1>;
  const Eigen::Matrix<double, 3, 1> A1{1.0, -2.0, 0.5};
  const Eigen::Matrix<double, 3, 1> A2{-0.25, 3.0, 1.5};
  const Vector3 b{0.75, -1.0, 2.0};
  const ScalarFactor actual(kKey1, A1, kKey2, A2, b);
  const JacobianFactor expected(kKey1, A1, kKey2, A2, b);
  const GaussianFactor& actualBase = actual;

  for (const bool reverse : {false, true}) {
    const KeyVector infoKeys =
        reverse ? KeyVector{kKey2, kKey1} : KeyVector{kKey1, kKey2};
    const std::vector<size_t> dimensions{1, 1, 1};
    SymmetricBlockMatrix expectedInfo(dimensions), wholeInfo(dimensions),
        rangedInfo(dimensions);
    expectedInfo.setZero();
    wholeInfo.setZero();
    rangedInfo.setZero();

    expected.updateHessian(infoKeys, &expectedInfo);
    actualBase.updateHessian(infoKeys, &wholeInfo);
    for (DenseIndex column = 0; column < rangedInfo.nBlocks(); ++column) {
      actualBase.updateHessian(infoKeys, &rangedInfo, column, column + 1);
    }

    EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                        Matrix(wholeInfo.selfadjointView()), 1e-12));
    EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                        Matrix(rangedInfo.selfadjointView()), 1e-12));
  }
}

// Verifies constrained ranged updates retain the generic exception behavior.
TEST(BinaryJacobianFactor, ConstrainedRangedUpdateThrows) {
  const SharedDiagonal model = noiseModel::Constrained::All(2);
  const Factor factor(kKey1, kA1, kKey2, kA2, kB, model);
  const GaussianFactor& factorBase = factor;
  const KeyVector infoKeys{kKey1, kKey2};
  SymmetricBlockMatrix info(std::vector<size_t>{3, 2, 1});
  info.setZero();

  CHECK_EXCEPTION(factorBase.updateHessian(infoKeys, &info, 0, 1),
                  std::invalid_argument);
}

// Verifies dimensions outside the explicit-instantiation registry retain the
// same inline fixed-size behavior as a generic JacobianFactor.
TEST(BinaryJacobianFactor, UnlistedDimensionsUseInlineFallback) {
  using UnlistedFactor = BinaryJacobianFactor<4, 2, 4>;
  const Eigen::Matrix<double, 4, 2> A1{
      {1.0, 2.0}, {-1.0, 0.5}, {3.0, -2.0}, {0.25, 4.0}};
  const Matrix4 A2{{2.0, 0.0, -1.0, 0.5},
                   {1.0, 3.0, 0.25, -2.0},
                   {-0.5, 2.0, 4.0, 1.0},
                   {3.0, -1.0, 0.0, 2.0}};
  const Vector4 b{0.5, -1.0, 2.0, 0.25};
  const UnlistedFactor actual(kKey1, A1, kKey2, A2, b);
  const JacobianFactor expected(kKey1, A1, kKey2, A2, b);
  const GaussianFactor& actualBase = actual;

  VectorValues testValues;
  testValues.insert(kKey1, Vector2{0.5, -1.0});
  testValues.insert(kKey2, Vector4{1.0, -0.5, 2.0, 0.25});
  DOUBLES_EQUAL(expected.deltaError(testValues),
                actualBase.deltaError(testValues), 1e-12);

  VectorValues expectedDiagonal, actualDiagonal;
  expected.hessianDiagonalAdd(expectedDiagonal);
  actualBase.hessianDiagonalAdd(actualDiagonal);
  EXPECT(assert_equal(expectedDiagonal, actualDiagonal, 1e-12));

  const KeyVector infoKeys{kKey2, kKey1};
  const std::vector<size_t> dimensions{4, 2, 1};
  SymmetricBlockMatrix expectedInfo(dimensions), wholeInfo(dimensions),
      rangedInfo(dimensions);
  expectedInfo.setZero();
  wholeInfo.setZero();
  rangedInfo.setZero();
  expected.updateHessian(infoKeys, &expectedInfo);
  actualBase.updateHessian(infoKeys, &wholeInfo);
  for (DenseIndex column = 0; column < rangedInfo.nBlocks(); ++column) {
    actualBase.updateHessian(infoKeys, &rangedInfo, column, column + 1);
  }
  EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                      Matrix(wholeInfo.selfadjointView()), 1e-12));
  EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                      Matrix(rangedInfo.selfadjointView()), 1e-12));
}

}  // namespace binary_jacobian_factor
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
