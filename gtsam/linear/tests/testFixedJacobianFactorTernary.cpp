/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testFixedJacobianFactorTernary.cpp
 * @brief Tests for ternary-arity FixedJacobianFactor operations.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <array>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace fixed_jacobian_factor_ternary {

using Factor = FixedJacobianFactor<2, 3, 2, 1>;

constexpr Key kKey1 = 10;
constexpr Key kKey2 = 20;
constexpr Key kKey3 = 30;
const Matrix23 kA1{{1.0, 2.0, -1.0}, {0.5, -3.0, 4.0}};
const Matrix2 kA2{{2.0, -0.5}, {1.5, 3.0}};
const Eigen::Matrix<double, 2, 1> kA3{1.25, -0.75};
const Vector2 kB{0.25, -2.0};
const KeyVector kKeys{kKey1, kKey2, kKey3};
const std::vector<Matrix> kJacobians{kA1, kA2, kA3};

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
  result.insert(kKey3, Vector1{0.75});
  return result;
}

Factor fixedFactor(const SharedDiagonal& model) {
  return Factor(kKeys, kJacobians, kB, model);
}

JacobianFactor genericFactor(const SharedDiagonal& model) {
  return JacobianFactor(kKey1, kA1, kKey2, kA2, kKey3, kA3, kB, model);
}

// Verifies fixed-size delta error and optional outputs match generic behavior.
TEST(FixedJacobianFactorTernary, DeltaErrorMatchesGeneric) {
  const VectorValues testValues = values();
  for (const SharedDiagonal& model : noiseModels()) {
    const Factor actual = fixedFactor(model);
    const JacobianFactor expected = genericFactor(model);
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
TEST(FixedJacobianFactorTernary, HessianDiagonalMatchesGeneric) {
  for (const SharedDiagonal& model : noiseModels()) {
    const Factor actual = fixedFactor(model);
    const JacobianFactor expected = genericFactor(model);
    const GaussianFactor& actualBase = actual;

    VectorValues expectedDiagonal, actualDiagonal;
    expected.hessianDiagonalAdd(expectedDiagonal);
    actualBase.hessianDiagonalAdd(actualDiagonal);
    EXPECT(assert_equal(expectedDiagonal, actualDiagonal, 1e-12));
  }
}

// Verifies fixed-size diagonal updates accumulate into existing entries.
TEST(FixedJacobianFactorTernary, HessianDiagonalAccumulates) {
  for (const SharedDiagonal& model : noiseModels()) {
    const Factor actual = fixedFactor(model);
    const JacobianFactor expected = genericFactor(model);
    const GaussianFactor& actualBase = actual;

    VectorValues expectedDiagonal, actualDiagonal;
    for (VectorValues* diagonal : {&expectedDiagonal, &actualDiagonal}) {
      diagonal->insert(kKey1, Vector3::Constant(2.0));
      diagonal->insert(kKey2, Vector2::Constant(-1.0));
      diagonal->insert(kKey3, Vector1::Constant(4.0));
      diagonal->insert(40, Vector1{7.0});
    }
    expected.hessianDiagonalAdd(expectedDiagonal);
    actualBase.hessianDiagonalAdd(actualDiagonal);
    EXPECT(assert_equal(expectedDiagonal, actualDiagonal, 1e-12));
  }
}

// Verifies whole and ranged Hessian updates for all physical key orderings.
TEST(FixedJacobianFactorTernary, RangedUpdateMatchesWholeAndGeneric) {
  const std::array<std::array<size_t, 3>, 6> permutations{{
      {{0, 1, 2}},
      {{0, 2, 1}},
      {{1, 0, 2}},
      {{1, 2, 0}},
      {{2, 0, 1}},
      {{2, 1, 0}},
  }};
  const std::array<Key, 3> keys{{kKey1, kKey2, kKey3}};
  const std::array<size_t, 3> blockDimensions{{3, 2, 1}};

  for (const SharedDiagonal& model : noiseModels()) {
    if (model && model->isConstrained()) continue;
    const Factor actual = fixedFactor(model);
    const JacobianFactor expected = genericFactor(model);
    const GaussianFactor& actualBase = actual;

    for (const auto& permutation : permutations) {
      KeyVector infoKeys;
      std::vector<size_t> dimensions;
      for (size_t index : permutation) {
        infoKeys.push_back(keys[index]);
        dimensions.push_back(blockDimensions[index]);
      }
      dimensions.push_back(1);

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

// Verifies constrained ranged updates retain the generic exception behavior.
TEST(FixedJacobianFactorTernary, ConstrainedRangedUpdateThrows) {
  const Factor factor = fixedFactor(noiseModel::Constrained::All(2));
  const GaussianFactor& factorBase = factor;
  const KeyVector infoKeys{kKey1, kKey2, kKey3};
  SymmetricBlockMatrix info(std::vector<size_t>{3, 2, 1, 1});
  info.setZero();

  CHECK_EXCEPTION(factorBase.updateHessian(infoKeys, &info, 0, 1),
                  std::invalid_argument);
}

// Verifies an uncommon dimension combination instantiates and executes the
// header-defined fixed-size helper fallback.
TEST(FixedJacobianFactorTernary, UnlistedDimensionsUseInlineFallback) {
  using UnlistedFactor = FixedJacobianFactor<4, 2, 4, 5>;
  const Eigen::Matrix<double, 4, 2> A1{
      {1.0, 2.0}, {-1.0, 0.5}, {3.0, -2.0}, {0.25, 4.0}};
  const Matrix4 A2{{2.0, 0.0, -1.0, 0.5},
                   {1.0, 3.0, 0.25, -2.0},
                   {-0.5, 2.0, 4.0, 1.0},
                   {3.0, -1.0, 0.0, 2.0}};
  const Eigen::Matrix<double, 4, 5> A3{{1.0, 0.0, 2.0, -1.0, 0.5},
                                       {-2.0, 1.0, 0.0, 3.0, 0.25},
                                       {0.5, -1.0, 4.0, 0.0, 2.0},
                                       {3.0, 2.0, -0.5, 1.0, -1.0}};
  const Vector4 b{0.5, -1.0, 2.0, 0.25};
  const UnlistedFactor actual(KeyVector{kKey1, kKey2, kKey3},
                              std::vector<Matrix>{A1, A2, A3}, b);
  const JacobianFactor expected(kKey1, A1, kKey2, A2, kKey3, A3, b);
  const GaussianFactor& actualBase = actual;

  VectorValues testValues;
  testValues.insert(kKey1, Vector2{0.5, -1.0});
  testValues.insert(kKey2, Vector4{1.0, -0.5, 2.0, 0.25});
  testValues.insert(kKey3, Vector5{0.25, 1.5, -2.0, 0.5, 3.0});
  DOUBLES_EQUAL(expected.deltaError(testValues),
                actualBase.deltaError(testValues), 1e-12);

  VectorValues expectedDiagonal, actualDiagonal;
  expected.hessianDiagonalAdd(expectedDiagonal);
  actualBase.hessianDiagonalAdd(actualDiagonal);
  EXPECT(assert_equal(expectedDiagonal, actualDiagonal, 1e-12));

  const KeyVector infoKeys{kKey3, kKey1, kKey2};
  const std::vector<size_t> dimensions{5, 2, 4, 1};
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

}  // namespace fixed_jacobian_factor_ternary
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
