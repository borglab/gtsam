/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testFixedJacobianFactor.cpp
 * @brief Tests for arbitrary-arity FixedJacobianFactor operations.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace fixed_jacobian_factor {

using Factor = FixedJacobianFactor<2, 1, 2, 1, 2>;

constexpr Key kKey1 = 10;
constexpr Key kKey2 = 20;
constexpr Key kKey3 = 30;
constexpr Key kKey4 = 40;
const Eigen::Matrix<double, 2, 1> kA1{1.0, -0.5};
const Matrix2 kA2{{2.0, -0.5}, {1.5, 3.0}};
const Eigen::Matrix<double, 2, 1> kA3{1.25, -0.75};
const Matrix2 kA4{{-1.0, 0.25}, {2.0, 0.5}};
const Vector2 kB{0.25, -2.0};
const KeyVector kKeys{kKey1, kKey2, kKey3, kKey4};
const std::vector<Matrix> kJacobians{kA1, kA2, kA3, kA4};

std::vector<SharedDiagonal> noiseModels() {
  return {SharedDiagonal(), noiseModel::Unit::Create(2),
          noiseModel::Isotropic::Sigma(2, 0.5),
          noiseModel::Diagonal::Sigmas(Vector2{0.5, 2.0}),
          noiseModel::Constrained::MixedSigmas(Vector2{0.0, 2.0})};
}

VectorValues values() {
  VectorValues result;
  result.insert(kKey1, Vector1{0.5});
  result.insert(kKey2, Vector2{-0.25, 1.5});
  result.insert(kKey3, Vector1{0.75});
  result.insert(kKey4, Vector2{1.25, -0.5});
  return result;
}

Factor fixedFactor(const SharedDiagonal& model = SharedDiagonal()) {
  return Factor(kKeys, kJacobians, kB, model);
}

JacobianFactor genericFactor(const SharedDiagonal& model = SharedDiagonal()) {
  const std::vector<std::pair<Key, Matrix>> terms{
      {kKey1, kA1}, {kKey2, kA2}, {kKey3, kA3}, {kKey4, kA4}};
  return JacobianFactor(terms, kB, model);
}

// Verifies arbitrary-arity residual and diagonal kernels match the generic
// path.
TEST(FixedJacobianFactor, QuaternaryResidualAndDiagonalMatchGeneric) {
  const VectorValues testValues = values();
  for (const SharedDiagonal& model : noiseModels()) {
    const Factor actual = fixedFactor(model);
    const JacobianFactor expected = genericFactor(model);
    const GaussianFactor& actualBase = actual;

    double expectedOld = 0.0, expectedNew = 0.0;
    double actualOld = 0.0, actualNew = 0.0;
    DOUBLES_EQUAL(expected.deltaError(testValues, &expectedOld, &expectedNew),
                  actualBase.deltaError(testValues, &actualOld, &actualNew),
                  1e-12);
    DOUBLES_EQUAL(expectedOld, actualOld, 1e-12);
    DOUBLES_EQUAL(expectedNew, actualNew, 1e-12);

    VectorValues expectedDiagonal, actualDiagonal;
    expected.hessianDiagonalAdd(expectedDiagonal);
    actualBase.hessianDiagonalAdd(actualDiagonal);
    EXPECT(assert_equal(expectedDiagonal, actualDiagonal, 1e-12));
  }
}

// Verifies every pairwise Hessian block for all quaternary key orderings.
TEST(FixedJacobianFactor, QuaternaryHessianMatchesGeneric) {
  const Factor actual = fixedFactor();
  const JacobianFactor expected = genericFactor();
  const GaussianFactor& actualBase = actual;
  std::array<size_t, 4> permutation{{0, 1, 2, 3}};
  const std::array<size_t, 4> blockDimensions{{1, 2, 1, 2}};

  do {
    KeyVector infoKeys;
    std::vector<size_t> dimensions;
    for (size_t index : permutation) {
      infoKeys.push_back(kKeys[index]);
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
  } while (std::next_permutation(permutation.begin(), permutation.end()));
}

class QuaternaryVectorFactor
    : public NoiseModelFactorT<Vector2, Vector1, Vector2, Vector1, Vector2> {
 public:
  using Base = NoiseModelFactorT<Vector2, Vector1, Vector2, Vector1, Vector2>;

  QuaternaryVectorFactor(const SharedNoiseModel& model)
      : Base(model, kKey1, kKey2, kKey3, kKey4) {}

  Vector2 evaluateError(const Vector1& x1, const Vector2& x2, const Vector1& x3,
                        const Vector2& x4, Matrix* H1 = nullptr,
                        Matrix* H2 = nullptr, Matrix* H3 = nullptr,
                        Matrix* H4 = nullptr) const override {
    if (H1) *H1 = kA1;
    if (H2) *H2 = kA2;
    if (H3) *H3 = kA3;
    if (H4) *H4 = kA4;
    return kA1 * x1 + kA2 * x2 + kA3 * x3 + kA4 * x4 - kB;
  }
};

class UnaryVectorFactor : public NoiseModelFactorT<Vector2, Vector2> {
 public:
  using Base = NoiseModelFactorT<Vector2, Vector2>;

  UnaryVectorFactor(const SharedNoiseModel& model) : Base(model, kKey1) {}

  Vector2 evaluateError(const Vector2& x, Matrix* H = nullptr) const override {
    if (H) *H = kA2;
    return kA2 * x - kB;
  }
};

// Verifies unary fixed-output factors also select the arbitrary-arity path.
TEST(FixedJacobianFactor, UnaryNonlinearLinearization) {
  using UnaryFactor = FixedJacobianFactor<2, 2>;
  const UnaryVectorFactor factor(noiseModel::Unit::Create(2));
  const Values nonlinearValues{{kKey1, genericValue(Vector2{0.5, -1.0})}};

  const auto generic = factor.NoiseModelFactor::linearize(nonlinearValues);
  const auto fixed = factor.linearize(nonlinearValues);
  CHECK((std::dynamic_pointer_cast<UnaryFactor>(fixed)));
  EXPECT(assert_equal(*generic, *fixed, 1e-12));

  const KeyVector infoKeys{kKey1};
  const std::vector<size_t> dimensions{2, 1};
  SymmetricBlockMatrix expectedInfo(dimensions), wholeInfo(dimensions),
      rangedInfo(dimensions);
  expectedInfo.setZero();
  wholeInfo.setZero();
  rangedInfo.setZero();
  generic->updateHessian(infoKeys, &expectedInfo);
  fixed->updateHessian(infoKeys, &wholeInfo);
  for (DenseIndex column = 0; column < rangedInfo.nBlocks(); ++column) {
    fixed->updateHessian(infoKeys, &rangedInfo, column, column + 1);
  }

  EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                      Matrix(wholeInfo.selfadjointView()), 1e-12));
  EXPECT(assert_equal(Matrix(expectedInfo.selfadjointView()),
                      Matrix(rangedInfo.selfadjointView()), 1e-12));
}

// Verifies fixed-output quaternary nonlinear factors select the fixed path.
TEST(FixedJacobianFactor, QuaternaryNonlinearLinearization) {
  const auto model = noiseModel::Diagonal::Sigmas(Vector2{0.5, 2.0});
  const QuaternaryVectorFactor factor(model);
  const Values nonlinearValues{{kKey1, genericValue(Vector1{0.5})},
                               {kKey2, genericValue(Vector2{-0.25, 1.5})},
                               {kKey3, genericValue(Vector1{0.75})},
                               {kKey4, genericValue(Vector2{1.25, -0.5})}};

  const auto generic = factor.NoiseModelFactor::linearize(nonlinearValues);
  const auto fixed = factor.linearize(nonlinearValues);
  CHECK((std::dynamic_pointer_cast<Factor>(fixed)));
  EXPECT(assert_equal(*generic, *fixed, 1e-12));
}

}  // namespace fixed_jacobian_factor
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
