/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeFixedJacobianFactor.cpp
 * @brief Compare generic and fixed-size quaternary Jacobian factors.
 */

#include <gtsam/base/Testable.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "internal/TimingUtils.h"

namespace {

using gtsam::DenseIndex;
using gtsam::FixedJacobianFactor;
using gtsam::GaussianFactor;
using gtsam::JacobianFactor;
using gtsam::Key;
using gtsam::KeyVector;
using gtsam::Matrix;
using gtsam::Matrix2;
using gtsam::SharedDiagonal;
using gtsam::SymmetricBlockMatrix;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector1;
using gtsam::Vector2;
using gtsam::VectorValues;
using std::size_t;

using FixedFactor = FixedJacobianFactor<2, 1, 2, 1, 2>;

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

class QuaternaryVectorFactor
    : public gtsam::NoiseModelFactorT<Vector2, Vector1, Vector2, Vector1,
                                      Vector2> {
 public:
  using Base =
      gtsam::NoiseModelFactorT<Vector2, Vector1, Vector2, Vector1, Vector2>;

  QuaternaryVectorFactor(const gtsam::SharedNoiseModel& model)
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

struct Samples {
  std::vector<double> linearize;
  std::vector<double> deltaError;
  std::vector<double> hessianDiagonal;
  std::vector<double> wholeHessian;
  std::vector<double> rangedHessian;
};

template <class Callable>
double microsecondsPerCall(size_t iterations, Callable&& callable) {
  const auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < iterations; ++i) callable();
  const auto end = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::micro>(end - start).count() /
         iterations;
}

JacobianFactor genericFactor() {
  const std::vector<std::pair<Key, Matrix>> terms{
      {kKey1, kA1}, {kKey2, kA2}, {kKey3, kA3}, {kKey4, kA4}};
  return JacobianFactor(terms, kB);
}

VectorValues deltaValues() {
  VectorValues result;
  result.insert(kKey1, Vector1{0.5});
  result.insert(kKey2, Vector2{-0.25, 1.5});
  result.insert(kKey3, Vector1{0.75});
  result.insert(kKey4, Vector2{1.25, -0.5});
  return result;
}

Values nonlinearValues() {
  return Values{{kKey1, gtsam::genericValue(Vector1{0.5})},
                {kKey2, gtsam::genericValue(Vector2{-0.25, 1.5})},
                {kKey3, gtsam::genericValue(Vector1{0.75})},
                {kKey4, gtsam::genericValue(Vector2{1.25, -0.5})}};
}

void verifyEquivalent(const QuaternaryVectorFactor& nonlinearFactor,
                      const Values& values, const JacobianFactor& generic,
                      const FixedFactor& fixed) {
  const auto genericLinear =
      nonlinearFactor.NoiseModelFactor::linearize(values);
  const auto fixedLinear = nonlinearFactor.linearize(values);
  if (!gtsam::assert_equal(*genericLinear, *fixedLinear, 1e-12)) {
    throw std::runtime_error("Linearized factors differ");
  }

  const VectorValues testValues = deltaValues();
  if (std::abs(generic.deltaError(testValues) - fixed.deltaError(testValues)) >
      1e-12) {
    throw std::runtime_error("Delta errors differ");
  }

  const KeyVector infoKeys{kKey4, kKey2, kKey1, kKey3};
  const std::vector<size_t> dimensions{2, 2, 1, 1, 1};
  SymmetricBlockMatrix genericInfo(dimensions), fixedInfo(dimensions);
  genericInfo.setZero();
  fixedInfo.setZero();
  generic.updateHessian(infoKeys, &genericInfo);
  fixed.updateHessian(infoKeys, &fixedInfo);
  if (!gtsam::assert_equal(Matrix(genericInfo.selfadjointView()),
                           Matrix(fixedInfo.selfadjointView()), 1e-12)) {
    throw std::runtime_error("Hessian updates differ");
  }
}

void runMode(bool useFixed, size_t linearizeIterations, size_t kernelIterations,
             const QuaternaryVectorFactor& nonlinearFactor,
             const Values& values, const JacobianFactor& generic,
             const FixedFactor& fixed, Samples* samples) {
  const GaussianFactor& factor =
      useFixed ? static_cast<const GaussianFactor&>(fixed)
               : static_cast<const GaussianFactor&>(generic);
  const VectorValues testValues = deltaValues();
  const KeyVector infoKeys{kKey4, kKey2, kKey1, kKey3};
  const std::vector<size_t> dimensions{2, 2, 1, 1, 1};
  SymmetricBlockMatrix wholeInfo(dimensions), rangedInfo(dimensions);
  wholeInfo.setZero();
  rangedInfo.setZero();
  VectorValues diagonal;
  volatile double sink = 0.0;

  samples->linearize.push_back(microsecondsPerCall(linearizeIterations, [&] {
    std::shared_ptr<GaussianFactor> linear =
        useFixed ? nonlinearFactor.linearize(values)
                 : nonlinearFactor.NoiseModelFactor::linearize(values);
    sink += linear->size();
  }));
  samples->deltaError.push_back(microsecondsPerCall(
      kernelIterations, [&] { sink += factor.deltaError(testValues); }));
  samples->hessianDiagonal.push_back(microsecondsPerCall(kernelIterations, [&] {
    factor.hessianDiagonalAdd(diagonal);
    sink += diagonal.at(kKey1)(0);
  }));
  samples->wholeHessian.push_back(microsecondsPerCall(kernelIterations, [&] {
    factor.updateHessian(infoKeys, &wholeInfo);
    sink += wholeInfo.blockView(0, 0)(0, 0);
  }));
  samples->rangedHessian.push_back(microsecondsPerCall(kernelIterations, [&] {
    for (DenseIndex column = 0; column < rangedInfo.nBlocks(); ++column) {
      factor.updateHessian(infoKeys, &rangedInfo, column, column + 1);
    }
    sink += rangedInfo.blockView(0, 0)(0, 0);
  }));
  if (sink == 0.0) throw std::runtime_error("Benchmark result was unused");
}

double median(const std::vector<double>& samples) {
  return gtsam::timing::summarizeSamples(
             samples, gtsam::timing::MedianPolicy::kAverageMiddle)
      .median;
}

void printResult(const char* name, const std::vector<double>& generic,
                 const std::vector<double>& fixed) {
  const double genericMedian = median(generic);
  const double fixedMedian = median(fixed);
  std::cout << name << " generic_us=" << genericMedian
            << " fixed_us=" << fixedMedian
            << " change_percent=" << 100.0 * (fixedMedian / genericMedian - 1.0)
            << '\n';
}

}  // namespace

int main(int argc, const char* argv[]) {
  gtsam::timing::Arguments arguments(argc, argv);
  const size_t trials = arguments.sizeValue("--trials", 9);
  const size_t linearizeIterations =
      arguments.sizeValue("--linearize-iterations", 100000);
  const size_t kernelIterations =
      arguments.sizeValue("--kernel-iterations", 200000);
  arguments.validateAllConsumed();

  const auto model = gtsam::noiseModel::Diagonal::Sigmas(Vector2{0.5, 2.0});
  const QuaternaryVectorFactor nonlinearFactor(model);
  const Values values = nonlinearValues();
  const JacobianFactor generic = genericFactor();
  const FixedFactor fixed(kKeys, kJacobians, kB);
  verifyEquivalent(nonlinearFactor, values, generic, fixed);

  Samples genericSamples, fixedSamples;
  for (size_t trial = 0; trial < trials + 1; ++trial) {
    for (size_t pass = 0; pass < 2; ++pass) {
      const bool useFixed = (trial + pass) % 2 == 0;
      Samples* destination = useFixed ? &fixedSamples : &genericSamples;
      runMode(useFixed, linearizeIterations, kernelIterations, nonlinearFactor,
              values, generic, fixed, destination);
    }
    if (trial == 0) {
      genericSamples = Samples{};
      fixedSamples = Samples{};
    }
  }

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Quaternary fixed Jacobian benchmark trials=" << trials
            << " linearize_iterations=" << linearizeIterations
            << " kernel_iterations=" << kernelIterations << '\n';
  printResult("linearize", genericSamples.linearize, fixedSamples.linearize);
  printResult("delta_error", genericSamples.deltaError,
              fixedSamples.deltaError);
  printResult("hessian_diagonal", genericSamples.hessianDiagonal,
              fixedSamples.hessianDiagonal);
  printResult("whole_hessian", genericSamples.wholeHessian,
              fixedSamples.wholeHessian);
  printResult("ranged_hessian", genericSamples.rangedHessian,
              fixedSamples.rangedHessian);
  return 0;
}
