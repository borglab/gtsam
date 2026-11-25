/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianNoiseSampler.cpp
 * @brief   Unit tests for GaussianNoiseSampler
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/config.h>  // for GTSAM_USE_TBB
#include <gtsam/planning/GaussianNoiseSampler.h>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

#include <stdexcept>

using namespace gtsam;

/* ************************************************************************* */
namespace gaussian_noise_sampler {

const size_t horizon = 4;
const std::uint64_t seed = 42;
const Matrix covariance{{0.04, 0.01}, {0.01, 0.09}};

/// A sampler with the test covariance and seed, and the given sample count.
GaussianNoiseSampler samplerWith(size_t numSamples,
                                 const Matrix& noiseCovariance = covariance,
                                 std::uint64_t samplerSeed = seed) {
  return GaussianNoiseSampler(noiseCovariance, numSamples, samplerSeed);
}

// Checks the accessors, and that an empty covariance is accepted while a
// non-square or non-positive-definite one is rejected.
TEST(GaussianNoiseSampler, Construction) {
  const GaussianNoiseSampler sampler = samplerWith(8);
  EXPECT_LONGS_EQUAL(2, sampler.dim());
  EXPECT_LONGS_EQUAL(8, sampler.numSamples());
  EXPECT_LONGS_EQUAL(42, sampler.seed());
  EXPECT_LONGS_EQUAL(0, sampler.stepCounter());

  EXPECT_LONGS_EQUAL(0, samplerWith(8, Matrix()).dim());
  CHECK_EXCEPTION(samplerWith(8, Matrix::Ones(2, 3)), std::invalid_argument);
  CHECK_EXCEPTION(samplerWith(8, Matrix{{1.0, 2.0}, {2.0, 1.0}}),
                  std::invalid_argument);
}

// Checks the M x (K*H) shape of the noise and that sampling advances the
// step counter.
TEST(GaussianNoiseSampler, NoiseShape) {
  const GaussianNoiseSampler sampler = samplerWith(8);
  const Matrix& noise = sampler.sampleNoise(horizon);
  EXPECT_LONGS_EQUAL(2, noise.rows());
  EXPECT_LONGS_EQUAL(8 * horizon, noise.cols());
  EXPECT_LONGS_EQUAL(1, sampler.stepCounter());
}

// Checks E = L Z: with the same seed, the noise for a covariance L L^T is L
// times the noise for the identity covariance.
TEST(GaussianNoiseSampler, NoiseIsCholeskyScaled) {
  const Matrix standard =
      samplerWith(8, Matrix::Identity(2, 2)).sampleNoise(horizon);
  const Matrix scaled = samplerWith(8).sampleNoise(horizon);
  const Matrix lower = Eigen::LLT<Matrix>(covariance).matrixL();
  EXPECT(assert_equal(Matrix(lower * standard), scaled, 1e-12));
}

// Checks that the same seed gives the same noise and another seed does not.
TEST(GaussianNoiseSampler, Seed) {
  const Matrix noise = samplerWith(8).sampleNoise(horizon);
  EXPECT(assert_equal(noise, Matrix(samplerWith(8).sampleNoise(horizon)),
                      0.0));
  EXPECT(!noise.isApprox(samplerWith(8, covariance, 7).sampleNoise(horizon)));
}

// Checks that each sampling call draws new noise, and that a new sampler
// repeats the same sequence of calls.
TEST(GaussianNoiseSampler, StepCounter) {
  const GaussianNoiseSampler sampler = samplerWith(8);
  const Matrix first = sampler.sampleNoise(horizon);
  const Matrix second = sampler.sampleNoise(horizon);
  EXPECT_LONGS_EQUAL(2, sampler.stepCounter());
  EXPECT(!first.isApprox(second));

  const GaussianNoiseSampler repeat = samplerWith(8);
  EXPECT(assert_equal(first, Matrix(repeat.sampleNoise(horizon)), 0.0));
  EXPECT(assert_equal(second, Matrix(repeat.sampleNoise(horizon)), 0.0));
}

// Checks that sample i is the same whatever the number of samples.
TEST(GaussianNoiseSampler, SampleIndependentOfCount) {
  const Matrix few = samplerWith(5).sampleNoise(horizon);
  const Matrix many = samplerWith(50).sampleNoise(horizon);
  EXPECT(assert_equal(few, Matrix(many.leftCols(5 * horizon)), 0.0));
}

// Checks that the noise is the same for any number of threads.
TEST(GaussianNoiseSampler, ThreadCountIndependent) {
  const Matrix expected = samplerWith(256).sampleNoise(horizon);
#ifdef GTSAM_USE_TBB
  for (int threads : {1, 2, 4}) {
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism,
                              static_cast<size_t>(threads));
    EXPECT(assert_equal(expected,
                        Matrix(samplerWith(256).sampleNoise(horizon)), 0.0));
  }
#endif
}

// Checks that, over many samples, each step's noise has zero mean and the
// given covariance, and different steps are uncorrelated.
TEST(GaussianNoiseSampler, NoiseStatistics) {
  const size_t numSamples = 20000, steps = 2;
  const Matrix noise = samplerWith(numSamples).sampleNoise(steps);
  for (size_t k = 0; k < steps; ++k) {
    Vector mean = Vector::Zero(2);
    Matrix second = Matrix::Zero(2, 2);
    for (size_t i = 0; i < numSamples; ++i) {
      const Vector sample = noise.col(i * steps + k);
      mean += sample;
      second += sample * sample.transpose();
    }
    mean /= static_cast<double>(numSamples);
    second /= static_cast<double>(numSamples);
    EXPECT(assert_equal(Vector(Vector::Zero(2)), mean, 0.01));
    EXPECT(assert_equal(covariance, second, 0.01));
  }
  Matrix cross = Matrix::Zero(2, 2);
  for (size_t i = 0; i < numSamples; ++i) {
    cross += noise.col(i * steps) * noise.col(i * steps + 1).transpose();
  }
  cross /= static_cast<double>(numSamples);
  EXPECT(assert_equal(Matrix(Matrix::Zero(2, 2)), cross, 0.01));
}

// Checks that the sampled controls are the nominal controls plus the noise,
// in the same layout.
TEST(GaussianNoiseSampler, SampleControls) {
  const Matrix nominal{{1.0, 2.0, 3.0, 4.0}, {-1.0, 0.0, 1.0, 2.0}};
  const Matrix noise = samplerWith(8).sampleNoise(horizon);
  const Matrix controls = samplerWith(8).sampleControls(nominal);
  EXPECT(assert_equal(Matrix(noise + nominal.replicate(1, 8)), controls));
  // Sample 5, step 2.
  EXPECT(assert_equal(Vector(nominal.col(2) + noise.col(5 * horizon + 2)),
                      Vector(controls.col(5 * horizon + 2))));
}

// Checks that the sampled controls equal the noise plus the nominal controls
// exactly, call after call, for a 2 x 2 and a full 4 x 4 covariance.
TEST(GaussianNoiseSampler, SampleControlsExact) {
  const Matrix factor{{1.0, 0.0, 0.0, 0.0},
                      {0.3, 0.5, 0.0, 0.0},
                      {-0.2, 0.1, 0.4, 0.0},
                      {0.1, -0.3, 0.2, 0.6}};
  for (const Matrix& noiseCovariance :
       {covariance, Matrix(factor * factor.transpose())}) {
    const Eigen::Index dimension = noiseCovariance.rows();
    Matrix nominal(dimension, horizon);
    for (Eigen::Index row = 0; row < dimension; ++row) {
      for (size_t k = 0; k < horizon; ++k) {
        nominal(row, k) = 1.0 + 0.5 * row - 0.25 * k;
      }
    }
    const GaussianNoiseSampler noiseSampler = samplerWith(64, noiseCovariance);
    const GaussianNoiseSampler controlSampler =
        samplerWith(64, noiseCovariance);
    for (int call = 0; call < 2; ++call) {
      const Matrix noise = noiseSampler.sampleNoise(horizon);
      const Matrix controls = controlSampler.sampleControls(nominal);
      EXPECT(assert_equal(Matrix(noise + nominal.replicate(1, 64)), controls,
                          0.0));
    }
    EXPECT_LONGS_EQUAL(2, controlSampler.stepCounter());
  }
}

// Checks that the sampled controls are the same for any number of threads.
TEST(GaussianNoiseSampler, SampleControlsThreadCountIndependent) {
  const Matrix nominal{{1.0, 2.0, 3.0, 4.0}, {-1.0, 0.0, 1.0, 2.0}};
  const Matrix expected = samplerWith(256).sampleControls(nominal);
#ifdef GTSAM_USE_TBB
  for (int threads : {1, 2, 4}) {
    tbb::global_control limit(tbb::global_control::max_allowed_parallelism,
                              static_cast<size_t>(threads));
    EXPECT(assert_equal(expected,
                        Matrix(samplerWith(256).sampleControls(nominal)), 0.0));
  }
#endif
}

// Checks that sampling throws if the covariance is not set or does not match
// the control dimension.
TEST(GaussianNoiseSampler, SamplingErrors) {
  const Matrix nominal = Matrix::Zero(2, horizon);
  const GaussianNoiseSampler unset = samplerWith(8, Matrix());
  CHECK_EXCEPTION(unset.sampleNoise(horizon), std::invalid_argument);
  CHECK_EXCEPTION(unset.sampleControls(nominal), std::invalid_argument);

  const GaussianNoiseSampler threeD = samplerWith(8, Matrix::Identity(3, 3));
  CHECK_EXCEPTION(threeD.sampleControls(nominal), std::invalid_argument);
}

}  // namespace gaussian_noise_sampler
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
