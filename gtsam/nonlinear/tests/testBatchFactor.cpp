/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBatchFactor.cpp
 * @brief Unit tests for BatchFactor class
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace std;

// Define a specific projection factor type for testing
using ProjectionFactor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>;

static std::shared_ptr<Cal3_S2> sharedK = std::make_shared<Cal3_S2>();

// Factor with a dynamic dimension (Vector), used to verify dynamic-dimension
// safety in key bookkeeping and linearization.
class DynamicVectorFactor : public NoiseModelFactor1<Vector> {
 private:
  Vector measurement_;

 public:
  DynamicVectorFactor(Key key, const Vector& measurement,
                      const SharedNoiseModel& model)
      : NoiseModelFactor1<Vector>(model, key), measurement_(measurement) {}

  Vector evaluateError(const Vector& x, OptionalMatrixType H) const override {
    if (H) *H = Matrix::Identity(x.size(), x.size());
    return x - measurement_;
  }
};

static JacobianFactor denseJacobian(
    const GaussianFactor::shared_ptr& gaussian) {
  auto compact = std::dynamic_pointer_cast<BatchJacobianFactorBase>(gaussian);
  if (compact) return compact->toJacobianFactor();
  auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);
  if (!jacobian) throw std::runtime_error("Expected a Jacobian-like factor.");
  return *jacobian;
}

/* ************************************************************************* */
// Verifies vector construction linearizes to a compact fixed-size batch factor.
TEST(BatchFactor, ConstructorAndLinearize) {
  // 1. Setup data
  std::vector<Key> poses = {Symbol('x', 0)};
  std::vector<Key> points;
  std::vector<Point2> measurements;

  // Create 10 points
  for (int i = 0; i < 10; ++i) {
    points.push_back(Symbol('l', i));
    measurements.push_back(Point2(double(i), double(i)));
  }

  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);

  // 2. Create factors manually
  std::vector<ProjectionFactor> factors;
  for (size_t i = 0; i < points.size(); ++i) {
    factors.emplace_back(measurements[i], noise, poses[0], points[i], sharedK);
  }

  // 3. Construct BatchFactor
  auto batch =
      std::make_shared<BatchFactor<ProjectionFactor, 2>>(std::move(factors));

  // 4. Linearize
  Values values;
  values.insert(Symbol('x', 0), Pose3());
  for (int i = 0; i < 10; ++i) {
    values.insert(Symbol('l', i), Point3(0, 0, 10));
  }

  auto gaussian = batch->linearize(values);
  auto compact = std::dynamic_pointer_cast<BatchJacobianFactorBase>(gaussian);
  const JacobianFactor jacobian = denseJacobian(gaussian);

  // 5. Verify
  CHECK(compact);
  LONGS_EQUAL(20, (long)jacobian.rows());
  LONGS_EQUAL(11, (long)jacobian.size());  // 1 pose + 10 points
}

/* ************************************************************************* */
// Verifies constrained row semantics are preserved in the dense Jacobian model.
TEST(BatchFactor, ConstrainedNoiseModel) {
  Key key = Symbol('x', 0);
  Vector constrainedSigmas{{1.0, 0.0, 2.0}};
  Vector constrainedMus{{3.0, 4.0, 5.0}};
  auto constrained =
      noiseModel::Constrained::MixedSigmas(constrainedMus, constrainedSigmas);

  std::vector<PriorFactor<Pose2>> factors;
  factors.emplace_back(key, Pose2(1.0, 2.0, 0.1), constrained);
  factors.emplace_back(key, Pose2(2.0, 3.0, 0.2), constrained);

  auto batch =
      std::make_shared<BatchFactor<PriorFactor<Pose2>, 3>>(std::move(factors));

  Values values;
  values.insert(key, Pose2(0.0, 0.0, 0.0));

  auto gaussian = batch->linearize(values);
  auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);
  CHECK(jacobian);
  CHECK(jacobian->get_model());
  CHECK(jacobian->get_model()->isConstrained());
  auto constrainedModel =
      std::dynamic_pointer_cast<noiseModel::Constrained>(jacobian->get_model());
  CHECK(constrainedModel);
  CHECK(jacobian->get_model()->isUnit() == false);
  Vector expectedMus(6);
  Vector expectedSigmas(6);
  Vector constrainedRowMus = constrainedMus;
  constrainedRowMus[0] = 1000.0;
  constrainedRowMus[2] = 1000.0;
  expectedMus << constrainedRowMus, constrainedRowMus;
  Vector expectedRowSigmas{{1.0, 0.0, 1.0}};
  expectedSigmas << expectedRowSigmas, expectedRowSigmas;
  EXPECT(assert_equal(expectedMus, constrainedModel->mu(), 1e-9));
  EXPECT(assert_equal(expectedSigmas, constrainedModel->sigmas(), 1e-9));
  LONGS_EQUAL(6, (long)jacobian->rows());
}

// Verifies constrained and unconstrained rows in constrained models are
// preserved correctly after whitening in linearization.
TEST(BatchFactor, ConstrainedNoiseModelUsesUnitSigmasForUnconstrainedRows) {
  Key key = Symbol('x', 0);
  Vector constrainedSigmas{{2.0, 0.0, 4.0}};
  Vector constrainedMus{{3.0, 7.0, 5.0}};
  auto constrained =
      noiseModel::Constrained::MixedSigmas(constrainedMus, constrainedSigmas);

  std::vector<PriorFactor<Pose2>> factors;
  factors.emplace_back(key, Pose2(2.0, 0.0, 1.0), constrained);
  BatchFactor<PriorFactor<Pose2>, 3> batch(std::move(factors));

  Values values;
  values.insert(key, Pose2(0.0, 0.0, 0.0));

  auto gaussian = batch.linearize(values);
  auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);
  CHECK(jacobian);
  auto constrainedModel =
      std::dynamic_pointer_cast<noiseModel::Constrained>(jacobian->get_model());
  CHECK(constrainedModel);

  Vector expectedSigmas{{1.0, 0.0, 1.0}};
  Vector expectedMus{{1000.0, 7.0, 1000.0}};

  EXPECT(assert_equal(expectedSigmas, constrainedModel->sigmas(), 1e-9));
  EXPECT(assert_equal(expectedMus, constrainedModel->mu(), 1e-9));
}

// Verifies updateKeys deduplicates by key only for fixed-size batch factors.
TEST(BatchFactor, UpdateKeysDeduplicatesByKeyOnly) {
  Key key1 = Symbol('x', 1);
  Key key2 = Symbol('x', 2);
  auto noise = noiseModel::Isotropic::Sigma(3, 1.0);

  std::vector<BetweenFactor<Pose2>> factors;
  factors.emplace_back(key1, key2, Pose2(1.0, 0.0, 0.0), noise);
  factors.emplace_back(key2, key1, Pose2(1.0, 0.0, 0.0), noise);

  BatchFactor<BetweenFactor<Pose2>, 3> batch(std::move(factors));

  Values values;
  values.insert(key1, Pose2(0.0, 0.0, 0.0));
  values.insert(key2, Pose2(0.0, 0.0, 0.0));

  auto gaussian = batch.linearize(values);
  auto compact = std::dynamic_pointer_cast<BatchJacobianFactorBase>(gaussian);
  CHECK(compact);

  auto jacobian = compact->toJacobianFactor();
  EXPECT_LONGS_EQUAL(2, (long)jacobian.size());
  EXPECT_LONGS_EQUAL(6, (long)jacobian.rows());
}

/* ************************************************************************* */
// Verifies dynamic-size variable types do not use stale dimension sentinels.
TEST(BatchFactor, DynamicDimensionSupport) {
  Key key = Symbol('x', 0);
  const SharedDiagonal model = noiseModel::Isotropic::Sigma(2, 1.0);
  std::vector<DynamicVectorFactor> factors;
  Vector measurement1{{1.0, 2.0}};
  Vector measurement2{{2.0, 3.0}};
  factors.emplace_back(key, measurement1, model);
  factors.emplace_back(key, measurement2, model);

  auto batch =
      std::make_shared<BatchFactor<DynamicVectorFactor, 2>>(std::move(factors));

  Values values;
  Vector dynamicX{{0.0, 0.0}};
  values.insert(key, dynamicX);

  auto gaussian = batch->linearize(values);
  auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);
  CHECK(jacobian);
  CHECK(jacobian->get_model());
  CHECK(jacobian->get_model()->isUnit());
  LONGS_EQUAL(4, (long)jacobian->rows());  // 2 factors * 2 error dim
  LONGS_EQUAL(1, (long)jacobian->size());
}

/* ************************************************************************* */
// Verifies the projection map constructor uses the compact batch linear factor.
TEST(BatchFactor, Constructor_Projection) {
  // 1. Setup data
  Key poseKey = Symbol('x', 0);
  std::map<Key, Point2> measurements;

  for (int i = 0; i < 10; ++i) {
    measurements[Symbol('l', i)] = Point2(double(i), double(i));
  }

  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);

  // 2. Construct using Map Constructor (ProjectionFactor style)
  // This should automatically detect the signature: (Measurement, Model, Key1,
  // Key2, K) We pass 'sharedK' as the extra argument.
  auto batch = std::make_shared<BatchFactor<ProjectionFactor, 2>>(
      poseKey, measurements, noise, sharedK);

  // 3. Verify
  Values values;
  values.insert(poseKey, Pose3());
  for (int i = 0; i < 10; ++i) {
    values.insert(Symbol('l', i), Point3(0, 0, 10));
  }

  auto gaussian = batch->linearize(values);
  auto compact = std::dynamic_pointer_cast<BatchJacobianFactorBase>(gaussian);
  const JacobianFactor jacobian = denseJacobian(gaussian);

  CHECK(compact);
  LONGS_EQUAL(20, (long)jacobian.rows());
  LONGS_EQUAL(11, (long)jacobian.size());
}

/* ************************************************************************* */
// Verifies non-SFM fixed-dimension factors also use compact batch storage.
TEST(BatchFactor, Constructor_Between) {
  // 1. Setup data
  Key key1 = Symbol('x', 0);
  std::map<Key, Pose2> measurements;

  for (int i = 1; i <= 10; ++i) {
    measurements[Symbol('x', i)] = Pose2(1.0, 0.0, 0.0);
  }

  auto noise = noiseModel::Isotropic::Sigma(3, 0.1);

  // 2. Construct using Map Constructor (Standard style)
  // This should detect: (Key1, Key2, Measurement, Model)
  // BetweenFactor takes (Key, Key, Measurement, Model)
  using Between = BetweenFactor<Pose2>;
  auto batch =
      std::make_shared<BatchFactor<Between, 3>>(key1, measurements, noise);

  // 3. Verify
  Values values;
  for (int i = 0; i <= 10; ++i) {
    values.insert(Symbol('x', i), Pose2(double(i), 0.0, 0.0));
  }

  auto gaussian = batch->linearize(values);
  auto compact = std::dynamic_pointer_cast<BatchJacobianFactorBase>(gaussian);
  const JacobianFactor jacobian = denseJacobian(gaussian);

  CHECK(compact);
  LONGS_EQUAL(30, (long)jacobian.rows());  // 10 factors * 3 dim
  LONGS_EQUAL(11, (long)jacobian.size());  // 11 poses
}

/* ************************************************************************* */
// Verifies dense conversion preserves structural zeros for multi-camera tracks.
TEST(BatchFactor, MultipleCamerasPerPoint) {
  // Create a batch factor with 2 factors:
  // Factor 1: cam1, point
  // Factor 2: cam2, point
  // This mimics the timeSFMBAL scenario where we batch by point.

  using Factor = GeneralSFMFactor<PinholeCamera<Cal3_S2>, Point3>;

  std::vector<Factor> factors;
  Key c1 = Symbol('c', 1);
  Key c2 = Symbol('c', 2);
  Key p0 = Symbol('p', 1);

  Point2 measurement(0, 0);
  auto noise = noiseModel::Unit::Create(2);
  auto K = std::make_shared<Cal3_S2>();

  factors.emplace_back(measurement, noise, c1, p0);
  factors.emplace_back(measurement, noise, c2, p0);

  BatchFactor<Factor, 2> batch(factors);

  // Create values
  Values values;
  values.insert(c1, PinholeCamera<Cal3_S2>(Pose3(), *K));
  values.insert(c2, PinholeCamera<Cal3_S2>(Pose3(Rot3(), Point3(1, 0, 0)), *K));
  values.insert(p0, Point3(0, 0, 10));

  auto gaussian = batch.linearize(values);
  auto compact = std::dynamic_pointer_cast<BatchJacobianFactorBase>(gaussian);
  const JacobianFactor jacobian = denseJacobian(gaussian);
  CHECK(compact);

  // Verify sparsity
  // keys_ should be {c1, c2, p1} (sorted)
  // Indices: c1->0, c2->1, p1->2

  // Jacobian has 4 rows (2 factors * 2 dim)
  // Block 0 (c1): Should be non-zero in rows 0-1, ZERO in rows 2-3
  // Block 1 (c2): Should be ZERO in rows 0-1, non-zero in rows 2-3
  // Block 2 (p1): Should be non-zero in all rows

  Matrix A_c1 = jacobian.getA(jacobian.begin());
  Matrix A_c2 = jacobian.getA(jacobian.begin() + 1);
  Matrix A_p1 = jacobian.getA(jacobian.begin() + 2);

  // Check dimensions
  EXPECT_LONGS_EQUAL(4, A_c1.rows());
  EXPECT_LONGS_EQUAL(11, A_c1.cols());
  EXPECT_LONGS_EQUAL(4, A_c2.rows());
  EXPECT_LONGS_EQUAL(11, A_c2.cols());
  EXPECT_LONGS_EQUAL(4, A_p1.rows());
  EXPECT_LONGS_EQUAL(3, A_p1.cols());

  // Check zeros
  // A_c1 (c1) should be zero in rows 2-3 (Factor 2)
  EXPECT(assert_equal(Matrix::Zero(2, 11), Matrix(A_c1.block(2, 0, 2, 11))));
  // A_c2 (c2) should be zero in rows 0-1 (Factor 1)
  EXPECT(assert_equal(Matrix::Zero(2, 11), Matrix(A_c2.block(0, 0, 2, 11))));
}

/* ************************************************************************* */
// Verifies the explicit Hessian option still returns a HessianFactor.
TEST(BatchFactor, JacobianVsHessian) {
  std::vector<ProjectionFactor> factors;
  Key poseKey = Symbol('x', 0);
  Key l1 = Symbol('l', 1);
  Key l2 = Symbol('l', 2);
  auto noise = noiseModel::Isotropic::Sigma(2, 1.0);
  factors.emplace_back(Point2(0, 0), noise, poseKey, l1, sharedK);
  factors.emplace_back(Point2(1, 1), noise, poseKey, l2, sharedK);
  BatchFactor<ProjectionFactor, 2> batch(factors);

  Values values;
  values.insert(poseKey, Pose3());
  values.insert(l1, Point3(0, 0, 10));
  values.insert(l2, Point3(0, 0, 10));

  const JacobianFactor jacobian = denseJacobian(batch.linearize(values));

  batch.setUseHessianFactor(true);
  auto hessian =
      std::dynamic_pointer_cast<HessianFactor>(batch.linearize(values));
  CHECK(hessian);

  const size_t n = jacobian.size();
  const Vector& b = jacobian.getb();
  double expectedF = b.squaredNorm();
  EXPECT_DOUBLES_EQUAL(expectedF, hessian->constantTerm(), 1e-9);

  for (size_t i = 0; i < n; ++i) {
    auto itI = jacobian.begin() + i;
    const Matrix& Ai = jacobian.getA(itI);
    Matrix expectedDiag = Ai.transpose() * Ai;
    EXPECT(assert_equal(expectedDiag, hessian->info().block(i, i), 1e-9));

    Vector expectedGi = Ai.transpose() * b;
    Matrix actualGiMat = hessian->linearTerm(hessian->begin() + i);
    EXPECT(assert_equal(expectedGi, actualGiMat.col(0), 1e-9));

    for (size_t j = i + 1; j < n; ++j) {
      auto itJ = jacobian.begin() + j;
      const Matrix& Aj = jacobian.getA(itJ);
      Matrix expectedOff = Ai.transpose() * Aj;
      EXPECT(assert_equal(expectedOff, hessian->info().block(i, j), 1e-9));
    }
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
