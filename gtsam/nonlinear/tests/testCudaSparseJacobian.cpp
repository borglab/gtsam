/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaSparseJacobian.cpp
 * @brief   Unit tests for sparse-Jacobian planning and streaming linearization
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/cuda/Context.h>
#include <gtsam/base/cuda/PinnedHostArray.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/cuda/internal/DeviceSparseJacobianNormalEquations.h>
#include <gtsam/nonlinear/cuda/internal/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/internal/JacobianNormalOperator.h>
#include <gtsam/nonlinear/cuda/internal/SparseJacobianPlan.h>
#include <gtsam/nonlinear/cuda/internal/StreamingSparseJacobianLinearizer.h>

#include <Eigen/Cholesky>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#ifndef GTSAM_TEST_EXPECT_SPGEMM_REUSE
#error "GTSAM_TEST_EXPECT_SPGEMM_REUSE must be configured for this test"
#endif

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace sparse_jacobian_fixture {

constexpr Key kPoseKey = 10;
constexpr Key kPointKey = 20;
constexpr Key kFirstStreamingKey = 30;
constexpr Key kSecondStreamingKey = 40;

static_assert(!std::is_copy_constructible_v<PinnedHostArray<double>>);
static_assert(!std::is_copy_assignable_v<PinnedHostArray<double>>);
static_assert(
    std::is_nothrow_move_constructible_v<PinnedHostArray<double>>);
static_assert(std::is_nothrow_move_assignable_v<PinnedHostArray<double>>);
static_assert(std::is_base_of_v<LinearOperator,
                                JacobianNormalOperator>);
static_assert(std::is_base_of_v<Preconditioner,
                                JacobianNormalPreconditioner>);
static_assert(std::is_same_v<
              decltype(std::declval<const DeviceSparseJacobianNormalEquations&>()
                           .linearOperator()),
              const LinearOperator&>);
static_assert(std::is_same_v<
              decltype(std::declval<const DeviceSparseJacobianNormalEquations&>()
                           .preconditioner()),
              const Preconditioner&>);

struct NonzeroValueInitialized {
  int value = 17;
};

static_assert(std::is_trivially_copyable_v<NonzeroValueInitialized>);
static_assert(NonzeroValueInitialized{}.value == 17);

bool isPinnedHostAllocation(const void* pointer) {
  cudaPointerAttributes attributes{};
  GTSAM_CUDA_CHECK(cudaPointerGetAttributes(&attributes, pointer));
  return attributes.type == cudaMemoryTypeHost;
}

class ReversedPointPoseFactor : public NoiseModelFactorN<Point2, Pose2> {
 public:
  using Base = NoiseModelFactorN<Point2, Pose2>;

  ReversedPointPoseFactor(Key pointKey, Key poseKey)
      : Base(noiseModel::Unit::Create(2), pointKey, poseKey) {}

  Vector evaluateError(const Point2& point, const Pose2& pose,
                       OptionalMatrixType Hpoint = OptionalNone,
                       OptionalMatrixType Hpose = OptionalNone) const override {
    if (Hpoint) {
      *Hpoint = Matrix::Identity(2, 2);
    }
    if (Hpose) {
      *Hpose = Matrix::Zero(2, 3);
      Hpose->block<2, 2>(0, 0) = -Matrix::Identity(2, 2);
    }
    return point - pose.translation();
  }
};

class StructuralFactor : public NonlinearFactor {
 public:
  StructuralFactor(const KeyVector& keys, size_t rowCount,
                   bool isSendable = true)
      : NonlinearFactor(keys), rowCount_(rowCount), isSendable_(isSendable) {}

  size_t dim() const override { return rowCount_; }

  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    return {};
  }

  bool sendable() const override { return isSendable_; }

 private:
  size_t rowCount_;
  bool isSendable_;
};

class ReturningGaussianFactor : public NonlinearFactor {
 public:
  ReturningGaussianFactor(const KeyVector& keys, size_t rowCount,
                          std::shared_ptr<GaussianFactor> result)
      : NonlinearFactor(keys),
        rowCount_(rowCount),
        result_(std::move(result)) {}

  size_t dim() const override { return rowCount_; }

  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    return result_;
  }

 private:
  size_t rowCount_;
  std::shared_ptr<GaussianFactor> result_;
};

class RecordingPointFactor : public NonlinearFactor {
 public:
  RecordingPointFactor(Key key, bool isSendable)
      : NonlinearFactor(KeyVector{key}), key_(key), isSendable_(isSendable) {}

  size_t dim() const override { return 1; }

  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    callCount_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(threadIdMutex_);
      threadId_ = std::this_thread::get_id();
    }

    const Matrix A = (Matrix(1, 2) << 1.0, 2.0).finished();
    const Vector b = (Vector(1) << 3.0).finished();
    return std::make_shared<JacobianFactor>(key_, A, b);
  }

  bool sendable() const override { return isSendable_.load(); }

  void setSendable(bool isSendable) { isSendable_.store(isSendable); }

  size_t callCount() const { return callCount_.load(); }

  std::thread::id threadId() const {
    std::lock_guard<std::mutex> lock(threadIdMutex_);
    return threadId_;
  }

 private:
  Key key_;
  std::atomic<bool> isSendable_;
  mutable std::atomic<size_t> callCount_{0};
  mutable std::mutex threadIdMutex_;
  mutable std::thread::id threadId_;
};

class CountingReturningGaussianFactor : public NonlinearFactor {
 public:
  CountingReturningGaussianFactor(const KeyVector& keys, size_t rowCount,
                                  bool isSendable,
                                  std::shared_ptr<GaussianFactor> result)
      : NonlinearFactor(keys),
        rowCount_(rowCount),
        isSendable_(isSendable),
        result_(std::move(result)) {}

  size_t dim() const override { return rowCount_; }

  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    callCount_.fetch_add(1);
    return result_;
  }

  bool sendable() const override { return isSendable_; }

  size_t callCount() const { return callCount_.load(); }

 private:
  size_t rowCount_;
  bool isSendable_;
  std::shared_ptr<GaussianFactor> result_;
  mutable std::atomic<size_t> callCount_{0};
};

class DelayedProfilePointFactor : public NonlinearFactor {
 public:
  DelayedProfilePointFactor(Key key, bool isSendable, size_t rowCount)
      : NonlinearFactor(KeyVector{key}),
        isSendable_(isSendable),
        rowCount_(rowCount),
        result_(std::make_shared<JacobianFactor>(
            key, Matrix::Ones(static_cast<Eigen::Index>(rowCount), 2),
            Vector::Ones(static_cast<Eigen::Index>(rowCount)))) {}

  size_t dim() const override { return rowCount_; }

  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return result_;
  }

  bool sendable() const override { return isSendable_; }

 private:
  bool isSendable_;
  size_t rowCount_;
  std::shared_ptr<GaussianFactor> result_;
};

class InactivePointFactor : public NoiseModelFactorN<Point2> {
 public:
  using Base = NoiseModelFactorN<Point2>;

  explicit InactivePointFactor(Key key)
      : Base(noiseModel::Unit::Create(2), key) {}

  bool active(const Values&) const override { return false; }

  Vector evaluateError(const Point2& point,
                       OptionalMatrixType H = OptionalNone) const override {
    if (H) {
      *H = Matrix::Identity(2, 2);
    }
    return point;
  }
};

struct DenseJacobianReference {
  Matrix jacobian;
  Vector rhs;
};

Matrix DenseFromCsr(const SparseJacobianPlan& plan,
                    const HostSparseJacobian& host) {
  Matrix result = Matrix::Zero(plan.rows(), plan.columns());
  for (int row = 0; row < plan.rows(); ++row) {
    for (int index = plan.rowPointers()[static_cast<size_t>(row)];
         index < plan.rowPointers()[static_cast<size_t>(row + 1)]; ++index) {
      result(row, plan.columnIndices()[static_cast<size_t>(index)]) =
          host.valuesData()[static_cast<size_t>(index)];
    }
  }
  return result;
}

Vector VectorFromHostRhs(const HostSparseJacobian& host) {
  Vector result(static_cast<Eigen::Index>(host.rhsSize()));
  for (size_t row = 0; row < host.rhsSize(); ++row) {
    result(static_cast<Eigen::Index>(row)) = host.rhsData()[row];
  }
  return result;
}

struct DownloadedSparseNormalEquations {
  std::vector<int> rowPointers;
  std::vector<int> columnIndices;
  Matrix hessian;
  Vector rhs;
};

std::pair<std::vector<int>, std::vector<int>> DownloadNormalEquationPattern(
    const DeviceSparseSpdSystem& system, cudaStream_t stream) {
  std::vector<int> rowPointers;
  std::vector<int> columnIndices;
  system.rowPointers().download(&rowPointers, stream);
  system.colIndices().download(&columnIndices, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  return {std::move(rowPointers), std::move(columnIndices)};
}

DownloadedSparseNormalEquations DownloadNormalEquations(
    const DeviceSparseSpdSystem& system, cudaStream_t stream) {
  DownloadedSparseNormalEquations downloaded;
  std::vector<double> values;
  std::vector<double> rhs;
  system.rowPointers().download(&downloaded.rowPointers, stream);
  system.colIndices().download(&downloaded.columnIndices, stream);
  system.values().download(&values, stream);
  system.rhs().download(&rhs, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  if (downloaded.rowPointers.size() != static_cast<size_t>(system.rows()) + 1 ||
      downloaded.columnIndices.size() != values.size() ||
      rhs.size() != static_cast<size_t>(system.rows())) {
    throw std::runtime_error("downloaded normal equations have bad sizes");
  }

  downloaded.hessian = Matrix::Zero(system.rows(), system.rows());
  for (int row = 0; row < system.rows(); ++row) {
    const int begin = downloaded.rowPointers.at(static_cast<size_t>(row));
    const int end = downloaded.rowPointers.at(static_cast<size_t>(row + 1));
    if (begin < 0 || end < begin ||
        end > static_cast<int>(downloaded.columnIndices.size())) {
      throw std::runtime_error("downloaded normal equations have bad CSR");
    }
    for (int index = begin; index < end; ++index) {
      const int column =
          downloaded.columnIndices.at(static_cast<size_t>(index));
      if (column < 0 || column >= system.rows()) {
        throw std::runtime_error(
            "downloaded normal equations have a bad column");
      }
      downloaded.hessian(row, column) = values.at(static_cast<size_t>(index));
    }
  }

  downloaded.rhs = Vector(static_cast<Eigen::Index>(rhs.size()));
  for (size_t row = 0; row < rhs.size(); ++row) {
    downloaded.rhs(static_cast<Eigen::Index>(row)) = rhs[row];
  }
  return downloaded;
}

DenseJacobianReference AssembleDenseReferenceBySlot(
    const NonlinearFactorGraph& graph, const Values& values,
    const KeyInfo& columns, const SparseJacobianPlan& plan) {
  DenseJacobianReference reference{Matrix::Zero(plan.rows(), plan.columns()),
                                   Vector::Zero(plan.rows())};
  const GaussianFactorGraph::shared_ptr linear = graph.linearize(values);
  if (linear->size() != graph.size()) {
    throw std::runtime_error("reference linearization lost graph slots");
  }

  for (size_t factorIndex = 0; factorIndex < linear->size(); ++factorIndex) {
    const GaussianFactor::shared_ptr& gaussian = (*linear)[factorIndex];
    if (!gaussian) {
      continue;
    }

    const auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);
    if (!jacobian) {
      throw std::runtime_error("reference requires JacobianFactor results");
    }

    const auto [localA, localB] = jacobian->jacobian();
    const SparseJacobianFactorWritePlan& factorPlan = plan.factor(factorIndex);
    Eigen::Index localColumn = 0;
    for (Key key : jacobian->keys()) {
      const KeyInfoEntry& column = columns.at(key);
      reference.jacobian.block(
          factorPlan.rowBegin, static_cast<Eigen::Index>(column.start),
          factorPlan.rowCount, static_cast<Eigen::Index>(column.dim)) =
          localA.middleCols(localColumn,
                            static_cast<Eigen::Index>(column.dim));
      localColumn += static_cast<Eigen::Index>(column.dim);
    }
    reference.rhs.segment(factorPlan.rowBegin, factorPlan.rowCount) = localB;
  }
  return reference;
}

Values makeStreamingValues() {
  Values values;
  values.insert(kFirstStreamingKey, Point2(3.0, -2.0));
  values.insert(kSecondStreamingKey, Point2(-1.0, 4.0));
  return values;
}

NonlinearFactorGraph makeStreamingGraph() {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Point2>>(
      kFirstStreamingKey, Point2(0.5, -0.25), noiseModel::Unit::Create(2));
  graph.emplace_shared<PriorFactor<Point2>>(
      kSecondStreamingKey, Point2(1.0, 1.5),
      noiseModel::Diagonal::Sigmas(Vector2(2.0, 0.5)));
  graph.emplace_shared<PriorFactor<Point2>>(
      kFirstStreamingKey, Point2(-2.0, 3.0),
      noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(1.345),
          noiseModel::Diagonal::Sigmas(Vector2(0.75, 1.25))));
  return graph;
}

NonlinearFactorGraph makeFingerprintPlanGraph(bool reverseFactorOrder) {
  NonlinearFactorGraph graph;
  const Key firstKey =
      reverseFactorOrder ? kSecondStreamingKey : kFirstStreamingKey;
  const Key secondKey =
      reverseFactorOrder ? kFirstStreamingKey : kSecondStreamingKey;
  graph.emplace_shared<RecordingPointFactor>(firstKey, false);
  graph.emplace_shared<RecordingPointFactor>(secondKey, false);
  return graph;
}

void SeedFactorRange(const SparseJacobianPlan& plan, size_t factorIndex,
                     double sentinel, HostSparseJacobian* host) {
  const SparseJacobianFactorWritePlan& factor = plan.factor(factorIndex);
  for (int row = factor.rowBegin; row < factor.rowBegin + factor.rowCount;
       ++row) {
    host->rhsData()[static_cast<size_t>(row)] = sentinel;
    for (int index = plan.rowPointers()[static_cast<size_t>(row)];
         index < plan.rowPointers()[static_cast<size_t>(row + 1)]; ++index) {
      host->valuesData()[static_cast<size_t>(index)] = sentinel;
    }
  }
}

bool FactorRangeEquals(const SparseJacobianPlan& plan, size_t factorIndex,
                       double expected, const HostSparseJacobian& host) {
  const SparseJacobianFactorWritePlan& factor = plan.factor(factorIndex);
  for (int row = factor.rowBegin; row < factor.rowBegin + factor.rowCount;
       ++row) {
    if (host.rhsData()[static_cast<size_t>(row)] != expected) {
      return false;
    }
    for (int index = plan.rowPointers()[static_cast<size_t>(row)];
         index < plan.rowPointers()[static_cast<size_t>(row + 1)]; ++index) {
      if (host.valuesData()[static_cast<size_t>(index)] != expected) {
        return false;
      }
    }
  }
  return true;
}

struct StreamingAndPackingFailureResult {
  DirectJacobianStatus streamingStatus;
  bool streamingRangeUnchanged = false;
  DirectJacobianStatus packingStatus;
  bool packingRangeUnchanged = false;
};

StreamingAndPackingFailureResult RunStreamingAndPackingFailure(
    const std::shared_ptr<GaussianFactor>& returned) {
  constexpr size_t kFailingFactorIndex = 1;
  constexpr double kSentinel = 83.0;
  StreamingAndPackingFailureResult result;

  Values values;
  values.insert(kFirstStreamingKey, Point2(0.0, 0.0));
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Point2>>(
      kFirstStreamingKey, Point2(0.0, 0.0), noiseModel::Unit::Create(2));
  graph.push_back(std::make_shared<ReturningGaussianFactor>(
      KeyVector{kFirstStreamingKey}, 2, returned));

  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  host.clear();
  SeedFactorRange(plan, kFailingFactorIndex, kSentinel, &host);

  const StreamingSparseJacobianLinearizer linearizer;
  result.streamingStatus =
      linearizer.linearize(graph, values, columns, plan, &host, nullptr, false);
  result.streamingRangeUnchanged =
      FactorRangeEquals(plan, kFailingFactorIndex, kSentinel, host);

  GaussianFactorGraph linear;
  linear.push_back(std::make_shared<JacobianFactor>(
      kFirstStreamingKey, Matrix::Identity(2, 2), Vector::Zero(2)));
  linear.push_back(returned);
  host.clear();
  SeedFactorRange(plan, kFailingFactorIndex, kSentinel, &host);

  result.packingStatus =
      linearizer.packGaussianFactorGraph(linear, plan, &host);
  result.packingRangeUnchanged =
      FactorRangeEquals(plan, kFailingFactorIndex, kSentinel, host);
  return result;
}

Values makeValues() {
  Values values;
  values.insert(kPoseKey, Pose2());
  values.insert(kPointKey, Point2(4.0, 5.0));
  return values;
}

NonlinearFactorGraph makeGraph() {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  graph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);
  return graph;
}

struct DeviceSystemAddresses {
  const int* rowPointers = nullptr;
  const int* columnIndices = nullptr;
  const double* values = nullptr;
  const double* rhs = nullptr;
};

[[maybe_unused]] DeviceSystemAddresses GetSystemAddresses(
    const DeviceSparseSpdSystem& system) {
  return {system.rowPointers().data(), system.colIndices().data(),
          system.values().data(), system.rhs().data()};
}

[[maybe_unused]] bool SystemAddressesEqual(
    const DeviceSparseSpdSystem& system,
    const DeviceSystemAddresses& expected) {
  return system.rowPointers().data() == expected.rowPointers &&
         system.colIndices().data() == expected.columnIndices &&
         system.values().data() == expected.values &&
         system.rhs().data() == expected.rhs;
}

DenseJacobianReference LinearizeSparseJacobian(
    const NonlinearFactorGraph& graph, const Values& values,
    const KeyInfo& columns, const SparseJacobianPlan& plan,
    HostSparseJacobian* host) {
  host->clear();
  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                    plan, host, nullptr, false);
  if (!status.ok()) {
    throw std::runtime_error("test sparse Jacobian linearization failed");
  }
  return {DenseFromCsr(plan, *host), VectorFromHostRhs(*host)};
}

struct DenseAttemptReference {
  Vector delta;
};

[[maybe_unused]] DenseAttemptReference MakeDenseAttemptReference(
    const DenseJacobianReference& linearized, const Matrix& dampedHessian) {
  DenseAttemptReference expected;
  expected.delta = dampedHessian.llt().solve(linearized.jacobian.transpose() *
                                             linearized.rhs);
  return expected;
}

// Verifies KeyInfo::UsesNaturalKeyOrderAndConvertsDeltas.
TEST(KeyInfo, UsesNaturalKeyOrderAndConvertsDeltas) {
  const Values values = makeValues();
  const KeyInfo layout(values.dims());

  EXPECT_LONGS_EQUAL(5, layout.numCols());
  EXPECT_LONGS_EQUAL(2, layout.size());

  EXPECT_LONGS_EQUAL(kPoseKey, layout.ordering()[0]);
  EXPECT_LONGS_EQUAL(3, layout.at(kPoseKey).dim);
  EXPECT_LONGS_EQUAL(0, layout.at(kPoseKey).start);

  EXPECT_LONGS_EQUAL(kPointKey, layout.ordering()[1]);
  EXPECT_LONGS_EQUAL(2, layout.at(kPointKey).dim);
  EXPECT_LONGS_EQUAL(3, layout.at(kPointKey).start);

  EXPECT_LONGS_EQUAL(0, layout.at(kPoseKey).start);
  EXPECT_LONGS_EQUAL(3, layout.at(kPointKey).start);

  const Vector flatDelta = (Vector(5) << 1.0, 2.0, 3.0, 4.0, 5.0).finished();
  const VectorValues delta = buildVectorValues(flatDelta, layout);
  EXPECT(assert_equal(Vector3(1.0, 2.0, 3.0), delta.at(kPoseKey)));
  EXPECT(assert_equal(Vector2(4.0, 5.0), delta.at(kPointKey)));
}

// Verifies KeyInfo::RejectsUnknownKeyAndWrongDeltaSize.
TEST(KeyInfo, RejectsUnknownKeyAndWrongDeltaSize) {
  const KeyInfo layout(makeValues().dims());

  CHECK_EXCEPTION(layout.at(999), std::out_of_range);
  CHECK_EXCEPTION(buildVectorValues(Vector::Zero(4), layout),
                  std::invalid_argument);
}

// Verifies SparseJacobianPlan::BuildsScalarCsrInGlobalColumnOrder.
TEST(SparseJacobianPlan, BuildsScalarCsrInGlobalColumnOrder) {
  const Values values = makeValues();
  const KeyInfo layout(values.dims());
  const SparseJacobianPlan plan(makeGraph(), layout);

  EXPECT_LONGS_EQUAL(5, plan.rows());
  EXPECT_LONGS_EQUAL(5, plan.columns());
  EXPECT_LONGS_EQUAL(19, plan.nonzeros());

  const std::vector<int> expectedRowPointers = {0, 3, 6, 9, 14, 19};
  const std::vector<int> expectedColumnIndices = {0, 1, 2, 0, 1, 2, 0, 1, 2, 0,
                                                  1, 2, 3, 4, 0, 1, 2, 3, 4};
  CHECK(plan.rowPointers() == expectedRowPointers);
  CHECK(plan.columnIndices() == expectedColumnIndices);

  const SparseJacobianFactorWritePlan& reversed = plan.factor(1);
  EXPECT_LONGS_EQUAL(3, reversed.rowBegin);
  EXPECT_LONGS_EQUAL(2, reversed.rowCount);
  EXPECT_LONGS_EQUAL(5, reversed.nonzerosPerRow);
  EXPECT_LONGS_EQUAL(2, reversed.blocks.size());

  EXPECT_LONGS_EQUAL(kPointKey, reversed.blocks[0].key);
  EXPECT_LONGS_EQUAL(0, reversed.blocks[0].localBlockIndex);
  EXPECT_LONGS_EQUAL(2, reversed.blocks[0].width);
  EXPECT_LONGS_EQUAL(3, reversed.blocks[0].globalColumnBegin);
  EXPECT_LONGS_EQUAL(3, reversed.blocks[0].valueOffsetWithinRow);

  EXPECT_LONGS_EQUAL(kPoseKey, reversed.blocks[1].key);
  EXPECT_LONGS_EQUAL(1, reversed.blocks[1].localBlockIndex);
  EXPECT_LONGS_EQUAL(3, reversed.blocks[1].width);
  EXPECT_LONGS_EQUAL(0, reversed.blocks[1].globalColumnBegin);
  EXPECT_LONGS_EQUAL(0, reversed.blocks[1].valueOffsetWithinRow);

  CHECK_EXCEPTION(plan.factor(2), std::out_of_range);
}

// Verifies SparseJacobianPlan::RejectsMissingAndRepeatedFactorKeys.
TEST(SparseJacobianPlan, RejectsMissingAndRepeatedFactorKeys) {
  const KeyInfo layout(makeValues().dims());

  NonlinearFactorGraph missing;
  missing.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  missing.emplace_shared<ReversedPointPoseFactor>(999, kPoseKey);
  CHECK_EXCEPTION(SparseJacobianPlan(missing, layout), std::invalid_argument);

  NonlinearFactorGraph repeated;
  repeated.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  repeated.push_back(
      std::make_shared<StructuralFactor>(KeyVector{kPointKey, kPointKey}, 2));
  CHECK_EXCEPTION(SparseJacobianPlan(repeated, layout), std::invalid_argument);
}

// Verifies SparseJacobianPlan::RejectsAnUncoveredValuesKey.
TEST(SparseJacobianPlan, RejectsAnUncoveredValuesKey) {
  const KeyInfo layout(makeValues().dims());

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());

  bool exceptionNamesKey = false;
  try {
    const SparseJacobianPlan plan(graph, layout);
    (void)plan;
  } catch (const std::invalid_argument& error) {
    exceptionNamesKey =
        std::string(error.what()).find(DefaultKeyFormatter(kPointKey)) !=
        std::string::npos;
  }
  CHECK(exceptionNamesKey);
}

// Verifies SparseJacobianPlan::ZeroRowFactorDoesNotProvideStructuralCoverage.
TEST(SparseJacobianPlan, ZeroRowFactorDoesNotProvideStructuralCoverage) {
  const KeyInfo layout(makeValues().dims());

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  graph.push_back(std::make_shared<StructuralFactor>(KeyVector{kPointKey}, 0));
  CHECK_EXCEPTION(SparseJacobianPlan(graph, layout), std::invalid_argument);
}

// Verifies SparseJacobianPlan::RejectsFactorCountsOutsideSignedIntCapacity.
TEST(SparseJacobianPlan, RejectsFactorCountsOutsideSignedIntCapacity) {
  Values values;
  values.insert(kPointKey, Point2(4.0, 5.0));
  const KeyInfo layout(values.dims());

  NonlinearFactorGraph rowOverflow;
  rowOverflow.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey},
      static_cast<size_t>(std::numeric_limits<int>::max()) + 1U));
  CHECK_EXCEPTION(SparseJacobianPlan(rowOverflow, layout),
                  std::invalid_argument);

  NonlinearFactorGraph nonzeroOverflow;
  nonzeroOverflow.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey},
      static_cast<size_t>(std::numeric_limits<int>::max())));
  CHECK_EXCEPTION(SparseJacobianPlan(nonzeroOverflow, layout),
                  std::invalid_argument);
}

// Verifies SparseJacobianPlan::PreservesNullSlotsAndLaterRowOffsets.
TEST(SparseJacobianPlan, PreservesNullSlotsAndLaterRowOffsets) {
  const Values values = makeValues();
  const KeyInfo layout(values.dims());

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  graph.push_back(std::shared_ptr<NonlinearFactor>());
  graph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);

  const SparseJacobianPlan plan(graph, layout);
  EXPECT_LONGS_EQUAL(3, plan.factors().size());
  EXPECT_LONGS_EQUAL(3, plan.factor(1).rowBegin);
  EXPECT_LONGS_EQUAL(0, plan.factor(1).rowCount);
  EXPECT_LONGS_EQUAL(0, plan.factor(1).nonzerosPerRow);
  EXPECT_LONGS_EQUAL(0, plan.factor(1).blocks.size());
  EXPECT_LONGS_EQUAL(3, plan.factor(2).rowBegin);
  EXPECT_LONGS_EQUAL(2, plan.factor(2).rowCount);
  EXPECT_LONGS_EQUAL(5, plan.rows());
  EXPECT_LONGS_EQUAL(19, plan.nonzeros());
}

// Verifies SparseJacobianPlan::MatchesOnlyIdenticalStructure.
TEST(SparseJacobianPlan, MatchesOnlyIdenticalStructure) {
  const Values values = makeValues();
  const KeyInfo layout(values.dims());
  const NonlinearFactorGraph graph = makeGraph();
  const SparseJacobianPlan plan(graph, layout);

  const Values identicalValues = makeValues();
  const KeyInfo identicalLayout(identicalValues.dims());
  const NonlinearFactorGraph identicalGraph = makeGraph();
  const SparseJacobianPlan identicalPlan(identicalGraph, identicalLayout);
  CHECK(plan.matches(identicalGraph, identicalLayout));
  CHECK(plan.structuralFingerprint() == identicalPlan.structuralFingerprint());

  Values changedValues;
  changedValues.insert(kPoseKey, Pose2());
  changedValues.insert(kPointKey, Point3(0.0, 0.0, 0.0));
  const KeyInfo changedLayout(changedValues.dims());
  CHECK(!plan.matches(graph, changedLayout));

  NonlinearFactorGraph reorderedGraph;
  reorderedGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  reorderedGraph.push_back(
      std::make_shared<StructuralFactor>(KeyVector{kPoseKey, kPointKey}, 2));
  const SparseJacobianPlan reorderedPlan(reorderedGraph, layout);
  CHECK(!plan.matches(reorderedGraph, layout));
  CHECK(plan.structuralFingerprint() != reorderedPlan.structuralFingerprint());

  NonlinearFactorGraph unsendableGraph;
  unsendableGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  unsendableGraph.push_back(std::make_shared<StructuralFactor>(
      KeyVector{kPointKey, kPoseKey}, 2, false));
  const SparseJacobianPlan unsendablePlan(unsendableGraph, layout);
  CHECK(!plan.matches(unsendableGraph, layout));
  CHECK(plan.structuralFingerprint() != unsendablePlan.structuralFingerprint());

  NonlinearFactorGraph changedRowsGraph;
  changedRowsGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  changedRowsGraph.push_back(
      std::make_shared<StructuralFactor>(KeyVector{kPointKey, kPoseKey}, 3));
  const SparseJacobianPlan changedRowsPlan(changedRowsGraph, layout);
  CHECK(!plan.matches(changedRowsGraph, layout));
  CHECK(plan.structuralFingerprint() !=
        changedRowsPlan.structuralFingerprint());
}

// Verifies SparseJacobianPlan::FingerprintAndMatchesIncludeNullMarkers.
TEST(SparseJacobianPlan, FingerprintAndMatchesIncludeNullMarkers) {
  const KeyInfo layout(makeValues().dims());

  NonlinearFactorGraph nullGraph;
  nullGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  nullGraph.push_back(std::shared_ptr<NonlinearFactor>());
  nullGraph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);
  const SparseJacobianPlan nullPlan(nullGraph, layout);

  NonlinearFactorGraph zeroRowGraph;
  zeroRowGraph.emplace_shared<PriorFactor<Pose2>>(kPoseKey, Pose2());
  zeroRowGraph.push_back(std::make_shared<StructuralFactor>(KeyVector{}, 0));
  zeroRowGraph.emplace_shared<ReversedPointPoseFactor>(kPointKey, kPoseKey);
  const SparseJacobianPlan zeroRowPlan(zeroRowGraph, layout);

  CHECK(!nullPlan.matches(zeroRowGraph, layout));
  CHECK(nullPlan.structuralFingerprint() !=
        zeroRowPlan.structuralFingerprint());
}

// Verifies PinnedHostArray::MoveConstructionTransfersOwnership.
TEST(PinnedHostArray, MoveConstructionTransfersOwnership) {
  PinnedHostArray<int> source(3);
  source.data()[0] = 4;
  source.data()[1] = 5;
  source.data()[2] = 6;
  int* sourceAddress = source.data();
  CHECK(isPinnedHostAllocation(sourceAddress));

  PinnedHostArray<int> destination(std::move(source));

  CHECK(source.data() == nullptr);
  EXPECT_LONGS_EQUAL(0, source.size());
  CHECK(destination.data() == sourceAddress);
  EXPECT_LONGS_EQUAL(3, destination.size());
  EXPECT_LONGS_EQUAL(4, destination.data()[0]);
  EXPECT_LONGS_EQUAL(5, destination.data()[1]);
  EXPECT_LONGS_EQUAL(6, destination.data()[2]);
}

// Verifies PinnedHostArray::ClearValueInitializesElements.
TEST(PinnedHostArray, ClearValueInitializesElements) {
  PinnedHostArray<NonzeroValueInitialized> array(3);
  for (size_t i = 0; i < array.size(); ++i) {
    array.data()[i].value = 0;
  }
  NonzeroValueInitialized* address = array.data();

  array.clear();

  CHECK(array.data() == address);
  for (size_t i = 0; i < array.size(); ++i) {
    EXPECT_LONGS_EQUAL(17, array.data()[i].value);
  }
}

// Verifies PinnedHostArray::MoveAssignmentTransfersOwnership.
TEST(PinnedHostArray, MoveAssignmentTransfersOwnership) {
  PinnedHostArray<int> source(3);
  source.data()[0] = 7;
  source.data()[1] = 8;
  source.data()[2] = 9;
  int* sourceAddress = source.data();

  PinnedHostArray<int> destination(2);
  destination.data()[0] = 1;
  destination.data()[1] = 2;
  destination = std::move(source);

  CHECK(source.data() == nullptr);
  EXPECT_LONGS_EQUAL(0, source.size());
  CHECK(destination.data() == sourceAddress);
  EXPECT_LONGS_EQUAL(3, destination.size());
  EXPECT_LONGS_EQUAL(7, destination.data()[0]);
  EXPECT_LONGS_EQUAL(8, destination.data()[1]);
  EXPECT_LONGS_EQUAL(9, destination.data()[2]);
  CHECK(isPinnedHostAllocation(destination.data()));
}

// Verifies PinnedHostArray::HandlesZeroSizeAndResize.
TEST(PinnedHostArray, HandlesZeroSizeAndResize) {
  PinnedHostArray<double> array;
  CHECK(array.data() == nullptr);
  EXPECT_LONGS_EQUAL(0, array.size());
  array.clear();
  array.resize(0);
  CHECK(array.data() == nullptr);
  EXPECT_LONGS_EQUAL(0, array.size());

  array.resize(4);
  CHECK(array.data() != nullptr);
  EXPECT_LONGS_EQUAL(4, array.size());
  CHECK(isPinnedHostAllocation(array.data()));
  for (size_t i = 0; i < array.size(); ++i) {
    array.data()[i] = static_cast<double>(i + 1);
  }

  double* sameSizeAddress = array.data();
  array.resize(4);
  CHECK(array.data() == sameSizeAddress);
  DOUBLES_EQUAL(4.0, array.data()[3], 0.0);

  array.resize(7);
  CHECK(array.data() != nullptr);
  EXPECT_LONGS_EQUAL(7, array.size());
  CHECK(isPinnedHostAllocation(array.data()));
  array.clear();
  for (size_t i = 0; i < array.size(); ++i) {
    DOUBLES_EQUAL(0.0, array.data()[i], 0.0);
  }

  array.resize(0);
  CHECK(array.data() == nullptr);
  EXPECT_LONGS_EQUAL(0, array.size());

  const PinnedHostArray<double> zeroSize(0);
  CHECK(zeroSize.data() == nullptr);
  EXPECT_LONGS_EQUAL(0, zeroSize.size());
}

// Verifies PinnedHostArray::OverflowLeavesExistingAllocationValid.
TEST(PinnedHostArray, OverflowLeavesExistingAllocationValid) {
  PinnedHostArray<double> array(3);
  array.data()[0] = 2.5;
  double* address = array.data();

  const size_t overflowingSize =
      std::numeric_limits<size_t>::max() / sizeof(double) + 1;
  CHECK_EXCEPTION(array.resize(overflowingSize), std::overflow_error);

  CHECK(array.data() == address);
  EXPECT_LONGS_EQUAL(3, array.size());
  DOUBLES_EQUAL(2.5, array.data()[0], 0.0);
  CHECK(isPinnedHostAllocation(array.data()));
}

// Verifies HostSparseJacobian::ClearsInPlaceWithoutChangingBufferAddresses.
TEST(HostSparseJacobian, ClearsInPlaceWithoutChangingBufferAddresses) {
  const Values values = makeValues();
  const KeyInfo layout(values.dims());
  const SparseJacobianPlan plan(makeGraph(), layout);
  HostSparseJacobian host(plan);

  EXPECT_LONGS_EQUAL(plan.nonzeros(), host.valuesSize());
  EXPECT_LONGS_EQUAL(plan.rows(), host.rhsSize());
  CHECK(isPinnedHostAllocation(host.valuesData()));
  CHECK(isPinnedHostAllocation(host.rhsData()));
  for (size_t i = 0; i < host.valuesSize(); ++i) {
    DOUBLES_EQUAL(0.0, host.valuesData()[i], 0.0);
    host.valuesData()[i] = static_cast<double>(i + 1);
  }
  for (size_t i = 0; i < host.rhsSize(); ++i) {
    DOUBLES_EQUAL(0.0, host.rhsData()[i], 0.0);
    host.rhsData()[i] = -static_cast<double>(i + 1);
  }
  double* valuesAddress = host.valuesData();
  double* rhsAddress = host.rhsData();

  host.clear();

  const HostSparseJacobian& constHost = host;
  EXPECT_LONGS_EQUAL(plan.nonzeros(), constHost.valuesSize());
  EXPECT_LONGS_EQUAL(plan.rows(), constHost.rhsSize());
  CHECK(constHost.valuesData() == valuesAddress);
  CHECK(constHost.rhsData() == rhsAddress);
  for (size_t i = 0; i < constHost.valuesSize(); ++i) {
    DOUBLES_EQUAL(0.0, constHost.valuesData()[i], 0.0);
  }
  for (size_t i = 0; i < constHost.rhsSize(); ++i) {
    DOUBLES_EQUAL(0.0, constHost.rhsData()[i], 0.0);
  }
}

// Verifies StreamingSparseJacobianLinearizer::StreamsUnitDiagonalAndRobustFactorsIntoPlannedCsr.
TEST(StreamingSparseJacobianLinearizer,
     StreamsUnitDiagonalAndRobustFactorsIntoPlannedCsr) {
  const Values values = makeStreamingValues();
  const NonlinearFactorGraph graph = makeStreamingGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  host.clear();

  const DenseJacobianReference expected =
      AssembleDenseReferenceBySlot(graph, values, columns, plan);
  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                    plan, &host);

  CHECK(status.ok());
  EXPECT(assert_equal(expected.jacobian, DenseFromCsr(plan, host), 1e-12));
  EXPECT(assert_equal(expected.rhs, VectorFromHostRhs(host), 1e-12));
}

// Verifies StreamingSparseJacobianLinearizer::WhitensReturnedDiagonalJacobianExactlyOnce.
TEST(StreamingSparseJacobianLinearizer,
     WhitensReturnedDiagonalJacobianExactlyOnce) {
  Values values;
  values.insert(kFirstStreamingKey, Point2(0.0, 0.0));

  const Matrix rawA = (Matrix(2, 2) << 4.0, 8.0, 6.0, 10.0).finished();
  const Vector rawB = (Vector(2) << 8.0, 15.0).finished();
  const auto returned = std::make_shared<JacobianFactor>(
      kFirstStreamingKey, rawA, rawB,
      noiseModel::Diagonal::Sigmas(Vector2(2.0, 5.0)));

  NonlinearFactorGraph graph;
  graph.push_back(std::make_shared<ReturningGaussianFactor>(
      KeyVector{kFirstStreamingKey}, 2, returned));
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  host.clear();

  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                    plan, &host);

  CHECK(status.ok());
  const Matrix expectedA = (Matrix(2, 2) << 2.0, 4.0, 1.2, 2.0).finished();
  const Vector expectedB = (Vector(2) << 4.0, 3.0).finished();
  EXPECT(assert_equal(expectedA, DenseFromCsr(plan, host), 1e-12));
  EXPECT(assert_equal(expectedB, VectorFromHostRhs(host), 1e-12));
  CHECK(returned->get_model() != nullptr);
  EXPECT(assert_equal(rawA, Matrix(returned->getA()), 1e-12));
  EXPECT(assert_equal(rawB, Vector(returned->getb()), 1e-12));
}

// Verifies StreamingSparseJacobianLinearizer::InactiveFactorLeavesItsReservedRowsCleared.
TEST(StreamingSparseJacobianLinearizer,
     InactiveFactorLeavesItsReservedRowsCleared) {
  Values values;
  values.insert(kFirstStreamingKey, Point2(2.0, 3.0));
  NonlinearFactorGraph graph;
  graph.push_back(std::make_shared<InactivePointFactor>(kFirstStreamingKey));
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  SeedFactorRange(plan, 0, 19.0, &host);
  host.clear();

  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                    plan, &host);

  CHECK(status.ok());
  CHECK(FactorRangeEquals(plan, 0, 0.0, host));
}

// Verifies StreamingSparseJacobianLinearizer::RejectsMalformedReturnedJacobiansWithoutPartialWrites.
TEST(StreamingSparseJacobianLinearizer,
     RejectsMalformedReturnedJacobiansWithoutPartialWrites) {
  constexpr double kSentinel = 73.0;
  const auto verifyMalformed =
      [&](const KeyVector& plannedKeys,
          const std::shared_ptr<GaussianFactor>& returned) {
        Values values;
        for (Key key : plannedKeys) {
          if (!values.exists(key)) {
            values.insert(key, Point2(0.0, 0.0));
          }
        }
        NonlinearFactorGraph graph;
        graph.push_back(std::make_shared<ReturningGaussianFactor>(plannedKeys,
                                                                  2, returned));
        const KeyInfo columns(values.dims());
        const SparseJacobianPlan plan(graph, columns);
        HostSparseJacobian host(plan);
        host.clear();
        SeedFactorRange(plan, 0, kSentinel, &host);

        const DirectJacobianStatus status =
            StreamingSparseJacobianLinearizer().linearize(graph, values,
                                                          columns, plan, &host);

        EXPECT(status.failure == DirectJacobianFailure::StructuralMismatch);
        EXPECT(status.factorIndex == 0);
        EXPECT(!status.detail.empty());
        EXPECT(FactorRangeEquals(plan, 0, kSentinel, host));
      };

  const Matrix wrongRowA = Matrix::Identity(1, 2);
  const Vector wrongRowB = Vector::Ones(1);
  verifyMalformed(KeyVector{kFirstStreamingKey},
                  std::make_shared<JacobianFactor>(kFirstStreamingKey,
                                                   wrongRowA, wrongRowB));

  const Matrix squareA = Matrix::Identity(2, 2);
  const Vector squareB = Vector::Ones(2);
  verifyMalformed(
      KeyVector{kFirstStreamingKey, kSecondStreamingKey},
      std::make_shared<JacobianFactor>(kFirstStreamingKey, squareA, squareB));

  verifyMalformed(KeyVector{kFirstStreamingKey, kSecondStreamingKey},
                  std::make_shared<JacobianFactor>(kSecondStreamingKey, squareA,
                                                   kFirstStreamingKey,
                                                   2.0 * squareA, squareB));

  const Matrix wrongWidthA = Matrix::Identity(2, 3);
  verifyMalformed(KeyVector{kFirstStreamingKey, kSecondStreamingKey},
                  std::make_shared<JacobianFactor>(kFirstStreamingKey, squareA,
                                                   kSecondStreamingKey,
                                                   wrongWidthA, squareB));
}

// Verifies StreamingSparseJacobianLinearizer::SchedulesSendableFactorsAndReportsStats.
TEST(StreamingSparseJacobianLinearizer,
     SchedulesSendableFactorsAndReportsStats) {
  constexpr size_t kSendableFactorCount = 512;

  Values values;
  values.insert(kFirstStreamingKey, Point2(0.0, 0.0));
  NonlinearFactorGraph graph;
  std::vector<std::shared_ptr<RecordingPointFactor>> sendableFactors;
  sendableFactors.reserve(kSendableFactorCount);
  for (size_t index = 0; index < kSendableFactorCount; ++index) {
    if (index == kSendableFactorCount / 2) {
      graph.push_back(NonlinearFactor::shared_ptr{});
    }
    auto factor =
        std::make_shared<RecordingPointFactor>(kFirstStreamingKey, true);
    sendableFactors.push_back(factor);
    graph.push_back(factor);
  }
  auto nonSendableFactor =
      std::make_shared<RecordingPointFactor>(kFirstStreamingKey, false);
  graph.push_back(nonSendableFactor);

  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  host.clear();
  const std::thread::id callerThread = std::this_thread::get_id();
  StreamingLinearizationStats stats{7, 9};

  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                    plan, &host, &stats);

  CHECK(status.ok());
  EXPECT_LONGS_EQUAL(kSendableFactorCount, stats.sendableFactors);
  EXPECT_LONGS_EQUAL(1, stats.nonSendableFactors);
  Matrix expectedJacobian = Matrix::Ones(plan.rows(), plan.columns());
  expectedJacobian.col(1).setConstant(2.0);
  const Vector expectedRhs = Vector::Constant(plan.rows(), 3.0);
  EXPECT(assert_equal(expectedJacobian, DenseFromCsr(plan, host), 1e-12));
  EXPECT(assert_equal(expectedRhs, VectorFromHostRhs(host), 1e-12));
  for (const auto& factor : sendableFactors) {
    EXPECT_LONGS_EQUAL(1, factor->callCount());
  }
  EXPECT_LONGS_EQUAL(1, nonSendableFactor->callCount());
  CHECK(nonSendableFactor->threadId() == callerThread);
}

// Verifies StreamingSparseJacobianLinearizer::ProfilesSendableAndNonSendableFactorsWithoutChangingResults.
TEST(StreamingSparseJacobianLinearizer,
     ProfilesSendableAndNonSendableFactorsWithoutChangingResults) {
  constexpr size_t kProfileRows = 2048;

  for (const bool isSendable : {true, false}) {
    Values values;
    values.insert(kFirstStreamingKey, Point2(0.0, 0.0));
    NonlinearFactorGraph graph;
    graph.push_back(std::make_shared<DelayedProfilePointFactor>(
        kFirstStreamingKey, isSendable, kProfileRows));

    const KeyInfo columns(values.dims());
    const SparseJacobianPlan plan(graph, columns);
    HostSparseJacobian unprofiledHost(plan);
    HostSparseJacobian profiledHost(plan);
    unprofiledHost.clear();
    profiledHost.clear();
    StreamingLinearizationStats unprofiledStats{7, 9};
    StreamingLinearizationStats profiledStats{11, 13};
    StreamingLinearizationProfile profile{
        std::numeric_limits<double>::quiet_NaN(), -1.0};
    const StreamingSparseJacobianLinearizer linearizer;

    const DirectJacobianStatus unprofiledStatus = linearizer.linearize(
        graph, values, columns, plan, &unprofiledHost, &unprofiledStats);
    const DirectJacobianStatus profiledStatus =
        linearizer.linearize(graph, values, columns, plan, &profiledHost,
                             &profiledStats, true, &profile);

    CHECK(unprofiledStatus.failure == profiledStatus.failure);
    EXPECT_LONGS_EQUAL(unprofiledStatus.factorIndex,
                       profiledStatus.factorIndex);
    CHECK(unprofiledStatus.detail == profiledStatus.detail);
    EXPECT_LONGS_EQUAL(unprofiledStats.sendableFactors,
                       profiledStats.sendableFactors);
    EXPECT_LONGS_EQUAL(unprofiledStats.nonSendableFactors,
                       profiledStats.nonSendableFactors);
    EXPECT_LONGS_EQUAL(isSendable ? 1 : 0, profiledStats.sendableFactors);
    EXPECT_LONGS_EQUAL(isSendable ? 0 : 1, profiledStats.nonSendableFactors);
    EXPECT(assert_equal(DenseFromCsr(plan, unprofiledHost),
                        DenseFromCsr(plan, profiledHost), 0.0));
    EXPECT(assert_equal(VectorFromHostRhs(unprofiledHost),
                        VectorFromHostRhs(profiledHost), 0.0));
    CHECK(std::isfinite(profile.factorLinearizationCpuSum));
    CHECK(std::isfinite(profile.csrPackingCpuSum));
    CHECK(profile.factorLinearizationCpuSum > 0.0);
    CHECK(profile.csrPackingCpuSum > 0.0);
  }
}

// Verifies StreamingSparseJacobianLinearizer::ResetsProfileBeforeReturningStructuralFailure.
TEST(StreamingSparseJacobianLinearizer,
     ResetsProfileBeforeReturningStructuralFailure) {
  const Values values = makeStreamingValues();
  const NonlinearFactorGraph graph = makeStreamingGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  StreamingLinearizationProfile profile{
      std::numeric_limits<double>::quiet_NaN(), -1.0};

  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(
          graph, values, columns, plan, nullptr, nullptr, true, &profile);

  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);
  DOUBLES_EQUAL(0.0, profile.factorLinearizationCpuSum, 0.0);
  DOUBLES_EQUAL(0.0, profile.csrPackingCpuSum, 0.0);
}

// Verifies StreamingSparseJacobianLinearizer::ReturnsLowestFactorFailureAfterEvaluatingEveryScheduleClass.
TEST(StreamingSparseJacobianLinearizer,
     ReturnsLowestFactorFailureAfterEvaluatingEveryScheduleClass) {
  Values values;
  values.insert(kFirstStreamingKey, Point2(0.0, 0.0));

  auto lowerNonSendable = std::make_shared<CountingReturningGaussianFactor>(
      KeyVector{kFirstStreamingKey}, 1, false,
      std::make_shared<JacobianFactor>(
          kFirstStreamingKey, Matrix::Identity(2, 2), Vector::Zero(2)));
  auto higherSendable = std::make_shared<CountingReturningGaussianFactor>(
      KeyVector{kFirstStreamingKey}, 1, true,
      std::make_shared<HessianFactor>(
          kFirstStreamingKey, Matrix::Identity(2, 2), Vector::Zero(2), 0.0));
  auto higherNonSendable =
      std::make_shared<RecordingPointFactor>(kFirstStreamingKey, false);
  NonlinearFactorGraph graph;
  graph.push_back(lowerNonSendable);
  graph.push_back(higherSendable);
  graph.push_back(higherNonSendable);

  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  host.clear();
  StreamingLinearizationStats stats;
  const std::thread::id callerThread = std::this_thread::get_id();

  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                    plan, &host, &stats);

  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);
  EXPECT_LONGS_EQUAL(0, status.factorIndex);
  EXPECT_LONGS_EQUAL(1, lowerNonSendable->callCount());
  EXPECT_LONGS_EQUAL(1, higherSendable->callCount());
  EXPECT_LONGS_EQUAL(1, higherNonSendable->callCount());
  CHECK(higherNonSendable->threadId() == callerThread);
  EXPECT_LONGS_EQUAL(1, stats.sendableFactors);
  EXPECT_LONGS_EQUAL(2, stats.nonSendableFactors);
}

// Verifies StreamingSparseJacobianLinearizer::RejectsStaleSendabilityBeforeEvaluatingInShallowMode.
TEST(StreamingSparseJacobianLinearizer,
     RejectsStaleSendabilityBeforeEvaluatingInShallowMode) {
  Values values;
  values.insert(kFirstStreamingKey, Point2(0.0, 0.0));
  auto factor =
      std::make_shared<RecordingPointFactor>(kFirstStreamingKey, true);
  NonlinearFactorGraph graph;
  graph.push_back(factor);

  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  host.clear();
  factor->setSendable(false);
  StreamingLinearizationStats stats{7, 9};

  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                    plan, &host, &stats, false);

  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);
  EXPECT_LONGS_EQUAL(0, status.factorIndex);
  EXPECT(!status.detail.empty());
  EXPECT_LONGS_EQUAL(0, factor->callCount());
  EXPECT_LONGS_EQUAL(0, stats.sendableFactors);
  EXPECT_LONGS_EQUAL(1, stats.nonSendableFactors);
}

// Verifies StreamingSparseJacobianLinearizer::RejectsReturnedHessianFactorsWithoutPartialWrites.
TEST(StreamingSparseJacobianLinearizer,
     RejectsReturnedHessianFactorsWithoutPartialWrites) {
  const StreamingAndPackingFailureResult result =
      RunStreamingAndPackingFailure(std::make_shared<HessianFactor>(
          kFirstStreamingKey, Matrix::Identity(2, 2), Vector::Zero(2), 0.0));
  EXPECT(result.streamingStatus.failure ==
         DirectJacobianFailure::UnsupportedGaussianFactor);
  EXPECT_LONGS_EQUAL(1, result.streamingStatus.factorIndex);
  EXPECT(!result.streamingStatus.detail.empty());
  EXPECT(result.streamingRangeUnchanged);
  EXPECT(result.packingStatus.failure ==
         DirectJacobianFailure::UnsupportedGaussianFactor);
  EXPECT_LONGS_EQUAL(1, result.packingStatus.factorIndex);
  EXPECT(!result.packingStatus.detail.empty());
  EXPECT(result.streamingStatus.detail == result.packingStatus.detail);
  EXPECT(result.packingRangeUnchanged);
}

// Verifies StreamingSparseJacobianLinearizer::RejectsReturnedConstrainedJacobiansWithoutPartialWrites.
TEST(StreamingSparseJacobianLinearizer,
     RejectsReturnedConstrainedJacobiansWithoutPartialWrites) {
  const StreamingAndPackingFailureResult result =
      RunStreamingAndPackingFailure(std::make_shared<JacobianFactor>(
          kFirstStreamingKey, Matrix::Identity(2, 2), Vector::Zero(2),
          noiseModel::Constrained::All(2)));
  EXPECT(result.streamingStatus.failure ==
         DirectJacobianFailure::ConstrainedFactor);
  EXPECT_LONGS_EQUAL(1, result.streamingStatus.factorIndex);
  EXPECT(!result.streamingStatus.detail.empty());
  EXPECT(result.streamingRangeUnchanged);
  EXPECT(result.packingStatus.failure ==
         DirectJacobianFailure::ConstrainedFactor);
  EXPECT_LONGS_EQUAL(1, result.packingStatus.factorIndex);
  EXPECT(!result.packingStatus.detail.empty());
  EXPECT(result.streamingStatus.detail == result.packingStatus.detail);
  EXPECT(result.packingRangeUnchanged);
}

// Verifies StreamingSparseJacobianLinearizer::RejectsNonFiniteReturnedJacobiansWithoutPartialWritesInShallowMode.
TEST(StreamingSparseJacobianLinearizer,
     RejectsNonFiniteReturnedJacobiansWithoutPartialWritesInShallowMode) {
  std::vector<StreamingAndPackingFailureResult> results;

  Matrix nonFiniteA = Matrix::Identity(2, 2);
  nonFiniteA(0, 1) = std::numeric_limits<double>::quiet_NaN();
  results.push_back(
      RunStreamingAndPackingFailure(std::make_shared<JacobianFactor>(
          kFirstStreamingKey, nonFiniteA, Vector::Zero(2))));

  nonFiniteA = Matrix::Identity(2, 2);
  nonFiniteA(1, 0) = std::numeric_limits<double>::infinity();
  results.push_back(
      RunStreamingAndPackingFailure(std::make_shared<JacobianFactor>(
          kFirstStreamingKey, nonFiniteA, Vector::Zero(2))));

  Vector nonFiniteB = Vector::Zero(2);
  nonFiniteB(0) = std::numeric_limits<double>::quiet_NaN();
  results.push_back(
      RunStreamingAndPackingFailure(std::make_shared<JacobianFactor>(
          kFirstStreamingKey, Matrix::Identity(2, 2), nonFiniteB)));

  nonFiniteB = Vector::Zero(2);
  nonFiniteB(1) = -std::numeric_limits<double>::infinity();
  results.push_back(
      RunStreamingAndPackingFailure(std::make_shared<JacobianFactor>(
          kFirstStreamingKey, Matrix::Identity(2, 2), nonFiniteB)));

  for (const StreamingAndPackingFailureResult& result : results) {
    EXPECT(result.streamingStatus.failure ==
           DirectJacobianFailure::NonFiniteValues);
    EXPECT_LONGS_EQUAL(1, result.streamingStatus.factorIndex);
    EXPECT(!result.streamingStatus.detail.empty());
    EXPECT(result.streamingRangeUnchanged);
    EXPECT(result.packingStatus.failure ==
           DirectJacobianFailure::NonFiniteValues);
    EXPECT_LONGS_EQUAL(1, result.packingStatus.factorIndex);
    EXPECT(!result.packingStatus.detail.empty());
    EXPECT(result.streamingStatus.detail == result.packingStatus.detail);
    EXPECT(result.packingRangeUnchanged);
  }
}

// Verifies StreamingSparseJacobianLinearizer::RejectsInvalidGlobalInputsBeforeLinearizing.
TEST(StreamingSparseJacobianLinearizer,
     RejectsInvalidGlobalInputsBeforeLinearizing) {
  const Values values = makeStreamingValues();
  const NonlinearFactorGraph graph = makeStreamingGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  const StreamingSparseJacobianLinearizer linearizer;

  DirectJacobianStatus status =
      linearizer.linearize(graph, values, columns, plan, nullptr);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  NonlinearFactorGraph largerPlanGraph = graph;
  largerPlanGraph.emplace_shared<PriorFactor<Point2>>(
      kSecondStreamingKey, Point2(0.0, 0.0), noiseModel::Unit::Create(2));
  const SparseJacobianPlan largerPlan(largerPlanGraph, columns);
  HostSparseJacobian wrongSizeOutput(largerPlan);
  status = linearizer.linearize(graph, values, columns, plan, &wrongSizeOutput);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  NonlinearFactorGraph shorterGraph = graph;
  shorterGraph.resize(graph.size() - 1);
  status = linearizer.linearize(shorterGraph, values, columns, plan, &host,
                                nullptr, false);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  NonlinearFactorGraph longerGraph = graph;
  longerGraph.emplace_shared<PriorFactor<Point2>>(
      kFirstStreamingKey, Point2(0.0, 0.0), noiseModel::Unit::Create(2));
  status = linearizer.linearize(longerGraph, values, columns, plan, &host,
                                nullptr, false);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  Values changedDimensions;
  changedDimensions.insert(kFirstStreamingKey, Point3(0.0, 0.0, 0.0));
  changedDimensions.insert(kSecondStreamingKey, Point2(0.0, 0.0));
  status = linearizer.linearize(graph, changedDimensions, columns, plan, &host);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  NonlinearFactorGraph changedStructure;
  changedStructure.emplace_shared<PriorFactor<Point2>>(
      kSecondStreamingKey, Point2(0.0, 0.0), noiseModel::Unit::Create(2));
  changedStructure.push_back(graph[1]);
  changedStructure.push_back(graph[2]);
  status = linearizer.linearize(changedStructure, values, columns, plan, &host);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);
}

// Verifies StreamingSparseJacobianLinearizer::PacksOnlySlotAlignedGaussianFactorGraphs.
TEST(StreamingSparseJacobianLinearizer,
     PacksOnlySlotAlignedGaussianFactorGraphs) {
  const Values values = makeStreamingValues();
  const NonlinearFactorGraph graph = makeStreamingGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian streamed(plan), packed(plan);
  streamed.clear();
  packed.clear();
  const StreamingSparseJacobianLinearizer linearizer;

  CHECK(linearizer.linearize(graph, values, columns, plan, &streamed).ok());
  const GaussianFactorGraph::shared_ptr linear = graph.linearize(values);
  CHECK(linearizer.packGaussianFactorGraph(*linear, plan, &packed).ok());
  EXPECT(assert_equal(DenseFromCsr(plan, streamed), DenseFromCsr(plan, packed),
                      1e-12));
  EXPECT(assert_equal(VectorFromHostRhs(streamed), VectorFromHostRhs(packed),
                      1e-12));

  DirectJacobianStatus status =
      linearizer.packGaussianFactorGraph(*linear, plan, nullptr);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  NonlinearFactorGraph largerPlanGraph = graph;
  largerPlanGraph.emplace_shared<PriorFactor<Point2>>(
      kFirstStreamingKey, Point2(0.0, 0.0), noiseModel::Unit::Create(2));
  const SparseJacobianPlan largerPlan(largerPlanGraph, columns);
  HostSparseJacobian wrongSizeOutput(largerPlan);
  status = linearizer.packGaussianFactorGraph(*linear, plan, &wrongSizeOutput);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  GaussianFactorGraph shorter = *linear;
  shorter.resize(linear->size() - 1);
  status = linearizer.packGaussianFactorGraph(shorter, plan, &packed);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  GaussianFactorGraph longer = *linear;
  longer.push_back((*linear)[0]);
  status = linearizer.packGaussianFactorGraph(longer, plan, &packed);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);

  GaussianFactorGraph misaligned = *linear;
  std::swap(misaligned[0], misaligned[1]);
  status = linearizer.packGaussianFactorGraph(misaligned, plan, &packed);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);
  CHECK(status.factorIndex == 0);
}

// Verifies DeviceSparseJacobianNormalEquations::RepeatedFormsMatchEigenAndPreserveFinalStorage.
TEST(DeviceSparseJacobianNormalEquations,
     RepeatedFormsMatchEigenAndPreserveFinalStorage) {
#if GTSAM_TEST_EXPECT_SPGEMM_REUSE
  Values firstValues;
  firstValues.insert(kPoseKey, Pose2(0.25, -0.5, 0.15));
  firstValues.insert(kPointKey, Point2(4.0, 5.0));

  Values secondValues;
  secondValues.insert(kPoseKey, Pose2(-0.75, 1.25, -0.3));
  secondValues.insert(kPointKey, Point2(-2.5, 3.75));

  const NonlinearFactorGraph graph = makeGraph();
  const KeyInfo columns(firstValues.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  const StreamingSparseJacobianLinearizer linearizer;
  Context context;
  DeviceSparseJacobianNormalEquations normalEquations;
  normalEquations.initialize(plan, context.stream());

  const DeviceSparseSpdSystem& initialSystem = normalEquations.system();
  const int* const initialRowPointersAddress =
      initialSystem.rowPointers().data();
  const int* const initialColumnIndicesAddress =
      initialSystem.colIndices().data();
  const double* const initialValuesAddress = initialSystem.values().data();
  const double* const initialRhsAddress = initialSystem.rhs().data();
  const auto initialPattern =
      DownloadNormalEquationPattern(initialSystem, context.stream());

  const Values* states[] = {&firstValues, &secondValues};
  for (const Values* values : states) {
    host.clear();
    const DirectJacobianStatus status = linearizer.linearize(
        graph, *values, columns, plan, &host, nullptr, false);
    CHECK(status.ok());
    const Matrix denseJacobian = DenseFromCsr(plan, host);
    const Vector denseRhs = VectorFromHostRhs(host);

    normalEquations.uploadNumerics(host, context.stream());
    normalEquations.formUndampedSystem(context.stream());
    const DeviceSparseSpdSystem& system = normalEquations.system();
    const DownloadedSparseNormalEquations downloaded =
        DownloadNormalEquations(system, context.stream());

    EXPECT(assert_equal(denseJacobian.transpose() * denseJacobian,
                        downloaded.hessian, 1e-10));
    EXPECT(assert_equal(denseJacobian.transpose() * denseRhs, downloaded.rhs,
                        1e-10));
    CHECK(system.rowPointers().data() == initialRowPointersAddress);
    CHECK(system.colIndices().data() == initialColumnIndicesAddress);
    CHECK(system.values().data() == initialValuesAddress);
    CHECK(system.rhs().data() == initialRhsAddress);
    CHECK(downloaded.rowPointers == initialPattern.first);
    CHECK(downloaded.columnIndices == initialPattern.second);
  }
#endif
}

// Verifies DeviceSparseJacobianNormalEquations::SolvesIdentityDampedAttemptsWithoutAccumulationAndEvaluatesModel.
TEST(DeviceSparseJacobianNormalEquations,
     SolvesIdentityDampedAttemptsWithoutAccumulationAndEvaluatesModel) {
#if GTSAM_TEST_EXPECT_SPGEMM_REUSE && GTSAM_ENABLE_CUDSS
  Values values;
  values.insert(kPoseKey, Pose2(0.25, -0.5, 0.15));
  values.insert(kPointKey, Point2(4.0, 5.0));

  const NonlinearFactorGraph graph = makeGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  LinearizeSparseJacobian(graph, values, columns, plan, &host);
  const DenseJacobianReference reference =
      AssembleDenseReferenceBySlot(graph, values, columns, plan);
  const GaussianFactorGraph::shared_ptr linearGraph = graph.linearize(values);
  const Matrix undampedHessian =
      reference.jacobian.transpose() * reference.jacobian;

  Context context;
  DeviceSparseJacobianNormalEquations normalEquations;
  normalEquations.initialize(plan, context.stream());
  normalEquations.uploadNumerics(host, context.stream());
  normalEquations.formUndampedSystem(context.stream());

  const DeviceSparseSpdSystem& initialSystem = normalEquations.system();
  const DeviceSystemAddresses addresses = GetSystemAddresses(initialSystem);
  const auto pattern =
      DownloadNormalEquationPattern(initialSystem, context.stream());

  normalEquations.prepareDamping(false, 0.25, 4.0, context.stream());
  LinearSolverOptions directOptions;
  directOptions.backend = LinearSolverType::Cudss;
  LinearSolverSession directSolver(directOptions);
  directSolver.analyze(normalEquations.mutableSystem(),
                       &normalEquations.deviceDelta(), context.stream());
  EXPECT_LONGS_EQUAL(1, directSolver.stats().analysisCount);

  for (const double lambda : {0.0, 0.1, 1.0}) {
    normalEquations.applyExplicitDamping(lambda, context.stream());
    directSolver.solve(normalEquations.mutableSystem(),
                       &normalEquations.deviceDelta(), context.stream());
    normalEquations.evaluateSolvedDelta(context.stream());
    const DeviceSparseJacobianAttemptResult attempt =
        normalEquations.downloadAttemptResult(context.stream());
    const Matrix expectedHessian =
        undampedHessian +
        lambda * Matrix::Identity(plan.columns(), plan.columns());
    const DenseAttemptReference expected =
        MakeDenseAttemptReference(reference, expectedHessian);
    EXPECT(assert_equal(expected.delta, attempt.delta, 1e-9));
    const double expectedOldError = linearGraph->error(
        buildVectorValues(Vector::Zero(plan.columns()), columns));
    const double expectedNewError =
        linearGraph->error(buildVectorValues(attempt.delta, columns));
    DOUBLES_EQUAL(expectedOldError, attempt.model.oldError, 1e-9);
    DOUBLES_EQUAL(expectedNewError, attempt.model.newError, 1e-9);
    DOUBLES_EQUAL(expectedOldError - expectedNewError, attempt.model.change(),
                  1e-9);

    const DeviceSparseSpdSystem& system = normalEquations.system();
    const DownloadedSparseNormalEquations downloaded =
        DownloadNormalEquations(system, context.stream());
    EXPECT(assert_equal(expectedHessian, downloaded.hessian, 1e-10));
    EXPECT(assert_equal(downloaded.hessian, downloaded.hessian.transpose(),
                        1e-12));
    CHECK(SystemAddressesEqual(system, addresses));
    CHECK(downloaded.rowPointers == pattern.first);
    CHECK(downloaded.columnIndices == pattern.second);
  }
#endif
}

// Verifies DeviceSparseJacobianNormalEquations::SolvesDiagonalDampedAttemptsWithoutAccumulationAndPreservesStorage.
TEST(DeviceSparseJacobianNormalEquations,
     SolvesDiagonalDampedAttemptsWithoutAccumulationAndPreservesStorage) {
#if GTSAM_TEST_EXPECT_SPGEMM_REUSE && GTSAM_ENABLE_CUDSS
  Values values;
  values.insert(kPoseKey, Pose2(-0.75, 1.25, -0.3));
  values.insert(kPointKey, Point2(-2.5, 3.75));

  const NonlinearFactorGraph graph = makeGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  LinearizeSparseJacobian(graph, values, columns, plan, &host);
  const DenseJacobianReference reference =
      AssembleDenseReferenceBySlot(graph, values, columns, plan);
  const GaussianFactorGraph::shared_ptr linearGraph = graph.linearize(values);
  const Matrix undampedHessian =
      reference.jacobian.transpose() * reference.jacobian;
  constexpr double minDiagonal = 1.25;
  constexpr double maxDiagonal = 1.75;
  Vector dampingDiagonal = undampedHessian.diagonal();
  for (Eigen::Index row = 0; row < dampingDiagonal.size(); ++row) {
    dampingDiagonal(row) =
        std::clamp(dampingDiagonal(row), minDiagonal, maxDiagonal);
  }

  Context context;
  DeviceSparseJacobianNormalEquations normalEquations;
  normalEquations.initialize(plan, context.stream());
  normalEquations.uploadNumerics(host, context.stream());
  normalEquations.formUndampedSystem(context.stream());

  const DeviceSparseSpdSystem& initialSystem = normalEquations.system();
  const DeviceSystemAddresses addresses = GetSystemAddresses(initialSystem);
  const auto pattern =
      DownloadNormalEquationPattern(initialSystem, context.stream());

  normalEquations.prepareDamping(true, minDiagonal, maxDiagonal,
                                 context.stream());
  LinearSolverOptions directOptions;
  directOptions.backend = LinearSolverType::Cudss;
  LinearSolverSession directSolver(directOptions);
  directSolver.analyze(normalEquations.mutableSystem(),
                       &normalEquations.deviceDelta(), context.stream());

  for (const double lambda : {0.2, 1.1}) {
    normalEquations.applyExplicitDamping(lambda, context.stream());
    directSolver.solve(normalEquations.mutableSystem(),
                       &normalEquations.deviceDelta(), context.stream());
    normalEquations.evaluateSolvedDelta(context.stream());
    const DeviceSparseJacobianAttemptResult attempt =
        normalEquations.downloadAttemptResult(context.stream());
    const Matrix expectedHessian =
        undampedHessian +
        (lambda * dampingDiagonal).asDiagonal().toDenseMatrix();
    const DenseAttemptReference expected =
        MakeDenseAttemptReference(reference, expectedHessian);
    EXPECT(assert_equal(expected.delta, attempt.delta, 1e-9));
    const double expectedOldError = linearGraph->error(
        buildVectorValues(Vector::Zero(plan.columns()), columns));
    const double expectedNewError =
        linearGraph->error(buildVectorValues(attempt.delta, columns));
    DOUBLES_EQUAL(expectedOldError, attempt.model.oldError, 1e-9);
    DOUBLES_EQUAL(expectedNewError, attempt.model.newError, 1e-9);
    DOUBLES_EQUAL(expectedOldError - expectedNewError, attempt.model.change(),
                  1e-9);

    const DeviceSparseSpdSystem& system = normalEquations.system();
    const DownloadedSparseNormalEquations downloaded =
        DownloadNormalEquations(system, context.stream());
    EXPECT(assert_equal(expectedHessian, downloaded.hessian, 1e-10));
    EXPECT(assert_equal(downloaded.hessian, downloaded.hessian.transpose(),
                        1e-12));
    CHECK(SystemAddressesEqual(system, addresses));
    CHECK(downloaded.rowPointers == pattern.first);
    CHECK(downloaded.columnIndices == pattern.second);
  }
#endif
}

// Verifies DeviceSparseJacobianNormalEquations::CachesAnalysisAcrossNumericalUpdatesAndResetsOnInitialize.
TEST(DeviceSparseJacobianNormalEquations,
     CachesAnalysisAcrossNumericalUpdatesAndResetsOnInitialize) {
#if GTSAM_TEST_EXPECT_SPGEMM_REUSE && GTSAM_ENABLE_CUDSS
  Values firstValues;
  firstValues.insert(kPoseKey, Pose2(0.25, -0.5, 0.15));
  firstValues.insert(kPointKey, Point2(4.0, 5.0));
  Values secondValues;
  secondValues.insert(kPoseKey, Pose2(-0.75, 1.25, -0.3));
  secondValues.insert(kPointKey, Point2(-2.5, 3.75));

  const NonlinearFactorGraph graph = makeGraph();
  const KeyInfo columns(firstValues.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian firstHost(plan);
  HostSparseJacobian secondHost(plan);
  Context context;
  DeviceSparseJacobianNormalEquations normalEquations;
  normalEquations.initialize(plan, context.stream());

  LinearizeSparseJacobian(graph, firstValues, columns, plan, &firstHost);
  normalEquations.uploadNumerics(firstHost, context.stream());
  normalEquations.formUndampedSystem(context.stream());
  normalEquations.prepareDamping(false, 0.0, 10.0, context.stream());
  LinearSolverOptions directOptions;
  directOptions.backend = LinearSolverType::Cudss;
  LinearSolverSession directSolver(directOptions);
  directSolver.analyze(normalEquations.mutableSystem(),
                       &normalEquations.deviceDelta(), context.stream());
  EXPECT_LONGS_EQUAL(1, directSolver.stats().analysisCount);

  const DeviceSystemAddresses addresses =
      GetSystemAddresses(normalEquations.system());
  LinearizeSparseJacobian(graph, secondValues, columns, plan, &secondHost);
  const DenseJacobianReference secondReference =
      AssembleDenseReferenceBySlot(graph, secondValues, columns, plan);
  const GaussianFactorGraph::shared_ptr secondLinearGraph =
      graph.linearize(secondValues);
  normalEquations.uploadNumerics(secondHost, context.stream());
  normalEquations.formUndampedSystem(context.stream());
  normalEquations.prepareDamping(true, 0.5, 5.0, context.stream());
  EXPECT_LONGS_EQUAL(1, directSolver.stats().analysisCount);
  CHECK(SystemAddressesEqual(normalEquations.system(), addresses));

  constexpr double lambda = 0.4;
  Vector diagonal =
      (secondReference.jacobian.transpose() * secondReference.jacobian)
          .diagonal();
  for (Eigen::Index row = 0; row < diagonal.size(); ++row) {
    diagonal(row) = std::clamp(diagonal(row), 0.5, 5.0);
  }
  Matrix secondDampedHessian =
      secondReference.jacobian.transpose() * secondReference.jacobian;
  secondDampedHessian += (lambda * diagonal).asDiagonal().toDenseMatrix();
  normalEquations.applyExplicitDamping(lambda, context.stream());
  directSolver.solve(normalEquations.mutableSystem(),
                     &normalEquations.deviceDelta(), context.stream());
  normalEquations.evaluateSolvedDelta(context.stream());
  const DeviceSparseJacobianAttemptResult secondAttempt =
      normalEquations.downloadAttemptResult(context.stream());
  const DenseAttemptReference secondExpected =
      MakeDenseAttemptReference(secondReference, secondDampedHessian);
  EXPECT(assert_equal(secondExpected.delta, secondAttempt.delta, 1e-9));
  const double expectedOldError = secondLinearGraph->error(
      buildVectorValues(Vector::Zero(plan.columns()), columns));
  const double expectedNewError =
      secondLinearGraph->error(buildVectorValues(secondAttempt.delta, columns));
  DOUBLES_EQUAL(expectedOldError, secondAttempt.model.oldError, 1e-9);
  DOUBLES_EQUAL(expectedNewError, secondAttempt.model.newError, 1e-9);
  DOUBLES_EQUAL(expectedOldError - expectedNewError,
                secondAttempt.model.change(), 1e-9);

  normalEquations.initialize(plan, context.stream());
  normalEquations.uploadNumerics(secondHost, context.stream());
  normalEquations.formUndampedSystem(context.stream());
  normalEquations.prepareDamping(true, 2.0, 2.0, context.stream());
  LinearSolverSession reinitializedSolver(directOptions);
  reinitializedSolver.analyze(normalEquations.mutableSystem(),
                              &normalEquations.deviceDelta(),
                              context.stream());
  EXPECT_LONGS_EQUAL(1, reinitializedSolver.stats().analysisCount);
  normalEquations.applyExplicitDamping(lambda, context.stream());
  reinitializedSolver.solve(normalEquations.mutableSystem(),
                            &normalEquations.deviceDelta(), context.stream());
  normalEquations.evaluateSolvedDelta(context.stream());
  const DeviceSparseJacobianAttemptResult reinitializedAttempt =
      normalEquations.downloadAttemptResult(context.stream());
  const Matrix reinitializedHessian =
      secondReference.jacobian.transpose() * secondReference.jacobian +
      (2.0 * lambda) * Matrix::Identity(plan.columns(), plan.columns());
  const DenseAttemptReference reinitializedExpected =
      MakeDenseAttemptReference(secondReference, reinitializedHessian);
  EXPECT(assert_equal(reinitializedExpected.delta, reinitializedAttempt.delta,
                      1e-9));
  const double reinitializedOldError = secondLinearGraph->error(
      buildVectorValues(Vector::Zero(plan.columns()), columns));
  const double reinitializedNewError = secondLinearGraph->error(
      buildVectorValues(reinitializedAttempt.delta, columns));
  DOUBLES_EQUAL(reinitializedOldError, reinitializedAttempt.model.oldError,
                1e-9);
  DOUBLES_EQUAL(reinitializedNewError, reinitializedAttempt.model.newError,
                1e-9);
#endif
}

// Verifies DeviceSparseJacobianNormalEquations::RejectsUninitializedOutOfOrderAndInvalidAttemptCalls.
TEST(DeviceSparseJacobianNormalEquations,
     RejectsUninitializedOutOfOrderAndInvalidAttemptCalls) {
  DeviceSparseJacobianNormalEquations uninitialized;
  CHECK_EXCEPTION(uninitialized.prepareDamping(false, 0.0, 1.0),
                  std::logic_error);
  CHECK_EXCEPTION(uninitialized.applyExplicitDamping(0.1), std::logic_error);
  CHECK_EXCEPTION(uninitialized.evaluateSolvedDelta(), std::logic_error);
  CHECK_EXCEPTION(uninitialized.downloadAttemptResult(), std::logic_error);

#if GTSAM_TEST_EXPECT_SPGEMM_REUSE
  const Values values = makeValues();
  const NonlinearFactorGraph graph = makeGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  LinearizeSparseJacobian(graph, values, columns, plan, &host);
  Context context;
  DeviceSparseJacobianNormalEquations normalEquations;
  normalEquations.initialize(plan, context.stream());

  CHECK_EXCEPTION(normalEquations.formUndampedSystem(context.stream()),
                  std::logic_error);
  CHECK_EXCEPTION(
      normalEquations.prepareDamping(false, 0.0, 1.0, context.stream()),
      std::logic_error);
  CHECK_EXCEPTION(normalEquations.downloadAttemptResult(context.stream()),
                  std::logic_error);

  normalEquations.uploadNumerics(host, context.stream());
  CHECK_EXCEPTION(
      normalEquations.prepareDamping(false, 0.0, 1.0, context.stream()),
      std::logic_error);
  normalEquations.formUndampedSystem(context.stream());

  CHECK_EXCEPTION(normalEquations.prepareDamping(
                      false, std::numeric_limits<double>::quiet_NaN(), 1.0,
                      context.stream()),
                  std::invalid_argument);
  CHECK_EXCEPTION(normalEquations.prepareDamping(
                      false, std::numeric_limits<double>::infinity(), 1.0,
                      context.stream()),
                  std::invalid_argument);
  CHECK_EXCEPTION(normalEquations.prepareDamping(
                      false, -std::numeric_limits<double>::infinity(), 1.0,
                      context.stream()),
                  std::invalid_argument);
  CHECK_EXCEPTION(normalEquations.prepareDamping(
                      false, 0.0, std::numeric_limits<double>::quiet_NaN(),
                      context.stream()),
                  std::invalid_argument);
  CHECK_EXCEPTION(normalEquations.prepareDamping(
                      false, 0.0, std::numeric_limits<double>::infinity(),
                      context.stream()),
                  std::invalid_argument);
  CHECK_EXCEPTION(normalEquations.prepareDamping(
                      false, 0.0, -std::numeric_limits<double>::infinity(),
                      context.stream()),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      normalEquations.prepareDamping(false, 2.0, 1.0, context.stream()),
      std::invalid_argument);
  CHECK_EXCEPTION(
      normalEquations.prepareDamping(false, -1.0, 1.0, context.stream()),
      std::invalid_argument);
  CHECK_EXCEPTION(
      normalEquations.prepareDamping(true, -2.0, -1.0, context.stream()),
      std::invalid_argument);
  CHECK_EXCEPTION(
      normalEquations.applyExplicitDamping(0.1, context.stream()),
      std::logic_error);

  normalEquations.prepareDamping(false, 0.0, 1.0, context.stream());
  CHECK_EXCEPTION(normalEquations.applyExplicitDamping(-0.1, context.stream()),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      normalEquations.applyExplicitDamping(
          std::numeric_limits<double>::quiet_NaN(), context.stream()),
      std::invalid_argument);
  CHECK_EXCEPTION(
      normalEquations.applyExplicitDamping(
          std::numeric_limits<double>::infinity(), context.stream()),
      std::invalid_argument);
  CHECK_EXCEPTION(normalEquations.downloadAttemptResult(context.stream()),
                  std::logic_error);
#endif
}

// Verifies DeviceSparseJacobianNormalEquations::RejectsWrongStreamForAttemptLifecycle.
TEST(DeviceSparseJacobianNormalEquations,
     RejectsWrongStreamForAttemptLifecycle) {
#if GTSAM_TEST_EXPECT_SPGEMM_REUSE
  const Values values = makeValues();
  const NonlinearFactorGraph graph = makeGraph();
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  LinearizeSparseJacobian(graph, values, columns, plan, &host);

  Context context;
  cudaStream_t otherStream = nullptr;
  GTSAM_CUDA_CHECK(cudaStreamCreate(&otherStream));
  {
    DeviceSparseJacobianNormalEquations normalEquations;
    normalEquations.initialize(plan, context.stream());
    normalEquations.uploadNumerics(host, context.stream());
    normalEquations.formUndampedSystem(context.stream());

    CHECK_EXCEPTION(
        normalEquations.prepareDamping(false, 0.0, 1.0, otherStream),
        std::invalid_argument);
    normalEquations.prepareDamping(false, 0.0, 1.0, context.stream());
    CHECK_EXCEPTION(normalEquations.applyExplicitDamping(0.1, otherStream),
                    std::invalid_argument);
    CHECK_EXCEPTION(normalEquations.evaluateSolvedDelta(otherStream),
                    std::invalid_argument);
    CHECK_EXCEPTION(normalEquations.downloadAttemptResult(otherStream),
                    std::invalid_argument);
  }
  GTSAM_CUDA_CHECK(cudaStreamDestroy(otherStream));
#endif
}

// Verifies StreamingSparseJacobianLinearizer::RejectsMismatchedHostFingerprintBeforeFactorEvaluation.
TEST(StreamingSparseJacobianLinearizer,
     RejectsMismatchedHostFingerprintBeforeFactorEvaluation) {
  const Values values = makeStreamingValues();
  const KeyInfo columns(values.dims());
  const NonlinearFactorGraph firstGraph = makeFingerprintPlanGraph(false);
  const NonlinearFactorGraph secondGraph = makeFingerprintPlanGraph(true);
  const SparseJacobianPlan firstPlan(firstGraph, columns);
  const SparseJacobianPlan secondPlan(secondGraph, columns);
  constexpr double kSentinel = 97.0;
  HostSparseJacobian secondHost(secondPlan);
  SeedFactorRange(firstPlan, 0, kSentinel, &secondHost);
  SeedFactorRange(firstPlan, 1, kSentinel, &secondHost);

  const auto firstFactor =
      std::dynamic_pointer_cast<RecordingPointFactor>(firstGraph[0]);
  const auto secondFactor =
      std::dynamic_pointer_cast<RecordingPointFactor>(firstGraph[1]);
  CHECK(firstFactor != nullptr);
  CHECK(secondFactor != nullptr);

  const StreamingSparseJacobianLinearizer linearizer;
  const DirectJacobianStatus status = linearizer.linearize(
      firstGraph, values, columns, firstPlan, &secondHost, nullptr, false);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);
  CHECK(status.detail.find("structural fingerprint") != std::string::npos);
  CHECK(firstFactor->callCount() == 0);
  CHECK(secondFactor->callCount() == 0);
  CHECK(FactorRangeEquals(firstPlan, 0, kSentinel, secondHost));
  CHECK(FactorRangeEquals(firstPlan, 1, kSentinel, secondHost));
}

// Verifies StreamingSparseJacobianLinearizer::PackingRejectsMismatchedHostFingerprintBeforeWrites.
TEST(StreamingSparseJacobianLinearizer,
     PackingRejectsMismatchedHostFingerprintBeforeWrites) {
  const Values values = makeStreamingValues();
  const KeyInfo columns(values.dims());
  const NonlinearFactorGraph firstGraph = makeFingerprintPlanGraph(false);
  const NonlinearFactorGraph secondGraph = makeFingerprintPlanGraph(true);
  const SparseJacobianPlan firstPlan(firstGraph, columns);
  const SparseJacobianPlan secondPlan(secondGraph, columns);
  constexpr double kSentinel = 97.0;
  HostSparseJacobian secondHost(secondPlan);
  SeedFactorRange(firstPlan, 0, kSentinel, &secondHost);
  SeedFactorRange(firstPlan, 1, kSentinel, &secondHost);

  const Matrix A = (Matrix(1, 2) << 1.0, 2.0).finished();
  const Vector b = (Vector(1) << 3.0).finished();
  GaussianFactorGraph linear;
  linear.push_back(std::make_shared<JacobianFactor>(kFirstStreamingKey, A, b));
  linear.push_back(std::make_shared<JacobianFactor>(kSecondStreamingKey, A, b));

  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().packGaussianFactorGraph(
          linear, firstPlan, &secondHost);
  CHECK(status.failure == DirectJacobianFailure::StructuralMismatch);
  CHECK(status.detail.find("structural fingerprint") != std::string::npos);
  CHECK(FactorRangeEquals(firstPlan, 0, kSentinel, secondHost));
  CHECK(FactorRangeEquals(firstPlan, 1, kSentinel, secondHost));
}

// Verifies DeviceSparseJacobianNormalEquations::RejectsEqualSizedHostWithDifferentStructuralFingerprint.
TEST(DeviceSparseJacobianNormalEquations,
     RejectsEqualSizedHostWithDifferentStructuralFingerprint) {
  const Values values = makeStreamingValues();
  const KeyInfo columns(values.dims());
  const NonlinearFactorGraph firstGraph = makeFingerprintPlanGraph(false);
  const NonlinearFactorGraph secondGraph = makeFingerprintPlanGraph(true);
  const SparseJacobianPlan firstPlan(firstGraph, columns);
  const SparseJacobianPlan secondPlan(secondGraph, columns);

  CHECK(firstPlan.rows() == secondPlan.rows());
  CHECK(firstPlan.columns() == secondPlan.columns());
  CHECK(firstPlan.nonzeros() == secondPlan.nonzeros());
  CHECK(firstPlan.rowPointers() == secondPlan.rowPointers());
  CHECK(firstPlan.columnIndices() != secondPlan.columnIndices());
  CHECK(firstPlan.structuralFingerprint() !=
        secondPlan.structuralFingerprint());

#if GTSAM_TEST_EXPECT_SPGEMM_REUSE
  HostSparseJacobian secondHost(secondPlan);

  Context context;
  DeviceSparseJacobianNormalEquations normalEquations;
  normalEquations.initialize(firstPlan, context.stream());

  bool detailedInvalidArgument = false;
  try {
    normalEquations.uploadNumerics(secondHost, context.stream());
  } catch (const std::invalid_argument& error) {
    detailedInvalidArgument =
        std::string(error.what()).find("structural fingerprint") !=
        std::string::npos;
  }
  CHECK(detailedInvalidArgument);
#endif
}

// Verifies DeviceSparseJacobianNormalEquations::RejectsEmptyRowsColumnsAndNonzeros.
TEST(DeviceSparseJacobianNormalEquations, RejectsEmptyRowsColumnsAndNonzeros) {
  const Values values;
  const NonlinearFactorGraph graph;
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  DeviceSparseJacobianNormalEquations normalEquations;

  bool detailedInvalidArgument = false;
  try {
    normalEquations.initialize(plan);
  } catch (const std::invalid_argument& error) {
    detailedInvalidArgument =
        std::string(error.what())
            .find("positive rows, columns, and nonzeros are required") !=
        std::string::npos;
  }
  CHECK(detailedInvalidArgument);
}

// Verifies DeviceSparseJacobianNormalEquations::ReportsConfiguredSpGemmCapability.
TEST(DeviceSparseJacobianNormalEquations, ReportsConfiguredSpGemmCapability) {
  const DeviceSparseJacobianCapability capability =
      DeviceSparseJacobianNormalEquations::preflightCapability();
#if GTSAM_TEST_EXPECT_SPGEMM_REUSE
  CHECK(capability.supported);
#else
  CHECK(!capability.supported);
  CHECK(!capability.detail.empty());
#endif
}

}  // namespace sparse_jacobian_fixture
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
