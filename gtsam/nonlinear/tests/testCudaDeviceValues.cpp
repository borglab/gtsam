/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaDeviceValues.cpp
 * @brief   Unit tests for device-resident Values and retraction
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/cuda/Context.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/cuda/internal/DeviceSparseJacobianNormalEquations.h>
#include <gtsam/nonlinear/cuda/internal/DeviceValues.h>
#include <gtsam/nonlinear/cuda/internal/DeviceVariableIndex.h>
#include <gtsam/nonlinear/cuda/internal/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/internal/SparseJacobianPlan.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace device_values_fixture {
constexpr uint32_t kCameraType = 1;
constexpr uint32_t kPointType = 2;
constexpr uint32_t kTinyType = 77;

struct TinyValue {
  double x;
  double y;
};

#if GTSAM_ENABLE_CUDSS
constexpr Key kProfileKey = 81;

bool IsFiniteNonnegative(double seconds) {
  return std::isfinite(seconds) && seconds >= 0.0;
}

bool AllDeviceProfileTimingsAreFiniteNonnegative(
    const DeviceSparseJacobianProfile& profile) {
  return IsFiniteNonnegative(profile.initializeWall) &&
         IsFiniteNonnegative(profile.patternH2d) &&
         IsFiniteNonnegative(profile.structureSetup) &&
         IsFiniteNonnegative(profile.setupD2h) &&
         IsFiniteNonnegative(profile.numericH2d) &&
         IsFiniteNonnegative(profile.transposeUpdate) &&
         IsFiniteNonnegative(profile.normalJtJ) &&
         IsFiniteNonnegative(profile.normalJtb) &&
         IsFiniteNonnegative(profile.diagonalExtraction) &&
         IsFiniteNonnegative(profile.oldModelError) &&
         IsFiniteNonnegative(profile.dampingPreparation) &&
         IsFiniteNonnegative(profile.dampingApplication) &&
         IsFiniteNonnegative(profile.newModelError) &&
         IsFiniteNonnegative(profile.attemptD2h) &&
         IsFiniteNonnegative(profile.attemptHostBuild);
}

bool AllDeviceProfileTimingsAreZero(
    const DeviceSparseJacobianProfile& profile) {
  return profile.initializeWall == 0.0 && profile.patternH2d == 0.0 &&
         profile.structureSetup == 0.0 && profile.setupD2h == 0.0 &&
         profile.numericH2d == 0.0 && profile.transposeUpdate == 0.0 &&
         profile.normalJtJ == 0.0 && profile.normalJtb == 0.0 &&
         profile.diagonalExtraction == 0.0 && profile.oldModelError == 0.0 &&
         profile.dampingPreparation == 0.0 &&
         profile.dampingApplication == 0.0 &&
         profile.newModelError == 0.0 && profile.attemptD2h == 0.0 &&
         profile.attemptHostBuild == 0.0;
}

struct ProfiledDeviceRun {
  DeviceSparseJacobianProfile profile;
  size_t expectedPatternH2dBytes = 0;
  size_t expectedNumericH2dBytes = 0;
  size_t expectedSetupD2hBytes = 0;
  size_t expectedAttemptD2hBytes = 0;
  size_t analysisCount = 0;
};

ProfiledDeviceRun RunProfiledDevicePipeline(bool collectProfile) {
  Context context;
  Values values;
  values.insert(kProfileKey, Point2(0.0, 0.0));
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Point2>>(kProfileKey, Point2(0.0, 0.0),
                                            noiseModel::Unit::Create(2));
  const KeyInfo columns(values.dims());
  const SparseJacobianPlan plan(graph, columns);
  HostSparseJacobian host(plan);
  if (plan.rows() != 2 || plan.columns() != 2 || plan.nonzeros() != 4) {
    throw std::runtime_error("unexpected profile-test Jacobian shape");
  }
  host.valuesData()[0] = 1.0;
  host.valuesData()[1] = 0.0;
  host.valuesData()[2] = 0.0;
  host.valuesData()[3] = 1.0;
  host.rhsData()[0] = 1.0;
  host.rhsData()[1] = 2.0;

  DeviceSparseJacobianNormalEquations device;
  device.initialize(plan, context.stream(), collectProfile);
  const int hRows = device.system().rows();
  const int hNonzeros = device.system().nonzeros();

  device.uploadNumerics(host, context.stream());
  device.formUndampedSystem(context.stream());
  device.prepareDamping(false, 0.0, 0.0, context.stream());
  LinearSolverSession session(
      LinearSolverOptions{LinearSolverType::Cudss});
  session.analyze(device.mutableSystem(), &device.deviceDelta(),
                  context.stream());

  device.applyExplicitDamping(0.1, context.stream());
  session.solve(device.mutableSystem(), &device.deviceDelta(), context.stream());
  device.evaluateSolvedDelta(context.stream());
  (void)device.downloadAttemptResult(context.stream());
  device.applyExplicitDamping(1.0, context.stream());
  session.solve(device.mutableSystem(), &device.deviceDelta(), context.stream());
  device.evaluateSolvedDelta(context.stream());
  (void)device.downloadAttemptResult(context.stream());

  ProfiledDeviceRun run;
  run.profile = device.profile();
  run.expectedPatternH2dBytes =
      sizeof(int) * (static_cast<size_t>(plan.rows() + 1 + plan.nonzeros()) +
                     static_cast<size_t>(hRows + 1 + hNonzeros + hRows));
  run.expectedNumericH2dBytes =
      sizeof(double) * static_cast<size_t>(plan.nonzeros() + plan.rows());
  run.expectedSetupD2hBytes =
      2 * sizeof(int) * static_cast<size_t>(hRows + 1 + hNonzeros);
  run.expectedAttemptD2hBytes =
      2 * sizeof(double) * static_cast<size_t>(plan.columns() + 2);
  run.analysisCount = session.stats().analysisCount;
  return run;
}

bool ProfileBytesAreExact(const ProfiledDeviceRun& run) {
  return run.profile.patternH2dBytes == run.expectedPatternH2dBytes &&
         run.profile.numericH2dBytes == run.expectedNumericH2dBytes &&
         run.profile.setupD2hBytes == run.expectedSetupD2hBytes &&
         run.profile.attemptD2hBytes == run.expectedAttemptD2hBytes &&
         run.profile.totalH2dBytes() ==
             run.expectedPatternH2dBytes + run.expectedNumericH2dBytes &&
         run.profile.totalD2hBytes() ==
             run.expectedSetupD2hBytes + run.expectedAttemptD2hBytes;
}
#endif
// Verifies DeviceVariableIndex::AddsAndFindsSlots.
TEST(DeviceVariableIndex, AddsAndFindsSlots) {
  DeviceVariableIndex index;
  const Key cameraKey = Symbol('c', 3);
  const Key pointKey = Symbol('p', 7);

  index.add(cameraKey, kCameraType, 3, 9);
  index.add(pointKey, kPointType, 7, 3);

  const DeviceVariableSlot camera = index.at(cameraKey);
  EXPECT_LONGS_EQUAL(kCameraType, camera.typeId);
  EXPECT_LONGS_EQUAL(3, camera.slot);
  EXPECT_LONGS_EQUAL(9, camera.tangentDim);
  EXPECT_LONGS_EQUAL(3, index.slot(cameraKey, kCameraType));

  const DeviceVariableSlot point = index.at(pointKey);
  EXPECT_LONGS_EQUAL(kPointType, point.typeId);
  EXPECT_LONGS_EQUAL(7, point.slot);
  EXPECT_LONGS_EQUAL(3, point.tangentDim);
}

// Verifies DeviceVariableIndex::RejectsWrongType.
TEST(DeviceVariableIndex, RejectsWrongType) {
  DeviceVariableIndex index;
  const Key cameraKey = Symbol('c', 1);
  index.add(cameraKey, kCameraType, 0, 9);
  CHECK_EXCEPTION(index.slot(cameraKey, kPointType), std::invalid_argument);
}

// Verifies DeviceVariableIndex::RejectsInvalidSlotMetadata.
TEST(DeviceVariableIndex, RejectsInvalidSlotMetadata) {
  DeviceVariableIndex index;
  const Key cameraKey = Symbol('c', 1);
  CHECK_EXCEPTION(index.add(cameraKey, kCameraType, -1, 9),
                  std::invalid_argument);
  CHECK_EXCEPTION(index.add(cameraKey, kCameraType, 0, 0),
                  std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, index.size());
}

// Verifies DeviceValues::AddsAndDownloadsTypedBlock.
TEST(DeviceValues, AddsAndDownloadsTypedBlock) {
  Context context;
  DeviceValues values;

  std::vector<Key> keys = {Symbol('t', 0), Symbol('t', 1)};
  std::vector<TinyValue> host = {{1.0, 2.0}, {3.0, 4.0}};

  DeviceValueBlock<TinyValue>& block =
      values.addBlock<TinyValue>(kTinyType, 2, keys, host, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, values.index().size());
  EXPECT_LONGS_EQUAL(2, block.values.size());
  EXPECT_LONGS_EQUAL(4, block.delta.size());
  EXPECT_LONGS_EQUAL(2, block.tangentDim);
  EXPECT_LONGS_EQUAL(1, values.index().slot(Symbol('t', 1), kTinyType));

  std::vector<TinyValue> actual;
  values.block<TinyValue>(kTinyType).values.download(&actual, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(1.0, actual[0].x, 1e-12);
  DOUBLES_EQUAL(4.0, actual[1].y, 1e-12);
}

// Verifies DeviceValues::AddsUninitializedTypedBlock.
TEST(DeviceValues, AddsUninitializedTypedBlock) {
  DeviceValues values;

  std::vector<Key> keys = {Symbol('t', 0), Symbol('t', 1), Symbol('t', 2)};
  DeviceValueBlock<TinyValue>& block =
      values.addUninitializedBlock<TinyValue>(kTinyType, 2, keys);

  EXPECT_LONGS_EQUAL(3, values.index().size());
  EXPECT_LONGS_EQUAL(3, block.values.size());
  EXPECT_LONGS_EQUAL(6, block.delta.size());
  EXPECT_LONGS_EQUAL(2, block.tangentDim);
  EXPECT_LONGS_EQUAL(2, values.index().slot(Symbol('t', 2), kTinyType));
}

// Verifies DeviceValues::RejectsInvalidBlockMetadataBeforeUpload.
TEST(DeviceValues, RejectsInvalidBlockMetadataBeforeUpload) {
  DeviceValues values;

  std::vector<Key> duplicateKeys = {Symbol('t', 0), Symbol('t', 0)};
  std::vector<Key> keys = {Symbol('t', 0), Symbol('t', 1)};
  std::vector<TinyValue> host = {{1.0, 2.0}, {3.0, 4.0}};

  CHECK_EXCEPTION(values.addBlock<TinyValue>(kTinyType, 2, duplicateKeys, host),
                  std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, values.index().size());

  CHECK_EXCEPTION(values.addBlock<TinyValue>(kTinyType, 0, keys, host),
                  std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, values.index().size());
}

// Verifies DeviceSparseSpdSystem::UploadsCsrPatternAndRhs.
TEST(DeviceSparseSpdSystem, UploadsCsrPatternAndRhs) {
  Context context;
  DeviceSparseSpdSystem system;

  std::vector<int> rowPointers = {0, 2, 3};
  std::vector<int> colIndices = {0, 1, 1};
  system.uploadPattern(2, rowPointers, colIndices, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, system.rows());
  EXPECT_LONGS_EQUAL(3, system.nonzeros());
  EXPECT_LONGS_EQUAL(3, system.values().size());
  EXPECT_LONGS_EQUAL(2, system.rhs().size());
}

// Verifies DeviceSparseSpdSystem::ClearsValuesAndRhs.
TEST(DeviceSparseSpdSystem, ClearsValuesAndRhs) {
  Context context;
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3}, std::vector<int>{0, 1, 1},
                       context.stream());
  system.values().upload(std::vector<double>{1.0, 2.0, 3.0}, context.stream());
  system.rhs().upload(std::vector<double>{4.0, 5.0}, context.stream());

  system.zero(context.stream());

  std::vector<double> values;
  std::vector<double> rhs;
  system.values().download(&values, context.stream());
  system.rhs().download(&rhs, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(3, values.size());
  EXPECT_LONGS_EQUAL(2, rhs.size());
  DOUBLES_EQUAL(0.0, values[0], 1e-12);
  DOUBLES_EQUAL(0.0, values[1], 1e-12);
  DOUBLES_EQUAL(0.0, values[2], 1e-12);
  DOUBLES_EQUAL(0.0, rhs[0], 1e-12);
  DOUBLES_EQUAL(0.0, rhs[1], 1e-12);
}

// Verifies DeviceSparseSpdSystem::AddsDiagonalDamping.
TEST(DeviceSparseSpdSystem, AddsDiagonalDamping) {
  Context context;
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3}, std::vector<int>{0, 1, 1},
                       context.stream());
  system.values().upload(std::vector<double>{2.0, 0.5, 3.0}, context.stream());

  system.addDiagonalDamping(0.25, context.stream());

  std::vector<double> values;
  system.values().download(&values, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(2.25, values[0], 1e-12);
  DOUBLES_EQUAL(0.5, values[1], 1e-12);
  DOUBLES_EQUAL(3.25, values[2], 1e-12);
}

// Verifies DeviceSparseSpdSystem::RejectsDampingWithoutDiagonal.
TEST(DeviceSparseSpdSystem, RejectsDampingWithoutDiagonal) {
  Context context;
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, std::vector<int>{0, 1, 2}, std::vector<int>{1, 1},
                       context.stream());
  system.values().upload(std::vector<double>{0.5, 3.0}, context.stream());

  CHECK_EXCEPTION(system.addDiagonalDamping(0.25, context.stream()),
                  std::runtime_error);

  std::vector<double> values;
  system.values().download(&values, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(0.5, values[0], 1e-12);
  DOUBLES_EQUAL(3.0, values[1], 1e-12);
}

// Verifies DeviceSparseSpdSystem::RejectsMalformedCsrBeforeUpload.
TEST(DeviceSparseSpdSystem, RejectsMalformedCsrBeforeUpload) {
  DeviceSparseSpdSystem system;

  std::vector<int> badStart = {1, 2, 3};
  std::vector<int> badStartCols = {0, 1, 1};
  CHECK_EXCEPTION(system.uploadPattern(2, badStart, badStartCols),
                  std::invalid_argument);

  std::vector<int> badOrder = {0, 2, 1};
  std::vector<int> badOrderCols = {0};
  CHECK_EXCEPTION(system.uploadPattern(2, badOrder, badOrderCols),
                  std::invalid_argument);

  std::vector<int> badColumn = {0, 1};
  std::vector<int> badColumnCols = {1};
  CHECK_EXCEPTION(system.uploadPattern(1, badColumn, badColumnCols),
                  std::invalid_argument);
}

#if !GTSAM_ENABLE_CUDSS
// Verifies LinearSolverSession::ThrowsWhenCudssDisabled.
TEST(LinearSolverSession, ThrowsWhenCudssDisabled) {
  DeviceSparseSpdSystem system;
  system.uploadPattern(1, {0, 1}, {0});
  DeviceArray<double> solution;
  LinearSolverSession solver({LinearSolverType::Cudss});

  CHECK_EXCEPTION(solver.analyze(system, &solution), std::runtime_error);
}
#endif

#if GTSAM_ENABLE_CUDSS
// Verifies DeviceSparseJacobianProfile::AccountsExactBytesAndFiniteTimings.
TEST(DeviceSparseJacobianProfile, AccountsExactBytesAndFiniteTimings) {
  const ProfiledDeviceRun run = RunProfiledDevicePipeline(true);

  CHECK(ProfileBytesAreExact(run));
  CHECK(AllDeviceProfileTimingsAreFiniteNonnegative(run.profile));
  CHECK(run.profile.initializeWall > 0.0);
  const double cudaEventTotal =
      run.profile.patternH2d + run.profile.structureSetup +
      run.profile.setupD2h + run.profile.numericH2d +
      run.profile.transposeUpdate + run.profile.normalJtJ +
      run.profile.normalJtb + run.profile.diagonalExtraction +
      run.profile.oldModelError + run.profile.dampingPreparation +
      run.profile.dampingApplication + run.profile.newModelError +
      run.profile.attemptD2h;
  CHECK(cudaEventTotal > 0.0);
  EXPECT_LONGS_EQUAL(1, run.analysisCount);
}

// Verifies DeviceSparseJacobianProfile::DisabledTimingStillAccountsExactBytes.
TEST(DeviceSparseJacobianProfile, DisabledTimingStillAccountsExactBytes) {
  const ProfiledDeviceRun run = RunProfiledDevicePipeline(false);

  CHECK(ProfileBytesAreExact(run));
  CHECK(AllDeviceProfileTimingsAreZero(run.profile));
  EXPECT_LONGS_EQUAL(1, run.analysisCount);
}

// Verifies LinearSolverSession::ReusesAnalysisForChangedValues.
TEST(LinearSolverSession, ReusesCudssAnalysisForChangedValues) {
  Context context;
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3}, std::vector<int>{0, 1, 1},
                       context.stream());
  system.values().upload(std::vector<double>{4.0, 1.0, 3.0}, context.stream());
  system.rhs().upload(std::vector<double>{1.0, 2.0}, context.stream());

  DeviceArray<double> solution;
  LinearSolverSession solver({LinearSolverType::Cudss});
  solver.analyze(system, &solution, context.stream());
  solver.solve(system, &solution, context.stream());

  std::vector<double> actual;
  solution.download(&actual, context.stream());
  context.synchronize();
  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-10);
  DOUBLES_EQUAL(7.0 / 11.0, actual[1], 1e-10);

  system.values().upload(std::vector<double>{2.0, 0.5, 1.5}, context.stream());
  system.rhs().upload(std::vector<double>{1.0, 0.0}, context.stream());
  solver.solve(system, &solution, context.stream());

  solution.download(&actual, context.stream());
  context.synchronize();
  DOUBLES_EQUAL(6.0 / 11.0, actual[0], 1e-10);
  DOUBLES_EQUAL(-2.0 / 11.0, actual[1], 1e-10);
}

// Verifies LinearSolverSession::SurfacesIndefiniteFactorizationInfo.
TEST(LinearSolverSession, SurfacesCudssIndefiniteFactorizationInfo) {
  Context context;
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3}, std::vector<int>{0, 1, 1},
                       context.stream());
  system.values().upload(std::vector<double>{1.0, 2.0, 1.0}, context.stream());
  system.rhs().upload(std::vector<double>{1.0, 1.0}, context.stream());

  DeviceArray<double> solution;
  LinearSolverSession solver({LinearSolverType::Cudss});
  solver.analyze(system, &solution, context.stream());
  try {
    solver.solve(system, &solution, context.stream());
    CHECK(false);
  } catch (const std::runtime_error& e) {
    const std::string message = e.what();
    CHECK(message.find("CUDSS_DATA_INFO") != std::string::npos);
    CHECK(message.find("first non-positive minor") != std::string::npos);
  }
}
#endif

}  // namespace device_values_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
