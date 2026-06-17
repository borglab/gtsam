#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/nonlinear/cuda/DeviceVariableIndex.h>

#include <CppUnitLite/TestHarness.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

namespace {
constexpr uint32_t kCameraType = 1;
constexpr uint32_t kPointType = 2;
constexpr uint32_t kTinyType = 77;

struct TinyValue {
  double x;
  double y;
};
}  // namespace

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

TEST(DeviceVariableIndex, RejectsWrongType) {
  DeviceVariableIndex index;
  const Key cameraKey = Symbol('c', 1);
  index.add(cameraKey, kCameraType, 0, 9);
  CHECK_EXCEPTION(index.slot(cameraKey, kPointType), std::invalid_argument);
}

TEST(DeviceVariableIndex, RejectsInvalidSlotMetadata) {
  DeviceVariableIndex index;
  const Key cameraKey = Symbol('c', 1);
  CHECK_EXCEPTION(index.add(cameraKey, kCameraType, -1, 9),
                  std::invalid_argument);
  CHECK_EXCEPTION(index.add(cameraKey, kCameraType, 0, 0),
                  std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, index.size());
}

TEST(DeviceValues, AddsAndDownloadsTypedBlock) {
  CudaContext context;
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

TEST(DeviceSparseNormalEquations, UploadsCsrPatternAndRhs) {
  CudaContext context;
  DeviceSparseNormalEquations system;

  std::vector<int> rowPointers = {0, 2, 3};
  std::vector<int> colIndices = {0, 1, 1};
  system.uploadPattern(2, rowPointers, colIndices, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, system.rows());
  EXPECT_LONGS_EQUAL(3, system.nonzeros());
  EXPECT_LONGS_EQUAL(3, system.values().size());
  EXPECT_LONGS_EQUAL(2, system.rhs().size());
}

TEST(DeviceSparseNormalEquations, ClearsValuesAndRhs) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3},
                       std::vector<int>{0, 1, 1}, context.stream());
  system.values().upload(std::vector<double>{1.0, 2.0, 3.0},
                         context.stream());
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

TEST(DeviceSparseNormalEquations, AddsDiagonalDamping) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3},
                       std::vector<int>{0, 1, 1}, context.stream());
  system.values().upload(std::vector<double>{2.0, 0.5, 3.0},
                         context.stream());

  system.addDiagonalDamping(0.25, context.stream());

  std::vector<double> values;
  system.values().download(&values, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(2.25, values[0], 1e-12);
  DOUBLES_EQUAL(0.5, values[1], 1e-12);
  DOUBLES_EQUAL(3.25, values[2], 1e-12);
}

TEST(DeviceSparseNormalEquations, RejectsDampingWithoutDiagonal) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 1, 2},
                       std::vector<int>{1, 1}, context.stream());
  system.values().upload(std::vector<double>{0.5, 3.0}, context.stream());

  CHECK_EXCEPTION(system.addDiagonalDamping(0.25, context.stream()),
                  std::runtime_error);
}

TEST(DeviceSparseNormalEquations, RejectsMalformedCsrBeforeUpload) {
  DeviceSparseNormalEquations system;

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
TEST(CudssLinearSolver, ThrowsWhenCudssDisabled) {
  DeviceSparseNormalEquations system;
  CudaDeviceArray<double> solution;
  CudssLinearSolver solver;

  try {
    solver.solveSpd(system, &solution);
    CHECK(false);
  } catch (const std::runtime_error& e) {
    CHECK(std::string(e.what()).find("requires cuDSS") != std::string::npos);
  }
}
#endif

#if GTSAM_ENABLE_CUDSS
TEST(CudssLinearSolver, SolvesSmallSpdSystem) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3},
                       std::vector<int>{0, 1, 1}, context.stream());
  system.values().upload(std::vector<double>{4.0, 1.0, 3.0},
                         context.stream());
  system.rhs().upload(std::vector<double>{1.0, 2.0}, context.stream());

  CudaDeviceArray<double> solution;
  CudssLinearSolver solver;
  solver.solveSpd(system, &solution, context.stream());

  std::vector<double> actual;
  solution.download(&actual, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-10);
  DOUBLES_EQUAL(7.0 / 11.0, actual[1], 1e-10);
}
#endif

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
