#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>

#include <CppUnitLite/TestHarness.h>

#include <utility>
#include <vector>

using namespace gtsam::cuda;

TEST(CudaDeviceArray, UploadDownloadRoundTrip) {
  CudaContext context;
  std::vector<double> host = {1.0, 2.0, 3.5, -4.0};

  CudaDeviceArray<double> device;
  device.upload(host, context.stream());

  std::vector<double> actual;
  device.download(&actual, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(host.size(), actual.size());
  for (size_t i = 0; i < host.size(); ++i) {
    DOUBLES_EQUAL(host[i], actual[i], 1e-12);
  }
}

TEST(CudaDeviceArray, ZeroesAndCopiesDeviceData) {
  CudaContext context;
  CudaDeviceArray<double> source;
  source.upload(std::vector<double>{1.0, 2.0, 3.0}, context.stream());

  CudaDeviceArray<double> target;
  target.copyFrom(source, context.stream());

  std::vector<double> actual;
  target.download(&actual, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(3, actual.size());
  DOUBLES_EQUAL(1.0, actual[0], 1e-12);
  DOUBLES_EQUAL(2.0, actual[1], 1e-12);
  DOUBLES_EQUAL(3.0, actual[2], 1e-12);

  target.zero(context.stream());
  target.download(&actual, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(3, actual.size());
  DOUBLES_EQUAL(0.0, actual[0], 1e-12);
  DOUBLES_EQUAL(0.0, actual[1], 1e-12);
  DOUBLES_EQUAL(0.0, actual[2], 1e-12);
}

TEST(CudaDeviceArray, MoveTransfersOwnership) {
  CudaContext context;
  CudaDeviceArray<int> original(3);
  std::vector<int> host = {4, 5, 6};
  original.upload(host, context.stream());

  CudaDeviceArray<int> moved(std::move(original));
  std::vector<int> actual;
  moved.download(&actual, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(0, original.size());
  EXPECT_LONGS_EQUAL(3, moved.size());
  EXPECT_LONGS_EQUAL(4, actual[0]);
  EXPECT_LONGS_EQUAL(5, actual[1]);
  EXPECT_LONGS_EQUAL(6, actual[2]);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
