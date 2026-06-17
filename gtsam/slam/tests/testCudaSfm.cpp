#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>
#include <gtsam/sfm/SfmData.h>

#include <CppUnitLite/TestHarness.h>

#include <cmath>
#include <cstddef>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::P;

namespace {
SfmData makeTinySfmData() {
  SfmData data;
  data.cameras.emplace_back(Pose3(Rot3(), Point3(1.0, 2.0, 3.0)),
                            Cal3Bundler(100.0, 0.01, 0.001));
  data.cameras.emplace_back(Pose3(Rot3::RzRyRx(0.01, 0.02, 0.03),
                                  Point3(1.0, 0.0, 0.0)),
                            Cal3Bundler(120.0, 0.02, 0.002));

  SfmTrack track0(Point3(0.0, 0.0, 5.0));
  track0.measurements.emplace_back(0, Point2(10.0, 20.0));
  track0.measurements.emplace_back(1, Point2(11.0, 21.0));
  data.tracks.push_back(track0);

  SfmTrack track1(Point3(1.0, 0.0, 6.0));
  track1.measurements.emplace_back(0, Point2(30.0, 40.0));
  data.tracks.push_back(track1);

  return data;
}

SfmData makeTinyBalData() {
  SfmData data;
  data.cameras.emplace_back(Pose3(), Cal3Bundler(100.0, 0.0, 0.0));
  data.cameras.emplace_back(Pose3(), Cal3Bundler(100.0, 0.0, 0.0));

  SfmTrack track0(Point3(0.0, 0.0, 5.0));
  track0.measurements.emplace_back(0, Point2(1.0, 2.0));
  track0.measurements.emplace_back(1, Point2(3.0, 4.0));
  data.tracks.push_back(track0);

  SfmTrack track1(Point3(1.0, 0.0, 5.0));
  track1.measurements.emplace_back(1, Point2(5.0, 6.0));
  data.tracks.push_back(track1);

  return data;
}

bool Point3Equals(const Point3& expected, const Point3& actual) {
  constexpr double kTolerance = 1e-12;
  return std::abs(expected.x() - actual.x()) <= kTolerance &&
         std::abs(expected.y() - actual.y()) <= kTolerance &&
         std::abs(expected.z() - actual.z()) <= kTolerance;
}

bool CameraEquals(const SfmCamera& expected, const SfmCamera& actual) {
  constexpr double kTolerance = 1e-12;
  const Matrix3 expectedR = expected.pose().rotation().matrix();
  const Matrix3 actualR = actual.pose().rotation().matrix();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (std::abs(expectedR(r, c) - actualR(r, c)) > kTolerance) {
        return false;
      }
    }
  }

  return Point3Equals(expected.pose().translation(), actual.pose().translation()) &&
         std::abs(expected.calibration().fx() - actual.calibration().fx()) <=
             kTolerance &&
         std::abs(expected.calibration().k1() - actual.calibration().k1()) <=
             kTolerance &&
         std::abs(expected.calibration().k2() - actual.calibration().k2()) <=
             kTolerance;
}
}  // namespace

TEST(CudaSfmProjectionBatch, PacksOnlyTracksWithAtLeastTwoMeasurements) {
  const SfmData data = makeTinySfmData();
  CudaContext context;

  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());

  EXPECT_LONGS_EQUAL(2, batch.numObservations());

  std::vector<CudaSfmObservation> observations;
  batch.observations().download(&observations, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, observations.size());
  EXPECT_LONGS_EQUAL(0, observations[0].cameraSlot);
  EXPECT_LONGS_EQUAL(0, observations[0].pointSlot);
  DOUBLES_EQUAL(10.0, observations[0].measuredU, 1e-12);
  DOUBLES_EQUAL(20.0, observations[0].measuredV, 1e-12);

  EXPECT_LONGS_EQUAL(1, observations[1].cameraSlot);
  EXPECT_LONGS_EQUAL(0, observations[1].pointSlot);
  DOUBLES_EQUAL(21.0, observations[1].measuredV, 1e-12);
}

TEST(CudaSfmValues, PacksCamerasInGtsamConvention) {
  const SfmData data = makeTinySfmData();
  CudaContext context;

  DeviceValues values = PackSfmValues(data, context.stream());
  std::vector<DevicePinholeCameraCal3Bundler> cameras;
  std::vector<DevicePoint3> points;
  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);
  cameraBlock.values.download(&cameras, context.stream());
  pointBlock.values.download(&points, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(4, values.index().size());
  EXPECT_LONGS_EQUAL(
      1, values.index().slot(C(1), kDevicePinholeCameraCal3BundlerType));
  EXPECT_LONGS_EQUAL(1, values.index().slot(P(1), kDevicePoint3Type));
  EXPECT_LONGS_EQUAL(kDevicePinholeCameraCal3BundlerTangentDim,
                     cameraBlock.tangentDim);
  EXPECT_LONGS_EQUAL(kDevicePoint3TangentDim, pointBlock.tangentDim);
  EXPECT_LONGS_EQUAL(2, cameraBlock.keys.size());
  EXPECT_LONGS_EQUAL(C(0), cameraBlock.keys[0]);
  EXPECT_LONGS_EQUAL(C(1), cameraBlock.keys[1]);
  EXPECT_LONGS_EQUAL(2, pointBlock.keys.size());
  EXPECT_LONGS_EQUAL(P(0), pointBlock.keys[0]);
  EXPECT_LONGS_EQUAL(P(1), pointBlock.keys[1]);
  EXPECT_LONGS_EQUAL(2, cameras.size());
  EXPECT_LONGS_EQUAL(2, points.size());

  DOUBLES_EQUAL(1.0, cameras[0].R[0], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[1], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[2], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[3], 1e-12);
  DOUBLES_EQUAL(1.0, cameras[0].R[4], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[5], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[6], 1e-12);
  DOUBLES_EQUAL(0.0, cameras[0].R[7], 1e-12);
  DOUBLES_EQUAL(1.0, cameras[0].R[8], 1e-12);
  DOUBLES_EQUAL(1.0, cameras[0].t[0], 1e-12);
  DOUBLES_EQUAL(2.0, cameras[0].t[1], 1e-12);
  DOUBLES_EQUAL(3.0, cameras[0].t[2], 1e-12);
  DOUBLES_EQUAL(100.0, cameras[0].f, 1e-12);
  DOUBLES_EQUAL(0.01, cameras[0].k1, 1e-12);
  DOUBLES_EQUAL(0.001, cameras[0].k2, 1e-12);

  DOUBLES_EQUAL(1.0, points[1].x, 1e-12);
  DOUBLES_EQUAL(0.0, points[1].y, 1e-12);
  DOUBLES_EQUAL(6.0, points[1].z, 1e-12);
}

TEST(CudaSfmValues, DownloadsValuesWithOriginalKeys) {
  const SfmData data = makeTinySfmData();
  CudaContext context;

  DeviceValues values = PackSfmValues(data, context.stream());
  const Values downloaded = DownloadSfmValues(values, context.stream());

  EXPECT_LONGS_EQUAL(4, downloaded.size());
  CHECK(downloaded.exists(C(0)));
  CHECK(downloaded.exists(C(1)));
  CHECK(downloaded.exists(P(0)));
  CHECK(downloaded.exists(P(1)));

  CHECK(CameraEquals(data.camera(0), downloaded.at<SfmCamera>(C(0))));
  CHECK(CameraEquals(data.camera(1), downloaded.at<SfmCamera>(C(1))));
  CHECK(Point3Equals(data.track(0).point3(), downloaded.at<Point3>(P(0))));
  CHECK(Point3Equals(data.track(1).point3(), downloaded.at<Point3>(P(1))));
}

TEST(CudaBalCsrStructure, BuildsUpperTrianglePatternForMeasuredTrack) {
  const SfmData data = makeTinyBalData();
  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);

  EXPECT_LONGS_EQUAL(24, structure.dimension());
  EXPECT_LONGS_EQUAL(2, structure.numCameras());
  EXPECT_LONGS_EQUAL(2, structure.numPoints());

  CHECK(structure.hasEntry(0, 0));
  CHECK(structure.hasEntry(8, 8));
  CHECK(structure.hasEntry(9, 9));
  CHECK(structure.hasEntry(17, 17));
  CHECK(structure.hasEntry(18, 18));
  CHECK(structure.hasEntry(20, 20));
  CHECK(structure.hasEntry(21, 21));
  CHECK(structure.hasEntry(23, 23));

  CHECK(structure.hasEntry(0, 18));
  CHECK(structure.hasEntry(9, 18));
  CHECK(!structure.hasEntry(0, 21));
  CHECK(!structure.hasEntry(9, 21));

  const std::vector<int>& rowPointers = structure.rowPointers();
  const std::vector<int>& colIndices = structure.colIndices();
  EXPECT_LONGS_EQUAL(structure.dimension() + 1, rowPointers.size());
  EXPECT_LONGS_EQUAL(colIndices.size(), rowPointers.back());
  for (size_t row = 0; row + 1 < rowPointers.size(); ++row) {
    CHECK(rowPointers[row] <= rowPointers[row + 1]);
    for (int k = rowPointers[row] + 1; k < rowPointers[row + 1]; ++k) {
      CHECK(colIndices[k - 1] < colIndices[k]);
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
