#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/sfm/SfmData.h>

#include <CppUnitLite/TestHarness.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::P;

namespace {
using BundlerCamera = PinholeCamera<Cal3Bundler>;
using BundlerProjectionFactor = GeneralSFMFactor<BundlerCamera, Point3>;

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

SfmData makeTrueBalLikeData() {
  SfmData data;
  data.cameras.emplace_back(
      Pose3(Rot3::RzRyRx(0.03, -0.02, 0.01), Point3(0.2, -0.1, -0.3)),
      Cal3Bundler(150.0, 0.01, -0.0005));
  data.cameras.emplace_back(
      Pose3(Rot3::RzRyRx(-0.02, 0.015, -0.025), Point3(0.8, 0.1, -0.2)),
      Cal3Bundler(165.0, -0.008, 0.0007));

  const std::vector<Point3> points = {
      Point3(-0.8, -0.4, 4.5), Point3(0.6, -0.2, 5.0),
      Point3(-0.2, 0.7, 5.8), Point3(0.9, 0.5, 6.4)};

  for (const Point3& point : points) {
    SfmTrack track(point);
    for (size_t cameraSlot = 0; cameraSlot < data.numberCameras();
         ++cameraSlot) {
      track.measurements.emplace_back(cameraSlot,
                                      data.camera(cameraSlot).project2(point));
    }
    data.tracks.push_back(track);
  }
  return data;
}

SfmData makePerturbedBalLikeData(const SfmData& measuredData) {
  SfmData data = measuredData;

  Vector delta0(9);
  delta0 << 0.003, -0.002, 0.001, 0.04, -0.03, 0.02, 1.5, 0.0004,
      -0.00003;
  Vector delta1(9);
  delta1 << -0.002, 0.0015, -0.0025, -0.05, 0.02, -0.01, -2.0, -0.0003,
      0.00004;
  data.cameras[0] = measuredData.camera(0).retract(delta0);
  data.cameras[1] = measuredData.camera(1).retract(delta1);

  const std::vector<Point3> pointDeltas = {
      Point3(0.03, -0.02, 0.04), Point3(-0.025, 0.015, -0.03),
      Point3(0.02, 0.025, 0.05), Point3(-0.015, -0.02, -0.04)};
  for (size_t i = 0; i < data.tracks.size(); ++i) {
    const Point3 p = measuredData.track(i).point3();
    const Point3 d = pointDeltas[i];
    data.tracks[i].p = Point3(p.x() + d.x(), p.y() + d.y(), p.z() + d.z());
  }

  return data;
}

SfmData makeBehindCameraData() {
  SfmData data;
  data.cameras.emplace_back(Pose3(), Cal3Bundler(100.0, 0.01, 0.001));
  data.cameras.emplace_back(Pose3(Rot3::RzRyRx(0.01, 0.0, 0.0),
                                  Point3(0.1, 0.0, 0.0)),
                            Cal3Bundler(110.0, -0.01, 0.002));

  SfmTrack track(Point3(0.1, -0.2, -5.0));
  track.measurements.emplace_back(0, Point2(3.0, 4.0));
  track.measurements.emplace_back(1, Point2(5.0, 6.0));
  data.tracks.push_back(track);
  return data;
}

BundlerCamera hostCameraFromDevice(
    const DevicePinholeCameraCal3Bundler& deviceCamera) {
  Matrix3 R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = deviceCamera.R[3 * r + c];
    }
  }
  const Pose3 pose(Rot3(R), Point3(deviceCamera.t[0], deviceCamera.t[1],
                                   deviceCamera.t[2]));
  return BundlerCamera(
      pose, Cal3Bundler(deviceCamera.f, deviceCamera.k1, deviceCamera.k2));
}

Point3 hostPointFromDevice(const DevicePoint3& devicePoint) {
  return Point3(devicePoint.x, devicePoint.y, devicePoint.z);
}

bool DeviceCameraEquals(const DevicePinholeCameraCal3Bundler& expected,
                        const DevicePinholeCameraCal3Bundler& actual,
                        double tolerance) {
  for (int i = 0; i < 9; ++i) {
    if (std::abs(expected.R[i] - actual.R[i]) > tolerance) {
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    if (std::abs(expected.t[i] - actual.t[i]) > tolerance) {
      return false;
    }
  }
  return std::abs(expected.f - actual.f) <= tolerance &&
         std::abs(expected.k1 - actual.k1) <= tolerance &&
         std::abs(expected.k2 - actual.k2) <= tolerance;
}

Vector2 hostResidual(const BundlerCamera& camera, const Point3& point,
                     const CudaSfmObservation& observation) {
  const BundlerProjectionFactor factor(
      Point2(observation.measuredU, observation.measuredV),
      noiseModel::Unit::Create(2), C(observation.cameraSlot),
      P(observation.pointSlot));
  return factor.evaluateError(camera, point, {}, {});
}

struct HostNumericJacobians {
  double camera[18];
  double point[6];
};

HostNumericJacobians hostNumericJacobians(
    const BundlerCamera& camera, const Point3& point,
    const CudaSfmObservation& observation) {
  constexpr double kEps = 1e-6;
  HostNumericJacobians result{};

  for (int col = 0; col < 9; ++col) {
    Vector plusDelta = Vector::Zero(9);
    Vector minusDelta = Vector::Zero(9);
    plusDelta(col) = kEps;
    minusDelta(col) = -kEps;
    const Vector2 plus =
        hostResidual(camera.retract(plusDelta), point, observation);
    const Vector2 minus =
        hostResidual(camera.retract(minusDelta), point, observation);
    for (int row = 0; row < 2; ++row) {
      result.camera[row * 9 + col] = (plus(row) - minus(row)) / (2.0 * kEps);
    }
  }

  for (int col = 0; col < 3; ++col) {
    double plusCoords[3] = {point.x(), point.y(), point.z()};
    double minusCoords[3] = {point.x(), point.y(), point.z()};
    plusCoords[col] += kEps;
    minusCoords[col] -= kEps;
    const Vector2 plus =
        hostResidual(camera, Point3(plusCoords[0], plusCoords[1], plusCoords[2]),
                     observation);
    const Vector2 minus = hostResidual(
        camera, Point3(minusCoords[0], minusCoords[1], minusCoords[2]),
        observation);
    for (int row = 0; row < 2; ++row) {
      result.point[row * 3 + col] = (plus(row) - minus(row)) / (2.0 * kEps);
    }
  }

  return result;
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

TEST(CudaSfmProjectionLinearization,
     MatchesHostResidualsAndNumericJacobians) {
  constexpr double kResidualTolerance = 1e-7;
  constexpr double kJacobianTolerance = 5e-4;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<CudaSfmObservation> observations;
  std::vector<DevicePinholeCameraCal3Bundler> deviceCameras;
  std::vector<DevicePoint3> devicePoints;
  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  batch.observations().download(&observations, context.stream());
  values.block<DevicePinholeCameraCal3Bundler>(
            kDevicePinholeCameraCal3BundlerType)
      .values.download(&deviceCameras, context.stream());
  values.block<DevicePoint3>(kDevicePoint3Type)
      .values.download(&devicePoints, context.stream());
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(8, observations.size());
  EXPECT_LONGS_EQUAL(2 * observations.size(), residuals.size());
  EXPECT_LONGS_EQUAL(18 * observations.size(), cameraJacobians.size());
  EXPECT_LONGS_EQUAL(6 * observations.size(), pointJacobians.size());

  double expectedError = 0.0;
  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const BundlerCamera camera =
        hostCameraFromDevice(deviceCameras[observation.cameraSlot]);
    const Point3 point =
        hostPointFromDevice(devicePoints[observation.pointSlot]);

    const Vector2 expectedResidual = hostResidual(camera, point, observation);
    expectedError += 0.5 * expectedResidual.squaredNorm();
    for (int row = 0; row < 2; ++row) {
      DOUBLES_EQUAL(expectedResidual(row), residuals[2 * i + row],
                    kResidualTolerance);
    }

    const HostNumericJacobians expectedJacobians =
        hostNumericJacobians(camera, point, observation);
    for (int k = 0; k < 18; ++k) {
      DOUBLES_EQUAL(expectedJacobians.camera[k], cameraJacobians[18 * i + k],
                    kJacobianTolerance);
    }
    for (int k = 0; k < 6; ++k) {
      DOUBLES_EQUAL(expectedJacobians.point[k], pointJacobians[6 * i + k],
                    kJacobianTolerance);
    }
  }

  const double actualError =
      ComputeCudaSfmProjectionError(values, batch, context.stream());
  DOUBLES_EQUAL(expectedError, actualError, kResidualTolerance);
}

TEST(CudaSfmProjectionLinearization, ReturnsZerosForCheiralityFailures) {
  const SfmData data = makeBehindCameraData();
  CudaContext context;

  DeviceValues values = PackSfmValues(data, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  std::vector<CudaSfmObservation> observations;
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  batch.observations().download(&observations, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, batch.numObservations());
  for (const CudaSfmObservation& observation : observations) {
    const Vector2 expectedResidual =
        hostResidual(data.camera(observation.cameraSlot),
                     data.track(observation.pointSlot).point3(), observation);
    DOUBLES_EQUAL(0.0, expectedResidual(0), 1e-12);
    DOUBLES_EQUAL(0.0, expectedResidual(1), 1e-12);
  }
  for (double residual : residuals) {
    DOUBLES_EQUAL(0.0, residual, 1e-12);
  }
  for (double jacobian : cameraJacobians) {
    DOUBLES_EQUAL(0.0, jacobian, 1e-12);
  }
  for (double jacobian : pointJacobians) {
    DOUBLES_EQUAL(0.0, jacobian, 1e-12);
  }
}

TEST(CudaSfmProjectionLinearization, RejectsMismatchedValueShapes) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData smallerValuesData = makeTinyBalData();
  CudaContext context;

  DeviceValues values = PackSfmValues(smallerValuesData, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());
  CudaSfmProjectionLinearization linearization;

  CHECK_EXCEPTION(LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                                  context.stream()),
                  std::invalid_argument);
}

TEST(DeviceGeometryKernels, RetractCameraMatchesHostCameraRetract) {
  const SfmData data = makeTrueBalLikeData();
  const DevicePinholeCameraCal3Bundler camera =
      PackPinholeCameraCal3Bundler(data.camera(1));

  const double deltaArray[9] = {0.004, -0.003, 0.002, 0.05, -0.04,
                                0.03,  2.0,    -0.0007, 0.00008};
  Vector delta(9);
  for (int i = 0; i < 9; ++i) {
    delta(i) = deltaArray[i];
  }

  const DevicePinholeCameraCal3Bundler actual =
      RetractCamera(camera, deltaArray);
  const DevicePinholeCameraCal3Bundler expected =
      PackPinholeCameraCal3Bundler(data.camera(1).retract(delta));

  CHECK(DeviceCameraEquals(expected, actual, 1e-10));
}

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
