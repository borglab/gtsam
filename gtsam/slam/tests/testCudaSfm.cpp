#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/sfm/SfmData.h>

#include <CppUnitLite/TestHarness.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::P;

namespace {
using BundlerCamera = PinholeCamera<Cal3Bundler>;
using BundlerProjectionFactor = GeneralSFMFactor<BundlerCamera, Point3>;

class CountingBundlerProjectionFactor : public BundlerProjectionFactor {
 public:
  using BundlerProjectionFactor::BundlerProjectionFactor;

  static int errorCalls;

  double error(const Values& values) const override {
    ++errorCalls;
    return BundlerProjectionFactor::error(values);
  }
};

int CountingBundlerProjectionFactor::errorCalls = 0;

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

  Vector9 delta0{0.003, -0.002, 0.001,  0.04,    -0.03,
                 0.02,  1.5,    0.0004, -0.00003};
  Vector9 delta1{-0.002, 0.0015, -0.0025, -0.05,  0.02,
                 -0.01,  -2.0,   -0.0003, 0.00004};
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

SfmData makeHighDegreeBalLikeData() {
  SfmData measured;
  const Point3 point(0.2, -0.1, 6.0);
  SfmTrack track(point);

  for (size_t i = 0; i < 18; ++i) {
    const double x = 0.03 * static_cast<double>(i);
    const double yaw = 0.001 * static_cast<double>(i);
    measured.cameras.emplace_back(
        Pose3(Rot3::RzRyRx(0.0, 0.0, yaw), Point3(x, 0.0, 0.0)),
        Cal3Bundler(140.0 + static_cast<double>(i), 0.001, -0.00001));
    track.measurements.emplace_back(i,
                                    measured.camera(i).project2(point));
  }
  measured.tracks.push_back(track);

  SfmData perturbed = measured;
  for (size_t i = 0; i < perturbed.numberCameras(); ++i) {
    Vector9 delta{0.0001 * static_cast<double>(i + 1),
                  -0.0002,
                  0.00015,
                  0.002,
                  -0.001,
                  0.0015,
                  0.03,
                  0.00001,
                  -0.000001};
    perturbed.cameras[i] = measured.camera(i).retract(delta);
  }
  perturbed.tracks[0].p = Point3(point.x() + 0.01, point.y() - 0.02,
                                 point.z() + 0.03);
  return perturbed;
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

CudaSfmSqrtInfo2 MakeSqrtInfo(double r00, double r01, double r11) {
  return CudaSfmSqrtInfo2{r00, r01, r11};
}

CudaSfmRobustModel MakeRobustModel(
    CudaSfmRobustModelKind kind, double parameter,
    CudaSfmRobustReweightScheme reweightScheme =
        CudaSfmRobustReweightScheme::Block) {
  return CudaSfmRobustModel{kind, reweightScheme, parameter};
}

bool ConvertedSfmDataEquals(const CudaSfmFactorGraphData& expected,
                            const CudaSfmFactorGraphData& actual) {
  if (expected.cameraKeys.size() != actual.cameraKeys.size() ||
      expected.pointKeys.size() != actual.pointKeys.size()) {
    return false;
  }
  for (size_t i = 0; i < expected.cameraKeys.size(); ++i) {
    if (expected.cameraKeys[i] != actual.cameraKeys[i] ||
        !CameraEquals(expected.data.camera(i), actual.data.camera(i))) {
      return false;
    }
  }
  for (size_t i = 0; i < expected.pointKeys.size(); ++i) {
    if (expected.pointKeys[i] != actual.pointKeys[i] ||
        !Point3Equals(expected.data.track(i).point3(),
                      actual.data.track(i).point3()) ||
        expected.data.track(i).numberMeasurements() !=
            actual.data.track(i).numberMeasurements()) {
      return false;
    }
    for (size_t j = 0; j < expected.data.track(i).numberMeasurements(); ++j) {
      const SfmMeasurement& expectedMeasurement =
          expected.data.track(i).measurement(j);
      const SfmMeasurement& actualMeasurement =
          actual.data.track(i).measurement(j);
      if (expectedMeasurement.first != actualMeasurement.first ||
          std::abs(expectedMeasurement.second(0) -
                   actualMeasurement.second(0)) > 1e-12 ||
          std::abs(expectedMeasurement.second(1) -
                   actualMeasurement.second(1)) > 1e-12) {
        return false;
      }
    }
  }
  if (expected.hasNonUnitNoise != actual.hasNonUnitNoise ||
      expected.hasRobustNoise != actual.hasRobustNoise ||
      expected.sqrtInfoByTrack.size() != actual.sqrtInfoByTrack.size()) {
    return false;
  }
  for (size_t i = 0; i < expected.sqrtInfoByTrack.size(); ++i) {
    if (expected.sqrtInfoByTrack[i].size() !=
        actual.sqrtInfoByTrack[i].size()) {
      return false;
    }
    for (size_t j = 0; j < expected.sqrtInfoByTrack[i].size(); ++j) {
      if (std::abs(expected.sqrtInfoByTrack[i][j].r00 -
                   actual.sqrtInfoByTrack[i][j].r00) > 1e-12 ||
          std::abs(expected.sqrtInfoByTrack[i][j].r01 -
                   actual.sqrtInfoByTrack[i][j].r01) > 1e-12 ||
          std::abs(expected.sqrtInfoByTrack[i][j].r11 -
                   actual.sqrtInfoByTrack[i][j].r11) > 1e-12) {
        return false;
      }
    }
  }
  if (expected.robustModelsByTrack.size() !=
      actual.robustModelsByTrack.size()) {
    return false;
  }
  for (size_t i = 0; i < expected.robustModelsByTrack.size(); ++i) {
    if (expected.robustModelsByTrack[i].size() !=
        actual.robustModelsByTrack[i].size()) {
      return false;
    }
    for (size_t j = 0; j < expected.robustModelsByTrack[i].size(); ++j) {
      if (expected.robustModelsByTrack[i][j].kind !=
              actual.robustModelsByTrack[i][j].kind ||
          expected.robustModelsByTrack[i][j].reweightScheme !=
              actual.robustModelsByTrack[i][j].reweightScheme ||
          std::abs(expected.robustModelsByTrack[i][j].parameter -
                   actual.robustModelsByTrack[i][j].parameter) > 1e-12) {
        return false;
      }
    }
  }
  return true;
}

Vector2 WhitenResidual(const CudaSfmSqrtInfo2& sqrtInfo,
                       const Vector2& residual) {
  return Vector2(sqrtInfo.r00 * residual(0) +
                     sqrtInfo.r01 * residual(1),
                 sqrtInfo.r11 * residual(1));
}

double RobustWeight(const CudaSfmRobustModel& model, double distance) {
  const double absDistance = std::abs(distance);
  switch (model.kind) {
    case CudaSfmRobustModelKind::None:
      return 1.0;
    case CudaSfmRobustModelKind::Huber:
      return absDistance <= model.parameter ? 1.0
                                            : model.parameter / absDistance;
    case CudaSfmRobustModelKind::Tukey: {
      if (absDistance > model.parameter) {
        return 0.0;
      }
      const double normalized = distance / model.parameter;
      const double oneMinusSquared = 1.0 - normalized * normalized;
      return oneMinusSquared * oneMinusSquared;
    }
  }
  return 1.0;
}

double RobustLoss(const CudaSfmRobustModel& model, double distance) {
  const double absDistance = std::abs(distance);
  switch (model.kind) {
    case CudaSfmRobustModelKind::None:
      return 0.5 * distance * distance;
    case CudaSfmRobustModelKind::Huber:
      if (absDistance <= model.parameter) {
        return 0.5 * distance * distance;
      }
      return model.parameter * (absDistance - 0.5 * model.parameter);
    case CudaSfmRobustModelKind::Tukey: {
      const double c2 = model.parameter * model.parameter;
      if (absDistance > model.parameter) {
        return c2 / 6.0;
      }
      const double normalized = distance / model.parameter;
      const double oneMinusSquared = 1.0 - normalized * normalized;
      return c2 *
             (1.0 - oneMinusSquared * oneMinusSquared * oneMinusSquared) /
             6.0;
    }
  }
  return 0.5 * distance * distance;
}

void RobustRowScales(const CudaSfmRobustModel& model,
                     const Vector2& whitenedResidual, double* row0Scale,
                     double* row1Scale) {
  if (model.reweightScheme == CudaSfmRobustReweightScheme::Scalar) {
    *row0Scale = std::sqrt(RobustWeight(model, whitenedResidual(0)));
    *row1Scale = std::sqrt(RobustWeight(model, whitenedResidual(1)));
  } else {
    const double scale = std::sqrt(RobustWeight(
        model, std::sqrt(whitenedResidual.squaredNorm())));
    *row0Scale = scale;
    *row1Scale = scale;
  }
}

std::vector<std::vector<CudaSfmSqrtInfo2>> MakeMixedSqrtInfoByTrack(
    const SfmData& data) {
  const CudaSfmSqrtInfo2 models[] = {
      MakeSqrtInfo(2.0, 0.0, 2.0), MakeSqrtInfo(4.0, 0.0, 2.0),
      MakeSqrtInfo(3.0, 0.25, 4.0)};
  std::vector<std::vector<CudaSfmSqrtInfo2>> sqrtInfoByTrack(
      data.numberTracks());
  size_t index = 0;
  for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
    const SfmTrack& track = data.track(pointSlot);
    sqrtInfoByTrack[pointSlot].reserve(track.numberMeasurements());
    for (size_t measurementSlot = 0;
         measurementSlot < track.numberMeasurements(); ++measurementSlot) {
      sqrtInfoByTrack[pointSlot].push_back(models[index % 3]);
      ++index;
    }
  }
  return sqrtInfoByTrack;
}

std::vector<std::vector<CudaSfmRobustModel>> MakeMixedRobustModelsByTrack(
    const SfmData& data) {
  const CudaSfmRobustModel models[] = {
      MakeRobustModel(CudaSfmRobustModelKind::Huber, 0.25),
      MakeRobustModel(CudaSfmRobustModelKind::Huber, 0.20,
                      CudaSfmRobustReweightScheme::Scalar),
      MakeRobustModel(CudaSfmRobustModelKind::Tukey, 0.75)};
  std::vector<std::vector<CudaSfmRobustModel>> robustModelsByTrack(
      data.numberTracks());
  size_t index = 0;
  for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
    const SfmTrack& track = data.track(pointSlot);
    robustModelsByTrack[pointSlot].reserve(track.numberMeasurements());
    for (size_t measurementSlot = 0;
         measurementSlot < track.numberMeasurements(); ++measurementSlot) {
      robustModelsByTrack[pointSlot].push_back(models[index % 3]);
      ++index;
    }
  }
  return robustModelsByTrack;
}
}  // namespace

TEST(CudaSfmLevenbergMarquardtParams, MapsLinearSolverStringAliases) {
  CudaSfmLevenbergMarquardtParams params;

  CHECK(params.linearSolver == CudaSfmLinearSolverType::DenseSchur);
  CHECK(params.getLinearSolver() == "dense-schur");

  params.setLinearSolver("cudss-full-normal");
  CHECK(params.linearSolver == CudaSfmLinearSolverType::CudssFullNormal);
  CHECK(params.getLinearSolver() == "cudss-full-normal");

  params.setLinearSolver("DENSE_SCHUR");
  CHECK(params.linearSolver == CudaSfmLinearSolverType::DenseSchur);

  params.setLinearSolver("CUDSS_FULL_NORMAL");
  CHECK(params.linearSolver == CudaSfmLinearSolverType::CudssFullNormal);

  CHECK_EXCEPTION(params.setLinearSolver("not-a-solver"),
                  std::invalid_argument);
  CHECK(params.linearSolver == CudaSfmLinearSolverType::CudssFullNormal);
}

TEST(CudaSfmLevenbergMarquardtParams, ProvidesLmDefaults) {
  const CudaSfmLevenbergMarquardtParams legacy =
      CudaSfmLevenbergMarquardtParams::LegacyDefaults();
  CHECK(legacy.maxIterations == 100);
  DOUBLES_EQUAL(1e-5, legacy.lambdaInitial, 0.0);
  DOUBLES_EQUAL(10.0, legacy.lambdaFactor, 0.0);
  CHECK(!legacy.diagonalDamping);
  CHECK(!legacy.enableDetailedProfiling);

  const CudaSfmLevenbergMarquardtParams ceres =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  CHECK(ceres.maxIterations == 50);
  DOUBLES_EQUAL(1e-4, ceres.lambdaInitial, 0.0);
  DOUBLES_EQUAL(2.0, ceres.lambdaFactor, 0.0);
  CHECK(ceres.diagonalDamping);
  CHECK(!ceres.useFixedLambdaFactor);
  CHECK(!ceres.enableDetailedProfiling);
}

TEST(CudaSfmLevenbergMarquardtOptimizer, ExposesCudaParams) {
  NonlinearFactorGraph graph;
  Values initial;
  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.linearSolver = CudaSfmLinearSolverType::CudssFullNormal;
  params.maxIterations = 7;

  CudaSfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);

  CHECK(optimizer.params().linearSolver ==
        CudaSfmLinearSolverType::CudssFullNormal);
  CHECK(optimizer.params().getLinearSolver() == "cudss-full-normal");
  CHECK(optimizer.params().maxIterations == 7);
}

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

TEST(CudaSfmProjectionLinearization,
     AppliesPerObservationGaussianWhitening) {
  constexpr double kResidualTolerance = 1e-7;
  constexpr double kJacobianTolerance = 3e-3;
  constexpr double kSystemTolerance = 1e-5;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<std::vector<CudaSfmSqrtInfo2>> sqrtInfoByTrack =
      MakeMixedSqrtInfoByTrack(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch = CudaSfmProjectionBatch::FromSfmData(
      measuredData, sqrtInfoByTrack, context.stream());
  CHECK(batch.noiseMode() == CudaSfmProjectionNoiseMode::Whitened);
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  const CudaBalCsrStructure structure =
      CudaBalCsrStructure::FromSfmData(measuredData);
  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());
  AccumulateCudaSfmNormalEquations(
      values, batch, static_cast<int>(structure.numCameras()), &system,
      context.stream());

  CudaDeviceArray<double> actualDeviceDiagonal;
  ComputeCudaSfmHessianDiagonal(values, batch,
                                static_cast<int>(structure.numCameras()), 1e-6,
                                1e32, &actualDeviceDiagonal, context.stream());

  std::vector<CudaSfmObservation> observations;
  std::vector<CudaSfmSqrtInfo2> sqrtInfos;
  std::vector<DevicePinholeCameraCal3Bundler> deviceCameras;
  std::vector<DevicePoint3> devicePoints;
  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  std::vector<double> actualValues;
  std::vector<double> actualRhs;
  std::vector<double> actualDiagonal;
  batch.observations().download(&observations, context.stream());
  batch.sqrtInfos().download(&sqrtInfos, context.stream());
  values.block<DevicePinholeCameraCal3Bundler>(
            kDevicePinholeCameraCal3BundlerType)
      .values.download(&deviceCameras, context.stream());
  values.block<DevicePoint3>(kDevicePoint3Type)
      .values.download(&devicePoints, context.stream());
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  system.values().download(&actualValues, context.stream());
  system.rhs().download(&actualRhs, context.stream());
  actualDeviceDiagonal.download(&actualDiagonal, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(observations.size(), sqrtInfos.size());
  EXPECT_LONGS_EQUAL(8, observations.size());

  double expectedError = 0.0;
  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const CudaSfmSqrtInfo2& sqrtInfo = sqrtInfos[i];
    const BundlerCamera camera =
        hostCameraFromDevice(deviceCameras[observation.cameraSlot]);
    const Point3 point =
        hostPointFromDevice(devicePoints[observation.pointSlot]);

    const Vector2 rawResidual = hostResidual(camera, point, observation);
    const Vector2 expectedResidual =
        WhitenResidual(sqrtInfo, rawResidual);
    expectedError += 0.5 * expectedResidual.squaredNorm();
    DOUBLES_EQUAL(expectedResidual(0), residuals[2 * i],
                  kResidualTolerance);
    DOUBLES_EQUAL(expectedResidual(1), residuals[2 * i + 1],
                  kResidualTolerance);

    const HostNumericJacobians rawJacobians =
        hostNumericJacobians(camera, point, observation);
    for (int col = 0; col < 9; ++col) {
      const double expectedRow0 =
          sqrtInfo.r00 * rawJacobians.camera[col] +
          sqrtInfo.r01 * rawJacobians.camera[9 + col];
      const double expectedRow1 =
          sqrtInfo.r11 * rawJacobians.camera[9 + col];
      DOUBLES_EQUAL(expectedRow0, cameraJacobians[18 * i + col],
                    kJacobianTolerance);
      DOUBLES_EQUAL(expectedRow1, cameraJacobians[18 * i + 9 + col],
                    kJacobianTolerance);
    }
    for (int col = 0; col < 3; ++col) {
      const double expectedRow0 =
          sqrtInfo.r00 * rawJacobians.point[col] +
          sqrtInfo.r01 * rawJacobians.point[3 + col];
      const double expectedRow1 =
          sqrtInfo.r11 * rawJacobians.point[3 + col];
      DOUBLES_EQUAL(expectedRow0, pointJacobians[6 * i + col],
                    kJacobianTolerance);
      DOUBLES_EQUAL(expectedRow1, pointJacobians[6 * i + 3 + col],
                    kJacobianTolerance);
    }
  }

  const double actualError =
      ComputeCudaSfmProjectionError(values, batch, context.stream());
  DOUBLES_EQUAL(expectedError, actualError, kResidualTolerance);

  const int dimension = structure.dimension();
  std::vector<double> expectedDense(static_cast<size_t>(dimension) *
                                    static_cast<size_t>(dimension));
  std::vector<double> expectedRhs(static_cast<size_t>(dimension));
  std::vector<double> expectedDiagonal(static_cast<size_t>(dimension));

  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const int cameraBase = 9 * observation.cameraSlot;
    const int pointBase =
        9 * static_cast<int>(structure.numCameras()) + 3 * observation.pointSlot;

    int global[12];
    double jacobianRow0[12];
    double jacobianRow1[12];
    for (int col = 0; col < 9; ++col) {
      global[col] = cameraBase + col;
      jacobianRow0[col] = cameraJacobians[18 * i + col];
      jacobianRow1[col] = cameraJacobians[18 * i + 9 + col];
    }
    for (int col = 0; col < 3; ++col) {
      global[9 + col] = pointBase + col;
      jacobianRow0[9 + col] = pointJacobians[6 * i + col];
      jacobianRow1[9 + col] = pointJacobians[6 * i + 3 + col];
    }

    const double residual0 = residuals[2 * i];
    const double residual1 = residuals[2 * i + 1];
    for (int a = 0; a < 12; ++a) {
      expectedDiagonal[global[a]] +=
          jacobianRow0[a] * jacobianRow0[a] +
          jacobianRow1[a] * jacobianRow1[a];
      expectedRhs[global[a]] +=
          -jacobianRow0[a] * residual0 - jacobianRow1[a] * residual1;
      for (int b = a; b < 12; ++b) {
        const int row = std::min(global[a], global[b]);
        const int col = std::max(global[a], global[b]);
        expectedDense[static_cast<size_t>(row) * dimension + col] +=
            jacobianRow0[a] * jacobianRow0[b] +
            jacobianRow1[a] * jacobianRow1[b];
      }
    }
  }
  for (double& value : expectedDiagonal) {
    value = std::min(1e32, std::max(1e-6, value));
  }

  EXPECT_LONGS_EQUAL(expectedDiagonal.size(), actualDiagonal.size());
  for (size_t i = 0; i < expectedDiagonal.size(); ++i) {
    DOUBLES_EQUAL(expectedDiagonal[i], actualDiagonal[i], kSystemTolerance);
  }

  EXPECT_LONGS_EQUAL(structure.colIndices().size(), actualValues.size());
  EXPECT_LONGS_EQUAL(dimension, actualRhs.size());
  for (int row = 0; row < dimension; ++row) {
    DOUBLES_EQUAL(expectedRhs[row], actualRhs[row], kSystemTolerance);
    for (int entry = structure.rowPointers()[row];
         entry < structure.rowPointers()[row + 1]; ++entry) {
      const int col = structure.colIndices()[entry];
      DOUBLES_EQUAL(expectedDense[static_cast<size_t>(row) * dimension + col],
                    actualValues[entry], kSystemTolerance);
    }
  }
}

TEST(CudaSfmProjectionLinearization,
     AppliesPerObservationRobustWhiteningAndLoss) {
  constexpr double kResidualTolerance = 1e-7;
  constexpr double kJacobianTolerance = 3e-3;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<std::vector<CudaSfmSqrtInfo2>> sqrtInfoByTrack =
      MakeMixedSqrtInfoByTrack(measuredData);
  const std::vector<std::vector<CudaSfmRobustModel>> robustModelsByTrack =
      MakeMixedRobustModelsByTrack(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch = CudaSfmProjectionBatch::FromSfmData(
      measuredData, sqrtInfoByTrack, robustModelsByTrack, context.stream());
  CHECK(batch.noiseMode() == CudaSfmProjectionNoiseMode::Robust);
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<CudaSfmObservation> observations;
  std::vector<CudaSfmSqrtInfo2> sqrtInfos;
  std::vector<CudaSfmRobustModel> robustModels;
  std::vector<DevicePinholeCameraCal3Bundler> deviceCameras;
  std::vector<DevicePoint3> devicePoints;
  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  batch.observations().download(&observations, context.stream());
  batch.sqrtInfos().download(&sqrtInfos, context.stream());
  batch.robustModels().download(&robustModels, context.stream());
  values.block<DevicePinholeCameraCal3Bundler>(
            kDevicePinholeCameraCal3BundlerType)
      .values.download(&deviceCameras, context.stream());
  values.block<DevicePoint3>(kDevicePoint3Type)
      .values.download(&devicePoints, context.stream());
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(observations.size(), sqrtInfos.size());
  EXPECT_LONGS_EQUAL(observations.size(), robustModels.size());
  EXPECT_LONGS_EQUAL(8, observations.size());

  double expectedError = 0.0;
  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const CudaSfmSqrtInfo2& sqrtInfo = sqrtInfos[i];
    const CudaSfmRobustModel& robustModel = robustModels[i];
    const BundlerCamera camera =
        hostCameraFromDevice(deviceCameras[observation.cameraSlot]);
    const Point3 point =
        hostPointFromDevice(devicePoints[observation.pointSlot]);

    const Vector2 rawResidual = hostResidual(camera, point, observation);
    const Vector2 whitenedResidual =
        WhitenResidual(sqrtInfo, rawResidual);
    double row0Scale = 1.0;
    double row1Scale = 1.0;
    RobustRowScales(robustModel, whitenedResidual, &row0Scale, &row1Scale);
    const Vector2 expectedResidual(row0Scale * whitenedResidual(0),
                                   row1Scale * whitenedResidual(1));
    expectedError += RobustLoss(
        robustModel, std::sqrt(whitenedResidual.squaredNorm()));

    DOUBLES_EQUAL(expectedResidual(0), residuals[2 * i],
                  kResidualTolerance);
    DOUBLES_EQUAL(expectedResidual(1), residuals[2 * i + 1],
                  kResidualTolerance);

    const HostNumericJacobians rawJacobians =
        hostNumericJacobians(camera, point, observation);
    for (int col = 0; col < 9; ++col) {
      const double whitenedRow0 =
          sqrtInfo.r00 * rawJacobians.camera[col] +
          sqrtInfo.r01 * rawJacobians.camera[9 + col];
      const double whitenedRow1 =
          sqrtInfo.r11 * rawJacobians.camera[9 + col];
      DOUBLES_EQUAL(row0Scale * whitenedRow0,
                    cameraJacobians[18 * i + col], kJacobianTolerance);
      DOUBLES_EQUAL(row1Scale * whitenedRow1,
                    cameraJacobians[18 * i + 9 + col], kJacobianTolerance);
    }
    for (int col = 0; col < 3; ++col) {
      const double whitenedRow0 =
          sqrtInfo.r00 * rawJacobians.point[col] +
          sqrtInfo.r01 * rawJacobians.point[3 + col];
      const double whitenedRow1 =
          sqrtInfo.r11 * rawJacobians.point[3 + col];
      DOUBLES_EQUAL(row0Scale * whitenedRow0,
                    pointJacobians[6 * i + col], kJacobianTolerance);
      DOUBLES_EQUAL(row1Scale * whitenedRow1,
                    pointJacobians[6 * i + 3 + col], kJacobianTolerance);
    }
  }

  const double actualError =
      ComputeCudaSfmProjectionError(values, batch, context.stream());
  DOUBLES_EQUAL(expectedError, actualError, kResidualTolerance);
}

TEST(CudaSfmProjectionLinearization,
     AccumulatesProjectionNormalEquationsIntoCsr) {
  constexpr double kTolerance = 1e-6;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  const CudaBalCsrStructure structure =
      CudaBalCsrStructure::FromSfmData(measuredData);
  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());
  system.values().upload(
      std::vector<double>(structure.colIndices().size(), 123.0),
      context.stream());
  system.rhs().upload(std::vector<double>(structure.dimension(), -456.0),
                      context.stream());

  AccumulateCudaSfmNormalEquations(
      values, batch, static_cast<int>(structure.numCameras()), &system,
      context.stream());

  std::vector<CudaSfmObservation> observations;
  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  std::vector<double> actualValues;
  std::vector<double> actualRhs;
  batch.observations().download(&observations, context.stream());
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  system.values().download(&actualValues, context.stream());
  system.rhs().download(&actualRhs, context.stream());
  context.synchronize();

  const int dimension = structure.dimension();
  std::vector<double> expectedDense(static_cast<size_t>(dimension) *
                                    static_cast<size_t>(dimension));
  std::vector<double> expectedRhs(static_cast<size_t>(dimension));

  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const int cameraBase = 9 * observation.cameraSlot;
    const int pointBase =
        9 * static_cast<int>(structure.numCameras()) + 3 * observation.pointSlot;

    int global[12];
    double jacobianRow0[12];
    double jacobianRow1[12];
    for (int col = 0; col < 9; ++col) {
      global[col] = cameraBase + col;
      jacobianRow0[col] = cameraJacobians[18 * i + col];
      jacobianRow1[col] = cameraJacobians[18 * i + 9 + col];
    }
    for (int col = 0; col < 3; ++col) {
      global[9 + col] = pointBase + col;
      jacobianRow0[9 + col] = pointJacobians[6 * i + col];
      jacobianRow1[9 + col] = pointJacobians[6 * i + 3 + col];
    }

    const double residual0 = residuals[2 * i];
    const double residual1 = residuals[2 * i + 1];
    for (int a = 0; a < 12; ++a) {
      expectedRhs[global[a]] +=
          -jacobianRow0[a] * residual0 - jacobianRow1[a] * residual1;
      for (int b = a; b < 12; ++b) {
        const int row = std::min(global[a], global[b]);
        const int col = std::max(global[a], global[b]);
        expectedDense[static_cast<size_t>(row) * dimension + col] +=
            jacobianRow0[a] * jacobianRow0[b] +
            jacobianRow1[a] * jacobianRow1[b];
      }
    }
  }

  EXPECT_LONGS_EQUAL(structure.colIndices().size(), actualValues.size());
  EXPECT_LONGS_EQUAL(dimension, actualRhs.size());
  for (int row = 0; row < dimension; ++row) {
    DOUBLES_EQUAL(expectedRhs[row], actualRhs[row], kTolerance);
    for (int entry = structure.rowPointers()[row];
         entry < structure.rowPointers()[row + 1]; ++entry) {
      const int col = structure.colIndices()[entry];
      DOUBLES_EQUAL(expectedDense[static_cast<size_t>(row) * dimension + col],
                    actualValues[entry], kTolerance);
    }
  }
}

TEST(CudaSfmProjectionLinearization, ComputesClampedHessianDiagonal) {
  constexpr double kTolerance = 1e-6;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  CudaDeviceArray<double> actualDeviceDiagonal;
  ComputeCudaSfmHessianDiagonal(values, batch, 2, 1e-6, 1e32,
                                &actualDeviceDiagonal, context.stream());

  std::vector<CudaSfmObservation> observations;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  std::vector<double> actualDiagonal;
  batch.observations().download(&observations, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  actualDeviceDiagonal.download(&actualDiagonal, context.stream());
  context.synchronize();

  std::vector<double> expectedDiagonal(9 * measuredData.numberCameras() +
                                           3 * measuredData.numberTracks(),
                                       0.0);
  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const int cameraBase = 9 * observation.cameraSlot;
    const int pointBase =
        9 * static_cast<int>(measuredData.numberCameras()) +
        3 * observation.pointSlot;
    for (int col = 0; col < 9; ++col) {
      const double j0 = cameraJacobians[18 * i + col];
      const double j1 = cameraJacobians[18 * i + 9 + col];
      expectedDiagonal[cameraBase + col] += j0 * j0 + j1 * j1;
    }
    for (int col = 0; col < 3; ++col) {
      const double j0 = pointJacobians[6 * i + col];
      const double j1 = pointJacobians[6 * i + 3 + col];
      expectedDiagonal[pointBase + col] += j0 * j0 + j1 * j1;
    }
  }
  for (double& value : expectedDiagonal) {
    value = std::min(1e32, std::max(1e-6, value));
  }

  LONGS_EQUAL(expectedDiagonal.size(), actualDiagonal.size());
  for (size_t i = 0; i < expectedDiagonal.size(); ++i) {
    DOUBLES_EQUAL(expectedDiagonal[i], actualDiagonal[i], kTolerance);
  }
}

TEST(CudaSfmProjectionLinearization, ComputesLinearizedErrorChange) {
  constexpr double kTolerance = 1e-6;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());
  CudaDeviceArray<double> delta;
  SolveCudaSfmDenseSchur(values, batch,
                         static_cast<int>(measuredData.numberCameras()),
                         1e-3, &delta, context.stream());

  double oldLinearizedError = 0.0;
  double newLinearizedError = 0.0;
  const double actualChange = ComputeCudaSfmLinearizedErrorChange(
      values, batch, static_cast<int>(measuredData.numberCameras()), delta,
      &oldLinearizedError, &newLinearizedError, context.stream());

  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());
  std::vector<CudaSfmObservation> observations;
  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  std::vector<double> hostDelta;
  batch.observations().download(&observations, context.stream());
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  delta.download(&hostDelta, context.stream());
  context.synchronize();

  double expectedOld = 0.0;
  double expectedNew = 0.0;
  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const int cameraBase = 9 * observation.cameraSlot;
    const int pointBase =
        9 * static_cast<int>(measuredData.numberCameras()) +
        3 * observation.pointSlot;
    double predicted0 = residuals[2 * i];
    double predicted1 = residuals[2 * i + 1];
    expectedOld += 0.5 * (predicted0 * predicted0 + predicted1 * predicted1);
    for (int col = 0; col < 9; ++col) {
      predicted0 += cameraJacobians[18 * i + col] *
                    hostDelta[cameraBase + col];
      predicted1 += cameraJacobians[18 * i + 9 + col] *
                    hostDelta[cameraBase + col];
    }
    for (int col = 0; col < 3; ++col) {
      predicted0 += pointJacobians[6 * i + col] *
                    hostDelta[pointBase + col];
      predicted1 += pointJacobians[6 * i + 3 + col] *
                    hostDelta[pointBase + col];
    }
    expectedNew += 0.5 * (predicted0 * predicted0 + predicted1 * predicted1);
  }

  DOUBLES_EQUAL(expectedOld, oldLinearizedError, kTolerance);
  DOUBLES_EQUAL(expectedNew, newLinearizedError, kTolerance);
  DOUBLES_EQUAL(expectedOld - expectedNew, actualChange, kTolerance);
}

TEST(CudaSfmProjectionLinearization, RejectsIncompleteCsrPattern) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());

  const CudaBalCsrStructure structure =
      CudaBalCsrStructure::FromSfmData(measuredData);
  std::vector<int> rowPointers = structure.rowPointers();
  std::vector<int> colIndices = structure.colIndices();

  const int missingRow = 0;
  const int missingCol = 9 * static_cast<int>(structure.numCameras());
  int missingEntry = -1;
  for (int entry = rowPointers[missingRow]; entry < rowPointers[missingRow + 1];
       ++entry) {
    if (colIndices[entry] == missingCol) {
      missingEntry = entry;
      break;
    }
  }
  CHECK(missingEntry >= 0);
  colIndices.erase(colIndices.begin() + missingEntry);
  for (size_t row = static_cast<size_t>(missingRow) + 1;
       row < rowPointers.size(); ++row) {
    --rowPointers[row];
  }

  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), rowPointers, colIndices,
                       context.stream());

  CHECK_EXCEPTION(AccumulateCudaSfmNormalEquations(
                      values, batch, static_cast<int>(structure.numCameras()),
                      &system, context.stream()),
                  std::runtime_error);
}

TEST(CudaSfmProjectionLinearization,
     RejectsInvalidSystemWithoutClearingExistingValues) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(initialData, context.stream());
  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());

  const CudaBalCsrStructure structure =
      CudaBalCsrStructure::FromSfmData(measuredData);
  std::vector<int> rowPointers = structure.rowPointers();
  std::vector<int> colIndices = structure.colIndices();
  rowPointers.push_back(rowPointers.back() + 1);
  colIndices.push_back(structure.dimension());

  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension() + 1, rowPointers, colIndices,
                       context.stream());
  system.values().upload(std::vector<double>(colIndices.size(), 7.0),
                         context.stream());
  system.rhs().upload(std::vector<double>(structure.dimension() + 1, -8.0),
                      context.stream());

  CHECK_EXCEPTION(AccumulateCudaSfmNormalEquations(
                      values, batch, static_cast<int>(structure.numCameras()),
                      &system, context.stream()),
                  std::invalid_argument);

  std::vector<double> actualValues;
  std::vector<double> actualRhs;
  system.values().download(&actualValues, context.stream());
  system.rhs().download(&actualRhs, context.stream());
  context.synchronize();

  for (double value : actualValues) {
    DOUBLES_EQUAL(7.0, value, 1e-12);
  }
  for (double value : actualRhs) {
    DOUBLES_EQUAL(-8.0, value, 1e-12);
  }
}

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
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
#endif

TEST(CudaSfmProjectionLinearization, RejectsMismatchedValueShapes) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData smallerValuesData = makeTinyBalData();
  SfmData fewerCamerasData = measuredData;
  fewerCamerasData.cameras.resize(1);
  CudaContext context;

  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(measuredData, context.stream());
  CudaSfmProjectionLinearization linearization;

  DeviceValues fewerCameraValues =
      PackSfmValues(fewerCamerasData, context.stream());
  CHECK_EXCEPTION(LinearizeCudaSfmProjectionBatch(
                      fewerCameraValues, batch, &linearization,
                      context.stream()),
                  std::invalid_argument);

  DeviceValues fewerPointValues =
      PackSfmValues(smallerValuesData, context.stream());
  CHECK_EXCEPTION(LinearizeCudaSfmProjectionBatch(
                      fewerPointValues, batch, &linearization,
                      context.stream()),
                  std::invalid_argument);
}

#if !GTSAM_ENABLE_CUDSS
TEST(CudaSfmLevenbergMarquardt, DenseSchurRunsWithoutCudss) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  CudaSfmLevenbergMarquardtParams params;
  params.linearSolver = CudaSfmLinearSolverType::DenseSchur;
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  const CudaSfmLevenbergMarquardtResult result =
      OptimizeCudaSfmWithoutValueDownload(data, params);

  CHECK(result.innerIterations > 0);
  CHECK(result.finalError < result.initialError);
  CHECK(result.optimizedValues.empty());
}
#endif

#if GTSAM_ENABLE_CUDSS
TEST(CudaSfmFactorGraphConversion,
     ConvertsGeneralSfmFactorsWithArbitraryKeys) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  NonlinearFactorGraph graph;
  const auto model = noiseModel::Unit::Create(2);
  for (size_t pointSlot = 0; pointSlot < measuredData.numberTracks();
       ++pointSlot) {
    const SfmTrack& track = measuredData.track(pointSlot);
    for (const SfmMeasurement& measurement : track.measurements) {
      graph.emplace_shared<BundlerProjectionFactor>(
          measurement.second, model, cameraKeys[measurement.first],
          pointKeys[pointSlot]);
    }
  }

  const CudaSfmFactorGraphData converted =
      ConvertGeneralSfmGraphToCudaSfmData(graph, initial);

  EXPECT_LONGS_EQUAL(cameraKeys.size(), converted.cameraKeys.size());
  EXPECT_LONGS_EQUAL(pointKeys.size(), converted.pointKeys.size());
  EXPECT_LONGS_EQUAL(cameraKeys[0], converted.cameraKeys[0]);
  EXPECT_LONGS_EQUAL(cameraKeys[1], converted.cameraKeys[1]);
  EXPECT_LONGS_EQUAL(pointKeys[0], converted.pointKeys[0]);
  EXPECT_LONGS_EQUAL(pointKeys[3], converted.pointKeys[3]);
  EXPECT_LONGS_EQUAL(measuredData.numberCameras(),
                     converted.data.numberCameras());
  EXPECT_LONGS_EQUAL(measuredData.numberTracks(),
                     converted.data.numberTracks());
  EXPECT_LONGS_EQUAL(2, converted.data.track(0).numberMeasurements());
  DOUBLES_EQUAL(measuredData.track(0).measurement(0).second(0),
                converted.data.track(0).measurement(0).second(0), 1e-12);
  CHECK(CameraEquals(initialData.camera(0), converted.data.camera(0)));
  CHECK(Point3Equals(initialData.track(3).point3(),
                     converted.data.track(3).point3()));
}

TEST(CudaSfmFactorGraphConversion,
     ConvertsPointBatchedGeneralSfmFactors) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  NonlinearFactorGraph rawGraph;
  NonlinearFactorGraph batchGraph;
  const auto model = noiseModel::Unit::Create(2);
  for (size_t pointSlot = 0; pointSlot < measuredData.numberTracks();
       ++pointSlot) {
    std::map<Key, Point2> measurements;
    const SfmTrack& track = measuredData.track(pointSlot);
    for (const SfmMeasurement& measurement : track.measurements) {
      rawGraph.emplace_shared<BundlerProjectionFactor>(
          measurement.second, model, cameraKeys[measurement.first],
          pointKeys[pointSlot]);
      measurements[cameraKeys[measurement.first]] = measurement.second;
    }
    batchGraph.add(
        std::make_shared<BatchFactor<BundlerProjectionFactor, 2>>(
            measurements, pointKeys[pointSlot], model));
  }

  const CudaSfmFactorGraphData raw =
      ConvertGeneralSfmGraphToCudaSfmData(rawGraph, initial);
  const CudaSfmFactorGraphData batched =
      ConvertGeneralSfmGraphToCudaSfmData(batchGraph, initial);
  CHECK(ConvertedSfmDataEquals(raw, batched));
}

TEST(CudaSfmFactorGraphConversion,
     ConvertsCameraBatchedGeneralSfmFactors) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  NonlinearFactorGraph rawGraph;
  NonlinearFactorGraph batchGraph;
  std::vector<std::map<Key, Point2>> measurementsByCamera(cameraKeys.size());
  const auto model = noiseModel::Unit::Create(2);
  for (size_t pointSlot = 0; pointSlot < measuredData.numberTracks();
       ++pointSlot) {
    const SfmTrack& track = measuredData.track(pointSlot);
    for (const SfmMeasurement& measurement : track.measurements) {
      rawGraph.emplace_shared<BundlerProjectionFactor>(
          measurement.second, model, cameraKeys[measurement.first],
          pointKeys[pointSlot]);
      measurementsByCamera[measurement.first][pointKeys[pointSlot]] =
          measurement.second;
    }
  }
  for (size_t cameraSlot = 0; cameraSlot < measurementsByCamera.size();
       ++cameraSlot) {
    batchGraph.add(
        std::make_shared<BatchFactor<BundlerProjectionFactor, 2>>(
            cameraKeys[cameraSlot], measurementsByCamera[cameraSlot], model));
  }

  const CudaSfmFactorGraphData raw =
      ConvertGeneralSfmGraphToCudaSfmData(rawGraph, initial);
  const CudaSfmFactorGraphData batched =
      ConvertGeneralSfmGraphToCudaSfmData(batchGraph, initial);
  CHECK(ConvertedSfmDataEquals(raw, batched));
}

TEST(CudaSfmFactorGraphConversion,
     AcceptsFixedGaussianNoiseAndPreservesFlattenedWhiteningOrder) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  Matrix2 fullR{{3.0, 0.25}, {0.0, 4.0}};
  const SharedNoiseModel full = noiseModel::Gaussian::SqrtInformation(
      fullR, false);
  const SharedNoiseModel diagonal =
      noiseModel::Diagonal::Sigmas(Vector2(0.25, 0.5));
  const SharedNoiseModel isotropic = noiseModel::Isotropic::Sigma(2, 0.5);
  const SharedNoiseModel nullModel;

  NonlinearFactorGraph graph;
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(1).measurement(1).second, full, cameraKeys[1],
      pointKeys[1]);
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(1).second, diagonal, cameraKeys[1],
      pointKeys[0]);
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(1).measurement(0).second, isotropic, cameraKeys[0],
      pointKeys[1]);
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(0).second, nullModel, cameraKeys[0],
      pointKeys[0]);

  const CudaSfmFactorGraphData converted =
      ConvertGeneralSfmGraphToCudaSfmData(graph, initial);

  CHECK(!converted.hasRobustNoise);
  CHECK(converted.robustModelsByTrack.empty());
  CHECK(converted.hasNonUnitNoise);
  EXPECT_LONGS_EQUAL(measuredData.numberTracks(),
                     converted.sqrtInfoByTrack.size());
  EXPECT_LONGS_EQUAL(2, converted.sqrtInfoByTrack[0].size());
  EXPECT_LONGS_EQUAL(2, converted.sqrtInfoByTrack[1].size());
  DOUBLES_EQUAL(4.0, converted.sqrtInfoByTrack[0][0].r00, 1e-12);
  DOUBLES_EQUAL(0.0, converted.sqrtInfoByTrack[0][0].r01, 1e-12);
  DOUBLES_EQUAL(2.0, converted.sqrtInfoByTrack[0][0].r11, 1e-12);
  DOUBLES_EQUAL(1.0, converted.sqrtInfoByTrack[0][1].r00, 1e-12);
  DOUBLES_EQUAL(0.0, converted.sqrtInfoByTrack[0][1].r01, 1e-12);
  DOUBLES_EQUAL(1.0, converted.sqrtInfoByTrack[0][1].r11, 1e-12);
  DOUBLES_EQUAL(3.0, converted.sqrtInfoByTrack[1][0].r00, 1e-12);
  DOUBLES_EQUAL(0.25, converted.sqrtInfoByTrack[1][0].r01, 1e-12);
  DOUBLES_EQUAL(4.0, converted.sqrtInfoByTrack[1][0].r11, 1e-12);
  DOUBLES_EQUAL(2.0, converted.sqrtInfoByTrack[1][1].r00, 1e-12);
  DOUBLES_EQUAL(0.0, converted.sqrtInfoByTrack[1][1].r01, 1e-12);
  DOUBLES_EQUAL(2.0, converted.sqrtInfoByTrack[1][1].r11, 1e-12);

  CudaContext context;
  const CudaSfmProjectionBatch batch = CudaSfmProjectionBatch::FromSfmData(
      converted.data, converted.sqrtInfoByTrack, context.stream());
  CHECK(batch.noiseMode() == CudaSfmProjectionNoiseMode::Whitened);

  std::vector<CudaSfmObservation> observations;
  std::vector<CudaSfmSqrtInfo2> sqrtInfos;
  batch.observations().download(&observations, context.stream());
  batch.sqrtInfos().download(&sqrtInfos, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(4, observations.size());
  EXPECT_LONGS_EQUAL(4, sqrtInfos.size());
  EXPECT_LONGS_EQUAL(1, observations[0].cameraSlot);
  EXPECT_LONGS_EQUAL(0, observations[1].cameraSlot);
  EXPECT_LONGS_EQUAL(1, observations[2].cameraSlot);
  EXPECT_LONGS_EQUAL(0, observations[3].cameraSlot);
  DOUBLES_EQUAL(4.0, sqrtInfos[0].r00, 1e-12);
  DOUBLES_EQUAL(1.0, sqrtInfos[1].r00, 1e-12);
  DOUBLES_EQUAL(3.0, sqrtInfos[2].r00, 1e-12);
  DOUBLES_EQUAL(0.25, sqrtInfos[2].r01, 1e-12);
  DOUBLES_EQUAL(2.0, sqrtInfos[3].r00, 1e-12);
}

TEST(CudaSfmFactorGraphConversion,
     AcceptsHuberAndTukeyRobustNoiseAndPreservesFlattenedOrder) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  Matrix2 fullR{{3.0, 0.25}, {0.0, 4.0}};
  const SharedNoiseModel huber = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(
          0.25, noiseModel::mEstimator::Huber::Block),
      noiseModel::Unit::Create(2));
  const SharedNoiseModel scalarHuber = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(
          0.20, noiseModel::mEstimator::Huber::Scalar),
      noiseModel::Diagonal::Sigmas(Vector2(0.25, 0.5)));
  const SharedNoiseModel tukey = noiseModel::Robust::Create(
      noiseModel::mEstimator::Tukey::Create(
          0.75, noiseModel::mEstimator::Tukey::Block),
      noiseModel::Gaussian::SqrtInformation(fullR, false));

  NonlinearFactorGraph graph;
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(1).measurement(1).second, tukey, cameraKeys[1],
      pointKeys[1]);
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(1).second, scalarHuber, cameraKeys[1],
      pointKeys[0]);
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(1).measurement(0).second, huber, cameraKeys[0],
      pointKeys[1]);
  graph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(0).second, huber, cameraKeys[0],
      pointKeys[0]);

  const CudaSfmFactorGraphData converted =
      ConvertGeneralSfmGraphToCudaSfmData(graph, initial);

  CHECK(converted.hasNonUnitNoise);
  CHECK(converted.hasRobustNoise);
  EXPECT_LONGS_EQUAL(measuredData.numberTracks(),
                     converted.sqrtInfoByTrack.size());
  EXPECT_LONGS_EQUAL(measuredData.numberTracks(),
                     converted.robustModelsByTrack.size());
  EXPECT_LONGS_EQUAL(2, converted.robustModelsByTrack[0].size());
  EXPECT_LONGS_EQUAL(2, converted.robustModelsByTrack[1].size());
  CHECK(converted.robustModelsByTrack[0][0].kind ==
        CudaSfmRobustModelKind::Huber);
  CHECK(converted.robustModelsByTrack[0][0].reweightScheme ==
        CudaSfmRobustReweightScheme::Scalar);
  CHECK(converted.robustModelsByTrack[0][1].kind ==
        CudaSfmRobustModelKind::Huber);
  CHECK(converted.robustModelsByTrack[1][0].kind ==
        CudaSfmRobustModelKind::Tukey);
  DOUBLES_EQUAL(0.75, converted.robustModelsByTrack[1][0].parameter,
                1e-12);
  DOUBLES_EQUAL(3.0, converted.sqrtInfoByTrack[1][0].r00, 1e-12);
  DOUBLES_EQUAL(0.25, converted.sqrtInfoByTrack[1][0].r01, 1e-12);
  DOUBLES_EQUAL(4.0, converted.sqrtInfoByTrack[1][0].r11, 1e-12);
  DOUBLES_EQUAL(4.0, converted.sqrtInfoByTrack[0][0].r00, 1e-12);
  DOUBLES_EQUAL(2.0, converted.sqrtInfoByTrack[0][0].r11, 1e-12);
  DOUBLES_EQUAL(1.0, converted.sqrtInfoByTrack[0][1].r00, 1e-12);
  DOUBLES_EQUAL(1.0, converted.sqrtInfoByTrack[0][1].r11, 1e-12);

  CudaContext context;
  const CudaSfmProjectionBatch batch = CudaSfmProjectionBatch::FromSfmData(
      converted.data, converted.sqrtInfoByTrack,
      converted.robustModelsByTrack, context.stream());
  CHECK(batch.noiseMode() == CudaSfmProjectionNoiseMode::Robust);

  std::vector<CudaSfmRobustModel> robustModels;
  batch.robustModels().download(&robustModels, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(4, robustModels.size());
  CHECK(robustModels[0].kind == CudaSfmRobustModelKind::Huber);
  CHECK(robustModels[0].reweightScheme ==
        CudaSfmRobustReweightScheme::Scalar);
  CHECK(robustModels[2].kind == CudaSfmRobustModelKind::Tukey);
  DOUBLES_EQUAL(0.75, robustModels[2].parameter, 1e-12);
}

TEST(CudaSfmFactorGraphConversion, RejectsUnsupportedNoiseModels) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Values initial;
  initial.insert(C(0), initialData.camera(0));
  initial.insert(P(0), initialData.track(0).point3());

  NonlinearFactorGraph unsupportedRobustGraph;
  const SharedNoiseModel unsupportedRobust = noiseModel::Robust::Create(
      noiseModel::mEstimator::Cauchy::Create(1.345),
      noiseModel::Unit::Create(2));
  unsupportedRobustGraph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(0).second, unsupportedRobust, C(0),
      P(0));
  CHECK_EXCEPTION(
      ConvertGeneralSfmGraphToCudaSfmData(unsupportedRobustGraph, initial),
      std::invalid_argument);

  NonlinearFactorGraph constrainedGraph;
  constrainedGraph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(0).second,
      noiseModel::Constrained::All(2), C(0), P(0));
  CHECK_EXCEPTION(
      ConvertGeneralSfmGraphToCudaSfmData(constrainedGraph, initial),
      std::invalid_argument);

  NonlinearFactorGraph wrongDimensionGraph;
  wrongDimensionGraph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(0).second,
      noiseModel::Isotropic::Sigma(3, 0.5), C(0), P(0));
  CHECK_EXCEPTION(
      ConvertGeneralSfmGraphToCudaSfmData(wrongDimensionGraph, initial),
      std::invalid_argument);

  const auto checkRejectedSqrtInformation = [&](const Matrix2& R) {
    NonlinearFactorGraph graph;
    graph.emplace_shared<BundlerProjectionFactor>(
        measuredData.track(0).measurement(0).second,
        noiseModel::Gaussian::SqrtInformation(R, false), C(0), P(0));
    CHECK_EXCEPTION(ConvertGeneralSfmGraphToCudaSfmData(graph, initial),
                    std::invalid_argument);
  };

  Matrix2 nonfiniteStoredR{{1.0, std::numeric_limits<double>::infinity()},
                           {0.0, 1.0}};
  checkRejectedSqrtInformation(nonfiniteStoredR);

  Matrix2 nanLowerLeftR{{1.0, 0.0},
                        {std::numeric_limits<double>::quiet_NaN(), 1.0}};
  checkRejectedSqrtInformation(nanLowerLeftR);

  Matrix2 nonzeroLowerLeftR{{1.0, 0.0}, {1e-3, 1.0}};
  checkRejectedSqrtInformation(nonzeroLowerLeftR);

  Matrix2 negativeDiagonalR{{1.0, 0.0}, {0.0, -1.0}};
  checkRejectedSqrtInformation(negativeDiagonalR);
}

TEST(CudaSfmLevenbergMarquardtOptimizer,
     OptimizesGeneralSfmGraphWithArbitraryKeys) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }
  initial.insert(Symbol('u', 1), Point2(9.0, 8.0));

  NonlinearFactorGraph graph;
  const auto model = noiseModel::Unit::Create(2);
  for (size_t pointSlot = 0; pointSlot < measuredData.numberTracks();
       ++pointSlot) {
    const SfmTrack& track = measuredData.track(pointSlot);
    for (const SfmMeasurement& measurement : track.measurements) {
      graph.emplace_shared<BundlerProjectionFactor>(
          measurement.second, model, cameraKeys[measurement.first],
          pointKeys[pointSlot]);
    }
  }

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.maxIterations = 5;
  params.relativeErrorTol = 1e-12;
  CudaSfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const Values& result = optimizer.optimize();

  CHECK(graph.error(result) < graph.error(initial));
  CHECK(optimizer.result().innerIterations >=
        static_cast<int>(optimizer.iterations()));
  CHECK(optimizer.result().graphConversionElapsed > 0.0);
  CHECK(optimizer.result().graphBackendCallElapsed >=
        optimizer.result().totalMeasuredElapsed);
  CHECK(optimizer.result().graphConvertedDataDestructionElapsed >= 0.0);
  CHECK(optimizer.result().graphValueMergeElapsed > 0.0);
  CHECK(result.exists(cameraKeys[0]));
  CHECK(result.exists(pointKeys[3]));
  CHECK(result.exists(Symbol('u', 1)));
  CHECK(std::isfinite(result.at<SfmCamera>(cameraKeys[0]).calibration().fx()));
  const Point2& extra = result.at<Point2>(Symbol('u', 1));
  DOUBLES_EQUAL(9.0, extra.x(), 1e-12);
  DOUBLES_EQUAL(8.0, extra.y(), 1e-12);
}

TEST(CudaSfmLevenbergMarquardtOptimizer,
     UsesGraphGaussianNoiseForInitialError) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  Matrix2 fullR{{3.0, 0.25}, {0.0, 4.0}};
  const std::vector<SharedNoiseModel> models = {
      noiseModel::Isotropic::Sigma(2, 0.5),
      noiseModel::Diagonal::Sigmas(Vector2(0.25, 0.5)),
      noiseModel::Gaussian::SqrtInformation(fullR, false)};

  NonlinearFactorGraph graph;
  size_t modelIndex = 0;
  for (size_t pointSlot = 0; pointSlot < measuredData.numberTracks();
       ++pointSlot) {
    const SfmTrack& track = measuredData.track(pointSlot);
    for (const SfmMeasurement& measurement : track.measurements) {
      graph.emplace_shared<BundlerProjectionFactor>(
          measurement.second, models[modelIndex % models.size()],
          cameraKeys[measurement.first], pointKeys[pointSlot]);
      ++modelIndex;
    }
  }

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.maxIterations = 0;
  CudaSfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  optimizer.optimize();

  const double expectedInitialError = graph.error(initial);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().initialError, 1e-6);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().finalError, 1e-6);
}

TEST(CudaSfmLevenbergMarquardtOptimizer,
     DoesNotEvaluateCpuGraphErrorDuringCudaOptimize) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  NonlinearFactorGraph graph;
  const auto model = noiseModel::Unit::Create(2);
  for (size_t pointSlot = 0; pointSlot < measuredData.numberTracks();
       ++pointSlot) {
    const SfmTrack& track = measuredData.track(pointSlot);
    for (const SfmMeasurement& measurement : track.measurements) {
      graph.emplace_shared<CountingBundlerProjectionFactor>(
          measurement.second, model, cameraKeys[measurement.first],
          pointKeys[pointSlot]);
    }
  }

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.maxIterations = 0;

  CountingBundlerProjectionFactor::errorCalls = 0;
  CudaSfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  optimizer.optimize();

  EXPECT_LONGS_EQUAL(0, CountingBundlerProjectionFactor::errorCalls);
  DOUBLES_EQUAL(optimizer.result().finalError, optimizer.error(), 1e-6);
}

TEST(CudaSfmLevenbergMarquardtOptimizer,
     UsesGraphRobustNoiseForInitialError) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<Key> cameraKeys = {Symbol('x', 10), Symbol('x', 20)};
  const std::vector<Key> pointKeys = {Symbol('l', 100), Symbol('l', 200),
                                      Symbol('l', 300), Symbol('l', 400)};

  Values initial;
  for (size_t i = 0; i < cameraKeys.size(); ++i) {
    initial.insert(cameraKeys[i], initialData.camera(i));
  }
  for (size_t i = 0; i < pointKeys.size(); ++i) {
    initial.insert(pointKeys[i], initialData.track(i).point3());
  }

  Matrix2 fullR{{1.5, 0.1}, {0.0, 2.0}};
  const std::vector<SharedNoiseModel> models = {
      noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(0.25),
          noiseModel::Unit::Create(2)),
      noiseModel::Robust::Create(
          noiseModel::mEstimator::Tukey::Create(0.75),
          noiseModel::Gaussian::SqrtInformation(fullR, false))};

  NonlinearFactorGraph graph;
  size_t modelIndex = 0;
  for (size_t pointSlot = 0; pointSlot < measuredData.numberTracks();
       ++pointSlot) {
    const SfmTrack& track = measuredData.track(pointSlot);
    for (const SfmMeasurement& measurement : track.measurements) {
      graph.emplace_shared<BundlerProjectionFactor>(
          measurement.second, models[modelIndex % models.size()],
          cameraKeys[measurement.first], pointKeys[pointSlot]);
      ++modelIndex;
    }
  }

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.maxIterations = 0;
  CudaSfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  optimizer.optimize();

  const double expectedInitialError = graph.error(initial);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().initialError, 1e-6);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().finalError, 1e-6);
}

TEST(CudaSfmLevenbergMarquardt, ReducesTinyBalErrorAndDownloadsValues) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  CudaSfmLevenbergMarquardtParams params;
  params.maxIterations = 5;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  const CudaSfmLevenbergMarquardtResult result =
      OptimizeCudaSfm(data, params);

  CHECK(result.iterations > 0);
  CHECK(result.acceptedSteps > 0);
  CHECK(result.solveLoopElapsed > 0.0);
  CHECK(result.finalError < result.initialError);
  CHECK(result.optimizedValues.exists(C(0)));
  CHECK(result.optimizedValues.exists(P(0)));

  const auto& camera0 = result.optimizedValues.at<SfmCamera>(C(0));
  const auto& point0 = result.optimizedValues.at<Point3>(P(0));
  CHECK(std::isfinite(camera0.calibration().fx()));
  CHECK(camera0.calibration().fx() > 0.0);
  CHECK(std::isfinite(point0.x()));
}

TEST(CudaSfmLevenbergMarquardt, CanSkipOptimizedValueDownload) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  CudaSfmLevenbergMarquardtParams params;
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;
  const CudaSfmLevenbergMarquardtResult result =
      OptimizeCudaSfmWithoutValueDownload(data, params);

  CHECK(result.iterations > 0);
  CHECK(result.optimizedValues.empty());
}

TEST(CudaSfmLevenbergMarquardt, DetailedProfilingIsDisabledByDefault) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  CHECK(!params.enableDetailedProfiling);
  const CudaSfmLevenbergMarquardtResult result =
      OptimizeCudaSfm(data, params);

  CHECK(result.solveLoopElapsed > 0.0);
  EXPECT_LONGS_EQUAL(0, result.totalH2dBytes);
  EXPECT_LONGS_EQUAL(0, result.totalD2hBytes);
  DOUBLES_EQUAL(0.0, result.denseSchurSolveElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.linearizedErrorElapsed, 0.0);
  CHECK(result.iterationProfiles.empty());
}

TEST(CudaSfmLevenbergMarquardt, RecordsDetailedTimingBreakdown) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.maxIterations = 5;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;
  params.enableDetailedProfiling = true;

  const CudaSfmLevenbergMarquardtResult result =
      OptimizeCudaSfmWithoutValueDownload(data, params);

  CHECK(result.solveLoopElapsed > 0.0);
  CHECK(result.dampingDiagonalElapsed > 0.0);
  CHECK(result.denseSchurSolveElapsed > 0.0);
  CHECK(result.linearizedErrorElapsed > 0.0);
  CHECK(result.applyDeltaElapsed > 0.0);
  CHECK(result.trialErrorElapsed > 0.0);
  CHECK(result.lambdaUpdateElapsed > 0.0);
  CHECK(!result.iterationProfiles.empty());

  size_t attempts = 0;
  for (const auto& iteration : result.iterationProfiles) {
    CHECK(iteration.totalElapsed > 0.0);
    CHECK(iteration.dampingDiagonalElapsed >= 0.0);
    CHECK(!iteration.attemptProfiles.empty());
    attempts += iteration.attemptProfiles.size();
    for (const auto& attempt : iteration.attemptProfiles) {
      CHECK(attempt.totalElapsed > 0.0);
      CHECK(attempt.lambda > 0.0);
      CHECK(attempt.linearizedErrorElapsed > 0.0);
      CHECK(attempt.linearizedCostChange != 0.0);
      CHECK(attempt.denseSchurSolveElapsed > 0.0);
    }
  }
  EXPECT_LONGS_EQUAL(result.innerIterations, attempts);
}

TEST(CudaSfmLevenbergMarquardt, RecordsPureTransferTimingBreakdown) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::CeresDefaults();
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;
  params.enableDetailedProfiling = true;

  const CudaSfmLevenbergMarquardtResult result =
      OptimizeCudaSfm(data, params);

  const size_t expectedValueBytes =
      data.numberCameras() * sizeof(DevicePinholeCameraCal3Bundler) +
      data.numberTracks() * sizeof(DevicePoint3);
  const size_t expectedProjectionBytes =
      8 * sizeof(CudaSfmObservation) +
      (data.numberTracks() + 1) * sizeof(int);

  EXPECT_LONGS_EQUAL(expectedValueBytes, result.packValuesH2dBytes);
  EXPECT_LONGS_EQUAL(expectedProjectionBytes, result.projectionBatchH2dBytes);
  EXPECT_LONGS_EQUAL(expectedValueBytes + expectedProjectionBytes,
                     result.totalH2dBytes);
  EXPECT_LONGS_EQUAL(expectedValueBytes, result.downloadD2hBytes);
  EXPECT_LONGS_EQUAL(expectedValueBytes, result.totalD2hBytes);

  CHECK(result.packValuesHostBuildElapsed >= 0.0);
  CHECK(result.packValuesDeviceAllocElapsed >= 0.0);
  CHECK(result.packValuesH2dCopyElapsed >= 0.0);
  CHECK(result.projectionBatchHostBuildElapsed >= 0.0);
  CHECK(result.projectionBatchDeviceAllocElapsed >= 0.0);
  CHECK(result.projectionBatchH2dCopyElapsed >= 0.0);
  CHECK(result.downloadHostAllocElapsed >= 0.0);
  CHECK(result.downloadD2hCopyElapsed >= 0.0);
  CHECK(result.downloadValuesBuildElapsed >= 0.0);
}

TEST(CudaSfmDenseSchurSolver, MatchesFullNormalEquationDeltaOnTinyBal) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(data, context.stream());
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);
  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());

  constexpr double lambda = 1e-3;
  AccumulateCudaSfmNormalEquations(values, batch,
                                   static_cast<int>(structure.numCameras()),
                                   &system, context.stream());
  system.addDiagonalDamping(lambda, context.stream());

  CudaDeviceArray<double> fullDelta;
  CudssSpdSolver fullSolver;
  fullSolver.analyze(system, &fullDelta, context.stream());
  fullSolver.solve(system, &fullDelta, context.stream());

  CudaDeviceArray<double> schurDelta;
  SolveCudaSfmDenseSchur(values, batch, static_cast<int>(data.numberCameras()),
                         lambda, &schurDelta, context.stream());

  std::vector<double> full;
  std::vector<double> schur;
  fullDelta.download(&full, context.stream());
  schurDelta.download(&schur, context.stream());
  context.synchronize();

  LONGS_EQUAL(full.size(), schur.size());
  for (size_t i = 0; i < full.size(); ++i) {
    DOUBLES_EQUAL(full[i], schur[i], 1e-6);
  }
}

TEST(CudaSfmDenseSchurSolver, MatchesFullNormalEquationDeltaWithDiagonalDamping) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  CudaContext context;

  DeviceValues values = PackSfmValues(data, context.stream());
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);
  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());

  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<CudaSfmObservation> observations;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  batch.observations().download(&observations, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  context.synchronize();

  std::vector<double> dampingDiagonal(structure.dimension(), 0.0);
  for (size_t i = 0; i < observations.size(); ++i) {
    const CudaSfmObservation& observation = observations[i];
    const int cameraBase = 9 * observation.cameraSlot;
    const int pointBase =
        9 * static_cast<int>(structure.numCameras()) + 3 * observation.pointSlot;
    for (int col = 0; col < 9; ++col) {
      const double j0 = cameraJacobians[18 * i + col];
      const double j1 = cameraJacobians[18 * i + 9 + col];
      dampingDiagonal[cameraBase + col] += j0 * j0 + j1 * j1;
    }
    for (int col = 0; col < 3; ++col) {
      const double j0 = pointJacobians[6 * i + col];
      const double j1 = pointJacobians[6 * i + 3 + col];
      dampingDiagonal[pointBase + col] += j0 * j0 + j1 * j1;
    }
  }
  for (double& value : dampingDiagonal) {
    value = std::min(1e32, std::max(1e-6, value));
  }

  CudaDeviceArray<double> deviceDampingDiagonal;
  deviceDampingDiagonal.upload(dampingDiagonal, context.stream());

  constexpr double lambda = 1e-3;
  AccumulateCudaSfmNormalEquations(values, batch,
                                   static_cast<int>(structure.numCameras()),
                                   &system, context.stream());
  system.addDiagonalDamping(lambda, deviceDampingDiagonal, context.stream());

  CudaDeviceArray<double> fullDelta;
  CudssSpdSolver fullSolver;
  fullSolver.analyze(system, &fullDelta, context.stream());
  fullSolver.solve(system, &fullDelta, context.stream());

  CudaDeviceArray<double> schurDelta;
  SolveCudaSfmDenseSchur(values, batch, static_cast<int>(data.numberCameras()),
                         lambda, deviceDampingDiagonal, &schurDelta,
                         context.stream());

  std::vector<double> full;
  std::vector<double> schur;
  fullDelta.download(&full, context.stream());
  schurDelta.download(&schur, context.stream());
  context.synchronize();

  LONGS_EQUAL(full.size(), schur.size());
  for (size_t i = 0; i < full.size(); ++i) {
    DOUBLES_EQUAL(full[i], schur[i], 1e-6);
  }
}

TEST(CudaSfmDenseSchurSolver,
     MatchesFullNormalEquationDeltaOnHighDegreeTrack) {
  const SfmData data = makeHighDegreeBalLikeData();
  CudaContext context;

  DeviceValues values = PackSfmValues(data, context.stream());
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);
  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());

  constexpr double lambda = 1e-3;
  AccumulateCudaSfmNormalEquations(values, batch,
                                   static_cast<int>(structure.numCameras()),
                                   &system, context.stream());
  system.addDiagonalDamping(lambda, context.stream());

  CudaDeviceArray<double> fullDelta;
  CudssSpdSolver fullSolver;
  fullSolver.analyze(system, &fullDelta, context.stream());
  fullSolver.solve(system, &fullDelta, context.stream());

  CudaDeviceArray<double> schurDelta;
  SolveCudaSfmDenseSchur(values, batch, static_cast<int>(data.numberCameras()),
                         lambda, &schurDelta, context.stream());

  std::vector<double> full;
  std::vector<double> schur;
  fullDelta.download(&full, context.stream());
  schurDelta.download(&schur, context.stream());
  context.synchronize();

  LONGS_EQUAL(full.size(), schur.size());
  for (size_t i = 0; i < full.size(); ++i) {
    DOUBLES_EQUAL(full[i], schur[i], 1e-6);
  }
}
#endif

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

  CHECK(batch.noiseMode() == CudaSfmProjectionNoiseMode::Unit);
  EXPECT_LONGS_EQUAL(0, batch.sqrtInfos().size());
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

TEST(CudaSfmProjectionBatch, PacksLongTrackPointSlots) {
  const SfmData data = makeHighDegreeBalLikeData();
  CudaContext context;

  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());

  std::vector<int> longTrackPointSlots;
  batch.longTrackPointSlots().download(&longTrackPointSlots,
                                       context.stream());
  context.synchronize();

  LONGS_EQUAL(1, longTrackPointSlots.size());
  LONGS_EQUAL(0, longTrackPointSlots[0]);
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

namespace {

// Synthetic BAL-like problem for GNC: every point is observed by every
// camera, so a track stays well constrained even after GNC down-weights its
// corrupted measurements to zero.
struct GncTestProblem {
  NonlinearFactorGraph graph;
  NonlinearFactorGraph inlierGraph;
  Values initial;
  std::vector<size_t> outlierFactorSlots;

  bool isOutlierSlot(size_t slot) const {
    return std::find(outlierFactorSlots.begin(), outlierFactorSlots.end(),
                     slot) != outlierFactorSlots.end();
  }
};

GncTestProblem makeGncBalLikeProblem() {
  // The geometry must be rigid (diverse viewpoints, many points) and the
  // corruptions moderate: with a weakly constrained problem or extreme
  // outliers, plain LM can absorb the corrupted measurements into a
  // consistent (wrong) solution with near-zero residuals, and GNC has no
  // signal left to reject them.
  constexpr size_t kNumCameras = 5;
  std::vector<Point3> points;
  for (size_t j = 0; j < 20; ++j) {
    const double a = 2.399963 * static_cast<double>(j);  // golden angle
    const double r = 0.3 + 0.08 * static_cast<double>(j % 7);
    points.emplace_back(r * std::cos(a), r * std::sin(a),
                        4.0 + 0.37 * static_cast<double>((j * 5) % 9));
  }

  std::vector<SfmCamera> cameras;
  const std::vector<Point3> centers = {
      Point3(-1.5, 0.3, -0.4), Point3(-0.7, -0.9, 0.3), Point3(0.1, 0.8, -0.2),
      Point3(0.9, -0.4, 0.5), Point3(1.6, 0.6, -0.3)};
  for (size_t i = 0; i < kNumCameras; ++i) {
    const double s = static_cast<double>(i);
    cameras.emplace_back(
        Pose3(Rot3::RzRyRx(0.15 - 0.08 * s, 0.25 - 0.12 * s, 0.05 * s),
              centers[i]),
        Cal3Bundler(160.0 + 4.0 * s, 1e-4, -1e-6));
  }

  // Corrupted (pointSlot, cameraSlot) pairs; each corrupted track keeps
  // four clean views.
  const std::vector<std::pair<size_t, size_t>> corrupted = {
      {1, 0}, {7, 2}, {14, 4}};
  const std::vector<Point2> corruptions = {
      Point2(14.0, -10.0), Point2(-12.0, 9.0), Point2(10.0, 13.0)};

  GncTestProblem problem;
  const auto model = noiseModel::Unit::Create(2);
  for (size_t pointSlot = 0; pointSlot < points.size(); ++pointSlot) {
    for (size_t cameraSlot = 0; cameraSlot < kNumCameras; ++cameraSlot) {
      Point2 measured = cameras[cameraSlot].project2(points[pointSlot]);
      const bool isOutlier =
          std::find(corrupted.begin(), corrupted.end(),
                    std::make_pair(pointSlot, cameraSlot)) != corrupted.end();
      if (isOutlier) {
        measured += corruptions[problem.outlierFactorSlots.size()];
        problem.outlierFactorSlots.push_back(problem.graph.size());
      }
      auto factor = std::make_shared<BundlerProjectionFactor>(
          measured, model, C(cameraSlot), P(pointSlot));
      problem.graph.push_back(factor);
      if (!isOutlier) {
        problem.inlierGraph.push_back(factor);
      }
    }
  }

  for (size_t i = 0; i < kNumCameras; ++i) {
    const double sign = (i % 2 == 0) ? 1.0 : -1.0;
    Vector9 delta{0.002 * sign, -0.0015,    0.001 * sign, 0.03, -0.02 * sign,
                  0.025,        0.8 * sign, 1e-5,         -1e-7};
    problem.initial.insert(C(i), cameras[i].retract(delta));
  }
  for (size_t j = 0; j < points.size(); ++j) {
    const double sign = (j % 2 == 0) ? 1.0 : -1.0;
    problem.initial.insert(
        P(j), Point3(points[j].x() + 0.02 * sign, points[j].y() - 0.015,
                     points[j].z() + 0.03 * sign));
  }
  return problem;
}

}  // namespace

TEST(CudaSfmLevenbergMarquardtParams, EqualsComparesFields) {
  const CudaSfmLevenbergMarquardtParams a =
      CudaSfmLevenbergMarquardtParams::LegacyDefaults();
  CudaSfmLevenbergMarquardtParams b = a;
  CHECK(a.equals(b));

  b.lambdaInitial = 2.0 * a.lambdaInitial + 1.0;
  CHECK(!a.equals(b));

  b = a;
  b.linearSolver = CudaSfmLinearSolverType::CudssFullNormal;
  CHECK(!a.equals(b));

  b = a;
  b.enableDetailedProfiling = !a.enableDetailedProfiling;
  CHECK(!a.equals(b));
}

TEST(GncCudaSfmOptimizer, TlsClassificationMatchesCpuGnc) {
  const GncTestProblem problem = makeGncBalLikeProblem();

  GncParams<LevenbergMarquardtParams> cpuGncParams{LevenbergMarquardtParams()};
  cpuGncParams.setLossType(GncLossType::TLS);
  GncOptimizer<GncParams<LevenbergMarquardtParams>> cpuGnc(
      problem.graph, problem.initial, cpuGncParams);
  const Values cpuResult = cpuGnc.optimize();

  GncParams<CudaSfmLevenbergMarquardtParams> cudaGncParams{
      CudaSfmLevenbergMarquardtParams::LegacyDefaults()};
  cudaGncParams.setLossType(GncLossType::TLS);
  GncOptimizer<GncParams<CudaSfmLevenbergMarquardtParams>> cudaGnc(
      problem.graph, problem.initial, cudaGncParams);
  const Values cudaResult = cudaGnc.optimize();

  const Vector& cpuWeights = cpuGnc.getWeights();
  const Vector& cudaWeights = cudaGnc.getWeights();
  EXPECT_LONGS_EQUAL(problem.graph.size(), cudaWeights.size());
  for (size_t slot = 0; slot < problem.graph.size(); ++slot) {
    if (problem.isOutlierSlot(slot)) {
      CHECK(cpuWeights[slot] < 0.05);
      CHECK(cudaWeights[slot] < 0.05);
    } else {
      CHECK(cpuWeights[slot] > 0.95);
      CHECK(cudaWeights[slot] > 0.95);
    }
  }

  const double initialInlierError = problem.inlierGraph.error(problem.initial);
  const double cpuInlierError = problem.inlierGraph.error(cpuResult);
  const double cudaInlierError = problem.inlierGraph.error(cudaResult);
  CHECK(cpuInlierError < 1e-3);
  CHECK(cudaInlierError < 1e-3);
  CHECK(cudaInlierError < initialInlierError);
}

TEST(GncCudaSfmOptimizer, GmClassificationMatchesCpuGnc) {
  const GncTestProblem problem = makeGncBalLikeProblem();

  GncParams<LevenbergMarquardtParams> cpuGncParams{LevenbergMarquardtParams()};
  cpuGncParams.setLossType(GncLossType::GM);
  GncOptimizer<GncParams<LevenbergMarquardtParams>> cpuGnc(
      problem.graph, problem.initial, cpuGncParams);
  const Values cpuResult = cpuGnc.optimize();

  GncParams<CudaSfmLevenbergMarquardtParams> cudaGncParams{
      CudaSfmLevenbergMarquardtParams::LegacyDefaults()};
  cudaGncParams.setLossType(GncLossType::GM);
  GncOptimizer<GncParams<CudaSfmLevenbergMarquardtParams>> cudaGnc(
      problem.graph, problem.initial, cudaGncParams);
  const Values cudaResult = cudaGnc.optimize();

  // GM weights do not converge to exactly {0, 1}; check the separation.
  const Vector& cpuWeights = cpuGnc.getWeights();
  const Vector& cudaWeights = cudaGnc.getWeights();
  for (size_t slot = 0; slot < problem.graph.size(); ++slot) {
    if (problem.isOutlierSlot(slot)) {
      CHECK(cpuWeights[slot] < 0.5);
      CHECK(cudaWeights[slot] < 0.5);
    } else {
      CHECK(cpuWeights[slot] > 0.9);
      CHECK(cudaWeights[slot] > 0.9);
    }
  }

  CHECK(problem.inlierGraph.error(cpuResult) < 1e-2);
  CHECK(problem.inlierGraph.error(cudaResult) < 1e-2);
}

TEST(GncCudaSfmOptimizer, KnownOutliersProduceZeroInformationGraph) {
  // GNC weights a known outlier by zero, which reaches the CUDA backend as a
  // zero-information Gaussian noise model. The corrupted measurement must be
  // ignored by the optimization.
  const GncTestProblem problem = makeGncBalLikeProblem();

  NonlinearFactorGraph weightedGraph;
  for (size_t slot = 0; slot < problem.graph.size(); ++slot) {
    const auto factor =
        std::static_pointer_cast<NoiseModelFactor>(problem.graph[slot]);
    if (problem.isOutlierSlot(slot)) {
      // Same construction as GncOptimizer::makeWeightedGraph with weight 0.
      const auto zeroInformation =
          noiseModel::Gaussian::Information(Matrix2::Zero());
      weightedGraph.push_back(factor->cloneWithNewNoiseModel(zeroInformation));
    } else {
      weightedGraph.push_back(factor);
    }
  }

  CudaSfmLevenbergMarquardtParams params =
      CudaSfmLevenbergMarquardtParams::LegacyDefaults();
  CudaSfmLevenbergMarquardtOptimizer optimizer(weightedGraph, problem.initial,
                                               params);
  const Values& result = optimizer.optimize();

  CHECK(problem.inlierGraph.error(result) < 1e-3);
  // The zero-information factors contribute exactly zero error.
  DOUBLES_EQUAL(problem.inlierGraph.error(result),
                weightedGraph.error(result), 1e-9);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
