#include <gtsam/base/cuda/Context.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/cuda/CudssSpdSolver.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
#include <gtsam/slam/cuda/BalCsrStructure.h>
#include <gtsam/slam/cuda/SfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/SfmFullNormalProblem.h>
#include <gtsam/slam/cuda/SfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/SfmProjectionLinearization.h>
#include <gtsam/slam/cuda/SfmProjectionBatch.h>
#include <gtsam/slam/cuda/SfmReducedCsrPlan.h>
#include <gtsam/slam/cuda/SfmSchurProblem.h>
#include <gtsam/slam/cuda/SfmValues.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/sfm/SfmData.h>

#include <CppUnitLite/TestHarness.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace sfm_fixture {
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::P;

static_assert(std::is_base_of_v<LevenbergMarquardtParams,
                                SfmLevenbergMarquardtParams>);

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

bool deviceCameraEquals(const DevicePinholeCameraCal3Bundler& expected,
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
                     const SfmObservation& observation) {
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
    const SfmObservation& observation) {
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

SfmSqrtInfo2 MakeSqrtInfo(double r00, double r01, double r11) {
  return SfmSqrtInfo2{r00, r01, r11};
}

SfmRobustModel MakeRobustModel(
    SfmRobustModelKind kind, double parameter,
    SfmRobustReweightScheme reweightScheme =
        SfmRobustReweightScheme::Block) {
  return SfmRobustModel{kind, reweightScheme, parameter};
}

#if GTSAM_ENABLE_CUDSS
bool ConvertedSfmDataEquals(const SfmFactorGraphData& expected,
                            const SfmFactorGraphData& actual) {
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
#endif

Vector2 WhitenResidual(const SfmSqrtInfo2& sqrtInfo,
                       const Vector2& residual) {
  return Vector2(sqrtInfo.r00 * residual(0) +
                     sqrtInfo.r01 * residual(1),
                 sqrtInfo.r11 * residual(1));
}

double RobustWeight(const SfmRobustModel& model, double distance) {
  const double absDistance = std::abs(distance);
  switch (model.kind) {
    case SfmRobustModelKind::None:
      return 1.0;
    case SfmRobustModelKind::Huber:
      return absDistance <= model.parameter ? 1.0
                                            : model.parameter / absDistance;
    case SfmRobustModelKind::Tukey: {
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

double RobustLoss(const SfmRobustModel& model, double distance) {
  const double absDistance = std::abs(distance);
  switch (model.kind) {
    case SfmRobustModelKind::None:
      return 0.5 * distance * distance;
    case SfmRobustModelKind::Huber:
      if (absDistance <= model.parameter) {
        return 0.5 * distance * distance;
      }
      return model.parameter * (absDistance - 0.5 * model.parameter);
    case SfmRobustModelKind::Tukey: {
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

void RobustRowScales(const SfmRobustModel& model,
                     const Vector2& whitenedResidual, double* row0Scale,
                     double* row1Scale) {
  if (model.reweightScheme == SfmRobustReweightScheme::Scalar) {
    *row0Scale = std::sqrt(RobustWeight(model, whitenedResidual(0)));
    *row1Scale = std::sqrt(RobustWeight(model, whitenedResidual(1)));
  } else {
    const double scale = std::sqrt(RobustWeight(
        model, std::sqrt(whitenedResidual.squaredNorm())));
    *row0Scale = scale;
    *row1Scale = scale;
  }
}

std::vector<std::vector<SfmSqrtInfo2>> MakeMixedSqrtInfoByTrack(
    const SfmData& data) {
  const SfmSqrtInfo2 models[] = {
      MakeSqrtInfo(2.0, 0.0, 2.0), MakeSqrtInfo(4.0, 0.0, 2.0),
      MakeSqrtInfo(3.0, 0.25, 4.0)};
  std::vector<std::vector<SfmSqrtInfo2>> sqrtInfoByTrack(
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

std::vector<std::vector<SfmRobustModel>> MakeMixedRobustModelsByTrack(
    const SfmData& data) {
  const SfmRobustModel models[] = {
      MakeRobustModel(SfmRobustModelKind::Huber, 0.25),
      MakeRobustModel(SfmRobustModelKind::Huber, 0.20,
                      SfmRobustReweightScheme::Scalar),
      MakeRobustModel(SfmRobustModelKind::Tukey, 0.75)};
  std::vector<std::vector<SfmRobustModel>> robustModelsByTrack(
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

// Verifies SfmReducedCsrPlan::BuildsStableCameraOnlyCovisibilityPattern.
TEST(SfmReducedCsrPlan, BuildsStableCameraOnlyCovisibilityPattern) {
  SfmData data;
  for (int camera = 0; camera < 4; ++camera) {
    data.cameras.emplace_back(Pose3(), Cal3Bundler(100.0, 0.0, 0.0));
  }
  const std::vector<std::vector<size_t>> tracks{{0, 1}, {1, 2}, {0, 2, 3}};
  for (size_t point = 0; point < tracks.size(); ++point) {
    SfmTrack track(Point3(static_cast<double>(point), 0.0, 5.0));
    for (const size_t camera : tracks[point]) {
      track.measurements.emplace_back(camera, Point2(0.0, 0.0));
    }
    data.tracks.push_back(track);
  }
  const std::vector<Key> cameraKeys{C(0), C(1), C(2), C(3)};
  const SfmReducedCsrPlan plan(data, cameraKeys);
  const SfmReducedCsrPlan repeated(data, cameraKeys);

  LONGS_EQUAL(36, plan.dimension());
  EXPECT(plan.rowPointers() == repeated.rowPointers());
  EXPECT(plan.columnIndices() == repeated.columnIndices());
  EXPECT(plan.hasCameraPair(0, 0));
  EXPECT(plan.hasCameraPair(0, 1));
  EXPECT(plan.hasCameraPair(0, 2));
  EXPECT(plan.hasCameraPair(0, 3));
  EXPECT(plan.hasCameraPair(1, 2));
  EXPECT(plan.hasCameraPair(2, 3));
  EXPECT(!plan.hasCameraPair(1, 3));
  LONGS_EQUAL(plan.valueOffset(0, 2, 4, 7),
              plan.valueOffset(2, 0, 7, 4));
  for (int row = 0; row < plan.dimension(); ++row) {
    const auto begin = plan.columnIndices().begin() + plan.rowPointers()[row];
    const auto end =
        plan.columnIndices().begin() + plan.rowPointers()[row + 1];
    EXPECT(std::is_sorted(begin, end));
  }

  const std::vector<int> scalarPermutation = compileScalarPermutation(
      plan.cameraBlocks(), Ordering{C(2), C(0), C(3), C(1)});
  std::vector<int> expected;
  for (const int camera : {2, 0, 3, 1}) {
    for (int scalar = 0; scalar < 9; ++scalar) {
      expected.push_back(9 * camera + scalar);
    }
  }
  EXPECT(expected == scalarPermutation);
  const Ordering automatic = plan.colamdOrdering();
  LONGS_EQUAL(4, automatic.size());
}

// Accepts canonical solver names with case and separator normalization only.
TEST(SfmLevenbergMarquardtParams, NormalizesCanonicalLinearSolverNames) {
  SfmLevenbergMarquardtParams params;

  CHECK(params.getLinearSolver() == "dense-schur");

  params.setLinearSolver("cudss-full-normal");
  CHECK(params.getLinearSolver() == "cudss-full-normal");

  params.setLinearSolver("DENSE_SCHUR");
  CHECK(params.getLinearSolver() == "dense-schur");

  params.setLinearSolver("CUDSS_FULL_NORMAL");
  CHECK(params.getLinearSolver() == "cudss-full-normal");

  CHECK_EXCEPTION(params.setLinearSolver("full-normal-pcg"),
                  std::invalid_argument);
  CHECK_EXCEPTION(params.setLinearSolver("schur-cudss"),
                  std::invalid_argument);
  CHECK_EXCEPTION(params.setLinearSolver("schur-pcg"),
                  std::invalid_argument);
  CHECK_EXCEPTION(params.setLinearSolver("not-a-solver"),
                  std::invalid_argument);
  CHECK(params.getLinearSolver() == "cudss-full-normal");
}

// Verifies SfmLevenbergMarquardtParams::SeparatesFormulationFromLinearBackend.
TEST(SfmLevenbergMarquardtParams,
     SeparatesFormulationFromLinearBackend) {
  SfmLevenbergMarquardtParams params;
  CHECK(params.formulation == SfmSystemFormulation::Schur);
  CHECK(params.linear.backend == LinearSolverType::DenseCholesky);

  params.setLinearSolverBackend("cudss");
  CHECK(params.getFormulation() == "schur");
  CHECK(params.getLinearSolverBackend() == "cudss");
  CHECK(params.getLinearSolver() == "cudss-schur");

  params.setFormulation("full_normal");
  CHECK(params.getFormulation() == "full-normal");
  CHECK(params.getLinearSolver() == "cudss-full-normal");

  params.setLinearSolverBackend("dense-cholesky");
  const SfmData data = makeTinyBalData();
  CHECK_EXCEPTION(optimizeSfmWithoutValueDownload(data, params),
                  std::invalid_argument);

  params.setFormulation("schur");
  params.setLinearSolverBackend("dense-cholesky");
  CHECK(params.getLinearSolver() == "dense-schur");
  CHECK_EXCEPTION(params.setFormulation("fullnormal"),
                  std::invalid_argument);
  CHECK_EXCEPTION(params.setLinearSolverBackend("dense"),
                  std::invalid_argument);
  params.setLinearSolverBackend("pcg");
  params.ordering = Ordering{C(0), C(1)};
  CHECK_EXCEPTION(optimizeSfmWithoutValueDownload(data, params),
                  std::invalid_argument);
}

// Verifies SfmLevenbergMarquardtParams::ProvidesLmDefaults.
TEST(SfmLevenbergMarquardtParams, ProvidesLmDefaults) {
  const SfmLevenbergMarquardtParams legacy =
      SfmLevenbergMarquardtParams::legacyDefaults();
  CHECK(legacy.maxIterations == 100);
  DOUBLES_EQUAL(1e-5, legacy.lambdaInitial, 0.0);
  DOUBLES_EQUAL(10.0, legacy.lambdaFactor, 0.0);
  CHECK(!legacy.dampingParams.diagonalDamping);
  CHECK(!legacy.enableDetailedProfiling);

  const SfmLevenbergMarquardtParams ceres =
      SfmLevenbergMarquardtParams::ceresDefaults();
  CHECK(ceres.maxIterations == 50);
  DOUBLES_EQUAL(1e-4, ceres.lambdaInitial, 0.0);
  DOUBLES_EQUAL(2.0, ceres.lambdaFactor, 0.0);
  CHECK(ceres.dampingParams.diagonalDamping);
  CHECK(!ceres.useFixedLambdaFactor);
  CHECK(!ceres.enableDetailedProfiling);
}

// Verifies SfmLevenbergMarquardtOptimizer::ExposesCudaParams.
TEST(SfmLevenbergMarquardtOptimizer, ExposesCudaParams) {
  NonlinearFactorGraph graph;
  Values initial;
  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.setLinearSolver("cudss-full-normal");
  params.maxIterations = 7;

  SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);

  CHECK(optimizer.params().getLinearSolver() == "cudss-full-normal");
  CHECK(optimizer.params().maxIterations == 7);
}

// The batch optimizer does not advertise an unsupported incremental API.
TEST(SfmLevenbergMarquardtOptimizer, UsesBatchOnlyInterface) {
  CHECK((!std::is_base_of_v<NonlinearOptimizer,
                             SfmLevenbergMarquardtOptimizer>));
}

// Verifies SfmProjectionLinearization::MatchesHostResidualsAndNumericJacobians.
TEST(SfmProjectionLinearization,
     MatchesHostResidualsAndNumericJacobians) {
  constexpr double kResidualTolerance = 1e-7;
  constexpr double kJacobianTolerance = 5e-4;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<SfmObservation> observations;
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
    const SfmObservation& observation = observations[i];
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
      computeSfmProjectionError(values, batch, context.stream());
  DOUBLES_EQUAL(expectedError, actualError, kResidualTolerance);
}

// Verifies SfmProjectionLinearization::AppliesPerObservationGaussianWhitening.
TEST(SfmProjectionLinearization,
     AppliesPerObservationGaussianWhitening) {
  constexpr double kResidualTolerance = 1e-7;
  constexpr double kJacobianTolerance = 3e-3;
  constexpr double kSystemTolerance = 1e-5;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<std::vector<SfmSqrtInfo2>> sqrtInfoByTrack =
      MakeMixedSqrtInfoByTrack(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch = SfmProjectionBatch::fromSfmData(
      measuredData, sqrtInfoByTrack, context.stream());
  CHECK(batch.noiseMode() == SfmProjectionNoiseMode::Whitened);
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  const BalCsrStructure structure =
      BalCsrStructure::fromSfmData(measuredData);
  DeviceSparseSpdSystem system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());
  accumulateSfmNormalEquations(
      values, batch, static_cast<int>(structure.numCameras()), &system,
      context.stream());

  DeviceArray<double> actualDeviceDiagonal;
  computeSfmHessianDiagonal(values, batch,
                                static_cast<int>(structure.numCameras()), 1e-6,
                                1e32, &actualDeviceDiagonal, context.stream());

  std::vector<SfmObservation> observations;
  std::vector<SfmSqrtInfo2> sqrtInfos;
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
    const SfmObservation& observation = observations[i];
    const SfmSqrtInfo2& sqrtInfo = sqrtInfos[i];
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
      computeSfmProjectionError(values, batch, context.stream());
  DOUBLES_EQUAL(expectedError, actualError, kResidualTolerance);

  const int dimension = structure.dimension();
  std::vector<double> expectedDense(static_cast<size_t>(dimension) *
                                    static_cast<size_t>(dimension));
  std::vector<double> expectedRhs(static_cast<size_t>(dimension));
  std::vector<double> expectedDiagonal(static_cast<size_t>(dimension));

  for (size_t i = 0; i < observations.size(); ++i) {
    const SfmObservation& observation = observations[i];
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

// Verifies SfmProjectionLinearization::AppliesPerObservationRobustWhiteningAndLoss.
TEST(SfmProjectionLinearization,
     AppliesPerObservationRobustWhiteningAndLoss) {
  constexpr double kResidualTolerance = 1e-7;
  constexpr double kJacobianTolerance = 3e-3;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  const std::vector<std::vector<SfmSqrtInfo2>> sqrtInfoByTrack =
      MakeMixedSqrtInfoByTrack(measuredData);
  const std::vector<std::vector<SfmRobustModel>> robustModelsByTrack =
      MakeMixedRobustModelsByTrack(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch = SfmProjectionBatch::fromSfmData(
      measuredData, sqrtInfoByTrack, robustModelsByTrack, context.stream());
  CHECK(batch.noiseMode() == SfmProjectionNoiseMode::Robust);
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<SfmObservation> observations;
  std::vector<SfmSqrtInfo2> sqrtInfos;
  std::vector<SfmRobustModel> robustModels;
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
    const SfmObservation& observation = observations[i];
    const SfmSqrtInfo2& sqrtInfo = sqrtInfos[i];
    const SfmRobustModel& robustModel = robustModels[i];
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
      computeSfmProjectionError(values, batch, context.stream());
  DOUBLES_EQUAL(expectedError, actualError, kResidualTolerance);
}

// Verifies SfmProjectionLinearization::AccumulatesProjectionNormalEquationsIntoCsr.
TEST(SfmProjectionLinearization,
     AccumulatesProjectionNormalEquationsIntoCsr) {
  constexpr double kTolerance = 1e-6;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  const BalCsrStructure structure =
      BalCsrStructure::fromSfmData(measuredData);
  DeviceSparseSpdSystem system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());
  system.values().upload(
      std::vector<double>(structure.colIndices().size(), 123.0),
      context.stream());
  system.rhs().upload(std::vector<double>(structure.dimension(), -456.0),
                      context.stream());

  accumulateSfmNormalEquations(
      values, batch, static_cast<int>(structure.numCameras()), &system,
      context.stream());

  std::vector<SfmObservation> observations;
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
    const SfmObservation& observation = observations[i];
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

// Verifies SfmProjectionLinearization::ComputesClampedHessianDiagonal.
TEST(SfmProjectionLinearization, ComputesClampedHessianDiagonal) {
  constexpr double kTolerance = 1e-6;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  DeviceArray<double> actualDeviceDiagonal;
  computeSfmHessianDiagonal(values, batch, 2, 1e-6, 1e32,
                                &actualDeviceDiagonal, context.stream());

  std::vector<SfmObservation> observations;
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
    const SfmObservation& observation = observations[i];
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

// Verifies SfmFullNormalProblem::ReusesOneLinearizationAcrossDampingAttempts.
TEST(SfmFullNormalProblem, ReusesOneLinearizationAcrossDampingAttempts) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  SfmFullNormalProblem problem;
  problem.initialize(batch, static_cast<int>(measuredData.numberCameras()),
                     context.stream());
  problem.linearize(linearization, context.stream());
  const SfmFullNormalView first = problem.prepare(1e-3, context.stream());
  const SfmFullNormalView second = problem.prepare(1e-2, context.stream());
  context.synchronize();

  CHECK(first.linearOperator != nullptr);
  CHECK(first.preconditioner != nullptr);
  CHECK(first.rhs != nullptr);
  CHECK(second.linearOperator == first.linearOperator);
  CHECK(second.preconditioner == first.preconditioner);
  CHECK(second.rhs == first.rhs);
  EXPECT_LONGS_EQUAL(1, problem.linearizationCount());
  EXPECT_LONGS_EQUAL(2, problem.preparationCount());
}

// Verifies SfmFullNormalProblem::RestoresExplicitSystemBetweenDampingAttempts.
TEST(SfmFullNormalProblem, RestoresExplicitSystemBetweenDampingAttempts) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;
  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());
  const BalCsrStructure structure =
      BalCsrStructure::fromSfmData(measuredData);

  SfmFullNormalProblem problem;
  problem.initializeSparse(batch,
                           static_cast<int>(measuredData.numberCameras()),
                           structure.rowPointers(), structure.colIndices(),
                           context.stream());
  problem.linearize(linearization, context.stream());

  DeviceSparseSpdSystem& first =
      problem.prepareSparse(1e-3, context.stream());
  std::vector<double> firstValues;
  std::vector<double> firstRhs;
  first.values().download(&firstValues, context.stream());
  first.rhs().download(&firstRhs, context.stream());
  context.synchronize();

  DeviceSparseSpdSystem& second =
      problem.prepareSparse(4e-3, context.stream());
  std::vector<double> secondValues;
  second.values().download(&secondValues, context.stream());
  context.synchronize();

  DeviceSparseSpdSystem& repeatedFirst =
      problem.prepareSparse(1e-3, context.stream());
  std::vector<double> repeatedFirstValues;
  std::vector<double> repeatedFirstRhs;
  repeatedFirst.values().download(&repeatedFirstValues, context.stream());
  repeatedFirst.rhs().download(&repeatedFirstRhs, context.stream());
  context.synchronize();

  EXPECT(firstValues != secondValues);
  LONGS_EQUAL(firstValues.size(), repeatedFirstValues.size());
  for (size_t i = 0; i < firstValues.size(); ++i) {
    DOUBLES_EQUAL(firstValues[i], repeatedFirstValues[i], 1e-12);
  }
  LONGS_EQUAL(firstRhs.size(), repeatedFirstRhs.size());
  for (size_t i = 0; i < firstRhs.size(); ++i) {
    DOUBLES_EQUAL(firstRhs[i], repeatedFirstRhs[i], 1e-12);
  }
  EXPECT_LONGS_EQUAL(1, problem.linearizationCount());
  EXPECT_LONGS_EQUAL(3, problem.preparationCount());
}

// Verifies SfmProjectionLinearization::ComputesLinearizedErrorChange.
TEST(SfmProjectionLinearization, ComputesLinearizedErrorChange) {
  constexpr double kTolerance = 1e-6;

  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());
  DeviceArray<double> delta;
  solveSfmDenseSchur(values, batch,
                         static_cast<int>(measuredData.numberCameras()),
                         1e-3, &delta, context.stream());

  double oldLinearizedError = 0.0;
  double newLinearizedError = 0.0;
  const double actualChange = computeSfmLinearizedErrorChange(
      values, batch, static_cast<int>(measuredData.numberCameras()), delta,
      &oldLinearizedError, &newLinearizedError, context.stream());

  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());
  std::vector<SfmObservation> observations;
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
    const SfmObservation& observation = observations[i];
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

// Verifies SfmProjectionLinearization::RejectsIncompleteCsrPattern.
TEST(SfmProjectionLinearization, RejectsIncompleteCsrPattern) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());

  const BalCsrStructure structure =
      BalCsrStructure::fromSfmData(measuredData);
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

  DeviceSparseSpdSystem system;
  system.uploadPattern(structure.dimension(), rowPointers, colIndices,
                       context.stream());

  CHECK_EXCEPTION(accumulateSfmNormalEquations(
                      values, batch, static_cast<int>(structure.numCameras()),
                      &system, context.stream()),
                  std::runtime_error);
}

// Verifies SfmProjectionLinearization::RejectsInvalidSystemWithoutClearingExistingValues.
TEST(SfmProjectionLinearization,
     RejectsInvalidSystemWithoutClearingExistingValues) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData initialData = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(initialData, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());

  const BalCsrStructure structure =
      BalCsrStructure::fromSfmData(measuredData);
  std::vector<int> rowPointers = structure.rowPointers();
  std::vector<int> colIndices = structure.colIndices();
  rowPointers.push_back(rowPointers.back() + 1);
  colIndices.push_back(structure.dimension());

  DeviceSparseSpdSystem system;
  system.uploadPattern(structure.dimension() + 1, rowPointers, colIndices,
                       context.stream());
  system.values().upload(std::vector<double>(colIndices.size(), 7.0),
                         context.stream());
  system.rhs().upload(std::vector<double>(structure.dimension() + 1, -8.0),
                      context.stream());

  CHECK_EXCEPTION(accumulateSfmNormalEquations(
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
// Verifies SfmProjectionLinearization::ReturnsZerosForCheiralityFailures.
TEST(SfmProjectionLinearization, ReturnsZerosForCheiralityFailures) {
  const SfmData data = makeBehindCameraData();
  Context context;

  DeviceValues values = packSfmValues(data, context.stream());
  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<double> residuals;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  std::vector<SfmObservation> observations;
  linearization.residuals.download(&residuals, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  batch.observations().download(&observations, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, batch.numObservations());
  for (const SfmObservation& observation : observations) {
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

// Verifies SfmProjectionLinearization::RejectsMismatchedValueShapes.
TEST(SfmProjectionLinearization, RejectsMismatchedValueShapes) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData smallerValuesData = makeTinyBalData();
  SfmData fewerCamerasData = measuredData;
  fewerCamerasData.cameras.resize(1);
  Context context;

  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(measuredData, context.stream());
  SfmProjectionLinearization linearization;

  DeviceValues fewerCameraValues =
      packSfmValues(fewerCamerasData, context.stream());
  CHECK_EXCEPTION(linearizeSfmProjectionBatch(
                      fewerCameraValues, batch, &linearization,
                      context.stream()),
                  std::invalid_argument);

  DeviceValues fewerPointValues =
      packSfmValues(smallerValuesData, context.stream());
  CHECK_EXCEPTION(linearizeSfmProjectionBatch(
                      fewerPointValues, batch, &linearization,
                      context.stream()),
                  std::invalid_argument);
}

#if !GTSAM_ENABLE_CUDSS
// Verifies SfmLevenbergMarquardt::DenseSchurRunsWithoutCudss.
TEST(SfmLevenbergMarquardt, DenseSchurRunsWithoutCudss) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params;
  params.setLinearSolver("dense-schur");
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  const SfmLevenbergMarquardtResult result =
      optimizeSfmWithoutValueDownload(data, params);

  CHECK(result.innerIterations > 0);
  CHECK(result.finalError < result.initialError);
  CHECK(result.optimizedValues.empty());
  CHECK(result.linearSolveStats.backend ==
        LinearSolverType::DenseCholesky);
  EXPECT_LONGS_EQUAL(1, result.linearSolveStats.analysisCount);
  CHECK(result.linearSolveStats.solveCount > 0);
}

#endif

// Verifies SfmLevenbergMarquardt::ImplicitSchurPcgMatchesDense.
TEST(SfmLevenbergMarquardt, ImplicitSchurPcgMatchesDense) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  SfmLevenbergMarquardtParams denseParams =
      SfmLevenbergMarquardtParams::ceresDefaults();
  denseParams.maxIterations = 2;
  denseParams.relativeErrorTol = 0.0;
  const SfmLevenbergMarquardtResult dense =
      optimizeSfmWithoutValueDownload(data, denseParams);

  SfmLevenbergMarquardtParams pcgParams = denseParams;
  pcgParams.setLinearSolver("pcg-schur");
  pcgParams.pcg.maxIterations = 200;
  pcgParams.pcg.relativeTolerance = 1e-10;
  pcgParams.pcg.convergenceCheckInterval = 1;
  pcgParams.pcg.warmStart = false;
  const SfmLevenbergMarquardtResult pcg =
      optimizeSfmWithoutValueDownload(data, pcgParams);

  DOUBLES_EQUAL(dense.finalError, pcg.finalError, 1e-5);
  CHECK(pcg.linearSolveStats.lastPcgConverged);
  CHECK(pcg.linearSolveStats.solveCount > 0);
  CHECK(pcg.linearSolveStats.pcgIterationsTotal > 0);
}

// Verifies SfmLevenbergMarquardt::FullNormalPcgUsesSharedIterativeSession.
TEST(SfmLevenbergMarquardt, FullNormalPcgUsesSharedIterativeSession) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.formulation = SfmSystemFormulation::FullNormal;
  params.linear.backend = LinearSolverType::Pcg;
  params.maxIterations = 1;
  params.pcg.maxIterations = 300;
  params.pcg.relativeTolerance = 1e-9;
  params.pcg.convergenceCheckInterval = 1;
  params.pcg.warmStart = false;
  params.enableDetailedProfiling = true;

  const SfmLevenbergMarquardtResult result =
      optimizeSfmWithoutValueDownload(data, params);
  CHECK(result.finalError < result.initialError);
  CHECK(result.linearSolveStats.backend == LinearSolverType::Pcg);
  CHECK(result.linearSystemKind == LinearSystemKind::Operator);
  EXPECT_LONGS_EQUAL(0, result.linearSystemNonzeros);
  CHECK(result.linearSolveStats.lastPcgConverged);
  CHECK(result.linearSolveStats.pcgIterationsTotal > 0);
  CHECK(result.linearSolveStats.pcgD2hBytes > 0);
  CHECK(result.totalD2hBytes >= result.linearSolveStats.pcgD2hBytes);
  CHECK(result.totalD2hCopyElapsed >=
        result.linearSolveStats.pcgD2hSeconds);
  CHECK(result.linearSolveStats.pcgD2hSeconds > 0.0);
  CHECK(result.linearSolveStats.pcgD2hSeconds <
        result.linearSolveStats.solveSeconds);
}

#if GTSAM_ENABLE_CUDSS

// Verifies SfmLevenbergMarquardt::SparseSchurCudssMatchesDenseAndAppliesCameraOrdering.
TEST(SfmLevenbergMarquardt,
     SparseSchurCudssMatchesDenseAndAppliesCameraOrdering) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  SfmLevenbergMarquardtParams denseParams =
      SfmLevenbergMarquardtParams::ceresDefaults();
  denseParams.maxIterations = 2;
  denseParams.relativeErrorTol = 0.0;
  const SfmLevenbergMarquardtResult dense =
      optimizeSfmWithoutValueDownload(data, denseParams);

  SfmLevenbergMarquardtParams sparseParams = denseParams;
  sparseParams.setLinearSolver("cudss-schur");
  sparseParams.ordering = Ordering{C(1), C(0)};
  const SfmLevenbergMarquardtResult sparse =
      optimizeSfmWithoutValueDownload(data, sparseParams);

  DOUBLES_EQUAL(dense.finalError, sparse.finalError, 1e-6);
  CHECK(sparse.linearSolveStats.userOrderingApplied);
  CHECK(sparse.linearSolveStats.analysisCount == 1);
  CHECK(sparse.linearSolveStats.solveCount > 0);
  CHECK(sparse.formulation == SfmSystemFormulation::Schur);
  CHECK(sparse.linearBackend == LinearSolverType::Cudss);
  EXPECT_LONGS_EQUAL(18, sparse.linearSystemDimension);
  CHECK(sparse.linearSystemNonzeros > 0);
  EXPECT(sparse.appliedScalarPermutation ==
         compileScalarPermutation(
             SfmReducedCsrPlan(data, {C(0), C(1)}).cameraBlocks(),
             *sparseParams.ordering));
}

// Verifies SfmLevenbergMarquardt::FullNormalCudssAppliesCameraAndPointOrdering.
TEST(SfmLevenbergMarquardt,
     FullNormalCudssAppliesCameraAndPointOrdering) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.setLinearSolver("cudss-full-normal");
  params.maxIterations = 1;
  params.ordering = Ordering{P(3), C(1), P(0), C(0), P(1), P(2)};

  const SfmLevenbergMarquardtResult result =
      optimizeSfmWithoutValueDownload(data, params);
  CHECK(result.linearSolveStats.userOrderingApplied);
  EXPECT_LONGS_EQUAL(1, result.linearSolveStats.analysisCount);
  std::vector<int> expected;
  for (const int scalar : {27, 28, 29, 9, 10, 11, 12, 13, 14, 15, 16, 17,
                           18, 19, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 21, 22,
                           23, 24, 25, 26}) {
    expected.push_back(scalar);
  }
  EXPECT(expected == result.appliedScalarPermutation);
}

// Verifies SfmFactorGraphConversion::ConvertsGeneralSfmFactorsWithArbitraryKeys.
TEST(SfmFactorGraphConversion,
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

  const SfmFactorGraphData converted =
      convertGeneralSfmGraph(graph, initial);

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

// Verifies SfmFactorGraphConversion::ConvertsPointBatchedGeneralSfmFactors.
TEST(SfmFactorGraphConversion,
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

  const SfmFactorGraphData raw =
      convertGeneralSfmGraph(rawGraph, initial);
  const SfmFactorGraphData batched =
      convertGeneralSfmGraph(batchGraph, initial);
  CHECK(ConvertedSfmDataEquals(raw, batched));
}

// Verifies SfmFactorGraphConversion::ConvertsCameraBatchedGeneralSfmFactors.
TEST(SfmFactorGraphConversion,
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

  const SfmFactorGraphData raw =
      convertGeneralSfmGraph(rawGraph, initial);
  const SfmFactorGraphData batched =
      convertGeneralSfmGraph(batchGraph, initial);
  CHECK(ConvertedSfmDataEquals(raw, batched));
}

// Verifies SfmFactorGraphConversion::AcceptsFixedGaussianNoiseAndPreservesFlattenedWhiteningOrder.
TEST(SfmFactorGraphConversion,
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

  const SfmFactorGraphData converted =
      convertGeneralSfmGraph(graph, initial);

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

  Context context;
  const SfmProjectionBatch batch = SfmProjectionBatch::fromSfmData(
      converted.data, converted.sqrtInfoByTrack, context.stream());
  CHECK(batch.noiseMode() == SfmProjectionNoiseMode::Whitened);

  std::vector<SfmObservation> observations;
  std::vector<SfmSqrtInfo2> sqrtInfos;
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

// Verifies SfmFactorGraphConversion::AcceptsHuberAndTukeyRobustNoiseAndPreservesFlattenedOrder.
TEST(SfmFactorGraphConversion,
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

  const SfmFactorGraphData converted =
      convertGeneralSfmGraph(graph, initial);

  CHECK(converted.hasNonUnitNoise);
  CHECK(converted.hasRobustNoise);
  EXPECT_LONGS_EQUAL(measuredData.numberTracks(),
                     converted.sqrtInfoByTrack.size());
  EXPECT_LONGS_EQUAL(measuredData.numberTracks(),
                     converted.robustModelsByTrack.size());
  EXPECT_LONGS_EQUAL(2, converted.robustModelsByTrack[0].size());
  EXPECT_LONGS_EQUAL(2, converted.robustModelsByTrack[1].size());
  CHECK(converted.robustModelsByTrack[0][0].kind ==
        SfmRobustModelKind::Huber);
  CHECK(converted.robustModelsByTrack[0][0].reweightScheme ==
        SfmRobustReweightScheme::Scalar);
  CHECK(converted.robustModelsByTrack[0][1].kind ==
        SfmRobustModelKind::Huber);
  CHECK(converted.robustModelsByTrack[1][0].kind ==
        SfmRobustModelKind::Tukey);
  DOUBLES_EQUAL(0.75, converted.robustModelsByTrack[1][0].parameter,
                1e-12);
  DOUBLES_EQUAL(3.0, converted.sqrtInfoByTrack[1][0].r00, 1e-12);
  DOUBLES_EQUAL(0.25, converted.sqrtInfoByTrack[1][0].r01, 1e-12);
  DOUBLES_EQUAL(4.0, converted.sqrtInfoByTrack[1][0].r11, 1e-12);
  DOUBLES_EQUAL(4.0, converted.sqrtInfoByTrack[0][0].r00, 1e-12);
  DOUBLES_EQUAL(2.0, converted.sqrtInfoByTrack[0][0].r11, 1e-12);
  DOUBLES_EQUAL(1.0, converted.sqrtInfoByTrack[0][1].r00, 1e-12);
  DOUBLES_EQUAL(1.0, converted.sqrtInfoByTrack[0][1].r11, 1e-12);

  Context context;
  const SfmProjectionBatch batch = SfmProjectionBatch::fromSfmData(
      converted.data, converted.sqrtInfoByTrack,
      converted.robustModelsByTrack, context.stream());
  CHECK(batch.noiseMode() == SfmProjectionNoiseMode::Robust);

  std::vector<SfmRobustModel> robustModels;
  batch.robustModels().download(&robustModels, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(4, robustModels.size());
  CHECK(robustModels[0].kind == SfmRobustModelKind::Huber);
  CHECK(robustModels[0].reweightScheme ==
        SfmRobustReweightScheme::Scalar);
  CHECK(robustModels[2].kind == SfmRobustModelKind::Tukey);
  DOUBLES_EQUAL(0.75, robustModels[2].parameter, 1e-12);
}

// Verifies SfmFactorGraphConversion::RejectsUnsupportedNoiseModels.
TEST(SfmFactorGraphConversion, RejectsUnsupportedNoiseModels) {
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
      convertGeneralSfmGraph(unsupportedRobustGraph, initial),
      std::invalid_argument);

  NonlinearFactorGraph constrainedGraph;
  constrainedGraph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(0).second,
      noiseModel::Constrained::All(2), C(0), P(0));
  CHECK_EXCEPTION(
      convertGeneralSfmGraph(constrainedGraph, initial),
      std::invalid_argument);

  NonlinearFactorGraph wrongDimensionGraph;
  wrongDimensionGraph.emplace_shared<BundlerProjectionFactor>(
      measuredData.track(0).measurement(0).second,
      noiseModel::Isotropic::Sigma(3, 0.5), C(0), P(0));
  CHECK_EXCEPTION(
      convertGeneralSfmGraph(wrongDimensionGraph, initial),
      std::invalid_argument);

  const auto checkRejectedSqrtInformation = [&](const Matrix2& R) {
    NonlinearFactorGraph graph;
    graph.emplace_shared<BundlerProjectionFactor>(
        measuredData.track(0).measurement(0).second,
        noiseModel::Gaussian::SqrtInformation(R, false), C(0), P(0));
    CHECK_EXCEPTION(convertGeneralSfmGraph(graph, initial),
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

// Verifies SfmLevenbergMarquardtOptimizer::OptimizesGeneralSfmGraphWithArbitraryKeys.
TEST(SfmLevenbergMarquardtOptimizer,
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

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.maxIterations = 5;
  params.relativeErrorTol = 1e-12;
  SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
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

// Verifies SfmLevenbergMarquardtOptimizer::UsesGraphGaussianNoiseForInitialError.
TEST(SfmLevenbergMarquardtOptimizer,
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

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.maxIterations = 0;
  SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  optimizer.optimize();

  const double expectedInitialError = graph.error(initial);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().initialError, 1e-6);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().finalError, 1e-6);
}

// Verifies SfmLevenbergMarquardtOptimizer::DoesNotEvaluateCpuGraphErrorDuringCudaOptimize.
TEST(SfmLevenbergMarquardtOptimizer,
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

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.maxIterations = 0;

  CountingBundlerProjectionFactor::errorCalls = 0;
  SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  optimizer.optimize();

  EXPECT_LONGS_EQUAL(0, CountingBundlerProjectionFactor::errorCalls);
  DOUBLES_EQUAL(optimizer.result().finalError, optimizer.error(), 1e-6);
}

// Verifies SfmLevenbergMarquardtOptimizer::UsesGraphRobustNoiseForInitialError.
TEST(SfmLevenbergMarquardtOptimizer,
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

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.maxIterations = 0;
  SfmLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  optimizer.optimize();

  const double expectedInitialError = graph.error(initial);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().initialError, 1e-6);
  DOUBLES_EQUAL(expectedInitialError, optimizer.result().finalError, 1e-6);
}

// Verifies SfmLevenbergMarquardt::ReducesTinyBalErrorAndDownloadsValues.
TEST(SfmLevenbergMarquardt, ReducesTinyBalErrorAndDownloadsValues) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params;
  params.maxIterations = 5;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  const SfmLevenbergMarquardtResult result =
      optimizeSfm(data, params);

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

// Verifies SfmLevenbergMarquardt::CanSkipOptimizedValueDownload.
TEST(SfmLevenbergMarquardt, CanSkipOptimizedValueDownload) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params;
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;
  const SfmLevenbergMarquardtResult result =
      optimizeSfmWithoutValueDownload(data, params);

  CHECK(result.iterations > 0);
  CHECK(result.optimizedValues.empty());
}

// Verifies SfmLevenbergMarquardt::DetailedProfilingIsDisabledByDefault.
TEST(SfmLevenbergMarquardt, DetailedProfilingIsDisabledByDefault) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  CHECK(!params.enableDetailedProfiling);
  const SfmLevenbergMarquardtResult result =
      optimizeSfm(data, params);

  CHECK(result.solveLoopElapsed > 0.0);
  CHECK(result.allocateTrialElapsed >= 0.0);
  CHECK(result.allocateTrialElapsed <= result.totalMeasuredElapsed);
  EXPECT_LONGS_EQUAL(0, result.totalH2dBytes);
  EXPECT_LONGS_EQUAL(0, result.totalD2hBytes);
  DOUBLES_EQUAL(0.0, result.packValuesElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.projectionBatchElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.denseSchurSolveElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.linearizedErrorElapsed, 0.0);
  CHECK(result.iterationProfiles.empty());
}

// Verifies SfmLevenbergMarquardt::DetailedProfilingIsDisabledByDefaultForCudss.
TEST(SfmLevenbergMarquardt,
     DetailedProfilingIsDisabledByDefaultForCudss) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.setLinearSolver("cudss-full-normal");
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  CHECK(!params.enableDetailedProfiling);
  const SfmLevenbergMarquardtResult result =
      optimizeSfmWithoutValueDownload(data, params);

  CHECK(result.solveLoopElapsed > 0.0);
  CHECK(result.csrStructureElapsed >= 0.0);
  CHECK(result.csrStructureElapsed <= result.totalMeasuredElapsed);
  DOUBLES_EQUAL(0.0, result.packValuesElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.projectionBatchElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.uploadPatternElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.firstCudssAnalyzeElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.normalEquationsElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.addDampingElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.cudssAnalyzeElapsed, 0.0);
  DOUBLES_EQUAL(0.0, result.cudssSolveElapsed, 0.0);
  CHECK(result.iterationProfiles.empty());
}

// Verifies SfmLevenbergMarquardt::RecordsDetailedTimingBreakdownForCudss.
TEST(SfmLevenbergMarquardt, RecordsDetailedTimingBreakdownForCudss) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.setLinearSolver("cudss-full-normal");
  params.enableDetailedProfiling = true;
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;

  const SfmLevenbergMarquardtResult result =
      optimizeSfmWithoutValueDownload(data, params);

  CHECK(result.packValuesElapsed > 0.0);
  CHECK(result.projectionBatchElapsed > 0.0);
  CHECK(result.uploadPatternElapsed > 0.0);
  CHECK(result.firstCudssAnalyzeElapsed > 0.0);
  CHECK(result.normalEquationsElapsed > 0.0);
  CHECK(result.addDampingElapsed > 0.0);
  CHECK(result.cudssAnalyzeElapsed > 0.0);
  CHECK(result.cudssSolveElapsed > 0.0);
  CHECK(!result.iterationProfiles.empty());
  double attributedNormalEquationsElapsed = 0.0;
  for (const auto& iteration : result.iterationProfiles) {
    CHECK(iteration.normalEquationsElapsed > 0.0);
    attributedNormalEquationsElapsed += iteration.normalEquationsElapsed;
  }
  DOUBLES_EQUAL(result.normalEquationsElapsed,
                attributedNormalEquationsElapsed, 1e-12);
}

// Verifies SfmLevenbergMarquardt::RecordsDetailedTimingBreakdown.
TEST(SfmLevenbergMarquardt, RecordsDetailedTimingBreakdown) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.maxIterations = 5;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;
  params.enableDetailedProfiling = true;

  const SfmLevenbergMarquardtResult result =
      optimizeSfmWithoutValueDownload(data, params);

  CHECK(result.solveLoopElapsed > 0.0);
  CHECK(result.packValuesElapsed > 0.0);
  CHECK(result.projectionBatchElapsed > 0.0);
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

// Verifies SfmLevenbergMarquardt::RecordsPureTransferTimingBreakdown.
TEST(SfmLevenbergMarquardt, RecordsPureTransferTimingBreakdown) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::ceresDefaults();
  params.maxIterations = 1;
  params.relativeErrorTol = 1e-12;
  params.lambdaInitial = 1e-3;
  params.enableDetailedProfiling = true;

  const SfmLevenbergMarquardtResult result =
      optimizeSfm(data, params);

  const size_t expectedValueBytes =
      data.numberCameras() * sizeof(DevicePinholeCameraCal3Bundler) +
      data.numberTracks() * sizeof(DevicePoint3);
  const size_t expectedProjectionBytes =
      8 * sizeof(SfmObservation) +
      (data.numberTracks() + 1) * sizeof(int);

  EXPECT_LONGS_EQUAL(expectedValueBytes, result.packValuesH2dBytes);
  EXPECT_LONGS_EQUAL(expectedProjectionBytes, result.projectionBatchH2dBytes);
  EXPECT_LONGS_EQUAL(expectedValueBytes + expectedProjectionBytes,
                     result.totalH2dBytes);
  EXPECT_LONGS_EQUAL(expectedValueBytes, result.downloadD2hBytes);
  // Initial objective, one linearized old/new pair, and one trial objective
  // each download one reduction block (four doubles total).
  const size_t expectedReductionBytes = 4 * sizeof(double);
  EXPECT_LONGS_EQUAL(expectedValueBytes + expectedReductionBytes,
                     result.totalD2hBytes);

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

// Verifies SfmDenseSchurSolver::MatchesFullNormalEquationDeltaOnTinyBal.
TEST(SfmDenseSchurSolver, MatchesFullNormalEquationDeltaOnTinyBal) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  const BalCsrStructure structure = BalCsrStructure::fromSfmData(data);
  DeviceSparseSpdSystem system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());

  constexpr double lambda = 1e-3;
  accumulateSfmNormalEquations(values, batch,
                                   static_cast<int>(structure.numCameras()),
                                   &system, context.stream());
  system.addDiagonalDamping(lambda, context.stream());

  DeviceArray<double> fullDelta;
  CudssSpdSolver fullSolver;
  fullSolver.analyze(system, &fullDelta, context.stream());
  fullSolver.solve(system, &fullDelta, context.stream());

  DeviceArray<double> schurDelta;
  solveSfmDenseSchur(values, batch, static_cast<int>(data.numberCameras()),
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

// Verifies SfmSchurProblem::ReusesLinearizationAndRebuildsDamping.
TEST(SfmSchurProblem, ReusesLinearizationAndRebuildsDamping) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;
  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());

  SfmSchurProblem problem;
  problem.initialize(batch, static_cast<int>(data.numberCameras()));
  problem.linearize(values, context.stream());
  const DenseSpdSystemView first =
      problem.prepareDense(1e-3, context.stream());
  std::vector<double> firstValues(first.dimension * first.dimension);
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      firstValues.data(), first.values, sizeof(double) * firstValues.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  context.synchronize();

  const DenseSpdSystemView second =
      problem.prepareDense(4e-3, context.stream());
  std::vector<double> secondValues(second.dimension * second.dimension);
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      secondValues.data(), second.values, sizeof(double) * secondValues.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  context.synchronize();

  const DenseSpdSystemView repeatedFirst =
      problem.prepareDense(1e-3, context.stream());
  std::vector<double> repeatedFirstValues(repeatedFirst.dimension *
                                          repeatedFirst.dimension);
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      repeatedFirstValues.data(), repeatedFirst.values,
      sizeof(double) * repeatedFirstValues.size(), cudaMemcpyDeviceToHost,
      context.stream()));
  context.synchronize();

  EXPECT_LONGS_EQUAL(1, problem.linearizationCount());
  EXPECT_LONGS_EQUAL(3, problem.denseAssemblyCount());
  EXPECT(firstValues != secondValues);
  for (size_t scalar = 0; scalar < firstValues.size(); ++scalar) {
    DOUBLES_EQUAL(firstValues[scalar], repeatedFirstValues[scalar], 1e-12);
  }
}

// Verifies SfmSchurProblem::BuildsPersistentNormalBlocksOncePerLinearization.
TEST(SfmSchurProblem, BuildsPersistentNormalBlocksOncePerLinearization) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;
  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());

  SfmSchurProblem problem;
  problem.initialize(batch, static_cast<int>(data.numberCameras()));
  problem.linearize(values, context.stream());

  const SfmSchurBlocks& blocks = problem.blocks();
  const size_t cameras = data.numberCameras();
  const size_t points = data.numberTracks();
  const size_t observations = batch.numObservations();
  EXPECT_LONGS_EQUAL(81 * cameras, blocks.cameraNormalBlocks.size());
  EXPECT_LONGS_EQUAL(9 * cameras, blocks.cameraGradient.size());
  EXPECT_LONGS_EQUAL(9 * points, blocks.pointNormalBlocks.size());
  EXPECT_LONGS_EQUAL(3 * points, blocks.pointGradient.size());
  EXPECT_LONGS_EQUAL(27 * observations, blocks.cameraPointBlocks.size());

  std::vector<SfmObservation> hostObservations;
  std::vector<double> residuals, cameraJacobians, pointJacobians;
  std::vector<double> actualU, actualGc, actualV, actualGp, actualW;
  batch.observations().download(&hostObservations, context.stream());
  problem.linearization().residuals.download(&residuals, context.stream());
  problem.linearization().cameraJacobians.download(&cameraJacobians,
                                                   context.stream());
  problem.linearization().pointJacobians.download(&pointJacobians,
                                                  context.stream());
  blocks.cameraNormalBlocks.download(&actualU, context.stream());
  blocks.cameraGradient.download(&actualGc, context.stream());
  blocks.pointNormalBlocks.download(&actualV, context.stream());
  blocks.pointGradient.download(&actualGp, context.stream());
  blocks.cameraPointBlocks.download(&actualW, context.stream());
  context.synchronize();

  std::vector<double> expectedU(81 * cameras, 0.0);
  std::vector<double> expectedGc(9 * cameras, 0.0);
  std::vector<double> expectedV(9 * points, 0.0);
  std::vector<double> expectedGp(3 * points, 0.0);
  std::vector<double> expectedW(27 * observations, 0.0);
  for (size_t observation = 0; observation < observations; ++observation) {
    const size_t camera = hostObservations[observation].cameraSlot;
    const size_t point = hostObservations[observation].pointSlot;
    const double r0 = residuals[2 * observation];
    const double r1 = residuals[2 * observation + 1];
    const double* Jc = cameraJacobians.data() + 18 * observation;
    const double* Jp = pointJacobians.data() + 6 * observation;
    for (size_t row = 0; row < 9; ++row) {
      expectedGc[9 * camera + row] += -Jc[row] * r0 - Jc[9 + row] * r1;
      for (size_t column = 0; column < 9; ++column) {
        expectedU[81 * camera + 9 * row + column] +=
            Jc[row] * Jc[column] + Jc[9 + row] * Jc[9 + column];
      }
      for (size_t column = 0; column < 3; ++column) {
        expectedW[27 * observation + 3 * row + column] =
            Jc[row] * Jp[column] + Jc[9 + row] * Jp[3 + column];
      }
    }
    for (size_t row = 0; row < 3; ++row) {
      expectedGp[3 * point + row] += -Jp[row] * r0 - Jp[3 + row] * r1;
      for (size_t column = 0; column < 3; ++column) {
        expectedV[9 * point + 3 * row + column] +=
            Jp[row] * Jp[column] + Jp[3 + row] * Jp[3 + column];
      }
    }
  }

  const auto check = [&](const std::vector<double>& expected,
                         const std::vector<double>& actual) {
    LONGS_EQUAL(expected.size(), actual.size());
    for (size_t i = 0; i < expected.size(); ++i) {
      DOUBLES_EQUAL(expected[i], actual[i], 1e-11);
    }
  };
  check(expectedU, actualU);
  check(expectedGc, actualGc);
  check(expectedV, actualV);
  check(expectedGp, actualGp);
  check(expectedW, actualW);

  (void)problem.prepareDense(1e-3, context.stream());
  (void)problem.prepareImplicit(2e-3, context.stream());
  EXPECT_LONGS_EQUAL(1, problem.blockBuildCount());
}

// Verifies SfmSchurProblem::DensePreparationConsumesPersistentNormalBlocks.
TEST(SfmSchurProblem, DensePreparationConsumesPersistentNormalBlocks) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;
  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  SfmSchurProblem problem;
  problem.initialize(batch, static_cast<int>(data.numberCameras()));
  problem.linearize(values, context.stream());

  constexpr double lambda = 1e-3;
  const DenseSpdSystemView before =
      problem.prepareDense(lambda, context.stream());
  std::vector<double> beforeValues(before.dimension * before.dimension);
  std::vector<double> beforeRhs(before.dimension);
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      beforeValues.data(), before.values, sizeof(double) * beforeValues.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(beforeRhs.data(), before.rhs,
                                   sizeof(double) * beforeRhs.size(),
                                   cudaMemcpyDeviceToHost, context.stream()));
  context.synchronize();

  SfmProjectionLinearization& projection =
      const_cast<SfmProjectionLinearization&>(problem.linearization());
  projection.residuals.zero(context.stream());
  projection.cameraJacobians.zero(context.stream());
  projection.pointJacobians.zero(context.stream());
  const DenseSpdSystemView after =
      problem.prepareDense(lambda, context.stream());
  std::vector<double> afterValues(after.dimension * after.dimension);
  std::vector<double> afterRhs(after.dimension);
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      afterValues.data(), after.values, sizeof(double) * afterValues.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(afterRhs.data(), after.rhs,
                                   sizeof(double) * afterRhs.size(),
                                   cudaMemcpyDeviceToHost, context.stream()));
  context.synchronize();

  LONGS_EQUAL(beforeValues.size(), afterValues.size());
  for (size_t i = 0; i < beforeValues.size(); ++i) {
    DOUBLES_EQUAL(beforeValues[i], afterValues[i], 1e-12);
  }
  LONGS_EQUAL(beforeRhs.size(), afterRhs.size());
  for (size_t i = 0; i < beforeRhs.size(); ++i) {
    DOUBLES_EQUAL(beforeRhs[i], afterRhs[i], 1e-12);
  }
}

// Verifies SfmSchurProblem::PointRecoveryConsumesPersistentNormalBlocks.
TEST(SfmSchurProblem, PointRecoveryConsumesPersistentNormalBlocks) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;
  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  SfmSchurProblem problem;
  problem.initialize(batch, static_cast<int>(data.numberCameras()));
  problem.linearize(values, context.stream());

  std::vector<double> hostCameraDelta(problem.cameraDimension());
  for (size_t i = 0; i < hostCameraDelta.size(); ++i) {
    hostCameraDelta[i] = 1e-3 * static_cast<double>(i + 1);
  }
  DeviceArray<double> cameraDelta;
  cameraDelta.upload(hostCameraDelta, context.stream());
  DeviceArray<double> before;
  DeviceArray<double> after;
  problem.recoverPoints(1e-3, cameraDelta, &before, context.stream());
  std::vector<double> beforeValues;
  before.download(&beforeValues, context.stream());
  context.synchronize();

  SfmProjectionLinearization& projection =
      const_cast<SfmProjectionLinearization&>(problem.linearization());
  projection.residuals.zero(context.stream());
  projection.cameraJacobians.zero(context.stream());
  projection.pointJacobians.zero(context.stream());
  problem.recoverPoints(1e-3, cameraDelta, &after, context.stream());
  std::vector<double> afterValues;
  after.download(&afterValues, context.stream());
  context.synchronize();

  LONGS_EQUAL(beforeValues.size(), afterValues.size());
  for (size_t i = 0; i < beforeValues.size(); ++i) {
    DOUBLES_EQUAL(beforeValues[i], afterValues[i], 1e-12);
  }
}

// Verifies SfmSchurProblem::SparseAssemblyMatchesDenseAssembly.
TEST(SfmSchurProblem, SparseAssemblyMatchesDenseAssembly) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;
  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  const std::vector<Key> cameraKeys{C(0), C(1)};
  const SfmReducedCsrPlan plan(data, cameraKeys);

  SfmSchurProblem problem;
  problem.initialize(batch, static_cast<int>(data.numberCameras()));
  problem.linearize(values, context.stream());
  constexpr double lambda = 1e-3;
  const DenseSpdSystemView dense =
      problem.prepareDense(lambda, context.stream());
  SfmProjectionLinearization& projection =
      const_cast<SfmProjectionLinearization&>(problem.linearization());
  projection.residuals.zero(context.stream());
  projection.cameraJacobians.zero(context.stream());
  projection.pointJacobians.zero(context.stream());
  DeviceSparseSpdSystem& sparse =
      problem.prepareSparse(lambda, plan, context.stream());

  std::vector<double> denseValues(dense.dimension * dense.dimension);
  std::vector<double> denseRhs(dense.dimension);
  std::vector<int> rowPointers;
  std::vector<int> columns;
  std::vector<double> sparseValues;
  std::vector<double> sparseRhs;
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      denseValues.data(), dense.values, sizeof(double) * denseValues.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      denseRhs.data(), dense.rhs, sizeof(double) * denseRhs.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  sparse.rowPointers().download(&rowPointers, context.stream());
  sparse.colIndices().download(&columns, context.stream());
  sparse.values().download(&sparseValues, context.stream());
  sparse.rhs().download(&sparseRhs, context.stream());
  context.synchronize();

  EXPECT(plan.rowPointers() == rowPointers);
  EXPECT(plan.columnIndices() == columns);
  for (int row = 0; row < dense.dimension; ++row) {
    DOUBLES_EQUAL(denseRhs[row], sparseRhs[row], 1e-10);
    for (int entry = rowPointers[row]; entry < rowPointers[row + 1]; ++entry) {
      const int column = columns[entry];
      const double denseSymmetricEntry =
          denseValues[static_cast<size_t>(row) * dense.dimension + column];
      DOUBLES_EQUAL(denseSymmetricEntry, sparseValues[entry], 1e-10);
    }
  }
}

// Verifies SfmSchurProblem::ImplicitOperatorMatchesDenseSchurProduct.
TEST(SfmSchurProblem, ImplicitOperatorMatchesDenseSchurProduct) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;
  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  SfmSchurProblem problem;
  problem.initialize(batch, static_cast<int>(data.numberCameras()));
  problem.linearize(values, context.stream());
  constexpr double lambda = 1e-3;
  const DenseSpdSystemView dense =
      problem.prepareDense(lambda, context.stream());
  SfmProjectionLinearization& projection =
      const_cast<SfmProjectionLinearization&>(problem.linearization());
  projection.residuals.zero(context.stream());
  projection.cameraJacobians.zero(context.stream());
  projection.pointJacobians.zero(context.stream());
  const SfmImplicitSchurView implicit =
      problem.prepareImplicit(lambda, context.stream());

  std::vector<double> hostInput(dense.dimension);
  for (int i = 0; i < dense.dimension; ++i) hostInput[i] = 0.25 + 0.1 * i;
  DeviceArray<double> input;
  DeviceArray<double> output(dense.dimension);
  input.upload(hostInput, context.stream());
  implicit.linearOperator->apply(input.data(), output.data(), context.stream());

  std::vector<double> denseValues(dense.dimension * dense.dimension);
  std::vector<double> denseRhs(dense.dimension);
  std::vector<double> actual;
  std::vector<double> implicitRhs(dense.dimension);
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      denseValues.data(), dense.values, sizeof(double) * denseValues.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(denseRhs.data(), dense.rhs,
                                   sizeof(double) * denseRhs.size(),
                                   cudaMemcpyDeviceToHost, context.stream()));
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(
      implicitRhs.data(), implicit.rhs, sizeof(double) * implicitRhs.size(),
      cudaMemcpyDeviceToHost, context.stream()));
  output.download(&actual, context.stream());
  context.synchronize();
  for (int row = 0; row < dense.dimension; ++row) {
    DOUBLES_EQUAL(denseRhs[row], implicitRhs[row], 1e-10);
  }
  for (int row = 0; row < dense.dimension; ++row) {
    double expected = 0.0;
    for (int column = 0; column < dense.dimension; ++column) {
      const int lowerRow = std::max(row, column);
      const int lowerColumn = std::min(row, column);
      expected += denseValues[lowerColumn * dense.dimension + lowerRow] *
                  hostInput[column];
    }
    DOUBLES_EQUAL(expected, actual[row], 1e-8);
  }
}

// Verifies SfmDenseSchurSolver::MatchesFullNormalEquationDeltaWithDiagonalDamping.
TEST(SfmDenseSchurSolver, MatchesFullNormalEquationDeltaWithDiagonalDamping) {
  const SfmData measuredData = makeTrueBalLikeData();
  const SfmData data = makePerturbedBalLikeData(measuredData);
  Context context;

  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  const BalCsrStructure structure = BalCsrStructure::fromSfmData(data);
  DeviceSparseSpdSystem system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());

  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization,
                                  context.stream());

  std::vector<SfmObservation> observations;
  std::vector<double> cameraJacobians;
  std::vector<double> pointJacobians;
  batch.observations().download(&observations, context.stream());
  linearization.cameraJacobians.download(&cameraJacobians, context.stream());
  linearization.pointJacobians.download(&pointJacobians, context.stream());
  context.synchronize();

  std::vector<double> dampingDiagonal(structure.dimension(), 0.0);
  for (size_t i = 0; i < observations.size(); ++i) {
    const SfmObservation& observation = observations[i];
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

  DeviceArray<double> deviceDampingDiagonal;
  deviceDampingDiagonal.upload(dampingDiagonal, context.stream());

  constexpr double lambda = 1e-3;
  accumulateSfmNormalEquations(values, batch,
                                   static_cast<int>(structure.numCameras()),
                                   &system, context.stream());
  system.addDiagonalDamping(lambda, deviceDampingDiagonal, context.stream());

  DeviceArray<double> fullDelta;
  CudssSpdSolver fullSolver;
  fullSolver.analyze(system, &fullDelta, context.stream());
  fullSolver.solve(system, &fullDelta, context.stream());

  DeviceArray<double> schurDelta;
  solveSfmDenseSchur(values, batch, static_cast<int>(data.numberCameras()),
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

// Verifies SfmDenseSchurSolver::MatchesFullNormalEquationDeltaOnHighDegreeTrack.
TEST(SfmDenseSchurSolver,
     MatchesFullNormalEquationDeltaOnHighDegreeTrack) {
  const SfmData data = makeHighDegreeBalLikeData();
  Context context;

  DeviceValues values = packSfmValues(data, context.stream());
  const SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());
  const BalCsrStructure structure = BalCsrStructure::fromSfmData(data);
  DeviceSparseSpdSystem system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());

  constexpr double lambda = 1e-3;
  accumulateSfmNormalEquations(values, batch,
                                   static_cast<int>(structure.numCameras()),
                                   &system, context.stream());
  system.addDiagonalDamping(lambda, context.stream());

  DeviceArray<double> fullDelta;
  CudssSpdSolver fullSolver;
  fullSolver.analyze(system, &fullDelta, context.stream());
  fullSolver.solve(system, &fullDelta, context.stream());

  DeviceArray<double> schurDelta;
  solveSfmDenseSchur(values, batch, static_cast<int>(data.numberCameras()),
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

// Verifies DeviceGeometryKernels::RetractCameraMatchesHostCameraRetract.
TEST(DeviceGeometryKernels, RetractCameraMatchesHostCameraRetract) {
  const SfmData data = makeTrueBalLikeData();
  const DevicePinholeCameraCal3Bundler camera =
      packPinholeCameraCal3Bundler(data.camera(1));

  const double deltaArray[9] = {0.004, -0.003, 0.002, 0.05, -0.04,
                                0.03,  2.0,    -0.0007, 0.00008};
  Vector delta(9);
  for (int i = 0; i < 9; ++i) {
    delta(i) = deltaArray[i];
  }

  const DevicePinholeCameraCal3Bundler actual =
      retractCamera(camera, deltaArray);
  const DevicePinholeCameraCal3Bundler expected =
      packPinholeCameraCal3Bundler(data.camera(1).retract(delta));

  CHECK(deviceCameraEquals(expected, actual, 1e-10));
}

// Verifies SfmProjectionBatch::PacksOnlyTracksWithAtLeastTwoMeasurements.
TEST(SfmProjectionBatch, PacksOnlyTracksWithAtLeastTwoMeasurements) {
  const SfmData data = makeTinySfmData();
  Context context;

  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());

  CHECK(batch.noiseMode() == SfmProjectionNoiseMode::Unit);
  EXPECT_LONGS_EQUAL(0, batch.sqrtInfos().size());
  EXPECT_LONGS_EQUAL(2, batch.numObservations());

  std::vector<SfmObservation> observations;
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

// Verifies SfmProjectionBatch::PacksLongTrackPointSlots.
TEST(SfmProjectionBatch, PacksLongTrackPointSlots) {
  const SfmData data = makeHighDegreeBalLikeData();
  Context context;

  SfmProjectionBatch batch =
      SfmProjectionBatch::fromSfmData(data, context.stream());

  std::vector<int> longTrackPointSlots;
  batch.longTrackPointSlots().download(&longTrackPointSlots,
                                       context.stream());
  context.synchronize();

  LONGS_EQUAL(1, longTrackPointSlots.size());
  LONGS_EQUAL(0, longTrackPointSlots[0]);
}

// Verifies SfmValues::PacksCamerasInGtsamConvention.
TEST(SfmValues, PacksCamerasInGtsamConvention) {
  const SfmData data = makeTinySfmData();
  Context context;

  DeviceValues values = packSfmValues(data, context.stream());
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

// Verifies SfmValues::DownloadsValuesWithOriginalKeys.
TEST(SfmValues, DownloadsValuesWithOriginalKeys) {
  const SfmData data = makeTinySfmData();
  Context context;

  DeviceValues values = packSfmValues(data, context.stream());
  const Values downloaded = downloadSfmValues(values, context.stream());

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

// Verifies BalCsrStructure::BuildsUpperTrianglePatternForMeasuredTrack.
TEST(BalCsrStructure, BuildsUpperTrianglePatternForMeasuredTrack) {
  const SfmData data = makeTinyBalData();
  const BalCsrStructure structure = BalCsrStructure::fromSfmData(data);

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


// Verifies SfmLevenbergMarquardtParams::EqualsComparesFields.
TEST(SfmLevenbergMarquardtParams, EqualsComparesFields) {
  const SfmLevenbergMarquardtParams a =
      SfmLevenbergMarquardtParams::legacyDefaults();
  SfmLevenbergMarquardtParams b = a;
  CHECK(a.equals(b));

  b.lambdaInitial = 2.0 * a.lambdaInitial + 1.0;
  CHECK(!a.equals(b));

  b = a;
  b.setLinearSolver("cudss-full-normal");
  CHECK(!a.equals(b));

  b = a;
  b.enableDetailedProfiling = !a.enableDetailedProfiling;
  CHECK(!a.equals(b));
}

// Verifies GncSfmOptimizer::TlsClassificationMatchesCpuGnc.
TEST(GncSfmOptimizer, TlsClassificationMatchesCpuGnc) {
  const GncTestProblem problem = makeGncBalLikeProblem();

  GncParams<LevenbergMarquardtParams> cpuGncParams{LevenbergMarquardtParams()};
  cpuGncParams.setLossType(GncLossType::TLS);
  GncOptimizer<GncParams<LevenbergMarquardtParams>> cpuGnc(
      problem.graph, problem.initial, cpuGncParams);
  const Values cpuResult = cpuGnc.optimize();

  GncParams<SfmLevenbergMarquardtParams> cudaGncParams{
      SfmLevenbergMarquardtParams::legacyDefaults()};
  cudaGncParams.setLossType(GncLossType::TLS);
  GncOptimizer<GncParams<SfmLevenbergMarquardtParams>> cudaGnc(
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

// Verifies GncSfmOptimizer::GmClassificationMatchesCpuGnc.
TEST(GncSfmOptimizer, GmClassificationMatchesCpuGnc) {
  const GncTestProblem problem = makeGncBalLikeProblem();

  GncParams<LevenbergMarquardtParams> cpuGncParams{LevenbergMarquardtParams()};
  cpuGncParams.setLossType(GncLossType::GM);
  GncOptimizer<GncParams<LevenbergMarquardtParams>> cpuGnc(
      problem.graph, problem.initial, cpuGncParams);
  const Values cpuResult = cpuGnc.optimize();

  GncParams<SfmLevenbergMarquardtParams> cudaGncParams{
      SfmLevenbergMarquardtParams::legacyDefaults()};
  cudaGncParams.setLossType(GncLossType::GM);
  GncOptimizer<GncParams<SfmLevenbergMarquardtParams>> cudaGnc(
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

// Verifies GncSfmOptimizer::KnownOutliersProduceZeroInformationGraph.
TEST(GncSfmOptimizer, KnownOutliersProduceZeroInformationGraph) {
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

  SfmLevenbergMarquardtParams params =
      SfmLevenbergMarquardtParams::legacyDefaults();
  SfmLevenbergMarquardtOptimizer optimizer(weightedGraph, problem.initial,
                                               params);
  const Values& result = optimizer.optimize();

  CHECK(problem.inlierGraph.error(result) < 1e-3);
  // The zero-information factors contribute exactly zero error.
  DOUBLES_EQUAL(problem.inlierGraph.error(result),
                weightedGraph.error(result), 1e-9);
}

}  // namespace sfm_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
