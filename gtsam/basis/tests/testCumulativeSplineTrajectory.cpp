/**
 * @file testCumulativeSplineTrajectory.cpp
 * @brief Unit tests for continuous cardinal spline trajectories.
 * @author Brett Downing
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/basis/CardinalSplineBasis.h>
#include <gtsam/basis/CumulativeSplineTrajectory.h>
#include <gtsam/basis/IrwinHall.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;

namespace {

constexpr double kEpsilon = 1e-4;
constexpr double kTolerance = kEpsilon / 4.0;

std::vector<Double_> scalarPath() {
  return {Double_(0.0), Double_(1.0), Double_(0.0), Double_(1.0)};
}

std::vector<Pose3_> posePath() {
  return {
      Pose3_(Pose3(Rot3::Rodrigues(0.3, 2.2, 0.1), Point3(0, 0, 0))),
      Pose3_(Pose3(Rot3::Rodrigues(0.2, 2.5, 0.1), Point3(0, 0, 0))),
      Pose3_(Pose3(Rot3::Rodrigues(0.0, 2.2, 0.1), Point3(0, 2, 0))),
      Pose3_(Pose3(Rot3::Rodrigues(0.2, 2.1, 0.4), Point3(0, 2, 0))),
  };
}

}  // namespace

/* ************************************************************************* */
namespace trajectory_bounds {

// Verifies rear padding reaches the first and last scalar control points.
TEST(CumulativeSplineTrajectory, BasicBounds) {
  Values values;
  const auto path = scalarPath();
  const KernelBase& kernel = kernels::IrwinHallCDF2;
  CumulativeSplineTrajectory<double> model(1.0, kernel, path);

  const Key timestampKey = Symbol('t', 0);
  values.insert(timestampKey, 0.0);
  const Double_ sample = model.sampleTrajectory(Double_(timestampKey));

  values.update(timestampKey, 0.0);
  EXPECT_DOUBLES_EQUAL(path.front().value(values), sample.value(values),
                       kTolerance);
  values.update(timestampKey,
                static_cast<double>(path.size() - 1) + kernel.getLength());
  EXPECT_DOUBLES_EQUAL(path.back().value(values), sample.value(values),
                       kTolerance);
}

// Verifies front padding shifts the valid scalar trajectory interval.
TEST(CumulativeSplineTrajectory, FrontPadding) {
  Values values;
  const auto path = scalarPath();
  const KernelBase& kernel = kernels::IrwinHallCDF2;
  CumulativeSplineTrajectory<double> model(1.0, kernel, path, true);

  const Key timestampKey = Symbol('t', 0);
  values.insert(timestampKey, 0.0);
  const Double_ sample = model.sampleTrajectory(Double_(timestampKey));

  values.update(timestampKey, -kernel.getLength());
  EXPECT_DOUBLES_EQUAL(path.front().value(values), sample.value(values),
                       kTolerance);
  values.update(timestampKey, static_cast<double>(path.size() - 1));
  EXPECT_DOUBLES_EQUAL(path.back().value(values), sample.value(values),
                       kTolerance);
}

// Verifies a padded control-point window matches the full expression.
TEST(CumulativeSplineTrajectory, WindowTruncation) {
  Values values;
  const std::vector<Double_> path = {
      Double_(4.0), Double_(-4.0), Double_(3.0), Double_(7.0),
      Double_(3.0), Double_(-8.0), Double_(2.0), Double_(6.0),
      Double_(4.0), Double_(1.0),  Double_(3.0), Double_(-2.0),
      Double_(5.0), Double_(2.0),  Double_(9.0), Double_(-2.0),
  };
  CumulativeSplineTrajectory<double> model(1.0, kernels::IrwinHallCDF2, path);

  const Key timestampKey = Symbol('t', 0);
  values.insert(timestampKey, 0.0);
  const Double_ timestamp(timestampKey);
  const Double_ full = model.sampleTrajectory(timestamp);
  const Double_ truncated = model.sampleTrajectory(timestamp, 6.0, 8.0);

  for (double time = 6.0; time < 8.0; time += 0.1) {
    values.update(timestampKey, time);
    EXPECT_DOUBLES_EQUAL(full.value(values), truncated.value(values),
                         kTolerance);
  }
}

// Verifies the reusable Basis functor matches the expression trajectory.
TEST(CumulativeSplineTrajectory, BasisFrameworkCompatibility) {
  Values values;
  const auto path = scalarPath();
  CumulativeSplineTrajectory<double> model(1.0, kernels::IrwinHallCDF2, path);
  const Key timestampKey = Symbol('t', 0);
  values.insert(timestampKey, 0.0);
  const Double_ sample = model.sampleTrajectory(Double_(timestampKey));
  const Vector parameters = (Vector(4) << 0.0, 1.0, 0.0, 1.0).finished();

  for (double time = 0.0; time < 7.0; time += 0.1) {
    values.update(timestampKey, time);
    const CardinalSplineBasis::EvaluationFunctor evaluate(4, time);
    EXPECT_DOUBLES_EQUAL(sample.value(values), evaluate(parameters),
                         kTolerance);
  }
}

}  // namespace trajectory_bounds
/* ************************************************************************* */

/* ************************************************************************* */
namespace trajectory_pose {

// Verifies Lie-group interpolation returns the expected translated pose.
TEST(CumulativeSplineTrajectory, Pose3Value) {
  Values values;
  const std::vector<Pose3_> path = {
      Pose3_(Pose3(Rot3(), Point3(0, 0, 0))),
      Pose3_(Pose3(Rot3(), Point3(0, 0, 0))),
      Pose3_(Pose3(Rot3(), Point3(0, 2, 0))),
      Pose3_(Pose3(Rot3(), Point3(0, 2, 0))),
  };
  CumulativeSplineTrajectory<Pose3> model(1.0, kernels::IrwinHallCDF2, path);

  const Key timestampKey = Symbol('t', 0);
  values.insert(timestampKey, 3.5);
  const Pose3 actual =
      model.sampleTrajectory(Double_(timestampKey)).value(values);
  CHECK(assert_equal(Pose3(Rot3(), Point3(0, 1, 0)), actual, kTolerance));
}

// Verifies numeric sampling uses the production expression implementation.
TEST(CumulativeSplineTrajectory, ConstantValueSampling) {
  CumulativeSplineTrajectory<Pose3> model;
  model.addControlPoint(Pose3(Rot3(), Point3(0, 0, 0)));
  model.addControlPoint(Pose3(Rot3(), Point3(0, 0, 0)));
  model.addControlPoint(Pose3(Rot3(), Point3(0, 2, 0)));
  model.addControlPoint(Pose3(Rot3(), Point3(0, 2, 0)));

  const Pose3 actual = model.sampleTrajectory(3.5);
  CHECK(assert_equal(Pose3(Rot3(), Point3(0, 1, 0)), actual, kTolerance));

  const Vector6 tangentRate = model.sampleTrajectoryDerivative(3.5);
  EXPECT_LONGS_EQUAL(6, tangentRate.size());
  EXPECT(tangentRate.allFinite());
}

bool derivativesMatch(double density) {
  Values values;
  CumulativeSplineTrajectory<Pose3> model(density, kernels::IrwinHallCDF2,
                                          posePath());
  const Key referenceKey = Symbol('t', 0);
  const Key offsetKey = Symbol('t', 1);
  values.insert(referenceKey, 0.0);
  values.insert(offsetKey, 0.0);
  const Double_ referenceTime(referenceKey);
  const Double_ offsetTime(offsetKey);

  const size_t maximumDerivative =
      kernels::IrwinHallCDF2.getValidDerivatives() - 1;
  for (size_t order = 0; order < maximumDerivative; ++order) {
    const auto reference =
        model.sampleTrajectoryDerivative(referenceTime, 0.0, -1.0, order);
    const auto offset =
        model.sampleTrajectoryDerivative(offsetTime, 0.0, -1.0, order);
    const auto next =
        model.sampleTrajectoryDerivative(referenceTime, 0.0, -1.0, order + 1);
    for (double time = 0.0; time < 7.0 / density; time += 0.1) {
      values.update(referenceKey, time);
      values.update(offsetKey, time + kEpsilon);
      if (!assert_equal(
              offset.value(values),
              expmap(reference, Double_(kEpsilon) * next).value(values),
              kTolerance)) {
        return false;
      }
    }
  }
  return true;
}

// Verifies analytic trajectory derivatives match finite time increments.
TEST(CumulativeSplineTrajectory, Pose3Derivatives) {
  EXPECT(derivativesMatch(1.0));
}

// Verifies derivative scaling remains correct at a higher point density.
TEST(CumulativeSplineTrajectory, Pose3DerivativeDensity) {
  EXPECT(derivativesMatch(10.0));
}

}  // namespace trajectory_pose
/* ************************************************************************* */

/* ************************************************************************* */
namespace trajectory_mesh {

// Verifies trajectory expressions can serve as another model's control points.
TEST(CumulativeSplineTrajectory, MeshModel) {
  Values values;
  const std::vector<std::vector<Double_>> mesh = {
      {Double_(4.0), Double_(-4.0), Double_(3.0), Double_(7.0)},
      {Double_(3.0), Double_(-8.0), Double_(2.0), Double_(6.0)},
      {Double_(4.0), Double_(1.0), Double_(3.0), Double_(-2.0)},
      {Double_(5.0), Double_(2.0), Double_(9.0), Double_(-2.0)},
  };
  const Key xKey = Symbol('x', 0);
  const Key yKey = Symbol('y', 0);
  values.insert(xKey, 0.0);
  values.insert(yKey, 0.0);

  std::vector<Double_> columnPath;
  for (const auto& rowPath : mesh) {
    CumulativeSplineTrajectory<double> row(1.0, kernels::IrwinHallCDF2,
                                           rowPath);
    columnPath.push_back(row.sampleTrajectory(Double_(xKey)));
  }
  CumulativeSplineTrajectory<double> column(1.0, kernels::IrwinHallCDF2,
                                            columnPath);
  const Double_ sample = column.sampleTrajectory(Double_(yKey));

  for (double x = 0.0; x < 7.0; x += 0.5) {
    for (double y = 0.0; y < 7.0; y += 0.5) {
      values.update(xKey, x);
      values.update(yKey, y);
      EXPECT(sample.value(values) < 10.0);
      EXPECT(sample.value(values) > -10.0);
    }
  }
}

}  // namespace trajectory_mesh
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
