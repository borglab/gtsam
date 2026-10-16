
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/GalileanEKF.h>

#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::galilean;

namespace {
const Vector3 kGravity(0, 0, -9.81);

/// Filter driven with a few random inputs so the estimate is non-trivial.
GalileanFilter exercisedFilter(std::mt19937& rng, int steps = 120) {
  GalileanFilterParams params;
  params.gravity = kGravity;
  params.earthRate = Vector3(1e-4, -2e-4, 7e-5);
  params.antennaLever = Point3(0.1, -0.05, 0.2);
  std::normal_distribution<double> n(0.0, 1.0);
  Vector10 bias = Vector10::Zero();
  for (int i = 0; i < 9; ++i) bias(i) = 0.02 * n(rng);
  GalileanFilter filter(params,
                        Gal3(Rot3::RzRyRx(0.1, -0.2, 0.7), Point3(1, 2, 3),
                             Vector3(4, -1, 0.5), 0.0),
                        0.083, bias, Matrix20::Identity() * 0.01);
  for (int k = 0; k < steps; ++k) {
    filter.predict(IMUInput(Vector3(0.3 * n(rng), 0.3 * n(rng), 0.3 * n(rng)),
                            Vector3(n(rng), n(rng), 9.81 + n(rng))),
                   0.005);
  }
  return filter;
}

/// Result of a closed-loop simulation run.
struct SimResult {
  double delay;
  double positionError;
  double velocityError;
  double attitudeError;
  int rejected = 0;
};

/**
 * Circular motion with vertical waves and varying attitude (in the spirit of
 * the paper's Sec. 4.2): 200 Hz IMU, 20 Hz GNSS with a fixed unknown delay.
 * The filter starts from δ̂ = 0 with wrong position, velocity and attitude.
 */
SimResult simulate(double trueDelay, double duration,
                   const GalileanFilterParams& base, unsigned seed = 9) {
  const double dt = 0.005, sigmaGyro = 1e-3, sigmaAcc = 1e-2, sigmaGnss = 0.02;
  const int gnssEvery = 10;
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 1.0);

  GalileanFilterParams params = base;
  params.gravity = kGravity;
  params.antennaLever = Point3(0.0, 0.1, -0.05);
  params.gyroNoiseDensity = sigmaGyro * std::sqrt(dt);
  params.accelNoiseDensity = sigmaAcc * std::sqrt(dt);
  params.virtualInputNoise = 1e-5;
  params.gyroBiasRandomWalk = 1e-4;
  params.accelBiasRandomWalk = 3e-4;
  params.virtualBiasRandomWalk = 1e-6;
  params.bufferWindow = std::max(0.6, trueDelay + 0.2);

  const Vector10 gN = EarthInput(kGravity, Vector3::Zero());
  const double radius = 10.0, w = 0.5;
  Gal3 T(Rot3(), Point3(radius, 0, 0), Vector3(0, radius * w, 1.0), 0.0);
  std::vector<Gal3> truth{T};
  std::vector<IMUInput> inputs;

  Matrix20 P0 = Matrix20::Zero();
  P0.diagonal() << Vector3::Constant(0.05 * 0.05), Vector3::Constant(0.25),
      Vector3::Constant(1.0), 0.15 * 0.15, Vector3::Constant(1e-6),
      Vector3::Constant(1e-4), Vector3::Constant(1e-8), 1e-8;
  P0(index::kDelay, index::kDelay) = std::pow(std::max(0.15, trueDelay), 2);
  GalileanFilter filter(params,
                        Gal3(Rot3::RzRyRx(0.02, -0.02, 0.03),
                             Point3(radius + 0.5, -0.4, 0.3),
                             Vector3(0.2, radius * w - 0.3, 1.1), 0.0),
                        0.0, Vector10::Zero(), P0);

  const auto truthAt = [&](double s) {  // exact under zero-order hold
    const size_t k = static_cast<size_t>(std::floor(s / dt + 1e-9));
    return Dynamics(State(truth[k], Vector10::Zero()), inputs[k], gN,
                    s - k * dt)
        .first;
  };

  SimResult result;
  const int steps = static_cast<int>(duration / dt);
  for (int k = 0; k < steps; ++k) {
    const double t = k * dt;
    const Vector3 accWorld(-radius * w * w * std::cos(w * t),
                           -radius * w * w * std::sin(w * t),
                           -2.0 * std::sin(2.0 * t));
    const Vector3 omega(0.4 * std::sin(0.9 * t), 0.3 * std::cos(0.6 * t),
                        0.2 + 0.2 * std::sin(0.3 * t));
    const IMUInput imu(omega, T.rotation().unrotate(accWorld - kGravity), t);
    inputs.push_back(imu);
    T = Dynamics(State(T, Vector10::Zero()), imu, gN, dt).first;
    truth.push_back(T);

    filter.predict(IMUInput(imu.gyr + sigmaGyro * Vector3(n(rng), n(rng), n(rng)),
                            imu.acc + sigmaAcc * Vector3(n(rng), n(rng), n(rng))),
                   dt);

    const double tNow = (k + 1) * dt;
    if ((k + 1) % gnssEvery == 0 && tNow > trueDelay + dt) {
      const Gal3 Td = truthAt(tNow - trueDelay);
      const Point3 y = Td.rotation() * params.antennaLever + Td.translation() +
                       sigmaGnss * Vector3(n(rng), n(rng), n(rng));
      if (!filter.update(GnssMeasurement(
              y, Matrix3::Identity() * sigmaGnss * sigmaGnss, tNow))) {
        ++result.rejected;
      }
    }
  }

  const Gal3 That = filter.navState();
  result.delay = filter.delay();
  result.positionError = (That.translation() - T.translation()).norm();
  result.velocityError = (That.velocity() - T.velocity()).norm();
  result.attitudeError =
      Rot3::Logmap(T.rotation().between(That.rotation())).norm();
  return result;
}
}  // namespace

/* ************************************************************************* */
TEST(GalileanFilter, ParamsMatrices) {
  GalileanFilterParams params;
  params.gyroNoiseDensity = 2.0;
  params.delayProcessVariance = 0.5;
  params.initialDelaySigma = 0.3;
  EXPECT_DOUBLES_EQUAL(4.0, params.inputNoise()(0, 0), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.5, params.stateProcessNoise()(index::kDelay,
                                                       index::kDelay), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.09, params.initialCovariance()(index::kDelay,
                                                        index::kDelay), 1e-12);
}

/* ************************************************************************* */
// Uninitialized filter ignores IMU and GNSS, and dt <= 0 is a no-op.
TEST(GalileanFilter, GuardsBeforeInitialization) {
  GalileanFilter filter{GalileanFilterParams()};
  EXPECT(!filter.isInitialized());
  filter.predict(IMUInput(Vector3(1, 2, 3), Vector3(4, 5, 6)), 0.01);
  EXPECT(filter.buffer().empty());
  EXPECT(!filter.update(GnssMeasurement(Point3(1, 2, 3), I_3x3)));

  filter.initializeFromIMU(IMUInput(Z_3x1, Vector3(0, 0, 9.81)));
  const Gal3 before = filter.navState();
  filter.predict(IMUInput(Vector3(1, 2, 3), Vector3(4, 5, 6)), 0.0);
  EXPECT(assert_equal(before, filter.navState(), 1e-12));
}

/* ************************************************************************* */
// Initialization rotates the measured specific force onto Earth "up".
TEST(GalileanFilter, InitializeFromIMU) {
  GalileanFilterParams params;
  params.initialDelay = 0.05;
  GalileanFilter filter(params);
  const Vector3 acc = Rot3::RzRyRx(0.3, -0.2, 0.0).unrotate(Vector3(0, 0, 9.81));
  filter.initializeFromIMU(IMUInput(Z_3x1, acc));
  EXPECT(filter.isInitialized());
  EXPECT(assert_equal(Vector3(0, 0, 9.81), Vector3(filter.attitude() * acc),
                      1e-9));
  EXPECT_DOUBLES_EQUAL(0.05, filter.delay(), 1e-12);
  EXPECT(assert_equal(Point3(0, 0, 0), filter.position(), 1e-9));
  EXPECT(assert_equal(Vector3(0, 0, 0), filter.velocity(), 1e-9));
  EXPECT(assert_equal(Matrix(params.initialCovariance()),
                      Matrix(filter.errorCovariance()), 1e-12));
}

/* ************************************************************************* */
// GNSS Jacobian w.r.t. the state chart (including ∂Υ/∂δ) and C.
TEST(GalileanFilter, MeasurementJacobians) {
  std::mt19937 rng(7);
  GalileanFilter filter = exercisedFilter(rng);
  const State xiHat = filter.state();

  Eigen::Matrix<double, 3, 20> H;
  filter.predictGnss(xiHat, H);
  const auto h = [&](const State& xi) -> Point3 {
    return filter.predictGnss(xi);
  };
  EXPECT(assert_equal(numericalDerivative11<Point3, State>(h, xiHat), Matrix(H),
                      1e-6));

  const Group Xhat = filter.groupEstimate();
  const auto hError = [&](const Vector20& eps) -> Point3 {
    return filter.predictGnss(Symmetry()(State::Identity().retract(eps), Xhat));
  };
  EXPECT(assert_equal(
      numericalDerivative11<Point3, Vector20>(hError, Vector20::Zero()),
      Matrix(filter.measurementMatrixC()), 1e-6));
}

/* ************************************************************************* */
// Navigation-state outputs are consistent: T(t-δ) Υ(δ) = Γ(δ) T(t).
TEST(GalileanFilter, NavStateConsistency) {
  std::mt19937 rng(11);
  GalileanFilter filter = exercisedFilter(rng);
  EXPECT_DOUBLES_EQUAL(0.0, filter.navState().time(), 1e-9);
  EXPECT(assert_equal(filter.Gamma(filter.delay()) * filter.navState(),
                      filter.delayedNavState() * filter.buffer().at(filter.delay()),
                      1e-9));
}

/* ************************************************************************* */
TEST(GalileanFilter, RejectsInvalidCovariance) {
  std::mt19937 rng(12);
  GalileanFilter filter = exercisedFilter(rng);
  Matrix3 bad = I_3x3;
  bad(0, 0) = -1.0;
  CHECK_EXCEPTION(filter.update(GnssMeasurement(Point3(1, 2, 3), bad)),
                  std::invalid_argument);
  bad = I_3x3;
  bad(0, 1) = 0.5;  // not symmetric
  CHECK_EXCEPTION(filter.update(GnssMeasurement(Point3(1, 2, 3), bad)),
                  std::invalid_argument);
}

/* ************************************************************************* */
// The NIS gate rejects a gross outlier without touching the state.
TEST(GalileanFilter, OutlierGate) {
  std::mt19937 rng(13);
  GalileanFilterParams params;
  params.gnssOutlierThreshold = 16.27;
  params.gnssGateWarmup = 0;  // gate from the first update
  GalileanFilter filter(params, Gal3(), 0.0, Vector10::Zero(),
                        Matrix20::Identity() * 0.01);
  for (int k = 0; k < 20; ++k) {
    filter.predict(IMUInput(Z_3x1, Vector3(0, 0, 9.81)), 0.005);
  }
  const Gal3 before = filter.navState();
  const Matrix20 Pbefore = filter.errorCovariance();
  EXPECT(!filter.update(GnssMeasurement(Point3(100, 0, 0), I_3x3 * 0.01)));
  EXPECT(filter.lastNis() > 16.27);
  EXPECT(assert_equal(before, filter.navState(), 1e-12));
  EXPECT(assert_equal(Matrix(Pbefore), Matrix(filter.errorCovariance()), 1e-12));
  EXPECT(filter.update(GnssMeasurement(Point3(0.05, 0, 0), I_3x3 * 0.01)));
}

/* ************************************************************************* */
// During warm-up the gate is off, so a badly initialized filter can recover.
TEST(GalileanFilter, GateWarmup) {
  GalileanFilterParams params;
  params.gnssOutlierThreshold = 16.27;
  params.gnssGateWarmup = 2;
  GalileanFilter filter(params, Gal3(), 0.0, Vector10::Zero(),
                        Matrix20::Identity() * 0.01);
  const GnssMeasurement far(Point3(100, 0, 0), I_3x3 * 0.01);
  EXPECT(filter.update(far));  // accepted: warm-up
  EXPECT(filter.update(far));  // accepted: warm-up
  EXPECT_LONGS_EQUAL(2, filter.acceptedUpdates());
  EXPECT(!filter.update(GnssMeasurement(Point3(-500, 0, 0), I_3x3 * 0.01)));
}

/* ************************************************************************* */
// A run of rejections reopens the gate so the filter cannot lock itself out.
TEST(GalileanFilter, GateReopensAfterConsecutiveRejections) {
  GalileanFilterParams params;
  params.gnssOutlierThreshold = 16.27;
  params.gnssGateWarmup = 0;
  params.gnssMaxConsecutiveRejections = 3;
  GalileanFilter filter(params, Gal3(), 0.0, Vector10::Zero(),
                        Matrix20::Identity() * 0.01);
  const GnssMeasurement far(Point3(100, 0, 0), I_3x3 * 0.01);
  EXPECT(!filter.update(far));
  EXPECT(!filter.update(far));
  EXPECT(!filter.update(far));
  EXPECT_LONGS_EQUAL(3, filter.consecutiveRejections());
  EXPECT(filter.update(far));  // gate reopened
  EXPECT_LONGS_EQUAL(0, filter.consecutiveRejections());
}

/* ************************************************************************* */
TEST(GalileanFilter, InitializeFromNavState) {
  GalileanFilterParams params;
  params.initialDelay = 0.1;
  GalileanFilter filter(params);
  const Gal3 T0(Rot3::Yaw(0.4), Point3(1, 2, 3), Vector3(4, 5, 6), 0.0);
  Vector10 b = Vector10::Zero();
  b.head<3>() << 0.01, -0.02, 0.03;
  filter.initialize(T0, b);
  EXPECT(filter.isInitialized());
  EXPECT(assert_equal(T0, filter.navState(), 1e-9));
  EXPECT_DOUBLES_EQUAL(0.1, filter.delay(), 1e-12);
  EXPECT(assert_equal(Vector3(0.01, -0.02, 0.03), filter.gyroBias(), 1e-12));
}

/* ************************************************************************* */
// delayProcessVariance inflates only the delay variance during prediction.
TEST(GalileanFilter, DelayRandomWalk) {
  GalileanFilterParams quiet, walking;
  walking.delayProcessVariance = 1e-2;
  const Matrix20 P0 = Matrix20::Identity() * 1e-4;
  GalileanFilter a(quiet, Gal3(), 0.1, Vector10::Zero(), P0);
  GalileanFilter b(walking, Gal3(), 0.1, Vector10::Zero(), P0);
  for (int k = 0; k < 200; ++k) {  // 1 s
    a.predict(IMUInput(Z_3x1, Vector3(0, 0, 9.81)), 0.005);
    b.predict(IMUInput(Z_3x1, Vector3(0, 0, 9.81)), 0.005);
  }
  const double extra = b.errorCovariance()(index::kDelay, index::kDelay) -
                       a.errorCovariance()(index::kDelay, index::kDelay);
  EXPECT_DOUBLES_EQUAL(1e-2, extra, 1e-6);
}

/* ************************************************************************* */
TEST(GalileanFilter, EstimatesUnknownDelay) {
  const SimResult r = simulate(0.12, 40.0, GalileanFilterParams());
  EXPECT_DOUBLES_EQUAL(0.12, r.delay, 0.005);
  EXPECT(r.positionError < 0.05);
  EXPECT(r.velocityError < 0.05);
  EXPECT(r.attitudeError < 0.01);
}

/* ************************************************************************* */
TEST(GalileanFilter, EstimatesLargeDelay) {
  const SimResult r = simulate(0.5, 40.0, GalileanFilterParams());
  EXPECT_DOUBLES_EQUAL(0.5, r.delay, 0.01);
  EXPECT(r.positionError < 0.05);
}

/* ************************************************************************* */
// With a sane NIS gate, clean data is (almost) never rejected.
TEST(GalileanFilter, GateAcceptsCleanData) {
  GalileanFilterParams params;
  params.gnssOutlierThreshold = 16.27;
  const SimResult r = simulate(0.12, 20.0, params);
  EXPECT(r.rejected <= 3);  // 400 fixes, 99.9% gate
  EXPECT_DOUBLES_EQUAL(0.12, r.delay, 0.005);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
