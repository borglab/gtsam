#include <gtsam_unstable/navigation/GalileanEKF.h>

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace gtsam {
namespace galilean {

/* ************************************************************************* */
Matrix20 GalileanFilterParams::inputNoise() const {
  const auto sq = [](double x) { return x * x; };
  Vector20 d;
  d << Vector3::Constant(sq(gyroNoiseDensity)),
      Vector3::Constant(sq(accelNoiseDensity)),
      Vector3::Constant(sq(virtualInputNoise)), sq(virtualInputNoise),
      Vector3::Constant(sq(gyroBiasRandomWalk)),
      Vector3::Constant(sq(accelBiasRandomWalk)),
      Vector3::Constant(sq(virtualBiasRandomWalk)), sq(virtualBiasRandomWalk);
  return d.asDiagonal();
}

/* ************************************************************************* */
Matrix20 GalileanFilterParams::stateProcessNoise() const {
  Matrix20 Q = Matrix20::Zero();
  Q.diagonal().segment<3>(index::kAttitude).setConstant(attitudeProcessVariance);
  Q.diagonal().segment<3>(index::kVelocity).setConstant(velocityProcessVariance);
  Q.diagonal().segment<3>(index::kPosition).setConstant(positionProcessVariance);
  Q(index::kDelay, index::kDelay) = delayProcessVariance;
  return Q;
}

/* ************************************************************************* */
Matrix20 GalileanFilterParams::initialCovariance() const {
  const auto sq = [](double x) { return x * x; };
  Vector20 d;
  d << Vector3::Constant(sq(initialAttitudeSigma)),
      Vector3::Constant(sq(initialVelocitySigma)),
      Vector3::Constant(sq(initialPositionSigma)), sq(initialDelaySigma),
      Vector3::Constant(sq(initialGyroBiasSigma)),
      Vector3::Constant(sq(initialAccelBiasSigma)),
      Vector3::Constant(sq(initialVirtualBiasSigma)),
      sq(initialVirtualBiasSigma);
  return d.asDiagonal();
}

/* ************************************************************************* */
Group GalileanFilter::InitialGroup(const Params& params, const Gal3& navState,
                                   double delay, const Vector10& bias) {
  // φ(ξ₀, (A, a)) = (A, -Ad_{A⁻¹} a) = (F₀, b)  ⇒  A = F₀, a = -Ad_{F₀} b.
  const Vector10 gN = EarthInput(params.gravity, params.earthRate);
  const Gal3 F0 = Gal3::Expmap(gN * delay) * navState;  // Γ(δ) T
  return Group(F0, -F0.AdjointMap() * bias);
}

/* ************************************************************************* */
GalileanFilter::GalileanFilter(const Params& params)
    : Base(State::Identity(), params.initialCovariance(), Group::Identity()),
      params_(params),
      gN_(EarthInput(params.gravity, params.earthRate)),
      buffer_(params.bufferWindow) {}

/* ************************************************************************* */
GalileanFilter::GalileanFilter(const Params& params, const Gal3& navState,
                               double delay, const Vector10& bias,
                               const Matrix20& P0)
    : Base(State::Identity(), P0, InitialGroup(params, navState, delay, bias)),
      params_(params),
      gN_(EarthInput(params.gravity, params.earthRate)),
      buffer_(params.bufferWindow),
      initialized_(true) {}

/* ************************************************************************* */
void GalileanFilter::initializeFromIMU(const IMUInput& imu) {
  // At rest the specific force is -gravity: rotate it onto Earth "up".
  const Vector3 up = -params_.gravity.normalized();
  Vector3 measured = imu.acc;
  if (measured.norm() < 1e-9) measured = up;
  const Rot3 R0(Eigen::Quaterniond::FromTwoVectors(measured.normalized(), up));

  initialize(Gal3(R0, Point3::Zero(), Vector3::Zero(), 0.0));
}

/* ************************************************************************* */
void GalileanFilter::initialize(const Gal3& navState, const Vector10& bias) {
  resetReferenceAndGroup(
      State::Identity(), params_.initialCovariance(),
      InitialGroup(params_, navState, params_.initialDelay, bias));
  buffer_ = UpsilonBuffer(params_.bufferWindow);
  initialized_ = true;
  acceptedUpdates_ = 0;
  consecutiveRejections_ = 0;
  lastNis_ = 0.0;
}

/* ************************************************************************* */
Gal3 GalileanFilter::Gamma(double delta) const {
  return Gal3::Expmap(gN_ * delta);
}

/* ************************************************************************* */
Gal3 GalileanFilter::navState() const {
  // eq. 23: T(t) = Γ(δ)⁻¹ F
  return Gamma(delay()).inverse() * galileanFrame(state());
}

/* ************************************************************************* */
Gal3 GalileanFilter::delayedNavState() const {
  // eq. 17: T(t - δ) = F Υ(δ)⁻¹
  return galileanFrame(state()) * buffer_.at(delay()).inverse();
}

/* ************************************************************************* */
double GalileanFilter::delaySigma() const {
  return std::sqrt(errorCovariance()(index::kDelay, index::kDelay));
}

/* ************************************************************************* */
void GalileanFilter::predict(const IMUInput& imu, double dt) {
  if (!initialized_ || dt <= 0.0) {
    return;
  }

  const State xi = state();
  const Vector10 wTilde = imu.extended() - bias(xi);
  const Group& X = groupEstimate();

  const Matrix20 Phi = EqFStateMatrixA(X, gN_, wTilde, dt);
  const Matrix20 B = EqFInputMatrixB(X, gN_, wTilde, dt);
  const Matrix20 Qd = B * params_.inputNoise() * B.transpose() / dt +
                      params_.stateProcessNoise() * dt;

  // Alg. 1 propagation: X̂ ← X̂ Λ (exact discrete lift), Σ ← Φ Σ Φᵀ + Qd.
  predictWithStep(LiftDiscrete(xi, imu, gN_, dt), Phi, Qd);
  // Alg. 1 propagation: Υ_k ← Υ_k exp(w̃^ dt) for all k.
  buffer_.push(wTilde, dt);
}

/* ************************************************************************* */
Point3 GalileanFilter::predictGnss(const State& xi,
                                   OptionalJacobian<3, 20> H) const {
  const Gal3& F = galileanFrame(xi);
  Vector10 wStart;
  const Gal3 Upsilon = buffer_.at(F.time(), &wStart);
  Matrix3x10 HF;
  const Point3 y = GnssOutput(F, Upsilon, wStart, params_.antennaLever,
                              H ? &HF : nullptr);
  if (H) {
    H->setZero();
    H->leftCols<10>() = HF;
  }
  return y;
}

/* ************************************************************************* */
Matrix3x20 GalileanFilter::measurementMatrixC() const {
  Matrix3x10 HF;
  const Gal3& F = galileanFrame(state());
  Vector10 wStart;
  const Gal3 Upsilon = buffer_.at(F.time(), &wStart);
  GnssOutput(F, Upsilon, wStart, params_.antennaLever, HF);
  return EqFOutputMatrixC(groupEstimate(), HF);
}

/* ************************************************************************* */
bool GalileanFilter::update(const GnssMeasurement& measurement) {
  const Matrix3& R = measurement.covariance;
  if (!R.allFinite() || !R.isApprox(R.transpose()) ||
      R.llt().info() != Eigen::Success) {
    throw std::invalid_argument(
        "GalileanFilter::update: GNSS covariance must be symmetric positive "
        "definite");
  }
  if (!initialized_) {
    return false;
  }

  const Point3 yHat = predictGnss(state());
  const Matrix3x20 C = measurementMatrixC();  // eqs. 40-42

  // Outlier gate on the normalized innovation squared.
  const Vector3 r = measurement.position - yHat;
  const Matrix3 S = C * errorCovariance() * C.transpose() + R;
  lastNis_ = r.dot(S.ldlt().solve(r));
  const bool gateActive =
      acceptedUpdates_ >= params_.gnssGateWarmup &&
      consecutiveRejections_ < params_.gnssMaxConsecutiveRejections;
  if (gateActive && lastNis_ > params_.gnssOutlierThreshold) {
    ++consecutiveRejections_;
    return false;
  }
  ++acceptedUpdates_;
  consecutiveRejections_ = 0;

  // Alg. 1 update: K, Δ, Joseph update, X̂ ← exp(Δ) X̂.
  Vector20 epsilonBar = Vector20::Zero();
  Vector20 deltaX = Vector20::Zero();
  updateWithVector(Vector(yHat), Matrix(C), Vector(measurement.position),
                   Matrix(R), [&](const Vector20& deltaXi) {
                     epsilonBar = deltaXi;
                     deltaX = LiftInnovation(deltaXi);
                     return deltaX;
                   });

  // Alg. 1 update ⋆⋆: covariance reset.
  if (params_.resetStep) {
    const Matrix20 J = ResetJacobian(epsilonBar, deltaX);
    this->P_ = J * this->P_ * J.transpose();
  }
  assert(!errorCovariance().hasNaN());
  return true;
}

}  // namespace galilean
}  // namespace gtsam

