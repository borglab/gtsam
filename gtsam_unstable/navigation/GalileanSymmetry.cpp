

#include <gtsam_unstable/navigation/GalileanSymmetry.h>

#include <algorithm>

namespace gtsam {
namespace galilean {

namespace {
/// Common blocks of eqs. 34-36 for input ẘ = Ad_{X_F} w̃ at the origin.
struct ErrorBlocks {
  Matrix10 AdX;  ///< Ad_{X_F}
  Matrix10 A1;   ///< Ad_{exp(-g_N dt)}
  Matrix10 JL;   ///< left Jacobian of exp at -ẘ dt
  Matrix10 A2;   ///< A1 Ad_{exp(ẘ dt)}
};

ErrorBlocks errorBlocks(const Group& X, const Vector10& gN,
                        const Vector10& wTilde, double dt) {
  ErrorBlocks blocks;
  blocks.AdX = X.first.AdjointMap();
  const Vector10 wOrigin = blocks.AdX * wTilde;
  blocks.A1 = Gal3::Expmap(-gN * dt).AdjointMap();
  blocks.JL = Gal3::ExpmapDerivative(-wOrigin * dt);
  blocks.A2 = blocks.A1 * Gal3::Expmap(wOrigin * dt).AdjointMap();
  return blocks;
}
}  // namespace

/* ************************************************************************* */
State Symmetry::operator()(const State& xi, const Group& X,
                           OptionalJacobian<20, 20> H_xi,
                           OptionalJacobian<20, 20> H_X) const {
  const Gal3& F = xi.first;
  const Vector10& b = xi.second;
  const Gal3& A = X.first;
  const Vector10& a = X.second;

  // eq. 28: φ(ξ, X) = (F A, Ad_{A⁻¹}(b - a))
  const Matrix10 AdAinv = A.inverse().AdjointMap();
  const Vector10 w = AdAinv * (b - a);

  if (H_xi) {
    // (F e^η) A = F A e^{Ad_{A⁻¹} η};  the b-part is linear in b.
    H_xi->setZero();
    H_xi->topLeftCorner<10, 10>() = AdAinv;
    H_xi->bottomRightCorner<10, 10>() = AdAinv;
  }
  if (H_X) {
    // X e^ζ ≈ (A e^{ζu}, a + Ad_A ζv):
    //   F-part: F A e^{ζu}            -> I
    //   b-part: e^{-ad ζu} w - ζv      -> [ad_w, -I]
    H_X->setZero();
    H_X->topLeftCorner<10, 10>() = Matrix10::Identity();
    H_X->bottomLeftCorner<10, 10>() = Gal3::adjointMap(w);
    H_X->bottomRightCorner<10, 10>() = -Matrix10::Identity();
  }
  return State(F * A, w);
}

/* ************************************************************************* */
State Dynamics(const State& xi, const IMUInput& imu, const Vector10& gN,
               double dt) {
  const Gal3& F = xi.first;
  const Vector10& b = xi.second;
  // eq. 25
  const Gal3 Fn = Gal3::Expmap(-gN * dt) * F *
                  Gal3::Expmap((imu.extended() - b) * dt);
  return State(Fn, b);
}

/* ************************************************************************* */
Group LiftDiscrete(const State& xi, const IMUInput& imu, const Vector10& gN,
                   double dt) {
  const Gal3& F = xi.first;
  const Vector10& b = xi.second;
  // eq. 30: gravity pulled through the state, then the biased IMU increment.
  const Vector10 gBody = F.inverse().AdjointMap() * gN;
  const Gal3 LambdaF = Gal3::Expmap(-gBody * dt) *
                       Gal3::Expmap((imu.extended() - b) * dt);
  // eq. 31 with τ = 0 (bias random walk enters only through the noise).
  const Vector10 LambdaB = b - LambdaF.AdjointMap() * b;
  return Group(LambdaF, LambdaB);
}

/* ************************************************************************* */
Matrix20 EqFStateMatrixA(const Group& X, const Vector10& gN,
                         const Vector10& wTilde, double dt) {
  const ErrorBlocks k = errorBlocks(X, gN, wTilde, dt);
  // eq. 34 in GTSAM error coordinates (S A S).
  Matrix20 Phi = Matrix20::Zero();
  Phi.topLeftCorner<10, 10>() = k.A1;
  Phi.topRightCorner<10, 10>() = -k.A1 * k.JL * dt;
  Phi.bottomRightCorner<10, 10>() = k.A2;
  return Phi;
}

/* ************************************************************************* */
Matrix20 EqFInputMatrixB(const Group& X, const Vector10& gN,
                         const Vector10& wTilde, double dt) {
  const ErrorBlocks k = errorBlocks(X, gN, wTilde, dt);
  // eq. 36 in GTSAM error coordinates (S B).
  Matrix20 B = Matrix20::Zero();
  B.topLeftCorner<10, 10>() = -k.A1 * k.JL * k.AdX * dt;
  B.bottomRightCorner<10, 10>() = k.A2 * k.AdX * dt;
  return B;
}

/* ************************************************************************* */
Point3 GnssOutput(const Gal3& F, const Gal3& Upsilon, const Vector10& wStart,
                  const Point3& leverArm, OptionalJacobian<3, 10> H) {
  const Gal3 T = F * Upsilon.inverse();  // T(t - δ), eq. 17
  const Matrix3 R = T.rotation().matrix();
  const Point3 y = R * leverArm + T.translation();  // eq. 38

  if (H) {
    // y w.r.t. a right perturbation T e^η, η = (ω, ν, ρ, τ):
    //   η^ (p0, 0, 1) = (ω×p0 + ρ, τ, 0)  ->  R(ω×p0 + ρ) + v_T τ
    Matrix3x10 HT = Matrix3x10::Zero();
    HT.block<3, 3>(0, 0) = -R * skewSymmetric(leverArm);
    HT.block<3, 3>(0, 6) = R;
    HT.col(9) = T.velocity();
    // T = F Υ(δ_F)⁻¹ with δ_F = time(F):
    //   F e^η -> T e^{Ad_Υ η};  Υ⁻¹(δ + dδ) = Υ⁻¹ e^{-w̃(t-δ) dδ}, dδ = η_τ
    Matrix10 dT = Upsilon.AdjointMap();
    dT.col(9) -= wStart;
    *H = HT * dT;
  }
  return y;
}

/* ************************************************************************* */
Matrix3x20 EqFOutputMatrixC(const Group& X, const Matrix3x10& H_F) {
  // Dφ_X at ξ₀: maps error coordinates to the chart at the current estimate.
  Matrix20 Dphi;
  Symmetry()(State::Identity(), X, Dphi);
  return H_F * Dphi.topRows<10>();  // biases do not enter (paper eq. 40)
}

/* ************************************************************************* */
Vector20 LiftInnovation(const Vector20& deltaXi) {
  Vector20 deltaX = deltaXi;
  deltaX.tail<10>() *= -1.0;  // S = diag(I, -I) = pinv(Dφ₀)
  return deltaX;
}

/* ************************************************************************* */
Matrix20 ResetJacobian(const Vector20& epsilonBar, const Vector20& deltaX) {
  // After X̂ ← exp(δx) X̂ the error becomes e' = φ(e, exp(-δx)).
  // ε' = Local(ξ₀, φ(Retract(ξ₀, ε), exp(-δx))), linearized at ε = ε̄.
  const State xi0 = State::Identity();
  Matrix20 Hret, Hxi, Hloc;
  const State e = xi0.retract(epsilonBar, {}, Hret);
  const State eNew = Symmetry()(e, Group::Expmap(-deltaX), Hxi);
  xi0.localCoordinates(eNew, {}, Hloc);
  return Hloc * Hxi * Hret;
}

/* ************************************************************************* */
void UpsilonBuffer::push(const Vector10& wTilde, double dt) {
  entries_.push_back({now_, wTilde, Gal3()});
  const Gal3 increment = Gal3::Expmap(wTilde * dt);
  for (Entry& e : entries_) e.upsilon = e.upsilon * increment;  // eq. 20
  now_ += dt;
  // Keep exactly one entry starting at or before now - window.
  while (entries_.size() > 1 && entries_[1].start <= now_ - window_) {
    entries_.pop_front();
  }
}

/* ************************************************************************* */
double UpsilonBuffer::span() const {
  return entries_.empty() ? 0.0 : now_ - entries_.front().start;
}

/* ************************************************************************* */
Gal3 UpsilonBuffer::at(double delta, Vector10* wStart) const {
  if (entries_.empty()) {
    if (wStart) wStart->setZero();
    return Gal3();
  }
  const double s = now_ - std::clamp(delta, 0.0, span());
  // Last entry whose held interval starts at or before s.
  auto it = std::upper_bound(
      entries_.begin(), entries_.end(), s,
      [](double value, const Entry& e) { return value < e.start; });
  if (it != entries_.begin()) --it;
  if (wStart) *wStart = it->input;
  // Trim [s_k, s) off the front of Υ_k: exact under zero-order hold.
  return Gal3::Expmap(-it->input * (s - it->start)) * it->upsilon;
}

}  // namespace galilean
}  // namespace gtsam
