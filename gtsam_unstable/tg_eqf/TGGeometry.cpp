#include <gtsam_unstable/tg_eqf/TGGeometry.h>

#include <iostream>

namespace gtsam {
namespace tgeqf {

// Chart: theta = log_G . phi_this^-1.
State State::retract(const State::TangentVector& delta) const {
  return phi(TGElement::Expmap(delta), *this);
}

State::TangentVector State::localCoordinates(const State& other) const {
  return TGElement::Logmap(phiInverse(*this, other));
}

}  // namespace tgeqf
}  // namespace gtsam

namespace gtsam {

bool traits<tgeqf::State>::Equals(const tgeqf::State& a, const tgeqf::State& b,
                                  double tol) {
  return a.T.equals(b.T, tol) && (a.b - b.b).norm() < tol;
}

void traits<tgeqf::State>::Print(const tgeqf::State& xi,
                                 const std::string& str) {
  std::cout << str;
  xi.T.print("  T: ");
  std::cout << "  b: " << xi.b.transpose() << "\n";
}

}  // namespace gtsam

namespace gtsam {
namespace tgeqf {

State phi(const TGElement& X, const State& xi) {
  State result;
  result.T = xi.T * X.first;
  result.b = X.first.inverse().AdjointMap() * (xi.b - X.second);
  return result;
}

// Nav block is the ExtendedPose3 "between" of xi_ref.T and xi_est.T; bias
// block follows from b_est = Ad_{A^-1}(b_ref - a), so a = b_ref - Ad_A(b_est).
TGElement phiInverse(const State& xi_ref, const State& xi_est) {
  TGElement X(xi_ref.T.inverse() * xi_est.T, Vector9::Zero());
  const Vector9 Ad_b_est = X.first.AdjointMap() * xi_est.b;
  X.second = xi_ref.b - Ad_b_est;
  return X;
}

TGSymmetry::Orbit::Orbit(const State& xi_ref) : xi_ref(xi_ref) {}

// d(phi)/dX is exactly I_18 everywhere, by freeness of phi.
State TGSymmetry::Orbit::operator()(const TGElement& X,
                                    State::Jacobian* H) const {
  const State result = phi(X, xi_ref);

  if (H) {
    H->setIdentity();
  }

  return result;
}

TGSymmetry::Diffeomorphism::Diffeomorphism(const TGElement& X) : X(X) {}

// d(phi)/dxi is exactly Ad_{X^-1}, by the right-action axiom and freeness.
State TGSymmetry::Diffeomorphism::operator()(const State& xi,
                                             State::Jacobian* H) const {
  const State result = phi(X, xi);

  if (H) {
    *H = X.inverse().AdjointMap();
  }

  return result;
}

}  // namespace tgeqf
}  // namespace gtsam
