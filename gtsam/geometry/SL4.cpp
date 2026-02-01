/**
 * @file  SL4.cpp
 * @brief Projective Special Linear Group (PSL(4, R)) Pose
 * @author: Hyungtae Lim
 */

#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/SO4.h>

// To use exp(), log()
#include <cmath>
#include <limits>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>

using namespace std;

namespace {
using gtsam::Matrix44;
using gtsam::Vector6;

constexpr double kInvSqrt2 = 0.7071067811865475244;
constexpr double kInvSqrt6 = 0.4082482904638630164;
constexpr double kInvSqrt12 = 0.2886751345948128823;

Vector6 SkewToSO4(const Vector6& r) {
  Vector6 so4;
  so4 << r(5), -r(4), r(2), -r(3), r(1), -r(0);
  return so4;
}

Vector6 SO4ToSkew(const Vector6& so4) {
  Vector6 r;
  r << -so4(5), so4(4), so4(2), -so4(3), -so4(1), so4(0);
  return r;
}

Matrix44 HatSym4(const Vector6& s) {
  Matrix44 A = Matrix44::Zero();
  A(0, 1) = s(0);
  A(1, 0) = s(0);
  A(0, 2) = s(1);
  A(2, 0) = s(1);
  A(0, 3) = s(2);
  A(3, 0) = s(2);
  A(1, 2) = s(3);
  A(2, 1) = s(3);
  A(1, 3) = s(4);
  A(3, 1) = s(4);
  A(2, 3) = s(5);
  A(3, 2) = s(5);
  return A;
}

Vector6 VeeSym4(const Matrix44& A) {
  Vector6 s;
  s << A(0, 1), A(0, 2), A(0, 3), A(1, 2), A(1, 3), A(2, 3);
  return s;
}

Eigen::Matrix<double, 16, 15> setVecToAlgMatrix() {
  Eigen::Matrix<double, 16, 15> alg = Eigen::Matrix<double, 16, 15>::Zero();

  int k = 0;
  auto set_skew = [&](int i, int j) {
    alg(i * 4 + j, k) = kInvSqrt2;
    alg(j * 4 + i, k) = -kInvSqrt2;
    ++k;
  };
  auto set_sym = [&](int i, int j) {
    alg(i * 4 + j, k) = kInvSqrt2;
    alg(j * 4 + i, k) = kInvSqrt2;
    ++k;
  };

  // Rotations (skew-symmetric).
  set_skew(0, 1);
  set_skew(0, 2);
  set_skew(0, 3);
  set_skew(1, 2);
  set_skew(1, 3);
  set_skew(2, 3);

  // Symmetric off-diagonal shears.
  set_sym(0, 1);
  set_sym(0, 2);
  set_sym(0, 3);
  set_sym(1, 2);
  set_sym(1, 3);
  set_sym(2, 3);

  // Traceless diagonal scalings.
  alg(0, k) = kInvSqrt2;
  alg(5, k) = -kInvSqrt2;
  ++k;

  alg(0, k) = kInvSqrt6;
  alg(5, k) = kInvSqrt6;
  alg(10, k) = -2.0 * kInvSqrt6;
  ++k;

  alg(0, k) = kInvSqrt12;
  alg(5, k) = kInvSqrt12;
  alg(10, k) = kInvSqrt12;
  alg(15, k) = -3.0 * kInvSqrt12;
  ++k;

  return alg;
}

Eigen::Matrix<double, 15, 16> setAlgtoVecMatrix(
    const Eigen::Matrix<double, 16, 15>& vec_to_alg) {
  return vec_to_alg.transpose();
}

// For the orthonormal basis, ALG_TO_VEC * VEC_TO_ALG is the identity.
const Eigen::Matrix<double, 16, 15> VEC_TO_ALG = setVecToAlgMatrix();
const Eigen::Matrix<double, 15, 16> ALG_TO_VEC = setAlgtoVecMatrix(VEC_TO_ALG);

}  // namespace
namespace gtsam {

SL4::SL4(const Matrix44& pose) {
  // Compute SVD: pose = U * S * V^T
  const Eigen::JacobiSVD<Matrix44> svd(pose, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Matrix44 U = svd.matrixU();
  const Matrix44 V = svd.matrixV();
  const Vector4 S = svd.singularValues();

  // Handle Orientation (Negative Determinant / Reflection)
  const double detUV = (U * V.transpose()).determinant();
  
  if (detUV < 0.0) {
    U.col(3) = -U.col(3);
  }

  // Reconstruct the matrix with corrected orientation
  const Matrix44 M_corrected = U * S.asDiagonal() * V.transpose();
  const double current_det_mag = S.prod();
  
   // Check for Singularity
  if (current_det_mag <= std::numeric_limits<double>::epsilon() || !std::isfinite(current_det_mag)) {
    throw std::runtime_error(
        "SL4 Constructor: Input matrix is singular or invalid. " 
        "SVD singular values product = " + std::to_string(current_det_mag));
  }

  // Normalize: T = M / det^(1/4)
  const double scale = std::pow(current_det_mag, 0.25);
  T_ = M_corrected / scale;
}

/* ************************************************************************* */
void SL4::print(const std::string& s) const { cout << s << T_ << "\n"; }

/* ************************************************************************* */
bool SL4::equals(const SL4& sl4, double tol) const {
  return T_.isApprox(sl4.T_, tol);
}
/* ************************************************************************* */
SL4 SL4::ChartAtOrigin::Retract(const Vector15& v, ChartJacobian H) {
  if (H) throw std::runtime_error("SL4::Retract: Jacobian not implemented.");

  const Matrix44 candidate = I_4x4 + Hat(v);
  const double det = candidate.determinant();

  // Use fast first-order retraction when it stays inside SL(4); fall back to
  // the true exponential map otherwise to avoid invalid determinants.
  if (det > 0.0 && std::isfinite(det)) {
    return SL4(candidate);
  }

  return Expmap(v);
}

/* ************************************************************************* */
Vector15 SL4::ChartAtOrigin::Local(const SL4& sl4, ChartJacobian H) {
  Vector xi = Vee(sl4.T_ - I_4x4);
  if (H) throw std::runtime_error("SL4::Local: Jacobian not implemented.");
  return xi;
}

/* ************************************************************************* */
SL4 SL4::Expmap(const Vector& xi, SL4Jacobian H) {
  if (xi.size() != 15) {
    throw std::runtime_error(
        "SL4::Expmap: xi must be a vector of size 15. Got size " +
        std::to_string(xi.size()));
  }
  const auto& A = Hat(xi);

  if (H) throw std::runtime_error("SL4::Expmap: Jacobian not implemented.");

  // NOTE(hlim):
  // The cost of the computation is approximately 20n^3 for matrices of size n.
  // The number 20 depends weakly on the norm of the matrix. See
  // https://eigen.tuxfamily.org/dox/unsupported/group__MatrixFunctions__Module.html

  // For SL(4), the Lie algebra consists of trace-zero 4x4 matrices.
  // The exponential of a trace-zero matrix should have determinant 1 by the property:
  // det(exp(A)) = exp(trace(A)) = exp(0) = 1.
  // However, for large tangent vectors, numerical errors in the matrix exponential
  // can cause the determinant to drift from 1. The constructor handles normalization.
  
  Matrix44 expA = A.exp();
  return SL4(expA);
}

/* ************************************************************************* */
Vector SL4::Logmap(const SL4& p, SL4Jacobian H) {
  if (H) throw std::runtime_error("SL4::Logmap: Jacobian not implemented.");
  return Vee(p.T_.log());
}

/* ************************************************************************* */
Matrix15x15 SL4::AdjointMap() const {
  Matrix44 H_inv_T = T_.inverse().transpose();
  Matrix16x16 C_H;

  // Kronecker product H ⊗ H^{-T}
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      C_H.block<4, 4>(i * 4, j * 4) = T_(i, j) * H_inv_T;

  return ALG_TO_VEC * C_H * VEC_TO_ALG;
}

/* ************************************************************************* */
Matrix44 SL4::Hat(const Vector& xi) {
  if (xi.size() != 15) {
    throw std::runtime_error(
        "SL4::Hat: xi must be a vector of size 15. Got size " +
        std::to_string(xi.size()));
  }
  const Vector6 r = kInvSqrt2 * xi.head<6>();
  const Vector6 s = kInvSqrt2 * xi.segment<6>(6);

  Matrix44 A = SO4::Hat(SkewToSO4(r)) + HatSym4(s);

  // Traceless diagonal scalings.
  const double a = kInvSqrt2 * xi(12);
  const double b = kInvSqrt6 * xi(13);
  const double c = kInvSqrt12 * xi(14);

  Vector4 diag;
  diag << a + b + c, -a + b + c, -2.0 * b + c, -3.0 * c;
  A.diagonal() += diag;

  return A;
}

/* ************************************************************************* */
// Used consistent notation with Hat()
Vector SL4::Vee(const Matrix44& A) {
  Vector xi(15);
  const Matrix44 skew = A - A.transpose();
  const Matrix44 sym = A + A.transpose();
  xi.head<6>() = kInvSqrt2 * SO4ToSkew(SO4::Vee(skew));
  xi.segment<6>(6) = kInvSqrt2 * VeeSym4(sym);
  xi(12) = kInvSqrt2 * (A(0, 0) - A(1, 1));
  xi(13) = kInvSqrt6 * (A(0, 0) + A(1, 1) - 2.0 * A(2, 2));
  xi(14) = kInvSqrt12 * (A(0, 0) + A(1, 1) + A(2, 2) - 3.0 * A(3, 3));
  return xi;
}

}  // namespace gtsam
