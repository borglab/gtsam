/**
 * @file  SL4.cpp
 * @brief Projective Special Linear Group (PSL(4, R)) Pose
 * @author: Hyungtae Lim
 */

#include <gtsam/geometry/SL4.h>

// To use exp(), log()
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;

namespace {
Eigen::Matrix<double, 15, 15> I_15x15 =
    Eigen::Matrix<double, 15, 15>::Identity();

Eigen::Matrix<double, 16, 15> setVecToAlgMatrix() {
  Eigen::Matrix<double, 16, 15> alg = Eigen::Matrix<double, 16, 15>::Zero();

  // 12 Off-diagonal E_ij generators
  int k = 0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i != j) {
        alg(i * 4 + j, k++) = 1.0;
      }
    }
  }

  // For Diagonal generators B1 = diag(1, -1, 0, 0)
  alg(0, 12) = 1.0;
  alg(5, 12) = -1.0;

  // For B2 = diag(0, 1, -1, 0)
  alg(5, 13) = 1.0;
  alg(10, 13) = -1.0;

  // For B3 = diag(0, 0, 1, -1)
  alg(10, 14) = 1.0;
  alg(15, 14) = -1.0;

  return alg;
}

Eigen::Matrix<double, 15, 16> setAlgtoVecMatrix() {
  Eigen::Matrix<double, 15, 16> mat;
  mat << 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
      1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 1., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., -1.;
  return mat;
}

// ALG_TO_VEC * VEC_TO_ALG is equals to I_15x15
const Eigen::Matrix<double, 16, 15> VEC_TO_ALG = setVecToAlgMatrix();
const Eigen::Matrix<double, 15, 16> ALG_TO_VEC = setAlgtoVecMatrix();
}  // namespace
namespace gtsam {

SL4::SL4(const Matrix44& pose) {
  double det = pose.determinant();
  if (det <= 0.0) {
    throw std::runtime_error(
        "Matrix determinant must be positive for SL(4) normalization. Got det "
        "= " +
        std::to_string(det));
  }

  T_ = pose / std::pow(det, 1.0 / 4.0);
}

/* ************************************************************************* */
void SL4::print(const std::string& s) const { cout << s << T_ << "\n"; }

/* ************************************************************************* */
bool SL4::equals(const SL4& sl4, double tol) const {
  return T_.isApprox(sl4.T_, tol);
}
/* ************************************************************************* */
SL4 SL4::ChartAtOrigin::Retract(const Vector15& v, ChartJacobian H) {
  assert(v.size() == 15);
  SL4 retracted(I_4x4 + Hat(v));
  if (H) throw std::runtime_error("SL4::Retract: Jacobian not implemented.");
  return retracted;
}

/* ************************************************************************* */
Vector15 SL4::ChartAtOrigin::Local(const SL4& sl4, ChartJacobian H) {
  Vector xi = Vee(sl4.T_ - I_4x4);
  if (H) throw std::runtime_error("SL4::Local: Jacobian not implemented.");
  return xi;
}

/* ************************************************************************* */
SL4 SL4::Expmap(const Vector& xi, SL4Jacobian H) {
  assert(xi.size() == 15);
  const auto& A = Hat(xi);

  if (H) throw std::runtime_error("SL4::Expmap: Jacobian not implemented.");

  // NOTE(hlim):
  // The cost of the computation is approximately 20n^3 for matrices of size n.
  // The number 20 depends weakly on the norm of the matrix.
  // See
  // https://eigen.tuxfamily.org/dox/unsupported/group__MatrixFunctions__Module.html

  return SL4(A.exp());
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
  assert(xi.size() == 15);
  Matrix44 A;
  const double d11 = xi(12);
  const double d22 = -xi(12) + xi(13);
  const double d33 = -xi(13) + xi(14);
  const double d44 = -xi(14);

  A << d11, xi(0), xi(1), xi(2), xi(3), d22, xi(4), xi(5), xi(6), xi(7), d33,
      xi(8), xi(9), xi(10), xi(11), d44;

  return A;
}

/* ************************************************************************* */
// NOTE(hlim): Why 'X'? - I just follow the convention of GTSAM
Vector SL4::Vee(const Matrix44& X) {
  Vector vec(15);
  const double x12 = X(0, 0);
  const double x13 = X(1, 1) + x12;
  const double x14 = -X(3, 3);
  vec << X(0, 1), X(0, 2), X(0, 3), X(1, 0), X(1, 2), X(1, 3), X(2, 0), X(2, 1),
      X(2, 3), X(3, 0), X(3, 1), X(3, 2), x12, x13, x14;
  return vec;
}

}  // namespace gtsam
