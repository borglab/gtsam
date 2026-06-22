/**
 * @file  Aff3.cpp
 * @brief Projective Special Linear Group (PSL(4, R)) Pose
 * @author: Hyungtae Lim
 */

#include <gtsam/geometry/SL4Aff3.h>

// To use exp(), log()
#include <cmath>
#include <limits>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>

using namespace std;

namespace {
using gtsam::Matrix44;
using gtsam::Vector6;


Eigen::Matrix<double, 16, 12> setVecToAlgMatrix() {
  Eigen::Matrix<double, 16, 12> alg = Eigen::Matrix<double, 16, 12>::Zero();

  // 12 Off-diagonal E_ij generators
  int k = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i != j) {
        alg(i * 4 + j, k++) = 1.0;
      }
    }
  }

  // For Diagonal generators B1 = diag(1, -1, 0, 0)
  alg(0, 9) = 1.0;
  alg(5, 9) = -1.0;

  // For B2 = diag(0, 1, -1, 0)
  alg(5, 10)  = 1.0;
  alg(10, 10) = -1.0;

  // For B3 = diag(0, 0, 1, -1)
  alg(10, 11) = 1.0;
  alg(15, 11) = -1.0;

  return alg;
}

inline Eigen::Matrix<double, 12, 16> setAlgtoVecMatrix() {
  Eigen::Matrix<double, 12, 16> mat;
  mat <<
    0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,  0.,
    1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,  0.,
    1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -1.;
  return mat;
}

// ALG_TO_VEC * VEC_TO_ALG is equals to I_12x12
const Eigen::Matrix<double, 16, 12> VEC_TO_ALG = setVecToAlgMatrix();
const Eigen::Matrix<double, 12, 16> ALG_TO_VEC = setAlgtoVecMatrix();

}  // namespace
namespace gtsam {

Aff3::Aff3(const Matrix44& pose) {
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
        "Aff3 Constructor: Input matrix is singular or invalid. " 
        "SVD singular values product = " + std::to_string(current_det_mag));
  }

  // Normalize: T = M / det^(1/4)
  const double scale = std::pow(current_det_mag, 0.25);
  T_ = M_corrected / scale;
}

/* ************************************************************************* */
void Aff3::print(const std::string& s) const { cout << s << T_ << "\n"; }

/* ************************************************************************* */
bool Aff3::equals(const Aff3& aff3, double tol) const {
  return T_.isApprox(aff3.T_, tol);
}
/* ************************************************************************* */
Aff3 Aff3::ChartAtOrigin::Retract(const Vector12& v, ChartJacobian H) {
  if (H) throw std::runtime_error("Aff3::Retract: Jacobian not implemented.");

  const Matrix44 candidate = I_4x4 + Hat(v);
  const double det = candidate.determinant();

  // Use fast first-order retraction when it stays inside SL(4); fall back to
  // the true exponential map otherwise to avoid invalid determinants.
  if (det > 0.0 && std::isfinite(det)) {
    return Aff3(candidate);
  }

  return Expmap(v);
}

/* ************************************************************************* */
Vector12 Aff3::ChartAtOrigin::Local(const Aff3& aff3, ChartJacobian H) {
  Vector xi = Vee(aff3.T_ - I_4x4);
  if (H) throw std::runtime_error("Aff3::Local: Jacobian not implemented.");
  return xi;
}

/* ************************************************************************* */
Aff3 Aff3::Expmap(const Vector& xi, Aff3Jacobian H) {
  if (xi.size() != 12) {
    throw std::runtime_error(
        "Aff3::Expmap: xi must be a vector of size 12. Got size " +
        std::to_string(xi.size()));
  }
  const auto& A = Hat(xi);

  if (H) throw std::runtime_error("Aff3::Expmap: Jacobian not implemented.");

  Matrix44 expA = A.exp();
  return Aff3(expA);
}

/* ************************************************************************* */
Vector Aff3::Logmap(const Aff3& p, Aff3Jacobian H) {
  if (H) throw std::runtime_error("Aff3::Logmap: Jacobian not implemented.");
  return Vee(p.T_.log());
}

/* ************************************************************************* */
Matrix12x12 Aff3::AdjointMap() const {
  Matrix44 H_inv_T = T_.inverse().transpose();
  Matrix16x16 C_H;

  // Kronecker product H ⊗ H^{-T}
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      C_H.block<4, 4>(i * 4, j * 4) = T_(i, j) * H_inv_T;

  return ALG_TO_VEC * C_H * VEC_TO_ALG;
}

/* ************************************************************************* */
Matrix44 Aff3::Hat(const Vector& xi) {
  if (xi.size() != 12) {
    throw std::runtime_error(
        "Aff3::Hat: xi must be a vector of size 12. Got size " +
        std::to_string(xi.size()));
  }
  Matrix44 mat;
    const double d11 =  xi(9);
    const double d22 = -xi(9) + xi(10);
    const double d33 = -xi(10) + xi(11);
    const double d44 = -xi(11);

    mat  <<   d11,  xi(0), xi(1), xi(2),
            xi(3),    d22, xi(4), xi(5),
            xi(6),  xi(7), d33,   xi(8),
            0.0,    0.0,    0.0,  d44;

    return mat;
}

/* ************************************************************************* */
// Used consistent notation with Hat()
Vector Aff3::Vee(const Matrix44& X) {
  Vector vec(12);
  const double x12 = X(0, 0);
  const double x13 = X(1, 1) + x12;
  const double x14 = -X(3, 3);
  vec <<  X(0, 1), X(0, 2), X(0, 3),
          X(1, 0), X(1, 2), X(1, 3),
          X(3, 0), X(3, 1), X(3, 2),
          x12, x13, x14;
  return vec;
}

}  // namespace gtsam
