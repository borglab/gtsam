/*
 * @file FundamentalMatrix.cpp
 * @brief FundamentalMatrix classes
 * @author Frank Dellaert
 * @date Oct 23, 2024
 */

#include <gtsam/geometry/FundamentalMatrix.h>

namespace gtsam {

GeneralFundamentalMatrix::GeneralFundamentalMatrix(const Matrix3& F) {
  // Perform SVD
  Eigen::JacobiSVD<Matrix3> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Extract U and V
  Matrix3 U = svd.matrixU();
  Matrix3 V = svd.matrixV();
  Vector3 singularValues = svd.singularValues();

  // Scale the singular values
  double scale = singularValues(0);
  if (scale != 0) {
    singularValues /= scale;  // Normalize the first singular value to 1.0
  }

  // Check if the third singular value is close to zero (valid F condition)
  if (std::abs(singularValues(2)) > 1e-9) {
    throw std::invalid_argument(
        "The input matrix does not represent a valid fundamental matrix.");
  }

  // Ensure the second singular value is recorded as s
  s_ = singularValues(1);

  // Check if U is a reflection
  if (U.determinant() < 0) {
    U = -U;
    s_ = -s_;  // Change sign of scalar if U is a reflection
  }

  // Check if V is a reflection
  if (V.determinant() < 0) {
    V = -V;
    s_ = -s_;  // Change sign of scalar if U is a reflection
  }

  // Assign the rotations
  U_ = Rot3(U);
  V_ = Rot3(V);
}

Matrix3 GeneralFundamentalMatrix::matrix() const {
  return U_.matrix() * Vector3(1, s_, 0).asDiagonal() * V_.transpose().matrix();
}

void GeneralFundamentalMatrix::print(const std::string& s) const {
  std::cout << s << "U:\n"
            << U_.matrix() << "\ns: " << s_ << "\nV:\n"
            << V_.matrix() << std::endl;
}

bool GeneralFundamentalMatrix::equals(const GeneralFundamentalMatrix& other,
                                      double tol) const {
  return U_.equals(other.U_, tol) && std::abs(s_ - other.s_) < tol &&
         V_.equals(other.V_, tol);
}

Vector GeneralFundamentalMatrix::localCoordinates(
    const GeneralFundamentalMatrix& F) const {
  Vector result(7);
  result.head<3>() = U_.localCoordinates(F.U_);
  result(3) = F.s_ - s_;  // Difference in scalar
  result.tail<3>() = V_.localCoordinates(F.V_);
  return result;
}

GeneralFundamentalMatrix GeneralFundamentalMatrix::retract(
    const Vector& delta) const {
  Rot3 newU = U_.retract(delta.head<3>());
  double newS = s_ + delta(3);  // Update scalar
  Rot3 newV = V_.retract(delta.tail<3>());
  return GeneralFundamentalMatrix(newU, newS, newV);
}

Matrix3 SimpleFundamentalMatrix::leftK() const {
  Matrix3 K;
  K << fa_, 0, ca_.x(), 0, fa_, ca_.y(), 0, 0, 1;
  return K;
}

Matrix3 SimpleFundamentalMatrix::rightK() const {
  Matrix3 K;
  K << fb_, 0, cb_.x(), 0, fb_, cb_.y(), 0, 0, 1;
  return K;
}

Matrix3 SimpleFundamentalMatrix::matrix() const {
  return leftK().transpose().inverse() * E_.matrix() * rightK().inverse();
}

void SimpleFundamentalMatrix::print(const std::string& s) const {
  std::cout << s << " E:\n"
            << E_.matrix() << "\nfa: " << fa_ << "\nfb: " << fb_
            << "\nca: " << ca_.transpose() << "\ncb: " << cb_.transpose()
            << std::endl;
}

bool SimpleFundamentalMatrix::equals(const SimpleFundamentalMatrix& other,
                                     double tol) const {
  return E_.equals(other.E_, tol) && std::abs(fa_ - other.fa_) < tol &&
         std::abs(fb_ - other.fb_) < tol && (ca_ - other.ca_).norm() < tol &&
         (cb_ - other.cb_).norm() < tol;
}

Vector SimpleFundamentalMatrix::localCoordinates(
    const SimpleFundamentalMatrix& F) const {
  Vector result(7);
  result.head<5>() = E_.localCoordinates(F.E_);
  result(5) = F.fa_ - fa_;  // Difference in fa
  result(6) = F.fb_ - fb_;  // Difference in fb
  return result;
}

SimpleFundamentalMatrix SimpleFundamentalMatrix::retract(
    const Vector& delta) const {
  EssentialMatrix newE = E_.retract(delta.head<5>());
  double newFa = fa_ + delta(5);  // Update fa
  double newFb = fb_ + delta(6);  // Update fb
  return SimpleFundamentalMatrix(newE, newFa, newFb, ca_, cb_);
}

}  // namespace gtsam
