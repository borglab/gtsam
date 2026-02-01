/*
 * @file FundamentalMatrix.cpp
 * @brief FundamentalMatrix classes
 * @author Frank Dellaert
 * @date October 2024
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

//*************************************************************************
Point2 EpipolarTransfer(const Matrix3& Fca, const Point2& pa,  //
                        const Matrix3& Fcb, const Point2& pb) {
  // Create lines in camera a from projections of the other two cameras
  Vector3 line_a = Fca * Vector3(pa.x(), pa.y(), 1);
  Vector3 line_b = Fcb * Vector3(pb.x(), pb.y(), 1);

  // Cross the lines to find the intersection point
  Vector3 intersectionPoint = line_a.cross(line_b);

  // Normalize the intersection point
  intersectionPoint /= intersectionPoint(2);

  return intersectionPoint.head<2>();  // Return the 2D point
}

//*************************************************************************
FundamentalMatrix::FundamentalMatrix(const Matrix3& U, double s,
                                     const Matrix3& V) {
  initialize(U, s, V);
}

FundamentalMatrix::FundamentalMatrix(const Matrix3& F) {
  // Perform SVD
  Eigen::JacobiSVD<Matrix3> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  #if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  if (svd.info() != Eigen::ComputationInfo::Success) {
    throw std::runtime_error("FundamentalMatrix::FundamentalMatrix: SVD computation failure");
  }
  #endif

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

  initialize(U, singularValues(1), V);
}

void FundamentalMatrix::initialize(Matrix3 U, double s, Matrix3 V) {
  // Check if U is a reflection and flip third column if so
  if (U.determinant() < 0) U.col(2) *= -1;

  // Same check for V
  if (V.determinant() < 0) V.col(2) *= -1;

  U_ = Rot3(U);
  s_ = s;
  V_ = Rot3(V);
}

Matrix3 FundamentalMatrix::matrix() const {
  return U_.matrix() * Vector3(1.0, s_, 0).asDiagonal() *
         V_.transpose().matrix();
}

Vector3 FundamentalMatrix::epipolarLine(const Point2& p,
                                        OptionalJacobian<3, 7> H) {
  Vector3 point(p.x(), p.y(), 1);  // Create a point in homogeneous coordinates
  Vector3 line = matrix() * point;  // Compute the epipolar line

  if (H) {
    // Compute the Jacobian if requested
    throw std::runtime_error(
        "FundamentalMatrix::epipolarLine: Jacobian not implemented yet.");
  }

  return line;  // Return the epipolar line
}

void FundamentalMatrix::print(const std::string& s) const {
  std::cout << s << matrix() << std::endl;
}

bool FundamentalMatrix::equals(const FundamentalMatrix& other,
                               double tol) const {
  return U_.equals(other.U_, tol) && std::abs(s_ - other.s_) < tol &&
         V_.equals(other.V_, tol);
}

Vector FundamentalMatrix::localCoordinates(const FundamentalMatrix& F) const {
  Vector result(7);
  result.head<3>() = U_.localCoordinates(F.U_);
  result(3) = F.s_ - s_;  // Difference in scalar
  result.tail<3>() = V_.localCoordinates(F.V_);
  return result;
}

FundamentalMatrix FundamentalMatrix::retract(const Vector& delta) const {
  const Rot3 newU = U_.retract(delta.head<3>());
  const double newS = s_ + delta(3);
  const Rot3 newV = V_.retract(delta.tail<3>());
  return FundamentalMatrix(newU, newS, newV);
}

//*************************************************************************
Matrix3 SimpleFundamentalMatrix::Ka() const {
  Matrix3 K;
  K << fa_, 0, ca_.x(), 0, fa_, ca_.y(), 0, 0, 1;
  return K;
}

Matrix3 SimpleFundamentalMatrix::Kb() const {
  Matrix3 K;
  K << fb_, 0, cb_.x(), 0, fb_, cb_.y(), 0, 0, 1;
  return K;
}

Matrix3 SimpleFundamentalMatrix::matrix() const {
  return Ka().transpose().inverse() * E_.matrix() * Kb().inverse();
}

Vector3 SimpleFundamentalMatrix::epipolarLine(const Point2& p,
                                              OptionalJacobian<3, 7> H) {
  Vector3 point(p.x(), p.y(), 1);  // Create a point in homogeneous coordinates
  Vector3 line = matrix() * point;  // Compute the epipolar line

  if (H) {
    // Compute the Jacobian if requested
    throw std::runtime_error(
        "SimpleFundamentalMatrix::epipolarLine: Jacobian not implemented yet.");
  }

  return line;  // Return the epipolar line
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

//*************************************************************************

}  // namespace gtsam
