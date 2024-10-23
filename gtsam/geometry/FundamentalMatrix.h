/*
 * @file FundamentalMatrix.h
 * @brief FundamentalMatrix class
 * @author Frank Dellaert
 * @date Oct 23, 2024
 */

#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>

namespace gtsam {

class FundamentalMatrix {
 private:
  Rot3 U_;    ///< Left rotation
  double s_;  ///< Scalar parameter for S
  Rot3 V_;    ///< Right rotation

 public:
  /// Default constructor
  FundamentalMatrix() : U_(Rot3()), s_(1.0), V_(Rot3()) {}

  /// Construct from U, V, and scalar s
  FundamentalMatrix(const Rot3& U, double s, const Rot3& V)
      : U_(U), s_(s), V_(V) {}

  /// Return the fundamental matrix representation
  Matrix3 matrix() const {
    return U_.matrix() * Vector3(1, s_, 1).asDiagonal() *
           V_.transpose().matrix();
  }

  /// @name Testable
  /// @{

  /// Print the FundamentalMatrix
  void print(const std::string& s = "") const {
    std::cout << s << "U:\n"
              << U_.matrix() << "\ns: " << s_ << "\nV:\n"
              << V_.matrix() << std::endl;
  }

  /// Check if the FundamentalMatrix is equal to another within a tolerance
  bool equals(const FundamentalMatrix& other, double tol = 1e-9) const {
    return U_.equals(other.U_, tol) && std::abs(s_ - other.s_) < tol &&
           V_.equals(other.V_, tol);
  }

  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = 7 };  // 3 for U, 1 for s, 3 for V
  inline static size_t Dim() { return dimension; }
  inline size_t dim() const { return dimension; }

  /// Return local coordinates with respect to another FundamentalMatrix
  Vector localCoordinates(const FundamentalMatrix& F) const {
    Vector result(7);
    result.head<3>() = U_.localCoordinates(F.U_);
    result(3) = F.s_ - s_;  // Difference in scalar
    result.tail<3>() = V_.localCoordinates(F.V_);
    return result;
  }

  /// Retract the given vector to get a new FundamentalMatrix
  FundamentalMatrix retract(const Vector& delta) const {
    Rot3 newU = U_.retract(delta.head<3>());
    double newS = s_ + delta(3);  // Update scalar
    Rot3 newV = V_.retract(delta.tail<3>());
    return FundamentalMatrix(newU, newS, newV);
  }

  /// @}
};

template <>
struct traits<FundamentalMatrix>
    : public internal::Manifold<FundamentalMatrix> {};

}  // namespace gtsam
