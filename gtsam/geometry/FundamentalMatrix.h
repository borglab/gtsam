/*
 * @file FundamentalMatrix.h
 * @brief FundamentalMatrix classes
 * @author Frank Dellaert
 * @date Oct 23, 2024
 */

#pragma once

#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>

namespace gtsam {

/**
 * @brief Abstract base class for FundamentalMatrix
 *
 * This class provides a common interface for all types of fundamental matrices.
 * It declares a virtual function `matrix()` that must be implemented by derived
 * classes. The `matrix()` function returns a 3x3 matrix representation of the
 * fundamental matrix.
 */
class FundamentalMatrix {
 public:
  /**
   * @brief Returns a 3x3 matrix representation of the fundamental matrix
   *
   * @return A 3x3 matrix representing the fundamental matrix
   */
  virtual Matrix3 matrix() const = 0;
};

/**
 * @class GeneralFundamentalMatrix
 * @brief Represents a general fundamental matrix.
 *
 * This class represents a general fundamental matrix, which is a 3x3 matrix
 * that describes the relationship between two images. It is parameterized by a
 * left rotation U, a scalar s, and a right rotation V.
 */
class GeneralFundamentalMatrix : public FundamentalMatrix {
 private:
  Rot3 U_;    ///< Left rotation
  double s_;  ///< Scalar parameter for S
  Rot3 V_;    ///< Right rotation

 public:
  /// Default constructor
  GeneralFundamentalMatrix() : U_(Rot3()), s_(1.0), V_(Rot3()) {}

  /**
   * @brief Construct from U, V, and scalar s
   *
   * Initializes the GeneralFundamentalMatrix with the given left rotation U,
   * scalar s, and right rotation V.
   *
   * @param U Left rotation matrix
   * @param s Scalar parameter for the fundamental matrix
   * @param V Right rotation matrix
   */
  GeneralFundamentalMatrix(const Rot3& U, double s, const Rot3& V)
      : U_(U), s_(s), V_(V) {}

  /**
   * @brief Construct from a 3x3 matrix using SVD
   *
   * Initializes the GeneralFundamentalMatrix by performing SVD on the given
   * matrix and ensuring U and V are not reflections.
   *
   * @param F A 3x3 matrix representing the fundamental matrix
   */
  GeneralFundamentalMatrix(const Matrix3& F) {
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

  /// Return the fundamental matrix representation
  Matrix3 matrix() const override {
    return U_.matrix() * Vector3(1, s_, 0).asDiagonal() *
           V_.transpose().matrix();
  }

  /// @name Testable
  /// @{

  /// Print the GeneralFundamentalMatrix
  void print(const std::string& s = "") const {
    std::cout << s << "U:\n"
              << U_.matrix() << "\ns: " << s_ << "\nV:\n"
              << V_.matrix() << std::endl;
  }

  /// Check if the GeneralFundamentalMatrix is equal to another within a
  /// tolerance
  bool equals(const GeneralFundamentalMatrix& other, double tol = 1e-9) const {
    return U_.equals(other.U_, tol) && std::abs(s_ - other.s_) < tol &&
           V_.equals(other.V_, tol);
  }

  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = 7 };  // 3 for U, 1 for s, 3 for V
  inline static size_t Dim() { return dimension; }
  inline size_t dim() const { return dimension; }

  /// Return local coordinates with respect to another GeneralFundamentalMatrix
  Vector localCoordinates(const GeneralFundamentalMatrix& F) const {
    Vector result(7);
    result.head<3>() = U_.localCoordinates(F.U_);
    result(3) = F.s_ - s_;  // Difference in scalar
    result.tail<3>() = V_.localCoordinates(F.V_);
    return result;
  }

  /// Retract the given vector to get a new GeneralFundamentalMatrix
  GeneralFundamentalMatrix retract(const Vector& delta) const {
    Rot3 newU = U_.retract(delta.head<3>());
    double newS = s_ + delta(3);  // Update scalar
    Rot3 newV = V_.retract(delta.tail<3>());
    return GeneralFundamentalMatrix(newU, newS, newV);
  }

  /// @}
};

/**
 * @class SimpleFundamentalMatrix
 * @brief Class for representing a simple fundamental matrix.
 *
 * This class represents a simple fundamental matrix, which is a
 * parameterization of the essential matrix and focal lengths for left and right
 * cameras. Principal points are not part of the manifold but a convenience.
 */
class SimpleFundamentalMatrix : FundamentalMatrix {
 private:
  EssentialMatrix E_;  ///< Essential matrix
  double fa_;          ///< Focal length for left camera
  double fb_;          ///< Focal length for right camera
  Point2 ca_;          ///< Principal point for left camera
  Point2 cb_;          ///< Principal point for right camera

 public:
  /// Default constructor
  SimpleFundamentalMatrix()
      : E_(), fa_(1.0), fb_(1.0), ca_(0.0, 0.0), cb_(0.0, 0.0) {}

  /// Construct from essential matrix and focal lengths
  SimpleFundamentalMatrix(const EssentialMatrix& E,  //
                          double fa, double fb,
                          const Point2& ca = Point2(0.0, 0.0),
                          const Point2& cb = Point2(0.0, 0.0))
      : E_(E), fa_(fa), fb_(fb), ca_(ca), cb_(cb) {}

  /// Return the left calibration matrix
  Matrix3 leftK() const {
    Matrix3 K;
    K << fa_, 0, ca_.x(), 0, fa_, ca_.y(), 0, 0, 1;
    return K;
  }

  /// Return the right calibration matrix
  Matrix3 rightK() const {
    Matrix3 K;
    K << fb_, 0, cb_.x(), 0, fb_, cb_.y(), 0, 0, 1;
    return K;
  }

  /// Return the fundamental matrix representation
  Matrix3 matrix() const override {
    return leftK().transpose().inverse() * E_.matrix() * rightK().inverse();
  }
  /// @name Testable
  /// @{

  /// Print the SimpleFundamentalMatrix
  void print(const std::string& s = "") const {
    std::cout << s << "E:\n"
              << E_.matrix() << "\nfa: " << fa_ << "\nfb: " << fb_
              << "\nca: " << ca_ << "\ncb: " << cb_ << std::endl;
  }

  /// Check equality within a tolerance
  bool equals(const SimpleFundamentalMatrix& other, double tol = 1e-9) const {
    return E_.equals(other.E_, tol) && std::abs(fa_ - other.fa_) < tol &&
           std::abs(fb_ - other.fb_) < tol && (ca_ - other.ca_).norm() < tol &&
           (cb_ - other.cb_).norm() < tol;
  }
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = 7 };  // 5 for E, 1 for fa, 1 for fb
  inline static size_t Dim() { return dimension; }
  inline size_t dim() const { return dimension; }

  /// Return local coordinates with respect to another
  /// SimpleFundamentalMatrix
  Vector localCoordinates(const SimpleFundamentalMatrix& F) const {
    Vector result(7);
    result.head<5>() = E_.localCoordinates(F.E_);
    result(5) = F.fa_ - fa_;  // Difference in fa
    result(6) = F.fb_ - fb_;  // Difference in fb
    return result;
  }

  /// Retract the given vector to get a new SimpleFundamentalMatrix
  SimpleFundamentalMatrix retract(const Vector& delta) const {
    EssentialMatrix newE = E_.retract(delta.head<5>());
    double newFa = fa_ + delta(5);  // Update fa
    double newFb = fb_ + delta(6);  // Update fb
    return SimpleFundamentalMatrix(newE, newFa, newFb, ca_, cb_);
  }
  /// @}
};

template <>
struct traits<GeneralFundamentalMatrix>
    : public internal::Manifold<GeneralFundamentalMatrix> {};

template <>
struct traits<SimpleFundamentalMatrix>
    : public internal::Manifold<SimpleFundamentalMatrix> {};

}  // namespace gtsam
