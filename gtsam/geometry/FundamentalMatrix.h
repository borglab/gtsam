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

  /**
   * @brief Virtual destructor to ensure proper cleanup of derived classes
   */
  virtual ~FundamentalMatrix() {}

  /**
   * @brief Transfer projections from cameras 1 and 2 to camera 0
   *
   * Take two fundamental matrices F01 and F02, and two points p1 and p2, and
   * returns the 2D point in camera 0 where the epipolar lines intersect.
   */
  static Point2 transfer(const Matrix3& F01, const Point2& p1,
                         const Matrix3& F02, const Point2& p2) {
    // Create lines in camera 0 from projections of the other two cameras
    Vector3 line1 = F01 * Vector3(p1.x(), p1.y(), 1);
    Vector3 line2 = F02 * Vector3(p2.x(), p2.y(), 1);

    // Cross the lines to find the intersection point
    Vector3 intersectionPoint = line1.cross(line2);

    // Normalize the intersection point
    intersectionPoint /= intersectionPoint(2);

    return intersectionPoint.head<2>();  // Return the 2D point
  }
};

/// Represents a set of three fundamental matrices for transferring points
/// between three cameras.
template <typename F>
struct TripleF {
  F F01, F12, F20;

  /// Transfers a point from cameras 1,2 to camera 0.
  Point2 transfer0(const Point2& p1, const Point2& p2) {
    return FundamentalMatrix::transfer(F01.matrix(), p1,
                                       F20.matrix().transpose(), p2);
  }

  /// Transfers a point from camera 0,2 to camera 1.
  Point2 transfer1(const Point2& p0, const Point2& p2) {
    return FundamentalMatrix::transfer(F01.matrix().transpose(), p0,
                                       F12.matrix(), p2);
  }

  /// Transfers a point from camera 0,1 to camera 2.
  Point2 transfer2(const Point2& p0, const Point2& p1) {
    return FundamentalMatrix::transfer(F01.matrix(), p0,
                                       F12.matrix().transpose(), p1);
  }
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
  GeneralFundamentalMatrix(const Matrix3& F);

  /// Return the fundamental matrix representation
  Matrix3 matrix() const override;

  /// @name Testable
  /// @{
  /// Print the GeneralFundamentalMatrix
  void print(const std::string& s = "") const;

  /// Check if the GeneralFundamentalMatrix is equal to another within a
  /// tolerance
  bool equals(const GeneralFundamentalMatrix& other, double tol = 1e-9) const;
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = 7 };  // 3 for U, 1 for s, 3 for V
  inline static size_t Dim() { return dimension; }
  inline size_t dim() const { return dimension; }

  /// Return local coordinates with respect to another GeneralFundamentalMatrix
  Vector localCoordinates(const GeneralFundamentalMatrix& F) const;

  /// Retract the given vector to get a new GeneralFundamentalMatrix
  GeneralFundamentalMatrix retract(const Vector& delta) const;
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
class SimpleFundamentalMatrix : public FundamentalMatrix {
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
  Matrix3 leftK() const;

  /// Return the right calibration matrix
  Matrix3 rightK() const;

  /// Return the fundamental matrix representation
  Matrix3 matrix() const override;

  /// @name Testable
  /// @{
  /// Print the SimpleFundamentalMatrix
  void print(const std::string& s = "") const;

  /// Check equality within a tolerance
  bool equals(const SimpleFundamentalMatrix& other, double tol = 1e-9) const;
  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = 7 };  // 5 for E, 1 for fa, 1 for fb
  inline static size_t Dim() { return dimension; }
  inline size_t dim() const { return dimension; }

  /// Return local coordinates with respect to another SimpleFundamentalMatrix
  Vector localCoordinates(const SimpleFundamentalMatrix& F) const;

  /// Retract the given vector to get a new SimpleFundamentalMatrix
  SimpleFundamentalMatrix retract(const Vector& delta) const;
  /// @}
};

template <>
struct traits<GeneralFundamentalMatrix>
    : public internal::Manifold<GeneralFundamentalMatrix> {};

template <>
struct traits<SimpleFundamentalMatrix>
    : public internal::Manifold<SimpleFundamentalMatrix> {};

}  // namespace gtsam
