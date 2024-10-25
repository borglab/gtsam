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
 * @class GeneralFundamentalMatrix
 * @brief Represents a general fundamental matrix.
 *
 * This class represents a general fundamental matrix, which is a 3x3 matrix
 * that describes the relationship between two images. It is parameterized by a
 * left rotation U, a scalar s, and a right rotation V.
 */
class GTSAM_EXPORT GeneralFundamentalMatrix {
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
  Matrix3 matrix() const;

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
class GTSAM_EXPORT SimpleFundamentalMatrix {
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
  Matrix3 matrix() const;

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

/**
 * @brief Transfer projections from cameras a and b to camera c
 *
 * Take two fundamental matrices Fca and Fcb, and two points pa and pb, and
 * returns the 2D point in view (c) where the epipolar lines intersect.
 */
GTSAM_EXPORT Point2 Transfer(const Matrix3& Fca, const Point2& pa,
                             const Matrix3& Fcb, const Point2& pb);

/// Represents a set of three fundamental matrices for transferring points
/// between three cameras.
template <typename F>
struct TripleF {
  F Fab, Fbc, Fca;

  /// Transfers a point from cameras b,c to camera a.
  Point2 transferToA(const Point2& pb, const Point2& pc) {
    return Transfer(Fab.matrix(), pb, Fca.matrix().transpose(), pc);
  }

  /// Transfers a point from camera a,c to camera b.
  Point2 transferToB(const Point2& pa, const Point2& pc) {
    return Transfer(Fab.matrix().transpose(), pa, Fbc.matrix(), pc);
  }

  /// Transfers a point from cameras a,b to camera c.
  Point2 transferToC(const Point2& pa, const Point2& pb) {
    return Transfer(Fca.matrix(), pa, Fbc.matrix().transpose(), pb);
  }
};

template <>
struct traits<GeneralFundamentalMatrix>
    : public internal::Manifold<GeneralFundamentalMatrix> {};

template <>
struct traits<SimpleFundamentalMatrix>
    : public internal::Manifold<SimpleFundamentalMatrix> {};

}  // namespace gtsam
