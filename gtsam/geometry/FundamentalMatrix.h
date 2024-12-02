/*
 * @file FundamentalMatrix.h
 * @brief FundamentalMatrix classes
 * @author Frank Dellaert
 * @date October 2024
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>

namespace gtsam {

/**
 * @class FundamentalMatrix
 * @brief Represents a fundamental matrix in computer vision, which encodes the
 * epipolar geometry between two views.
 *
 * The FundamentalMatrix class encapsulates the fundamental matrix, which
 * relates corresponding points in stereo images. It is parameterized by two
 * rotation matrices (U and V) and a scalar parameter (s).
 * Using these values, the fundamental matrix is represented as
 *
 *   F = U * diag(1, s, 0) * V^T
 */
class GTSAM_EXPORT FundamentalMatrix {
 private:
  Rot3 U_;    ///< Left rotation
  double s_;  ///< Scalar parameter for S
  Rot3 V_;    ///< Right rotation

 public:
  /// Default constructor
  FundamentalMatrix() : U_(Rot3()), s_(1.0), V_(Rot3()) {}

  /**
   * @brief Construct from U, V, and scalar s
   *
   * Initializes the FundamentalMatrix From the SVD representation
   * U*diag(1,s,0)*V^T. It will internally convert to using SO(3).
   */
  FundamentalMatrix(const Matrix3& U, double s, const Matrix3& V);

  /**
   * @brief Construct from a 3x3 matrix using SVD
   *
   * Initializes the FundamentalMatrix by performing SVD on the given
   * matrix and ensuring U and V are not reflections.
   *
   * @param F A 3x3 matrix representing the fundamental matrix
   */
  FundamentalMatrix(const Matrix3& F);

  /**
   * @brief Construct from essential matrix and calibration matrices
   *
   * Initializes the FundamentalMatrix from the given essential matrix E
   * and calibration matrices Ka and Kb, using
   *   F = Ka^(-T) * E * Kb^(-1)
   * and then calls constructor that decomposes F via SVD.
   *
   * @param E Essential matrix
   * @param Ka Calibration matrix for the left camera
   * @param Kb Calibration matrix for the right camera
   */
  FundamentalMatrix(const Matrix3& Ka, const EssentialMatrix& E,
                    const Matrix3& Kb)
      : FundamentalMatrix(Ka.transpose().inverse() * E.matrix() *
                          Kb.inverse()) {}

  /**
   * @brief Construct from calibration matrices Ka, Kb, and pose aPb
   *
   * Initializes the FundamentalMatrix from the given calibration
   * matrices Ka and Kb, and the pose aPb.
   *
   * @param Ka Calibration matrix for the left camera
   * @param aPb Pose from the left to the right camera
   * @param Kb Calibration matrix for the right camera
   */
  FundamentalMatrix(const Matrix3& Ka, const Pose3& aPb, const Matrix3& Kb)
      : FundamentalMatrix(Ka, EssentialMatrix::FromPose3(aPb), Kb) {}

  /// Return the fundamental matrix representation
  Matrix3 matrix() const;

  /// Computes the epipolar line in a (left) for a given point in b (right)
  Vector3 epipolarLine(const Point2& p, OptionalJacobian<3, 7> H = {});

  /// @name Testable
  /// @{
  /// Print the FundamentalMatrix
  void print(const std::string& s = "") const;

  /// Check if the FundamentalMatrix is equal to another within a
  /// tolerance
  bool equals(const FundamentalMatrix& other, double tol = 1e-9) const;
  /// @}

  /// @name Manifold
  /// @{
  inline constexpr static auto dimension = 7;  // 3 for U, 1 for s, 3 for V
  inline static size_t Dim() { return dimension; }
  inline size_t dim() const { return dimension; }

  /// Return local coordinates with respect to another FundamentalMatrix
  Vector localCoordinates(const FundamentalMatrix& F) const;

  /// Retract the given vector to get a new FundamentalMatrix
  FundamentalMatrix retract(const Vector& delta) const;
  /// @}
 private:
  /// Private constructor for internal use
  FundamentalMatrix(const Rot3& U, double s, const Rot3& V)
      : U_(U), s_(s), V_(V) {}

  /// Initialize SO(3) matrices from general O(3) matrices
  void initialize(Matrix3 U, double s, Matrix3 V);
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

  /// Return the left calibration matrix
  Matrix3 Ka() const;

  /// Return the right calibration matrix
  Matrix3 Kb() const;

 public:
  /// Default constructor
  SimpleFundamentalMatrix()
      : E_(), fa_(1.0), fb_(1.0), ca_(0.0, 0.0), cb_(0.0, 0.0) {}

  /**
   * @brief Construct from essential matrix and focal lengths
   * @param E Essential matrix
   * @param fa Focal length for left camera
   * @param fb Focal length for right camera
   * @param ca Principal point for left camera
   * @param cb Principal point for right camera
   */
  SimpleFundamentalMatrix(const EssentialMatrix& E,  //
                          double fa, double fb, const Point2& ca,
                          const Point2& cb)
      : E_(E), fa_(fa), fb_(fb), ca_(ca), cb_(cb) {}

  /// Return the fundamental matrix representation
  /// F = Ka^(-T) * E * Kb^(-1)
  Matrix3 matrix() const;

  /// Computes the epipolar line in a (left) for a given point in b (right)
  Vector3 epipolarLine(const Point2& p, OptionalJacobian<3, 7> H = {});

  /// @name Testable
  /// @{
  /// Print the SimpleFundamentalMatrix
  void print(const std::string& s = "") const;

  /// Check equality within a tolerance
  bool equals(const SimpleFundamentalMatrix& other, double tol = 1e-9) const;
  /// @}

  /// @name Manifold
  /// @{
  inline constexpr static auto dimension = 7;  // 5 for E, 1 for fa, 1 for fb
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
GTSAM_EXPORT Point2 EpipolarTransfer(const Matrix3& Fca, const Point2& pa,
                                     const Matrix3& Fcb, const Point2& pb);

/// Represents a set of three fundamental matrices for transferring points
/// between three cameras.
template <typename F>
struct TripleF {
  F Fab, Fbc, Fca;

  /// Transfers a point from cameras b,c to camera a.
  Point2 transferToA(const Point2& pb, const Point2& pc) {
    return EpipolarTransfer(Fab.matrix(), pb, Fca.matrix().transpose(), pc);
  }

  /// Transfers a point from camera a,c to camera b.
  Point2 transferToB(const Point2& pa, const Point2& pc) {
    return EpipolarTransfer(Fab.matrix().transpose(), pa, Fbc.matrix(), pc);
  }

  /// Transfers a point from cameras a,b to camera c.
  Point2 transferToC(const Point2& pa, const Point2& pb) {
    return EpipolarTransfer(Fca.matrix(), pa, Fbc.matrix().transpose(), pb);
  }
};

template <>
struct traits<FundamentalMatrix>
    : public internal::Manifold<FundamentalMatrix> {};

template <>
struct traits<SimpleFundamentalMatrix>
    : public internal::Manifold<SimpleFundamentalMatrix> {};

}  // namespace gtsam
