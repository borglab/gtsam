/** This class provides functionality for computing the (sparse) upper
 * triangular factor R and permutation P from a sparse QR factorization of the
 * form:
 *
 *   AP = QR
 *
 * This is useful for solving large-scale linear least-squares problems via
 * (well-conditioned) orthogonal decomposition.
 *
 * This small module is based upon Eigen's SPQRSupport, but avoids the
 * computation and storage of the orthogonal factor Q.
 *
 * Copyright(C) 2020 by David M.Rosen(dmrosen @mit.edu)
 **/

#pragma once

#include "Eigen/SPQRSupport"
#include "Preconditioners/Types.h"

namespace Preconditioners {

class LSChol {

  // Column normalization requires COLUMN-MAJOR storage order
  typedef SuiteSparse_long StorageIndex;
  typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor, StorageIndex>
      LSCholSparseMatrix;
  typedef SparseMatrix::RealScalar RealScalar;
  typedef SuiteSparse_long Index;

private:
  /// Data members

  /// FACTORIZATION ELEMENTS

  /// Cholmod configuration struct
  cholmod_common chol_com_;

  SparseMatrix R_; // The sparse matrix R in Eigen format
  Permutation P_;  // Permutation
  Index rank_;     // The rank of the matrix

  // Treat columns with 2-norm below this tolerance as zero during
  // factorization.
  RealScalar pivot_tol_;
  int ordering_;             // Ordering method to use, see SPQR's manual
  int num_fact_tol_;         // Allow to use some tolerance during numerical
  bool useDefaultThreshold_; // Use default threshold

  // Boolean value indicating whether the object contains a valid cached
  // factorization
  bool initialized_ = false;

  /// Helper function: initialize Cholmod
  void init();

public:
  /// Constructors

  /** Construct an empty LSChol object */
  LSChol();

  /** Construct an LSChol object containing a factorization
   * of the passed matrix A */
  LSChol(const SparseMatrix &A);

  /// Accessors

  /** Return the R-factor of the QR decomposition */
  const SparseMatrix &R() const { return R_; }

  /** Return the permutation P used in this factorization */
  const Permutation &P() const { return P_; }

  /** Get rank of computed triangular factor */
  Index rank() const { return rank_; }

  /// Mutators

  /// Set the fill-reducing ordering method to be used
  void setSPQROrdering(int ordering) { ordering_ = ordering; }

  /// Set the tolerance tol to treat columns with 2-norm <= tol as zero
  void setPivotThreshold(const RealScalar &tol) {
    useDefaultThreshold_ = false;
    pivot_tol_ = tol;
  }

  /** Compute the "Q-less" QR factorization of the matrix matrix A. */
  void compute(const SparseMatrix &A);

  /** Return a *mutable* pointer to the SPQR workspace */
  cholmod_common *cholmodCommon() { return &chol_com_; }

  /// Linear-algebraic operations

  /** Computes and returns the product P * R^{-1} * x */
  Vector PRinv(const Vector &x) const;

  /** Computes and returns the product R^{-T} * P^{-1} * x */
  Vector RinvTPinv(const Vector &x) const;

  /** Computes and returns the product R * P^{-1} * x */
  Vector RPinv(const Vector &x) const;

  /** Computes and returns the product
   *
   * P * R^{-1} * R^{-T} * P^{-1} * x = (A^T * A)^{-1} * x
   */
  Vector solve(const Vector &x) const;

  /// Destructor
  ~LSChol();
};

} // namespace Preconditioners
