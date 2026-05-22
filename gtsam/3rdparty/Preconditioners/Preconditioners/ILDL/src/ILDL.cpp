#include <exception>
#include <iostream>

#include "Eigen/Eigenvalues"
#include "ILDL/ILDL.h"
#include "ILDL/ILDL_utils.h"

namespace Preconditioners {

/// Constructors

// Basic constructor: just set the options
ILDL::ILDL(const ILDLOpts &options) { setOptions(options); }

// Advanced constructor: set the options, and then call compute() function to
// factor the passed matrix A
ILDL::ILDL(const SparseMatrix &A, const ILDLOpts &options) {
  setOptions(options);
  compute(A);
}

void ILDL::setOptions(const ILDLOpts &options) {
  /// Release any currently-held (cached) factorizations
  clear();

  /// Input checking

  if (options.max_fill_factor <= 0)
    throw std::invalid_argument("Maximum fill-factor must be a positive value");

  if (options.drop_tol < 0 || options.drop_tol > 1)
    throw std::invalid_argument(
        "Drop tolerance must be a value in the range [0,1]");

  if (options.BK_pivot_tol < 0 || options.BK_pivot_tol > 1)
    throw std::invalid_argument(
        "Bunch-Kaufman pivoting tolerance must be a value in the range [0,1]");

  // Save the passed options
  opts_ = options;
}

void ILDL::compute(const SparseMatrix &A) {

  // If we already have a cached factorization stored ...
  if (initialized_) {
    // Release it
    clear();
  }

  /// Argument checking
  if (A.rows() != A.cols())
    throw std::invalid_argument("Argument A must be a symmetric matrix!");

  // Dimension of A
  dim_ = A.rows();

  /// Preallocate storage to hold the incomplete factorization of A, computed
  /// using SYM-ILDL

  // LIL-C representation of A used by SYM-ILDL
  lilc_matrix<Scalar> Alilc;

  // Lower-triangular factor L
  lilc_matrix<Scalar> L;

  // Block-diagonal factor D
  block_diag_matrix<Scalar> D;

  // Row/col permutation
  std::vector<int> perm(dim_);

  /// Construct representation of A

  // Construct CSR representation of
  std::vector<int> row_ptr, col_idx;
  std::vector<Scalar> val;
  toCSR(A, row_ptr, col_idx, val);

  // Construct SYM-ILDL representation of passed matrix A.  Note that although
  // SYM-ILDL expects compressed COLUMN storage arguments, here we take
  // advantage of the fact that the CSR representation of A's UPPER TRIANGLE
  // actually coincides with the CSC representation of A's LOWER TRIANGLE :-)
  Alilc.load(row_ptr, col_idx, val);

  /// Equilibrate A using a diagonal scaling matrix S, if requested.
  // This will overwrite Alilc with SAS, and save the diagonal scaling matrix as
  // Alilc.S
  if (opts_.equilibration == Equilibration::Bunch)
    Alilc.sym_equil();

  /// Record scaling matrix S
  S_.resize(dim_);
  for (int k = 0; k < dim_; ++k)
    S_(k) = Alilc.S.main_diag[k];

  /// Compute fill-reducing reordering of A, if requested
  switch (opts_.order) {
  case Ordering::AMD:
    Alilc.sym_amd(perm);
    break;
  case Ordering::RCM:
    Alilc.sym_rcm(perm);
    break;
  case Ordering::None:
    // Set perm to be the identity permutation
    perm.resize(dim_);
    for (int k = 0; k < dim_; ++k)
      perm[k] = k;
    break;
  }

  // Apply this permutation to A_, if one was requested
  if (opts_.order != Ordering::None)
    Alilc.sym_perm(perm);

  /// Compute in-place LDL factorization of P*S*A*S*P
  Alilc.ildl(L, D, perm, opts_.max_fill_factor, opts_.drop_tol,
             opts_.BK_pivot_tol,
             (opts_.pivot_type == PivotType::Rook
                  ? lilc_matrix<Scalar>::pivot_type::ROOK
                  : lilc_matrix<Scalar>::pivot_type::BKP));

  /// Record the final permutation in P and Pinv
  P_.resize(dim_);
  Pinv_.resize(dim_);
  for (int k = 0; k < dim_; ++k) {
    P_(k) = perm[k];
    Pinv_[P_(k)] = k;
  }

  /// Construct lower-triangular Eigen matrix L_ from L
  std::vector<Eigen::Triplet<Scalar>> triplets;
  triplets.reserve(L.nnz());

  // From the lilc_matrix documentation: A(m_idx[k][j], k) = m_x[k][j]
  for (int k = 0; k < L.n_cols(); ++k)
    for (int j = 0; j < L.m_idx[k].size(); ++j)
      triplets.emplace_back(L.m_idx[k][j], k, L.m_x[k][j]);

  L_.resize(dim_, dim_);
  L_.setFromTriplets(triplets.begin(), triplets.end());

  /// Construct and record eigendecomposition for block diagonal matrix D

  // Get the number of 1- and 2-d blocks in D
  size_t num_2d_blocks = D.off_diag.size();
  size_t num_1d_blocks = dim_ - 2 * num_2d_blocks;
  size_t num_blocks = num_1d_blocks + num_2d_blocks;

  // Preallocate storage for this computation
  Lambda_.resize(dim_);
  block_start_idxs_.resize(num_blocks);
  block_sizes_.resize(num_blocks);

  // 2x2 matrix we will use to store any 2x2 blocks of D
  Matrix2d Di;
  // Eigensolver for computing an eigendecomposition of the 2x2 blocks of D
  Eigen::SelfAdjointEigenSolver<Matrix2d> eig;

  int idx = 0; // Starting (upper-left) index of the current block
  for (size_t i = 0; i < num_blocks; ++i) {
    // Record the starting index of this block
    block_start_idxs_[i] = idx;

    if (D.block_size(idx) > 1) {
      // This is a 2x2 block
      block_sizes_[i] = 2;

      // Extract 2x2 block from D

      // Extract diagonal elements
      Di(0, 0) = D.main_diag[idx];
      Di(1, 1) = D.main_diag[idx + 1];
      // Extract off-diagonal elements
      Di(0, 1) = D.off_diag.at(idx);
      Di(1, 0) = D.off_diag.at(idx);

      // Compute eigendecomposition of Di
      eig.compute(Di);

      // Record eigenvalues of this block
      Lambda_.segment<2>(idx) = eig.eigenvalues();

      // Record eigenvectors of this block
      Q_[i] = eig.eigenvectors();

      // Increment index
      idx += 2;
    } else {
      /// This is a 1x1 block
      block_sizes_[i] = 1;

      // Record eigenvalue
      Lambda_(idx) = D.main_diag[idx];

      // Increment index
      ++idx;
    }
  }

  // Record the fact that we now have a valid cached factorization
  initialized_ = true;
}

void ILDL::clear() {

  // If we have a cached factorization ...
  if (initialized_) {
    // Release the memory associated with this factorization
    block_start_idxs_.clear();
    block_sizes_.clear();
    Q_.clear();
  }

  // Record the fact that we no longer have a valid cached factorization
  initialized_ = false;
}

SparseMatrix ILDL::D(bool pos_def_mod) const {

  if (!initialized_)
    throw std::invalid_argument("Factorization has not yet been computed");

  // We rebuild D from its eigendecomposition according to whether we are
  // enforcing positive-definiteness

  std::vector<Eigen::Triplet<Scalar>> triplets;
  triplets.reserve(dim_ + 2 * num_2x2_blocks());

  // Preallocate working variables
  int idx; // Starting index of current block

  Matrix2d Di; // Working space for reconstructing 2x2 blocks

  // Iterate over the blocks of D
  for (size_t i = 0; i < num_blocks(); ++i) {
    idx = block_start_idxs_[i];
    if (block_sizes_[i] == 1) {
      triplets.emplace_back(idx, idx,
                            pos_def_mod ? fabs(Lambda_(idx)) : Lambda_(idx));
    } else {
      // Reconstruct the 2x2 block here
      const Matrix2d &Qi = Q_.at(i);

      if (pos_def_mod)
        Di = Qi * Lambda_.segment<2>(idx).cwiseAbs().asDiagonal() *
             Qi.transpose();
      else
        Di = Qi * Lambda_.segment<2>(idx).asDiagonal() * Qi.transpose();

      for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 2; ++c)
          triplets.emplace_back(idx + r, idx + c, Di(r, c));
    }
  }

  /// Reconstruct and return D
  SparseMatrix D(dim_, dim_);
  D.setFromTriplets(triplets.begin(), triplets.end());

  return D;
}

Inertia ILDL::inertia() const {
  if (!initialized_)
    throw std::invalid_argument("Factorization has not yet been computed");

  // Calculate number of positive eigenvalues
  size_t npos = (Lambda_.array() > 0.0).count();

  return std::make_pair(npos, dim_ - npos);
}

Vector ILDL::Dproduct(const Vector &x, bool pos_def_mod) const {
  /// Error checking
  if (!initialized_)
    throw std::invalid_argument("Factorization has not yet been computed");

  if (x.size() != dim_)
    throw std::invalid_argument("Argument x has incorrect dimension");

  // Preallocate output vector y = D*x
  Vector y(dim_);

  // We compute the output vector y blockwise
  for (int k = 0; k < num_blocks(); ++k) {

    // Get the starting index for this block
    const int &idx = block_start_idxs_[k];

    if (block_sizes_[k] == 1) {
      // This is a 1x1 block
      y(idx) = (pos_def_mod ? fabs(Lambda_[idx]) : Lambda_[idx]) * x(idx);
    } else {
      // This is a 2x2 block
      const Matrix2d &Qk = Q_.at(k);

      if (!pos_def_mod) {
        y.segment<2>(idx) = Qk * Lambda_.segment<2>(idx).asDiagonal() *
                            Qk.transpose() * x.segment<2>(idx);
      } else {
        // Take the absolute values of the eigenvalues of this block to enforce
        // positive-definiteness
        y.segment<2>(idx) = Qk *
                            Lambda_.segment<2>(idx).cwiseAbs().asDiagonal() *
                            Qk.transpose() * x.segment<2>(idx);
      }
    }
  }

  return y;
}

Vector ILDL::Dsolve(const Vector &b, bool pos_def_mod) const {
  /// Error checking
  if (!initialized_)
    throw std::invalid_argument("Factorization has not yet been computed");

  if (b.size() != dim_)
    throw std::invalid_argument("Argument b has incorrect dimension");

  // Preallocate output vector x = D^-1 * b
  Vector x(dim_);

  // We compute the output vector y blockwise
  for (int k = 0; k < num_blocks(); ++k) {

    // Get the starting index for this block
    const int &idx = block_start_idxs_[k];

    if (block_sizes_[k] == 1) {
      // This is a 1x1 block
      x(idx) = b(idx) / (pos_def_mod ? fabs(Lambda_[idx]) : Lambda_[idx]);
    } else {
      // This is a 2x2 block
      const Matrix2d &Qk = Q_.at(k);

      if (!pos_def_mod) {
        x.segment<2>(idx) =
            Qk * Lambda_.segment<2>(idx).cwiseInverse().asDiagonal() *
            Qk.transpose() * b.segment<2>(idx);
      } else {
        // Take the absolute values of the eigenvalues of this block to enforce
        // positive-definiteness
        x.segment<2>(idx) =
            Qk *
            Lambda_.segment<2>(idx).cwiseInverse().cwiseAbs().asDiagonal() *
            Qk.transpose() * b.segment<2>(idx);
      }
    }
  }

  return x;
}

Vector ILDL::sqrtDsolve(const Vector &b) const {
  /// Error checking
  if (!initialized_)
    throw std::invalid_argument("Factorization has not yet been computed");

  if (b.size() != dim_)
    throw std::invalid_argument("Argument b has incorrect dimension");

  // Preallocate output vector x = (D+)^{-1/2} * b
  Vector x(dim_);

  // We compute the output vector y blockwise
  for (int k = 0; k < num_blocks(); ++k) {

    // Get the starting index for this block
    const int &idx = block_start_idxs_[k];

    if (block_sizes_[k] == 1) {
      // This is a 1x1 block
      x(idx) = b(idx) / sqrt(fabs(Lambda_[idx]));
    } else {
      // This is a 2x2 block
      const Matrix2d &Qk = Q_.at(k);

      // Take the absolute values of the eigenvalues of this block to enforce
      // positive-definiteness
      x.segment<2>(idx) = Qk *
                          Lambda_.segment<2>(idx)
                              .cwiseAbs()
                              .cwiseSqrt()
                              .cwiseInverse()
                              .asDiagonal() *
                          Qk.transpose() * b.segment<2>(idx);
    }
  }

  return x;
}

Vector ILDL::LDLTsolve(const Vector &b, bool pos_def_mode) const {
  /// Error checking
  if (!initialized_)
    throw std::invalid_argument("Factorization has not yet been computed");

  if (b.size() != dim_)
    throw std::invalid_argument("Argument b has incorrect dimension");

  return L_.transpose().triangularView<Eigen::UnitUpper>().solve(
      Dsolve(L_.triangularView<Eigen::UnitLower>().solve(b), pos_def_mode));
}

Vector ILDL::sqrtDLTsolve(const Vector &b, bool transpose) const {
  if (!transpose)
    return L_.transpose().triangularView<Eigen::UnitUpper>().solve(
        sqrtDsolve(b));
  else
    return sqrtDsolve(L_.triangularView<Eigen::UnitLower>().solve(b));
}

Vector ILDL::solve(const Vector &b, bool pos_def_mod) const {
  /// If P'SASP ~ LDL', then A^-1 ~ SP (LDL')^-1 P'S
  return S_.cwiseProduct(
      P_.asPermutation() *
      LDLTsolve(Pinv_.asPermutation() * S_.cwiseProduct(b), pos_def_mod));
}

} // namespace Preconditioners
