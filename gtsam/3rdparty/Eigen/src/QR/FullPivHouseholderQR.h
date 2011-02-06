// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_FULLPIVOTINGHOUSEHOLDERQR_H
#define EIGEN_FULLPIVOTINGHOUSEHOLDERQR_H

/** \ingroup QR_Module
  *
  * \class FullPivHouseholderQR
  *
  * \brief Householder rank-revealing QR decomposition of a matrix with full pivoting
  *
  * \param MatrixType the type of the matrix of which we are computing the QR decomposition
  *
  * This class performs a rank-revealing QR decomposition of a matrix \b A into matrices \b P, \b Q and \b R
  * such that 
  * \f[
  *  \mathbf{A} \, \mathbf{P} = \mathbf{Q} \, \mathbf{R}
  * \f]
  * by using Householder transformations. Here, \b P is a permutation matrix, \b Q a unitary matrix and \b R an 
  * upper triangular matrix.
  *
  * This decomposition performs a very prudent full pivoting in order to be rank-revealing and achieve optimal
  * numerical stability. The trade-off is that it is slower than HouseholderQR and ColPivHouseholderQR.
  *
  * \sa MatrixBase::fullPivHouseholderQr()
  */
template<typename _MatrixType> class FullPivHouseholderQR
{
  public:

    typedef _MatrixType MatrixType;
    enum {
      RowsAtCompileTime = MatrixType::RowsAtCompileTime,
      ColsAtCompileTime = MatrixType::ColsAtCompileTime,
      Options = MatrixType::Options,
      MaxRowsAtCompileTime = MatrixType::MaxRowsAtCompileTime,
      MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime
    };
    typedef typename MatrixType::Scalar Scalar;
    typedef typename MatrixType::RealScalar RealScalar;
    typedef typename MatrixType::Index Index;
    typedef Matrix<Scalar, RowsAtCompileTime, RowsAtCompileTime, Options, MaxRowsAtCompileTime, MaxRowsAtCompileTime> MatrixQType;
    typedef typename ei_plain_diag_type<MatrixType>::type HCoeffsType;
    typedef Matrix<Index, 1, ColsAtCompileTime, RowMajor, 1, MaxColsAtCompileTime> IntRowVectorType;
    typedef PermutationMatrix<ColsAtCompileTime, MaxColsAtCompileTime> PermutationType;
    typedef typename ei_plain_col_type<MatrixType, Index>::type IntColVectorType;
    typedef typename ei_plain_row_type<MatrixType>::type RowVectorType;
    typedef typename ei_plain_col_type<MatrixType>::type ColVectorType;

    /** \brief Default Constructor.
      *
      * The default constructor is useful in cases in which the user intends to
      * perform decompositions via FullPivHouseholderQR::compute(const MatrixType&).
      */
    FullPivHouseholderQR()
      : m_qr(),
        m_hCoeffs(),
        m_rows_transpositions(),
        m_cols_transpositions(),
        m_cols_permutation(),
        m_temp(),
        m_isInitialized(false) {}

    /** \brief Default Constructor with memory preallocation
      *
      * Like the default constructor but with preallocation of the internal data
      * according to the specified problem \a size.
      * \sa FullPivHouseholderQR()
      */
    FullPivHouseholderQR(Index rows, Index cols)
      : m_qr(rows, cols),
        m_hCoeffs(std::min(rows,cols)),
        m_rows_transpositions(rows),
        m_cols_transpositions(cols),
        m_cols_permutation(cols),
        m_temp(std::min(rows,cols)),
        m_isInitialized(false) {}

    FullPivHouseholderQR(const MatrixType& matrix)
      : m_qr(matrix.rows(), matrix.cols()),
        m_hCoeffs(std::min(matrix.rows(), matrix.cols())),
        m_rows_transpositions(matrix.rows()),
        m_cols_transpositions(matrix.cols()),
        m_cols_permutation(matrix.cols()),
        m_temp(std::min(matrix.rows(), matrix.cols())),
        m_isInitialized(false)
    {
      compute(matrix);
    }

    /** This method finds a solution x to the equation Ax=b, where A is the matrix of which
      * *this is the QR decomposition, if any exists.
      *
      * \param b the right-hand-side of the equation to solve.
      *
      * \returns a solution.
      *
      * \note The case where b is a matrix is not yet implemented. Also, this
      *       code is space inefficient.
      *
      * \note_about_checking_solutions
      *
      * \note_about_arbitrary_choice_of_solution
      *
      * Example: \include FullPivHouseholderQR_solve.cpp
      * Output: \verbinclude FullPivHouseholderQR_solve.out
      */
    template<typename Rhs>
    inline const ei_solve_retval<FullPivHouseholderQR, Rhs>
    solve(const MatrixBase<Rhs>& b) const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return ei_solve_retval<FullPivHouseholderQR, Rhs>(*this, b.derived());
    }

    MatrixQType matrixQ(void) const;

    /** \returns a reference to the matrix where the Householder QR decomposition is stored
      */
    const MatrixType& matrixQR() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return m_qr;
    }

    FullPivHouseholderQR& compute(const MatrixType& matrix);

    const PermutationType& colsPermutation() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return m_cols_permutation;
    }

    const IntColVectorType& rowsTranspositions() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return m_rows_transpositions;
    }

    /** \returns the absolute value of the determinant of the matrix of which
      * *this is the QR decomposition. It has only linear complexity
      * (that is, O(n) where n is the dimension of the square matrix)
      * as the QR decomposition has already been computed.
      *
      * \note This is only for square matrices.
      *
      * \warning a determinant can be very big or small, so for matrices
      * of large enough dimension, there is a risk of overflow/underflow.
      * One way to work around that is to use logAbsDeterminant() instead.
      *
      * \sa logAbsDeterminant(), MatrixBase::determinant()
      */
    typename MatrixType::RealScalar absDeterminant() const;

    /** \returns the natural log of the absolute value of the determinant of the matrix of which
      * *this is the QR decomposition. It has only linear complexity
      * (that is, O(n) where n is the dimension of the square matrix)
      * as the QR decomposition has already been computed.
      *
      * \note This is only for square matrices.
      *
      * \note This method is useful to work around the risk of overflow/underflow that's inherent
      * to determinant computation.
      *
      * \sa absDeterminant(), MatrixBase::determinant()
      */
    typename MatrixType::RealScalar logAbsDeterminant() const;

    /** \returns the rank of the matrix of which *this is the QR decomposition.
      *
      * \note This is computed at the time of the construction of the QR decomposition. This
      *       method does not perform any further computation.
      */
    inline Index rank() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return m_rank;
    }

    /** \returns the dimension of the kernel of the matrix of which *this is the QR decomposition.
      *
      * \note Since the rank is computed at the time of the construction of the QR decomposition, this
      *       method almost does not perform any further computation.
      */
    inline Index dimensionOfKernel() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return m_qr.cols() - m_rank;
    }

    /** \returns true if the matrix of which *this is the QR decomposition represents an injective
      *          linear map, i.e. has trivial kernel; false otherwise.
      *
      * \note Since the rank is computed at the time of the construction of the QR decomposition, this
      *       method almost does not perform any further computation.
      */
    inline bool isInjective() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return m_rank == m_qr.cols();
    }

    /** \returns true if the matrix of which *this is the QR decomposition represents a surjective
      *          linear map; false otherwise.
      *
      * \note Since the rank is computed at the time of the construction of the QR decomposition, this
      *       method almost does not perform any further computation.
      */
    inline bool isSurjective() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return m_rank == m_qr.rows();
    }

    /** \returns true if the matrix of which *this is the QR decomposition is invertible.
      *
      * \note Since the rank is computed at the time of the construction of the QR decomposition, this
      *       method almost does not perform any further computation.
      */
    inline bool isInvertible() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return isInjective() && isSurjective();
    }

    /** \returns the inverse of the matrix of which *this is the QR decomposition.
      *
      * \note If this matrix is not invertible, the returned matrix has undefined coefficients.
      *       Use isInvertible() to first determine whether this matrix is invertible.
      */    inline const
    ei_solve_retval<FullPivHouseholderQR, typename MatrixType::IdentityReturnType>
    inverse() const
    {
      ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
      return ei_solve_retval<FullPivHouseholderQR,typename MatrixType::IdentityReturnType>
               (*this, MatrixType::Identity(m_qr.rows(), m_qr.cols()));
    }

    inline Index rows() const { return m_qr.rows(); }
    inline Index cols() const { return m_qr.cols(); }
    const HCoeffsType& hCoeffs() const { return m_hCoeffs; }

  protected:
    MatrixType m_qr;
    HCoeffsType m_hCoeffs;
    IntColVectorType m_rows_transpositions;
    IntRowVectorType m_cols_transpositions;
    PermutationType m_cols_permutation;
    RowVectorType m_temp;
    bool m_isInitialized;
    RealScalar m_precision;
    Index m_rank;
    Index m_det_pq;
};

template<typename MatrixType>
typename MatrixType::RealScalar FullPivHouseholderQR<MatrixType>::absDeterminant() const
{
  ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
  ei_assert(m_qr.rows() == m_qr.cols() && "You can't take the determinant of a non-square matrix!");
  return ei_abs(m_qr.diagonal().prod());
}

template<typename MatrixType>
typename MatrixType::RealScalar FullPivHouseholderQR<MatrixType>::logAbsDeterminant() const
{
  ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
  ei_assert(m_qr.rows() == m_qr.cols() && "You can't take the determinant of a non-square matrix!");
  return m_qr.diagonal().cwiseAbs().array().log().sum();
}

template<typename MatrixType>
FullPivHouseholderQR<MatrixType>& FullPivHouseholderQR<MatrixType>::compute(const MatrixType& matrix)
{
  Index rows = matrix.rows();
  Index cols = matrix.cols();
  Index size = std::min(rows,cols);
  m_rank = size;

  m_qr = matrix;
  m_hCoeffs.resize(size);

  m_temp.resize(cols);

  m_precision = NumTraits<Scalar>::epsilon() * size;

  m_rows_transpositions.resize(matrix.rows());
  m_cols_transpositions.resize(matrix.cols());
  Index number_of_transpositions = 0;

  RealScalar biggest(0);

  for (Index k = 0; k < size; ++k)
  {
    Index row_of_biggest_in_corner, col_of_biggest_in_corner;
    RealScalar biggest_in_corner;

    biggest_in_corner = m_qr.bottomRightCorner(rows-k, cols-k)
                            .cwiseAbs()
                            .maxCoeff(&row_of_biggest_in_corner, &col_of_biggest_in_corner);
    row_of_biggest_in_corner += k;
    col_of_biggest_in_corner += k;
    if(k==0) biggest = biggest_in_corner;

    // if the corner is negligible, then we have less than full rank, and we can finish early
    if(ei_isMuchSmallerThan(biggest_in_corner, biggest, m_precision))
    {
      m_rank = k;
      for(Index i = k; i < size; i++)
      {
        m_rows_transpositions.coeffRef(i) = i;
        m_cols_transpositions.coeffRef(i) = i;
        m_hCoeffs.coeffRef(i) = Scalar(0);
      }
      break;
    }

    m_rows_transpositions.coeffRef(k) = row_of_biggest_in_corner;
    m_cols_transpositions.coeffRef(k) = col_of_biggest_in_corner;
    if(k != row_of_biggest_in_corner) {
      m_qr.row(k).tail(cols-k).swap(m_qr.row(row_of_biggest_in_corner).tail(cols-k));
      ++number_of_transpositions;
    }
    if(k != col_of_biggest_in_corner) {
      m_qr.col(k).swap(m_qr.col(col_of_biggest_in_corner));
      ++number_of_transpositions;
    }

    RealScalar beta;
    m_qr.col(k).tail(rows-k).makeHouseholderInPlace(m_hCoeffs.coeffRef(k), beta);
    m_qr.coeffRef(k,k) = beta;

    m_qr.bottomRightCorner(rows-k, cols-k-1)
        .applyHouseholderOnTheLeft(m_qr.col(k).tail(rows-k-1), m_hCoeffs.coeffRef(k), &m_temp.coeffRef(k+1));
  }

  m_cols_permutation.setIdentity(cols);
  for(Index k = 0; k < size; ++k)
    m_cols_permutation.applyTranspositionOnTheRight(k, m_cols_transpositions.coeff(k));

  m_det_pq = (number_of_transpositions%2) ? -1 : 1;
  m_isInitialized = true;

  return *this;
}

template<typename _MatrixType, typename Rhs>
struct ei_solve_retval<FullPivHouseholderQR<_MatrixType>, Rhs>
  : ei_solve_retval_base<FullPivHouseholderQR<_MatrixType>, Rhs>
{
  EIGEN_MAKE_SOLVE_HELPERS(FullPivHouseholderQR<_MatrixType>,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    const Index rows = dec().rows(), cols = dec().cols();
    ei_assert(rhs().rows() == rows);

    // FIXME introduce nonzeroPivots() and use it here. and more generally,
    // make the same improvements in this dec as in FullPivLU.
    if(dec().rank()==0)
    {
      dst.setZero();
      return;
    }

    typename Rhs::PlainObject c(rhs());

    Matrix<Scalar,1,Rhs::ColsAtCompileTime> temp(rhs().cols());
    for (Index k = 0; k < dec().rank(); ++k)
    {
      Index remainingSize = rows-k;
      c.row(k).swap(c.row(dec().rowsTranspositions().coeff(k)));
      c.bottomRightCorner(remainingSize, rhs().cols())
       .applyHouseholderOnTheLeft(dec().matrixQR().col(k).tail(remainingSize-1),
                                  dec().hCoeffs().coeff(k), &temp.coeffRef(0));
    }

    if(!dec().isSurjective())
    {
      // is c is in the image of R ?
      RealScalar biggest_in_upper_part_of_c = c.topRows(   dec().rank()     ).cwiseAbs().maxCoeff();
      RealScalar biggest_in_lower_part_of_c = c.bottomRows(rows-dec().rank()).cwiseAbs().maxCoeff();
      // FIXME brain dead
      const RealScalar m_precision = NumTraits<Scalar>::epsilon() * std::min(rows,cols);
      if(!ei_isMuchSmallerThan(biggest_in_lower_part_of_c, biggest_in_upper_part_of_c, m_precision))
        return;
    }
    dec().matrixQR()
       .topLeftCorner(dec().rank(), dec().rank())
       .template triangularView<Upper>()
       .solveInPlace(c.topRows(dec().rank()));

    for(Index i = 0; i < dec().rank(); ++i) dst.row(dec().colsPermutation().indices().coeff(i)) = c.row(i);
    for(Index i = dec().rank(); i < cols; ++i) dst.row(dec().colsPermutation().indices().coeff(i)).setZero();
  }
};

/** \returns the matrix Q */
template<typename MatrixType>
typename FullPivHouseholderQR<MatrixType>::MatrixQType FullPivHouseholderQR<MatrixType>::matrixQ() const
{
  ei_assert(m_isInitialized && "FullPivHouseholderQR is not initialized.");
  // compute the product H'_0 H'_1 ... H'_n-1,
  // where H_k is the k-th Householder transformation I - h_k v_k v_k'
  // and v_k is the k-th Householder vector [1,m_qr(k+1,k), m_qr(k+2,k), ...]
  Index rows = m_qr.rows();
  Index cols = m_qr.cols();
  Index size = std::min(rows,cols);
  MatrixQType res = MatrixQType::Identity(rows, rows);
  Matrix<Scalar,1,MatrixType::RowsAtCompileTime> temp(rows);
  for (Index k = size-1; k >= 0; k--)
  {
    res.block(k, k, rows-k, rows-k)
       .applyHouseholderOnTheLeft(m_qr.col(k).tail(rows-k-1), ei_conj(m_hCoeffs.coeff(k)), &temp.coeffRef(k));
    res.row(k).swap(res.row(m_rows_transpositions.coeff(k)));
  }
  return res;
}

/** \return the full-pivoting Householder QR decomposition of \c *this.
  *
  * \sa class FullPivHouseholderQR
  */
template<typename Derived>
const FullPivHouseholderQR<typename MatrixBase<Derived>::PlainObject>
MatrixBase<Derived>::fullPivHouseholderQr() const
{
  return FullPivHouseholderQR<PlainObject>(eval());
}

#endif // EIGEN_FULLPIVOTINGHOUSEHOLDERQR_H
