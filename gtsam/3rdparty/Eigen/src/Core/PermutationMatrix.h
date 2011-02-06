// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_PERMUTATIONMATRIX_H
#define EIGEN_PERMUTATIONMATRIX_H

/** \class PermutationMatrix
  * \ingroup Core_Module
  *
  * \brief Permutation matrix
  *
  * \param SizeAtCompileTime the number of rows/cols, or Dynamic
  * \param MaxSizeAtCompileTime the maximum number of rows/cols, or Dynamic. This optional parameter defaults to SizeAtCompileTime. Most of the time, you should not have to specify it.
  *
  * This class represents a permutation matrix, internally stored as a vector of integers.
  * The convention followed here is that if \f$ \sigma \f$ is a permutation, the corresponding permutation matrix
  * \f$ P_\sigma \f$ is such that if \f$ (e_1,\ldots,e_p) \f$ is the canonical basis, we have:
  *  \f[ P_\sigma(e_i) = e_{\sigma(i)}. \f]
  * This convention ensures that for any two permutations \f$ \sigma, \tau \f$, we have:
  *  \f[ P_{\sigma\circ\tau} = P_\sigma P_\tau. \f]
  *
  * Permutation matrices are square and invertible.
  *
  * Notice that in addition to the member functions and operators listed here, there also are non-member
  * operator* to multiply a PermutationMatrix with any kind of matrix expression (MatrixBase) on either side.
  *
  * \sa class DiagonalMatrix
  */
template<typename PermutationType, typename MatrixType, int Side, bool Transposed=false> struct ei_permut_matrix_product_retval;

template<int SizeAtCompileTime, int MaxSizeAtCompileTime>
struct ei_traits<PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> >
 : ei_traits<Matrix<int,SizeAtCompileTime,SizeAtCompileTime,0,MaxSizeAtCompileTime,MaxSizeAtCompileTime> >
{};

template<int SizeAtCompileTime, int MaxSizeAtCompileTime>
class PermutationMatrix : public EigenBase<PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> >
{
  public:

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    typedef ei_traits<PermutationMatrix> Traits;
    typedef Matrix<int,SizeAtCompileTime,SizeAtCompileTime,0,MaxSizeAtCompileTime,MaxSizeAtCompileTime>
            DenseMatrixType;
    enum {
      Flags = Traits::Flags,
      CoeffReadCost = Traits::CoeffReadCost,
      RowsAtCompileTime = Traits::RowsAtCompileTime,
      ColsAtCompileTime = Traits::ColsAtCompileTime,
      MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
      MaxColsAtCompileTime = Traits::MaxColsAtCompileTime
    };
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Index Index;
    #endif

    typedef Matrix<int, SizeAtCompileTime, 1, 0, MaxSizeAtCompileTime, 1> IndicesType;

    inline PermutationMatrix()
    {}

    /** Constructs an uninitialized permutation matrix of given size.
      */
    inline PermutationMatrix(int size) : m_indices(size)
    {}

    /** Copy constructor. */
    template<int OtherSize, int OtherMaxSize>
    inline PermutationMatrix(const PermutationMatrix<OtherSize, OtherMaxSize>& other)
      : m_indices(other.indices()) {}

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    /** Standard copy constructor. Defined only to prevent a default copy constructor
      * from hiding the other templated constructor */
    inline PermutationMatrix(const PermutationMatrix& other) : m_indices(other.indices()) {}
    #endif

    /** Generic constructor from expression of the indices. The indices
      * array has the meaning that the permutations sends each integer i to indices[i].
      *
      * \warning It is your responsibility to check that the indices array that you passes actually
      * describes a permutation, i.e., each value between 0 and n-1 occurs exactly once, where n is the
      * array's size.
      */
    template<typename Other>
    explicit inline PermutationMatrix(const MatrixBase<Other>& indices) : m_indices(indices)
    {}

    /** Convert the Transpositions \a tr to a permutation matrix */
    template<int OtherSize, int OtherMaxSize>
    explicit PermutationMatrix(const Transpositions<OtherSize,OtherMaxSize>& tr)
      : m_indices(tr.size())
    {
      *this = tr;
    }

    /** Copies the other permutation into *this */
    template<int OtherSize, int OtherMaxSize>
    PermutationMatrix& operator=(const PermutationMatrix<OtherSize, OtherMaxSize>& other)
    {
      m_indices = other.indices();
      return *this;
    }

    /** Assignment from the Transpositions \a tr */
    template<int OtherSize, int OtherMaxSize>
    PermutationMatrix& operator=(const Transpositions<OtherSize,OtherMaxSize>& tr)
    {
      setIdentity(tr.size());
      for(Index k=size()-1; k>=0; --k)
        applyTranspositionOnTheRight(k,tr.coeff(k));
      return *this;
    }

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    /** This is a special case of the templated operator=. Its purpose is to
      * prevent a default operator= from hiding the templated operator=.
      */
    PermutationMatrix& operator=(const PermutationMatrix& other)
    {
      m_indices = other.m_indices;
      return *this;
    }
    #endif

    /** \returns the number of rows */
    inline Index rows() const { return m_indices.size(); }

    /** \returns the number of columns */
    inline Index cols() const { return m_indices.size(); }

    /** \returns the size of a side of the respective square matrix, i.e., the number of indices */
    inline Index size() const { return m_indices.size(); }

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    template<typename DenseDerived>
    void evalTo(MatrixBase<DenseDerived>& other) const
    {
      other.setZero();
      for (int i=0; i<rows();++i)
        other.coeffRef(m_indices.coeff(i),i) = typename DenseDerived::Scalar(1);
    }
    #endif

    /** \returns a Matrix object initialized from this permutation matrix. Notice that it
      * is inefficient to return this Matrix object by value. For efficiency, favor using
      * the Matrix constructor taking EigenBase objects.
      */
    DenseMatrixType toDenseMatrix() const
    {
      return *this;
    }

    /** const version of indices(). */
    const IndicesType& indices() const { return m_indices; }
    /** \returns a reference to the stored array representing the permutation. */
    IndicesType& indices() { return m_indices; }

    /** Resizes to given size.
      */
    inline void resize(Index size)
    {
      m_indices.resize(size);
    }

    /** Sets *this to be the identity permutation matrix */
    void setIdentity()
    {
      for(Index i = 0; i < m_indices.size(); ++i)
        m_indices.coeffRef(i) = i;
    }

    /** Sets *this to be the identity permutation matrix of given size.
      */
    void setIdentity(Index size)
    {
      resize(size);
      setIdentity();
    }

    /** Multiplies *this by the transposition \f$(ij)\f$ on the left.
      *
      * \returns a reference to *this.
      *
      * \warning This is much slower than applyTranspositionOnTheRight(int,int):
      * this has linear complexity and requires a lot of branching.
      *
      * \sa applyTranspositionOnTheRight(int,int)
      */
    PermutationMatrix& applyTranspositionOnTheLeft(Index i, Index j)
    {
      ei_assert(i>=0 && j>=0 && i<m_indices.size() && j<m_indices.size());
      for(Index k = 0; k < m_indices.size(); ++k)
      {
        if(m_indices.coeff(k) == i) m_indices.coeffRef(k) = j;
        else if(m_indices.coeff(k) == j) m_indices.coeffRef(k) = i;
      }
      return *this;
    }

    /** Multiplies *this by the transposition \f$(ij)\f$ on the right.
      *
      * \returns a reference to *this.
      *
      * This is a fast operation, it only consists in swapping two indices.
      *
      * \sa applyTranspositionOnTheLeft(int,int)
      */
    PermutationMatrix& applyTranspositionOnTheRight(Index i, Index j)
    {
      ei_assert(i>=0 && j>=0 && i<m_indices.size() && j<m_indices.size());
      std::swap(m_indices.coeffRef(i), m_indices.coeffRef(j));
      return *this;
    }

    /** \returns the inverse permutation matrix.
      *
      * \note \note_try_to_help_rvo
      */
    inline Transpose<PermutationMatrix> inverse() const
    { return *this; }
    /** \returns the tranpose permutation matrix.
      *
      * \note \note_try_to_help_rvo
      */
    inline Transpose<PermutationMatrix> transpose() const
    { return *this; }

    /**** multiplication helpers to hopefully get RVO ****/

#ifndef EIGEN_PARSED_BY_DOXYGEN
    template<int OtherSize, int OtherMaxSize>
    PermutationMatrix(const Transpose<PermutationMatrix<OtherSize,OtherMaxSize> >& other)
      : m_indices(other.nestedPermutation().size())
    {
      for (int i=0; i<rows();++i) m_indices.coeffRef(other.nestedPermutation().indices().coeff(i)) = i;
    }
  protected:
    enum Product_t {Product};
    PermutationMatrix(Product_t, const PermutationMatrix& lhs, const PermutationMatrix& rhs)
      : m_indices(lhs.m_indices.size())
    {
      ei_assert(lhs.cols() == rhs.rows());
      for (int i=0; i<rows();++i) m_indices.coeffRef(i) = lhs.m_indices.coeff(rhs.m_indices.coeff(i));
    }
#endif

  public:

    /** \returns the product permutation matrix.
      *
      * \note \note_try_to_help_rvo
      */
    template<int OtherSize, int OtherMaxSize>
    inline PermutationMatrix operator*(const PermutationMatrix<OtherSize, OtherMaxSize>& other) const
    { return PermutationMatrix(Product, *this, other); }

    /** \returns the product of a permutation with another inverse permutation.
      *
      * \note \note_try_to_help_rvo
      */
    template<int OtherSize, int OtherMaxSize>
    inline PermutationMatrix operator*(const Transpose<PermutationMatrix<OtherSize,OtherMaxSize> >& other) const
    { return PermutationMatrix(Product, *this, other.eval()); }

    /** \returns the product of an inverse permutation with another permutation.
      *
      * \note \note_try_to_help_rvo
      */
    template<int OtherSize, int OtherMaxSize> friend
    inline PermutationMatrix operator*(const Transpose<PermutationMatrix<OtherSize,OtherMaxSize> >& other, const PermutationMatrix& perm)
    { return PermutationMatrix(Product, other.eval(), perm); }

  protected:

    IndicesType m_indices;
};

/** \returns the matrix with the permutation applied to the columns.
  */
template<typename Derived, int SizeAtCompileTime, int MaxSizeAtCompileTime>
inline const ei_permut_matrix_product_retval<PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheRight>
operator*(const MatrixBase<Derived>& matrix,
          const PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> &permutation)
{
  return ei_permut_matrix_product_retval
           <PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheRight>
           (permutation, matrix.derived());
}

/** \returns the matrix with the permutation applied to the rows.
  */
template<typename Derived, int SizeAtCompileTime, int MaxSizeAtCompileTime>
inline const ei_permut_matrix_product_retval
               <PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheLeft>
operator*(const PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> &permutation,
          const MatrixBase<Derived>& matrix)
{
  return ei_permut_matrix_product_retval
           <PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheLeft>
           (permutation, matrix.derived());
}

template<typename PermutationType, typename MatrixType, int Side, bool Transposed>
struct ei_traits<ei_permut_matrix_product_retval<PermutationType, MatrixType, Side, Transposed> >
{
  typedef typename MatrixType::PlainObject ReturnType;
};

template<typename PermutationType, typename MatrixType, int Side, bool Transposed>
struct ei_permut_matrix_product_retval
 : public ReturnByValue<ei_permut_matrix_product_retval<PermutationType, MatrixType, Side, Transposed> >
{
    typedef typename ei_cleantype<typename MatrixType::Nested>::type MatrixTypeNestedCleaned;

    ei_permut_matrix_product_retval(const PermutationType& perm, const MatrixType& matrix)
      : m_permutation(perm), m_matrix(matrix)
    {}

    inline int rows() const { return m_matrix.rows(); }
    inline int cols() const { return m_matrix.cols(); }

    template<typename Dest> inline void evalTo(Dest& dst) const
    {
      const int n = Side==OnTheLeft ? rows() : cols();

      if(ei_is_same_type<MatrixTypeNestedCleaned,Dest>::ret && ei_extract_data(dst) == ei_extract_data(m_matrix))
      {
        // apply the permutation inplace
        Matrix<bool,PermutationType::RowsAtCompileTime,1,0,PermutationType::MaxRowsAtCompileTime> mask(m_permutation.size());
        mask.fill(false);
        int r = 0;
        while(r < m_permutation.size())
        {
          // search for the next seed
          while(r<m_permutation.size() && mask[r]) r++;
          if(r>=m_permutation.size())
            break;
          // we got one, let's follow it until we are back to the seed
          int k0 = r++;
          int kPrev = k0;
          mask.coeffRef(k0) = true;
          for(int k=m_permutation.indices().coeff(k0); k!=k0; k=m_permutation.indices().coeff(k))
          {
                  Block<Dest, Side==OnTheLeft ? 1 : Dest::RowsAtCompileTime, Side==OnTheRight ? 1 : Dest::ColsAtCompileTime>(dst, k)
            .swap(Block<Dest, Side==OnTheLeft ? 1 : Dest::RowsAtCompileTime, Side==OnTheRight ? 1 : Dest::ColsAtCompileTime>
                       (dst,((Side==OnTheLeft) ^ Transposed) ? k0 : kPrev));

            mask.coeffRef(k) = true;
            kPrev = k;
          }
        }
      }
      else
      {
        for(int i = 0; i < n; ++i)
        {
          Block<Dest, Side==OnTheLeft ? 1 : Dest::RowsAtCompileTime, Side==OnTheRight ? 1 : Dest::ColsAtCompileTime>
               (dst, ((Side==OnTheLeft) ^ Transposed) ? m_permutation.indices().coeff(i) : i)

          =

          Block<MatrixTypeNestedCleaned,Side==OnTheLeft ? 1 : MatrixType::RowsAtCompileTime,Side==OnTheRight ? 1 : MatrixType::ColsAtCompileTime>
               (m_matrix, ((Side==OnTheRight) ^ Transposed) ? m_permutation.indices().coeff(i) : i);
        }
      }
    }

  protected:
    const PermutationType& m_permutation;
    const typename MatrixType::Nested m_matrix;
};

/* Template partial specialization for transposed/inverse permutations */

template<int SizeAtCompileTime, int MaxSizeAtCompileTime>
struct ei_traits<Transpose<PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> > >
 : ei_traits<Matrix<int,SizeAtCompileTime,SizeAtCompileTime,0,MaxSizeAtCompileTime,MaxSizeAtCompileTime> >
{};

template<int SizeAtCompileTime, int MaxSizeAtCompileTime>
class Transpose<PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> >
  : public EigenBase<Transpose<PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> > >
{
    typedef PermutationMatrix<SizeAtCompileTime, MaxSizeAtCompileTime> PermutationType;
    typedef typename PermutationType::IndicesType IndicesType;
  public:

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    typedef ei_traits<PermutationType> Traits;
    typedef Matrix<int,SizeAtCompileTime,SizeAtCompileTime,0,MaxSizeAtCompileTime,MaxSizeAtCompileTime>
            DenseMatrixType;
    enum {
      Flags = Traits::Flags,
      CoeffReadCost = Traits::CoeffReadCost,
      RowsAtCompileTime = Traits::RowsAtCompileTime,
      ColsAtCompileTime = Traits::ColsAtCompileTime,
      MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
      MaxColsAtCompileTime = Traits::MaxColsAtCompileTime
    };
    typedef typename Traits::Scalar Scalar;
    #endif

    Transpose(const PermutationType& p) : m_permutation(p) {}

    inline int rows() const { return m_permutation.rows(); }
    inline int cols() const { return m_permutation.cols(); }

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    template<typename DenseDerived>
    void evalTo(MatrixBase<DenseDerived>& other) const
    {
      other.setZero();
      for (int i=0; i<rows();++i)
        other.coeffRef(i, m_permutation.indices().coeff(i)) = typename DenseDerived::Scalar(1);
    }
    #endif

    /** \return the equivalent permutation matrix */
    PermutationType eval() const { return *this; }

    DenseMatrixType toDenseMatrix() const { return *this; }

    /** \returns the matrix with the inverse permutation applied to the columns.
      */
    template<typename Derived> friend
    inline const ei_permut_matrix_product_retval<PermutationType, Derived, OnTheRight, true>
    operator*(const MatrixBase<Derived>& matrix, const Transpose& trPerm)
    {
      return ei_permut_matrix_product_retval<PermutationType, Derived, OnTheRight, true>(trPerm.m_permutation, matrix.derived());
    }

    /** \returns the matrix with the inverse permutation applied to the rows.
      */
    template<typename Derived>
    inline const ei_permut_matrix_product_retval<PermutationType, Derived, OnTheLeft, true>
    operator*(const MatrixBase<Derived>& matrix) const
    {
      return ei_permut_matrix_product_retval<PermutationType, Derived, OnTheLeft, true>(m_permutation, matrix.derived());
    }

    const PermutationType& nestedPermutation() const { return m_permutation; }

  protected:
    const PermutationType& m_permutation;
};

#endif // EIGEN_PERMUTATIONMATRIX_H
