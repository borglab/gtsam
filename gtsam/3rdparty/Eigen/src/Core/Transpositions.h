// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2010 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_TRANSPOSITIONS_H
#define EIGEN_TRANSPOSITIONS_H

/** \class Transpositions
  * \ingroup Core_Module
  *
  * \brief Represents a sequence of transpositions (row/column interchange)
  *
  * \param SizeAtCompileTime the number of transpositions, or Dynamic
  * \param MaxSizeAtCompileTime the maximum number of transpositions, or Dynamic. This optional parameter defaults to SizeAtCompileTime. Most of the time, you should not have to specify it.
  *
  * This class represents a permutation transformation as a sequence of \em n transpositions
  * \f$[T_{n-1} \ldots T_{i} \ldots T_{0}]\f$. It is internally stored as a vector of integers \c indices.
  * Each transposition \f$ T_{i} \f$ applied on the left of a matrix (\f$ T_{i} M\f$) interchanges
  * the rows \c i and \c indices[i] of the matrix \c M.
  * A transposition applied on the right (e.g., \f$ M T_{i}\f$) yields a column interchange.
  *
  * Compared to the class PermutationMatrix, such a sequence of transpositions is what is
  * computed during a decomposition with pivoting, and it is faster when applying the permutation in-place.
  * 
  * To apply a sequence of transpositions to a matrix, simply use the operator * as in the following example:
  * \code
  * Transpositions tr;
  * MatrixXf mat;
  * mat = tr * mat;
  * \endcode
  * In this example, we detect that the matrix appears on both side, and so the transpositions
  * are applied in-place without any temporary or extra copy.
  *
  * \sa class PermutationMatrix
  */
template<typename TranspositionType, typename MatrixType, int Side, bool Transposed=false> struct ei_transposition_matrix_product_retval;

template<int SizeAtCompileTime, int MaxSizeAtCompileTime>
class Transpositions
{
  public:

    typedef Matrix<DenseIndex, SizeAtCompileTime, 1, 0, MaxSizeAtCompileTime, 1> IndicesType;
    typedef typename IndicesType::Index Index;

    inline Transpositions() {}

    /** Copy constructor. */
    template<int OtherSize, int OtherMaxSize>
    inline Transpositions(const Transpositions<OtherSize, OtherMaxSize>& other)
      : m_indices(other.indices()) {}

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    /** Standard copy constructor. Defined only to prevent a default copy constructor
      * from hiding the other templated constructor */
    inline Transpositions(const Transpositions& other) : m_indices(other.indices()) {}
    #endif

    /** Generic constructor from expression of the transposition indices. */
    template<typename Other>
    explicit inline Transpositions(const MatrixBase<Other>& indices) : m_indices(indices)
    {}

    /** Copies the \a other transpositions into \c *this */
    template<int OtherSize, int OtherMaxSize>
    Transpositions& operator=(const Transpositions<OtherSize, OtherMaxSize>& other)
    {
      m_indices = other.indices();
      return *this;
    }

    #ifndef EIGEN_PARSED_BY_DOXYGEN
    /** This is a special case of the templated operator=. Its purpose is to
      * prevent a default operator= from hiding the templated operator=.
      */
    Transpositions& operator=(const Transpositions& other)
    {
      m_indices = other.m_indices;
      return *this;
    }
    #endif

    /** Constructs an uninitialized permutation matrix of given size.
      */
    inline Transpositions(Index size) : m_indices(size)
    {}

    /** \returns the number of transpositions */
    inline Index size() const { return m_indices.size(); }

    /** Direct access to the underlying index vector */
    inline const Index& coeff(Index i) const { return m_indices.coeff(i); }
    /** Direct access to the underlying index vector */
    inline Index& coeffRef(Index i) { return m_indices.coeffRef(i); }
    /** Direct access to the underlying index vector */
    inline const Index& operator()(Index i) const { return m_indices(i); }
    /** Direct access to the underlying index vector */
    inline Index& operator()(Index i) { return m_indices(i); }
    /** Direct access to the underlying index vector */
    inline const Index& operator[](Index i) const { return m_indices(i); }
    /** Direct access to the underlying index vector */
    inline Index& operator[](Index i) { return m_indices(i); }

    /** const version of indices(). */
    const IndicesType& indices() const { return m_indices; }
    /** \returns a reference to the stored array representing the transpositions. */
    IndicesType& indices() { return m_indices; }

    /** Resizes to given size. */
    inline void resize(int size)
    {
      m_indices.resize(size);
    }

    /** Sets \c *this to represents an identity transformation */
    void setIdentity()
    {
      for(int i = 0; i < m_indices.size(); ++i)
        m_indices.coeffRef(i) = i;
    }

    // FIXME: do we want such methods ?
    // might be usefull when the target matrix expression is complex, e.g.:
    // object.matrix().block(..,..,..,..) = trans * object.matrix().block(..,..,..,..);
    /*
    template<typename MatrixType>
    void applyForwardToRows(MatrixType& mat) const
    {
      for(Index k=0 ; k<size() ; ++k)
        if(m_indices(k)!=k)
          mat.row(k).swap(mat.row(m_indices(k)));
    }

    template<typename MatrixType>
    void applyBackwardToRows(MatrixType& mat) const
    {
      for(Index k=size()-1 ; k>=0 ; --k)
        if(m_indices(k)!=k)
          mat.row(k).swap(mat.row(m_indices(k)));
    }
    */

    /** \returns the inverse transformation */
    inline Transpose<Transpositions> inverse() const
    { return *this; }

    /** \returns the tranpose transformation */
    inline Transpose<Transpositions> transpose() const
    { return *this; }

#ifndef EIGEN_PARSED_BY_DOXYGEN
    template<int OtherSize, int OtherMaxSize>
    Transpositions(const Transpose<Transpositions<OtherSize,OtherMaxSize> >& other)
      : m_indices(other.size())
    {
      Index n = size();
      Index j = size-1;
      for(Index i=0; i<n;++i,--j)
        m_indices.coeffRef(j) = other.nestedTranspositions().indices().coeff(i);
    }
#endif

  protected:

    IndicesType m_indices;
};

/** \returns the \a matrix with the \a transpositions applied to the columns.
  */
template<typename Derived, int SizeAtCompileTime, int MaxSizeAtCompileTime>
inline const ei_transposition_matrix_product_retval<Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheRight>
operator*(const MatrixBase<Derived>& matrix,
          const Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime> &transpositions)
{
  return ei_transposition_matrix_product_retval
           <Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheRight>
           (transpositions, matrix.derived());
}

/** \returns the \a matrix with the \a transpositions applied to the rows.
  */
template<typename Derived, int SizeAtCompileTime, int MaxSizeAtCompileTime>
inline const ei_transposition_matrix_product_retval
               <Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheLeft>
operator*(const Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime> &transpositions,
          const MatrixBase<Derived>& matrix)
{
  return ei_transposition_matrix_product_retval
           <Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime>, Derived, OnTheLeft>
           (transpositions, matrix.derived());
}

template<typename TranspositionType, typename MatrixType, int Side, bool Transposed>
struct ei_traits<ei_transposition_matrix_product_retval<TranspositionType, MatrixType, Side, Transposed> >
{
  typedef typename MatrixType::PlainObject ReturnType;
};

template<typename TranspositionType, typename MatrixType, int Side, bool Transposed>
struct ei_transposition_matrix_product_retval
 : public ReturnByValue<ei_transposition_matrix_product_retval<TranspositionType, MatrixType, Side, Transposed> >
{
    typedef typename ei_cleantype<typename MatrixType::Nested>::type MatrixTypeNestedCleaned;
    typedef typename TranspositionType::Index Index;

    ei_transposition_matrix_product_retval(const TranspositionType& tr, const MatrixType& matrix)
      : m_transpositions(tr), m_matrix(matrix)
    {}

    inline int rows() const { return m_matrix.rows(); }
    inline int cols() const { return m_matrix.cols(); }

    template<typename Dest> inline void evalTo(Dest& dst) const
    {
      const int size = m_transpositions.size();
      Index j = 0;

      if(!(ei_is_same_type<MatrixTypeNestedCleaned,Dest>::ret && ei_extract_data(dst) == ei_extract_data(m_matrix)))
        dst = m_matrix;

      for(int k=(Transposed?size-1:0) ; Transposed?k>=0:k<size ; Transposed?--k:++k)
        if((j=m_transpositions.coeff(k))!=k)
        {
          if(Side==OnTheLeft)
            dst.row(k).swap(dst.row(j));
          else if(Side==OnTheRight)
            dst.col(k).swap(dst.col(j));
        }
    }

  protected:
    const TranspositionType& m_transpositions;
    const typename MatrixType::Nested m_matrix;
};

/* Template partial specialization for transposed/inverse transpositions */

template<int SizeAtCompileTime, int MaxSizeAtCompileTime>
class Transpose<Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime> >
{
    typedef Transpositions<SizeAtCompileTime, MaxSizeAtCompileTime> TranspositionType;
    typedef typename TranspositionType::IndicesType IndicesType;
  public:

    Transpose(const TranspositionType& t) : m_transpositions(t) {}

    inline int size() const { return m_transpositions.size(); }

    /** \returns the \a matrix with the inverse transpositions applied to the columns.
      */
    template<typename Derived> friend
    inline const ei_transposition_matrix_product_retval<TranspositionType, Derived, OnTheRight, true>
    operator*(const MatrixBase<Derived>& matrix, const Transpose& trt)
    {
      return ei_transposition_matrix_product_retval<TranspositionType, Derived, OnTheRight, true>(trt.m_transpositions, matrix.derived());
    }

    /** \returns the \a matrix with the inverse transpositions applied to the rows.
      */
    template<typename Derived>
    inline const ei_transposition_matrix_product_retval<TranspositionType, Derived, OnTheLeft, true>
    operator*(const MatrixBase<Derived>& matrix) const
    {
      return ei_transposition_matrix_product_retval<TranspositionType, Derived, OnTheLeft, true>(m_transpositions, matrix.derived());
    }

    const TranspositionType& nestedTranspositions() const { return m_transpositions; }

  protected:
    const TranspositionType& m_transpositions;
};

#endif // EIGEN_TRANSPOSITIONS_H
