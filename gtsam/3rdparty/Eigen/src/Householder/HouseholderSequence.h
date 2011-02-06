// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2010 Benoit Jacob <jacob.benoit.1@gmail.com>
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

#ifndef EIGEN_HOUSEHOLDER_SEQUENCE_H
#define EIGEN_HOUSEHOLDER_SEQUENCE_H

/** \ingroup Householder_Module
  * \householder_module
  * \class HouseholderSequence
  * \brief Represents a sequence of householder reflections with decreasing size
  *
  * This class represents a product sequence of householder reflections \f$ H = \Pi_0^{n-1} H_i \f$
  * where \f$ H_i \f$ is the i-th householder transformation \f$ I - h_i v_i v_i^* \f$,
  * \f$ v_i \f$ is the i-th householder vector \f$ [ 1, m_vectors(i+1,i), m_vectors(i+2,i), ...] \f$
  * and \f$ h_i \f$ is the i-th householder coefficient \c m_coeffs[i].
  *
  * Typical usages are listed below, where H is a HouseholderSequence:
  * \code
  * A.applyOnTheRight(H);             // A = A * H
  * A.applyOnTheLeft(H);              // A = H * A
  * A.applyOnTheRight(H.adjoint());   // A = A * H^*
  * A.applyOnTheLeft(H.adjoint());    // A = H^* * A
  * MatrixXd Q = H;                   // conversion to a dense matrix
  * \endcode
  * In addition to the adjoint, you can also apply the inverse (=adjoint), the transpose, and the conjugate.
  *
  * \sa MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */

template<typename VectorsType, typename CoeffsType, int Side>
struct ei_traits<HouseholderSequence<VectorsType,CoeffsType,Side> >
{
  typedef typename VectorsType::Scalar Scalar;
  typedef typename VectorsType::Index Index;
  typedef typename VectorsType::StorageKind StorageKind;
  enum {
    RowsAtCompileTime = Side==OnTheLeft ? ei_traits<VectorsType>::RowsAtCompileTime
                                        : ei_traits<VectorsType>::ColsAtCompileTime,
    ColsAtCompileTime = RowsAtCompileTime,
    MaxRowsAtCompileTime = Side==OnTheLeft ? ei_traits<VectorsType>::MaxRowsAtCompileTime
                                           : ei_traits<VectorsType>::MaxColsAtCompileTime,
    MaxColsAtCompileTime = MaxRowsAtCompileTime,
    Flags = 0
  };
};

template<typename VectorsType, typename CoeffsType, int Side>
struct ei_hseq_side_dependent_impl
{
  typedef Block<VectorsType, Dynamic, 1> EssentialVectorType;
  typedef HouseholderSequence<VectorsType, CoeffsType, OnTheLeft> HouseholderSequenceType;
  typedef typename VectorsType::Index Index;
  static inline const EssentialVectorType essentialVector(const HouseholderSequenceType& h, Index k)
  {
    Index start = k+1+h.m_shift;
    return Block<VectorsType,Dynamic,1>(h.m_vectors, start, k, h.rows()-start, 1);
  }
};

template<typename VectorsType, typename CoeffsType>
struct ei_hseq_side_dependent_impl<VectorsType, CoeffsType, OnTheRight>
{
  typedef Transpose<Block<VectorsType, 1, Dynamic> > EssentialVectorType;
  typedef HouseholderSequence<VectorsType, CoeffsType, OnTheRight> HouseholderSequenceType;
  typedef typename VectorsType::Index Index;
  static inline const EssentialVectorType essentialVector(const HouseholderSequenceType& h, Index k)
  {
    Index start = k+1+h.m_shift;
    return Block<VectorsType,1,Dynamic>(h.m_vectors, k, start, 1, h.rows()-start).transpose();
  }
};

template<typename OtherScalarType, typename MatrixType> struct ei_matrix_type_times_scalar_type
{
  typedef typename ei_scalar_product_traits<OtherScalarType, typename MatrixType::Scalar>::ReturnType
    ResultScalar;
  typedef Matrix<ResultScalar, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime,
                 0, MatrixType::MaxRowsAtCompileTime, MatrixType::MaxColsAtCompileTime> Type;
};

template<typename VectorsType, typename CoeffsType, int Side> class HouseholderSequence
  : public EigenBase<HouseholderSequence<VectorsType,CoeffsType,Side> >
{
    enum {
      RowsAtCompileTime = ei_traits<HouseholderSequence>::RowsAtCompileTime,
      ColsAtCompileTime = ei_traits<HouseholderSequence>::ColsAtCompileTime,
      MaxRowsAtCompileTime = ei_traits<HouseholderSequence>::MaxRowsAtCompileTime,
      MaxColsAtCompileTime = ei_traits<HouseholderSequence>::MaxColsAtCompileTime
    };
    typedef typename ei_traits<HouseholderSequence>::Scalar Scalar;
    typedef typename VectorsType::Index Index;

    typedef typename ei_hseq_side_dependent_impl<VectorsType,CoeffsType,Side>::EssentialVectorType
            EssentialVectorType;

  public:

    typedef HouseholderSequence<
      VectorsType,
      typename ei_meta_if<NumTraits<Scalar>::IsComplex,
        typename ei_cleantype<typename CoeffsType::ConjugateReturnType>::type,
        CoeffsType>::ret,
      Side
    > ConjugateReturnType;

    HouseholderSequence(const VectorsType& v, const CoeffsType& h, bool trans = false)
      : m_vectors(v), m_coeffs(h), m_trans(trans), m_actualVectors(v.diagonalSize()),
        m_shift(0)
    {
    }

    HouseholderSequence(const VectorsType& v, const CoeffsType& h, bool trans, Index actualVectors, Index shift)
      : m_vectors(v), m_coeffs(h), m_trans(trans), m_actualVectors(actualVectors), m_shift(shift)
    {
    }

    Index rows() const { return Side==OnTheLeft ? m_vectors.rows() : m_vectors.cols(); }
    Index cols() const { return rows(); }

    const EssentialVectorType essentialVector(Index k) const
    {
      ei_assert(k >= 0 && k < m_actualVectors);
      return ei_hseq_side_dependent_impl<VectorsType,CoeffsType,Side>::essentialVector(*this, k);
    }

    HouseholderSequence transpose() const
    { return HouseholderSequence(m_vectors, m_coeffs, !m_trans, m_actualVectors, m_shift); }

    ConjugateReturnType conjugate() const
    { return ConjugateReturnType(m_vectors, m_coeffs.conjugate(), m_trans, m_actualVectors, m_shift); }

    ConjugateReturnType adjoint() const
    { return ConjugateReturnType(m_vectors, m_coeffs.conjugate(), !m_trans, m_actualVectors, m_shift); }

    ConjugateReturnType inverse() const { return adjoint(); }

    /** \internal */
    template<typename DestType> void evalTo(DestType& dst) const
    {
      Index vecs = m_actualVectors;
      // FIXME find a way to pass this temporary if the user want to
      Matrix<Scalar, DestType::RowsAtCompileTime, 1,
             AutoAlign|ColMajor, DestType::MaxRowsAtCompileTime, 1> temp(rows());
      if(    ei_is_same_type<typename ei_cleantype<VectorsType>::type,DestType>::ret
          && ei_extract_data(dst) == ei_extract_data(m_vectors))
      {
        // in-place
        dst.diagonal().setOnes();
        dst.template triangularView<StrictlyUpper>().setZero();
        for(Index k = vecs-1; k >= 0; --k)
        {
          Index cornerSize = rows() - k - m_shift;
          if(m_trans)
            dst.bottomRightCorner(cornerSize, cornerSize)
            .applyHouseholderOnTheRight(essentialVector(k), m_coeffs.coeff(k), &temp.coeffRef(0));
          else
            dst.bottomRightCorner(cornerSize, cornerSize)
              .applyHouseholderOnTheLeft(essentialVector(k), m_coeffs.coeff(k), &temp.coeffRef(0));

          // clear the off diagonal vector
          dst.col(k).tail(rows()-k-1).setZero();
        }
        // clear the remaining columns if needed
        for(Index k = 0; k<cols()-vecs ; ++k)
          dst.col(k).tail(rows()-k-1).setZero();
      }
      else
      {
        dst.setIdentity(rows(), rows());
        for(Index k = vecs-1; k >= 0; --k)
        {
          Index cornerSize = rows() - k - m_shift;
          if(m_trans)
            dst.bottomRightCorner(cornerSize, cornerSize)
            .applyHouseholderOnTheRight(essentialVector(k), m_coeffs.coeff(k), &temp.coeffRef(0));
          else
            dst.bottomRightCorner(cornerSize, cornerSize)
              .applyHouseholderOnTheLeft(essentialVector(k), m_coeffs.coeff(k), &temp.coeffRef(0));
        }
      }
    }

    /** \internal */
    template<typename Dest> inline void applyThisOnTheRight(Dest& dst) const
    {
      Matrix<Scalar,1,Dest::RowsAtCompileTime> temp(dst.rows());
      for(Index k = 0; k < m_actualVectors; ++k)
      {
        Index actual_k = m_trans ? m_actualVectors-k-1 : k;
        dst.rightCols(rows()-m_shift-actual_k)
           .applyHouseholderOnTheRight(essentialVector(actual_k), m_coeffs.coeff(actual_k), &temp.coeffRef(0));
      }
    }

    /** \internal */
    template<typename Dest> inline void applyThisOnTheLeft(Dest& dst) const
    {
      Matrix<Scalar,1,Dest::ColsAtCompileTime> temp(dst.cols());
      for(Index k = 0; k < m_actualVectors; ++k)
      {
        Index actual_k = m_trans ? k : m_actualVectors-k-1;
        dst.bottomRows(rows()-m_shift-actual_k)
           .applyHouseholderOnTheLeft(essentialVector(actual_k), m_coeffs.coeff(actual_k), &temp.coeffRef(0));
      }
    }

    template<typename OtherDerived>
    typename ei_matrix_type_times_scalar_type<Scalar, OtherDerived>::Type operator*(const MatrixBase<OtherDerived>& other) const
    {
      typename ei_matrix_type_times_scalar_type<Scalar, OtherDerived>::Type
        res(other.template cast<typename ei_matrix_type_times_scalar_type<Scalar, OtherDerived>::ResultScalar>());
      applyThisOnTheLeft(res);
      return res;
    }

    template<typename OtherDerived> friend
    typename ei_matrix_type_times_scalar_type<Scalar, OtherDerived>::Type operator*(const MatrixBase<OtherDerived>& other, const HouseholderSequence& h)
    {
      typename ei_matrix_type_times_scalar_type<Scalar, OtherDerived>::Type
        res(other.template cast<typename ei_matrix_type_times_scalar_type<Scalar, OtherDerived>::ResultScalar>());
      h.applyThisOnTheRight(res);
      return res;
    }

    template<typename _VectorsType, typename _CoeffsType, int _Side> friend struct ei_hseq_side_dependent_impl;

  protected:
    typename VectorsType::Nested m_vectors;
    typename CoeffsType::Nested m_coeffs;
    bool m_trans;
    Index m_actualVectors;
    Index m_shift;
};

template<typename VectorsType, typename CoeffsType>
HouseholderSequence<VectorsType,CoeffsType> householderSequence(const VectorsType& v, const CoeffsType& h, bool trans=false)
{
  return HouseholderSequence<VectorsType,CoeffsType,OnTheLeft>(v, h, trans);
}

template<typename VectorsType, typename CoeffsType>
HouseholderSequence<VectorsType,CoeffsType> householderSequence
   (const VectorsType& v, const CoeffsType& h,
    bool trans, typename VectorsType::Index actualVectors, typename VectorsType::Index shift)
{
  return HouseholderSequence<VectorsType,CoeffsType,OnTheLeft>(v, h, trans, actualVectors, shift);
}

template<typename VectorsType, typename CoeffsType>
HouseholderSequence<VectorsType,CoeffsType> rightHouseholderSequence(const VectorsType& v, const CoeffsType& h, bool trans=false)
{
  return HouseholderSequence<VectorsType,CoeffsType,OnTheRight>(v, h, trans);
}

template<typename VectorsType, typename CoeffsType>
HouseholderSequence<VectorsType,CoeffsType> rightHouseholderSequence
  (const VectorsType& v, const CoeffsType& h, bool trans,
   typename VectorsType::Index actualVectors, typename VectorsType::Index shift)
{
  return HouseholderSequence<VectorsType,CoeffsType,OnTheRight>(v, h, trans, actualVectors, shift);
}

#endif // EIGEN_HOUSEHOLDER_SEQUENCE_H
