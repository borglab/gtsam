// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_BLASUTIL_H
#define EIGEN_BLASUTIL_H

// This file contains many lightweight helper classes used to
// implement and control fast level 2 and level 3 BLAS-like routines.

// forward declarations
template<typename LhsScalar, typename RhsScalar, typename Index, int mr, int nr, bool ConjugateLhs=false, bool ConjugateRhs=false>
struct ei_gebp_kernel;

template<typename Scalar, typename Index, int nr, int StorageOrder, bool Conjugate = false, bool PanelMode=false>
struct ei_gemm_pack_rhs;

template<typename Scalar, typename Index, int Pack1, int Pack2, int StorageOrder, bool Conjugate = false, bool PanelMode = false>
struct ei_gemm_pack_lhs;

template<
  typename Index,
  typename LhsScalar, int LhsStorageOrder, bool ConjugateLhs,
  typename RhsScalar, int RhsStorageOrder, bool ConjugateRhs,
  int ResStorageOrder>
struct ei_general_matrix_matrix_product;

template<typename Index, typename LhsScalar, int LhsStorageOrder, bool ConjugateLhs, typename RhsScalar, bool ConjugateRhs>
struct ei_general_matrix_vector_product;


template<bool Conjugate> struct ei_conj_if;

template<> struct ei_conj_if<true> {
  template<typename T>
  inline T operator()(const T& x) { return ei_conj(x); }
};

template<> struct ei_conj_if<false> {
  template<typename T>
  inline const T& operator()(const T& x) { return x; }
};

template<typename Scalar> struct ei_conj_helper<Scalar,Scalar,false,false>
{
  EIGEN_STRONG_INLINE Scalar pmadd(const Scalar& x, const Scalar& y, const Scalar& c) const { return  ei_pmadd(x,y,c); }
  EIGEN_STRONG_INLINE Scalar pmul(const Scalar& x, const Scalar& y) const { return  ei_pmul(x,y); }
};

template<typename RealScalar> struct ei_conj_helper<std::complex<RealScalar>, std::complex<RealScalar>, false,true>
{
  typedef std::complex<RealScalar> Scalar;
  EIGEN_STRONG_INLINE Scalar pmadd(const Scalar& x, const Scalar& y, const Scalar& c) const
  { return c + pmul(x,y); }

  EIGEN_STRONG_INLINE Scalar pmul(const Scalar& x, const Scalar& y) const
  { return Scalar(ei_real(x)*ei_real(y) + ei_imag(x)*ei_imag(y), ei_imag(x)*ei_real(y) - ei_real(x)*ei_imag(y)); }
};

template<typename RealScalar> struct ei_conj_helper<std::complex<RealScalar>, std::complex<RealScalar>, true,false>
{
  typedef std::complex<RealScalar> Scalar;
  EIGEN_STRONG_INLINE Scalar pmadd(const Scalar& x, const Scalar& y, const Scalar& c) const
  { return c + pmul(x,y); }

  EIGEN_STRONG_INLINE Scalar pmul(const Scalar& x, const Scalar& y) const
  { return Scalar(ei_real(x)*ei_real(y) + ei_imag(x)*ei_imag(y), ei_real(x)*ei_imag(y) - ei_imag(x)*ei_real(y)); }
};

template<typename RealScalar> struct ei_conj_helper<std::complex<RealScalar>, std::complex<RealScalar>, true,true>
{
  typedef std::complex<RealScalar> Scalar;
  EIGEN_STRONG_INLINE Scalar pmadd(const Scalar& x, const Scalar& y, const Scalar& c) const
  { return c + pmul(x,y); }

  EIGEN_STRONG_INLINE Scalar pmul(const Scalar& x, const Scalar& y) const
  { return Scalar(ei_real(x)*ei_real(y) - ei_imag(x)*ei_imag(y), - ei_real(x)*ei_imag(y) - ei_imag(x)*ei_real(y)); }
};

template<typename RealScalar,bool Conj> struct ei_conj_helper<std::complex<RealScalar>, RealScalar, Conj,false>
{
  typedef std::complex<RealScalar> Scalar;
  EIGEN_STRONG_INLINE Scalar pmadd(const Scalar& x, const RealScalar& y, const Scalar& c) const
  { return ei_padd(c, pmul(x,y)); }
  EIGEN_STRONG_INLINE Scalar pmul(const Scalar& x, const RealScalar& y) const
  { return ei_conj_if<Conj>()(x)*y; }
};

template<typename RealScalar,bool Conj> struct ei_conj_helper<RealScalar, std::complex<RealScalar>, false,Conj>
{
  typedef std::complex<RealScalar> Scalar;
  EIGEN_STRONG_INLINE Scalar pmadd(const RealScalar& x, const Scalar& y, const Scalar& c) const
  { return ei_padd(c, pmul(x,y)); }
  EIGEN_STRONG_INLINE Scalar pmul(const RealScalar& x, const Scalar& y) const
  { return x*ei_conj_if<Conj>()(y); }
};

template<typename From,typename To> struct ei_get_factor {
  EIGEN_STRONG_INLINE static To run(const From& x) { return x; }
};

template<typename Scalar> struct ei_get_factor<Scalar,typename NumTraits<Scalar>::Real> {
  EIGEN_STRONG_INLINE static typename NumTraits<Scalar>::Real run(const Scalar& x) { return ei_real(x); }
};

// Lightweight helper class to access matrix coefficients.
// Yes, this is somehow redundant with Map<>, but this version is much much lighter,
// and so I hope better compilation performance (time and code quality).
template<typename Scalar, typename Index, int StorageOrder>
class ei_blas_data_mapper
{
  public:
    ei_blas_data_mapper(Scalar* data, Index stride) : m_data(data), m_stride(stride) {}
    EIGEN_STRONG_INLINE Scalar& operator()(Index i, Index j)
    { return m_data[StorageOrder==RowMajor ? j + i*m_stride : i + j*m_stride]; }
  protected:
    Scalar* EIGEN_RESTRICT m_data;
    Index m_stride;
};

// lightweight helper class to access matrix coefficients (const version)
template<typename Scalar, typename Index, int StorageOrder>
class ei_const_blas_data_mapper
{
  public:
    ei_const_blas_data_mapper(const Scalar* data, Index stride) : m_data(data), m_stride(stride) {}
    EIGEN_STRONG_INLINE const Scalar& operator()(Index i, Index j) const
    { return m_data[StorageOrder==RowMajor ? j + i*m_stride : i + j*m_stride]; }
  protected:
    const Scalar* EIGEN_RESTRICT m_data;
    Index m_stride;
};


/* Helper class to analyze the factors of a Product expression.
 * In particular it allows to pop out operator-, scalar multiples,
 * and conjugate */
template<typename XprType> struct ei_blas_traits
{
  typedef typename ei_traits<XprType>::Scalar Scalar;
  typedef const XprType& ExtractType;
  typedef XprType _ExtractType;
  enum {
    IsComplex = NumTraits<Scalar>::IsComplex,
    IsTransposed = false,
    NeedToConjugate = false,
    HasUsableDirectAccess = (    (int(XprType::Flags)&DirectAccessBit)
                     && (  /* Uncomment this when the low-level matrix-vector product functions support strided vectors
                           bool(XprType::IsVectorAtCompileTime)
                         ||  */
                           int(ei_inner_stride_at_compile_time<XprType>::ret) == 1)
                   ) ?  1 : 0
  };
  typedef typename ei_meta_if<bool(HasUsableDirectAccess),
    ExtractType,
    typename _ExtractType::PlainObject
    >::ret DirectLinearAccessType;
  static inline ExtractType extract(const XprType& x) { return x; }
  static inline Scalar extractScalarFactor(const XprType&) { return Scalar(1); }
};

// pop conjugate
template<typename Scalar, typename NestedXpr>
struct ei_blas_traits<CwiseUnaryOp<ei_scalar_conjugate_op<Scalar>, NestedXpr> >
 : ei_blas_traits<NestedXpr>
{
  typedef ei_blas_traits<NestedXpr> Base;
  typedef CwiseUnaryOp<ei_scalar_conjugate_op<Scalar>, NestedXpr> XprType;
  typedef typename Base::ExtractType ExtractType;

  enum {
    IsComplex = NumTraits<Scalar>::IsComplex,
    NeedToConjugate = Base::NeedToConjugate ? 0 : IsComplex
  };
  static inline ExtractType extract(const XprType& x) { return Base::extract(x.nestedExpression()); }
  static inline Scalar extractScalarFactor(const XprType& x) { return ei_conj(Base::extractScalarFactor(x.nestedExpression())); }
};

// pop scalar multiple
template<typename Scalar, typename NestedXpr>
struct ei_blas_traits<CwiseUnaryOp<ei_scalar_multiple_op<Scalar>, NestedXpr> >
 : ei_blas_traits<NestedXpr>
{
  typedef ei_blas_traits<NestedXpr> Base;
  typedef CwiseUnaryOp<ei_scalar_multiple_op<Scalar>, NestedXpr> XprType;
  typedef typename Base::ExtractType ExtractType;
  static inline ExtractType extract(const XprType& x) { return Base::extract(x.nestedExpression()); }
  static inline Scalar extractScalarFactor(const XprType& x)
  { return x.functor().m_other * Base::extractScalarFactor(x.nestedExpression()); }
};

// pop opposite
template<typename Scalar, typename NestedXpr>
struct ei_blas_traits<CwiseUnaryOp<ei_scalar_opposite_op<Scalar>, NestedXpr> >
 : ei_blas_traits<NestedXpr>
{
  typedef ei_blas_traits<NestedXpr> Base;
  typedef CwiseUnaryOp<ei_scalar_opposite_op<Scalar>, NestedXpr> XprType;
  typedef typename Base::ExtractType ExtractType;
  static inline ExtractType extract(const XprType& x) { return Base::extract(x.nestedExpression()); }
  static inline Scalar extractScalarFactor(const XprType& x)
  { return - Base::extractScalarFactor(x.nestedExpression()); }
};

// pop/push transpose
template<typename NestedXpr>
struct ei_blas_traits<Transpose<NestedXpr> >
 : ei_blas_traits<NestedXpr>
{
  typedef typename NestedXpr::Scalar Scalar;
  typedef ei_blas_traits<NestedXpr> Base;
  typedef Transpose<NestedXpr> XprType;
  typedef Transpose<typename Base::_ExtractType>  ExtractType;
  typedef Transpose<typename Base::_ExtractType> _ExtractType;
  typedef typename ei_meta_if<bool(Base::HasUsableDirectAccess),
    ExtractType,
    typename ExtractType::PlainObject
    >::ret DirectLinearAccessType;
  enum {
    IsTransposed = Base::IsTransposed ? 0 : 1
  };
  static inline const ExtractType extract(const XprType& x) { return Base::extract(x.nestedExpression()); }
  static inline Scalar extractScalarFactor(const XprType& x) { return Base::extractScalarFactor(x.nestedExpression()); }
};

template<typename T, bool HasUsableDirectAccess=ei_blas_traits<T>::HasUsableDirectAccess>
struct ei_extract_data_selector {
  static const typename T::Scalar* run(const T& m)
  {
    return &ei_blas_traits<T>::extract(m).const_cast_derived().coeffRef(0,0); // FIXME this should be .data()
  }
};

template<typename T>
struct ei_extract_data_selector<T,false> {
  static typename T::Scalar* run(const T&) { return 0; }
};

template<typename T> const typename T::Scalar* ei_extract_data(const T& m)
{
  return ei_extract_data_selector<T>::run(m);
}

#endif // EIGEN_BLASUTIL_H
