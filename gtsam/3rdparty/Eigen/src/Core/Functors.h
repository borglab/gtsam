// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_FUNCTORS_H
#define EIGEN_FUNCTORS_H

// associative functors:

/** \internal
  * \brief Template functor to compute the sum of two scalars
  *
  * \sa class CwiseBinaryOp, MatrixBase::operator+, class VectorwiseOp, MatrixBase::sum()
  */
template<typename Scalar> struct ei_scalar_sum_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_sum_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const { return a + b; }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const Packet& b) const
  { return ei_padd(a,b); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Scalar predux(const Packet& a) const
  { return ei_predux(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_sum_op<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = ei_packet_traits<Scalar>::HasAdd
  };
};

/** \internal
  * \brief Template functor to compute the product of two scalars
  *
  * \sa class CwiseBinaryOp, Cwise::operator*(), class VectorwiseOp, MatrixBase::redux()
  */
template<typename LhsScalar,typename RhsScalar> struct ei_scalar_product_op {
  enum {
    Vectorizable = ei_is_same_type<LhsScalar,RhsScalar>::ret && ei_packet_traits<LhsScalar>::HasMul && ei_packet_traits<RhsScalar>::HasMul
  };
  typedef typename ei_scalar_product_traits<LhsScalar,RhsScalar>::ReturnType result_type;
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_product_op)
  EIGEN_STRONG_INLINE const result_type operator() (const LhsScalar& a, const RhsScalar& b) const { return a * b; }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const Packet& b) const
  { return ei_pmul(a,b); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const result_type predux(const Packet& a) const
  { return ei_predux_mul(a); }
};
template<typename LhsScalar,typename RhsScalar>
struct ei_functor_traits<ei_scalar_product_op<LhsScalar,RhsScalar> > {
  enum {
    Cost = (NumTraits<LhsScalar>::MulCost + NumTraits<RhsScalar>::MulCost)/2, // rough estimate!
    PacketAccess = ei_scalar_product_op<LhsScalar,RhsScalar>::Vectorizable
  };
};

/** \internal
  * \brief Template functor to compute the conjugate product of two scalars
  *
  * This is a short cut for ei_conj(x) * y which is needed for optimization purpose
  */
template<typename Scalar> struct ei_scalar_conj_product_op {
  enum { Conj = NumTraits<Scalar>::IsComplex };
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_conj_product_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const
  { return ei_conj_helper<Scalar,Scalar,Conj,false>().pmul(a,b); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const Packet& b) const
  { return ei_conj_helper<Packet,Packet,Conj,false>().pmul(a,b); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_conj_product_op<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::MulCost,
    PacketAccess = ei_packet_traits<Scalar>::HasMul
  };
};

/** \internal
  * \brief Template functor to compute the min of two scalars
  *
  * \sa class CwiseBinaryOp, MatrixBase::cwiseMin, class VectorwiseOp, MatrixBase::minCoeff()
  */
template<typename Scalar> struct ei_scalar_min_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_min_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const { return std::min(a, b); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const Packet& b) const
  { return ei_pmin(a,b); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Scalar predux(const Packet& a) const
  { return ei_predux_min(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_min_op<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = ei_packet_traits<Scalar>::HasMin
  };
};

/** \internal
  * \brief Template functor to compute the max of two scalars
  *
  * \sa class CwiseBinaryOp, MatrixBase::cwiseMax, class VectorwiseOp, MatrixBase::maxCoeff()
  */
template<typename Scalar> struct ei_scalar_max_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_max_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const { return std::max(a, b); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const Packet& b) const
  { return ei_pmax(a,b); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Scalar predux(const Packet& a) const
  { return ei_predux_max(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_max_op<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = ei_packet_traits<Scalar>::HasMax
  };
};

/** \internal
  * \brief Template functor to compute the hypot of two scalars
  *
  * \sa MatrixBase::stableNorm(), class Redux
  */
template<typename Scalar> struct ei_scalar_hypot_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_hypot_op)
//   typedef typename NumTraits<Scalar>::Real result_type;
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& _x, const Scalar& _y) const
  {
    Scalar p = std::max(_x, _y);
    Scalar q = std::min(_x, _y);
    Scalar qp = q/p;
    return p * ei_sqrt(Scalar(1) + qp*qp);
  }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_hypot_op<Scalar> > {
  enum { Cost = 5 * NumTraits<Scalar>::MulCost, PacketAccess=0 };
};

// other binary functors:

/** \internal
  * \brief Template functor to compute the difference of two scalars
  *
  * \sa class CwiseBinaryOp, MatrixBase::operator-
  */
template<typename Scalar> struct ei_scalar_difference_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_difference_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const { return a - b; }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const Packet& b) const
  { return ei_psub(a,b); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_difference_op<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = ei_packet_traits<Scalar>::HasSub
  };
};

/** \internal
  * \brief Template functor to compute the quotient of two scalars
  *
  * \sa class CwiseBinaryOp, Cwise::operator/()
  */
template<typename Scalar> struct ei_scalar_quotient_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_quotient_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const { return a / b; }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a, const Packet& b) const
  { return ei_pdiv(a,b); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_quotient_op<Scalar> > {
  enum {
    Cost = 2 * NumTraits<Scalar>::MulCost,
    PacketAccess = ei_packet_traits<Scalar>::HasDiv
  };
};

// unary functors:

/** \internal
  * \brief Template functor to compute the opposite of a scalar
  *
  * \sa class CwiseUnaryOp, MatrixBase::operator-
  */
template<typename Scalar> struct ei_scalar_opposite_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_opposite_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a) const { return -a; }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const
  { return ei_pnegate(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_opposite_op<Scalar> >
{ enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = ei_packet_traits<Scalar>::HasNegate };
};

/** \internal
  * \brief Template functor to compute the absolute value of a scalar
  *
  * \sa class CwiseUnaryOp, Cwise::abs
  */
template<typename Scalar> struct ei_scalar_abs_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_abs_op)
  typedef typename NumTraits<Scalar>::Real result_type;
  EIGEN_STRONG_INLINE const result_type operator() (const Scalar& a) const { return ei_abs(a); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const
  { return ei_pabs(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_abs_op<Scalar> >
{
  enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = ei_packet_traits<Scalar>::HasAbs
  };
};

/** \internal
  * \brief Template functor to compute the squared absolute value of a scalar
  *
  * \sa class CwiseUnaryOp, Cwise::abs2
  */
template<typename Scalar> struct ei_scalar_abs2_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_abs2_op)
  typedef typename NumTraits<Scalar>::Real result_type;
  EIGEN_STRONG_INLINE const result_type operator() (const Scalar& a) const { return ei_abs2(a); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const
  { return ei_pmul(a,a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_abs2_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasAbs2 }; };

/** \internal
  * \brief Template functor to compute the conjugate of a complex value
  *
  * \sa class CwiseUnaryOp, MatrixBase::conjugate()
  */
template<typename Scalar> struct ei_scalar_conjugate_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_conjugate_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a) const { return ei_conj(a); }
  template<typename Packet>
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const { return ei_pconj(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_conjugate_op<Scalar> >
{
  enum {
    Cost = NumTraits<Scalar>::IsComplex ? NumTraits<Scalar>::AddCost : 0,
    PacketAccess = ei_packet_traits<Scalar>::HasConj
  };
};

/** \internal
  * \brief Template functor to cast a scalar to another type
  *
  * \sa class CwiseUnaryOp, MatrixBase::cast()
  */
template<typename Scalar, typename NewType>
struct ei_scalar_cast_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_cast_op)
  typedef NewType result_type;
  EIGEN_STRONG_INLINE const NewType operator() (const Scalar& a) const { return ei_cast<Scalar, NewType>(a); }
};
template<typename Scalar, typename NewType>
struct ei_functor_traits<ei_scalar_cast_op<Scalar,NewType> >
{ enum { Cost = ei_is_same_type<Scalar, NewType>::ret ? 0 : NumTraits<NewType>::AddCost, PacketAccess = false }; };

/** \internal
  * \brief Template functor to extract the real part of a complex
  *
  * \sa class CwiseUnaryOp, MatrixBase::real()
  */
template<typename Scalar>
struct ei_scalar_real_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_real_op)
  typedef typename NumTraits<Scalar>::Real result_type;
  EIGEN_STRONG_INLINE result_type operator() (const Scalar& a) const { return ei_real(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_real_op<Scalar> >
{ enum { Cost = 0, PacketAccess = false }; };

/** \internal
  * \brief Template functor to extract the imaginary part of a complex
  *
  * \sa class CwiseUnaryOp, MatrixBase::imag()
  */
template<typename Scalar>
struct ei_scalar_imag_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_imag_op)
  typedef typename NumTraits<Scalar>::Real result_type;
  EIGEN_STRONG_INLINE result_type operator() (const Scalar& a) const { return ei_imag(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_imag_op<Scalar> >
{ enum { Cost = 0, PacketAccess = false }; };

/** \internal
  * \brief Template functor to extract the real part of a complex as a reference
  *
  * \sa class CwiseUnaryOp, MatrixBase::real()
  */
template<typename Scalar>
struct ei_scalar_real_ref_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_real_ref_op)
  typedef typename NumTraits<Scalar>::Real result_type;
  EIGEN_STRONG_INLINE result_type& operator() (const Scalar& a) const { return ei_real_ref(*const_cast<Scalar*>(&a)); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_real_ref_op<Scalar> >
{ enum { Cost = 0, PacketAccess = false }; };

/** \internal
  * \brief Template functor to extract the imaginary part of a complex as a reference
  *
  * \sa class CwiseUnaryOp, MatrixBase::imag()
  */
template<typename Scalar>
struct ei_scalar_imag_ref_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_imag_ref_op)
  typedef typename NumTraits<Scalar>::Real result_type;
  EIGEN_STRONG_INLINE result_type& operator() (const Scalar& a) const { return ei_imag_ref(*const_cast<Scalar*>(&a)); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_imag_ref_op<Scalar> >
{ enum { Cost = 0, PacketAccess = false }; };

/** \internal
  *
  * \brief Template functor to compute the exponential of a scalar
  *
  * \sa class CwiseUnaryOp, Cwise::exp()
  */
template<typename Scalar> struct ei_scalar_exp_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_exp_op)
  inline const Scalar operator() (const Scalar& a) const { return ei_exp(a); }
  typedef typename ei_packet_traits<Scalar>::type Packet;
  inline Packet packetOp(const Packet& a) const { return ei_pexp(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_exp_op<Scalar> >
{ enum { Cost = 5 * NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasExp }; };

/** \internal
  *
  * \brief Template functor to compute the logarithm of a scalar
  *
  * \sa class CwiseUnaryOp, Cwise::log()
  */
template<typename Scalar> struct ei_scalar_log_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_log_op)
  inline const Scalar operator() (const Scalar& a) const { return ei_log(a); }
  typedef typename ei_packet_traits<Scalar>::type Packet;
  inline Packet packetOp(const Packet& a) const { return ei_plog(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_log_op<Scalar> >
{ enum { Cost = 5 * NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasLog }; };

/** \internal
  * \brief Template functor to multiply a scalar by a fixed other one
  *
  * \sa class CwiseUnaryOp, MatrixBase::operator*, MatrixBase::operator/
  */
/* NOTE why doing the ei_pset1() in packetOp *is* an optimization ?
 * indeed it seems better to declare m_other as a Packet and do the ei_pset1() once
 * in the constructor. However, in practice:
 *  - GCC does not like m_other as a Packet and generate a load every time it needs it
 *  - on the other hand GCC is able to moves the ei_pset1() away the loop :)
 *  - simpler code ;)
 * (ICC and gcc 4.4 seems to perform well in both cases, the issue is visible with y = a*x + b*y)
 */
template<typename Scalar>
struct ei_scalar_multiple_op {
  typedef typename ei_packet_traits<Scalar>::type Packet;
  // FIXME default copy constructors seems bugged with std::complex<>
  EIGEN_STRONG_INLINE ei_scalar_multiple_op(const ei_scalar_multiple_op& other) : m_other(other.m_other) { }
  EIGEN_STRONG_INLINE ei_scalar_multiple_op(const Scalar& other) : m_other(other) { }
  EIGEN_STRONG_INLINE Scalar operator() (const Scalar& a) const { return a * m_other; }
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const
  { return ei_pmul(a, ei_pset1<Packet>(m_other)); }
  typename ei_makeconst<typename NumTraits<Scalar>::Nested>::type m_other;
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_multiple_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasMul }; };

template<typename Scalar1, typename Scalar2>
struct ei_scalar_multiple2_op {
  typedef typename ei_scalar_product_traits<Scalar1,Scalar2>::ReturnType result_type;
  EIGEN_STRONG_INLINE ei_scalar_multiple2_op(const ei_scalar_multiple2_op& other) : m_other(other.m_other) { }
  EIGEN_STRONG_INLINE ei_scalar_multiple2_op(const Scalar2& other) : m_other(other) { }
  EIGEN_STRONG_INLINE result_type operator() (const Scalar1& a) const { return a * m_other; }
  typename ei_makeconst<typename NumTraits<Scalar2>::Nested>::type m_other;
};
template<typename Scalar1,typename Scalar2>
struct ei_functor_traits<ei_scalar_multiple2_op<Scalar1,Scalar2> >
{ enum { Cost = NumTraits<Scalar1>::MulCost, PacketAccess = false }; };

template<typename Scalar, bool IsInteger>
struct ei_scalar_quotient1_impl {
  typedef typename ei_packet_traits<Scalar>::type Packet;
  // FIXME default copy constructors seems bugged with std::complex<>
  EIGEN_STRONG_INLINE ei_scalar_quotient1_impl(const ei_scalar_quotient1_impl& other) : m_other(other.m_other) { }
  EIGEN_STRONG_INLINE ei_scalar_quotient1_impl(const Scalar& other) : m_other(static_cast<Scalar>(1) / other) {}
  EIGEN_STRONG_INLINE Scalar operator() (const Scalar& a) const { return a * m_other; }
  EIGEN_STRONG_INLINE const Packet packetOp(const Packet& a) const
  { return ei_pmul(a, ei_pset1<Packet>(m_other)); }
  const Scalar m_other;
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_quotient1_impl<Scalar,false> >
{ enum { Cost = NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasMul }; };

template<typename Scalar>
struct ei_scalar_quotient1_impl<Scalar,true> {
  // FIXME default copy constructors seems bugged with std::complex<>
  EIGEN_STRONG_INLINE ei_scalar_quotient1_impl(const ei_scalar_quotient1_impl& other) : m_other(other.m_other) { }
  EIGEN_STRONG_INLINE ei_scalar_quotient1_impl(const Scalar& other) : m_other(other) {}
  EIGEN_STRONG_INLINE Scalar operator() (const Scalar& a) const { return a / m_other; }
  typename ei_makeconst<typename NumTraits<Scalar>::Nested>::type m_other;
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_quotient1_impl<Scalar,true> >
{ enum { Cost = 2 * NumTraits<Scalar>::MulCost, PacketAccess = false }; };

/** \internal
  * \brief Template functor to divide a scalar by a fixed other one
  *
  * This functor is used to implement the quotient of a matrix by
  * a scalar where the scalar type is not necessarily a floating point type.
  *
  * \sa class CwiseUnaryOp, MatrixBase::operator/
  */
template<typename Scalar>
struct ei_scalar_quotient1_op : ei_scalar_quotient1_impl<Scalar, NumTraits<Scalar>::IsInteger > {
  EIGEN_STRONG_INLINE ei_scalar_quotient1_op(const Scalar& other)
    : ei_scalar_quotient1_impl<Scalar, NumTraits<Scalar>::IsInteger >(other) {}
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_quotient1_op<Scalar> >
: ei_functor_traits<ei_scalar_quotient1_impl<Scalar, NumTraits<Scalar>::IsInteger> >
{};

// nullary functors

template<typename Scalar>
struct ei_scalar_constant_op {
  typedef typename ei_packet_traits<Scalar>::type Packet;
  EIGEN_STRONG_INLINE ei_scalar_constant_op(const ei_scalar_constant_op& other) : m_other(other.m_other) { }
  EIGEN_STRONG_INLINE ei_scalar_constant_op(const Scalar& other) : m_other(other) { }
  template<typename Index>
  EIGEN_STRONG_INLINE const Scalar operator() (Index, Index = 0) const { return m_other; }
  template<typename Index>
  EIGEN_STRONG_INLINE const Packet packetOp(Index, Index = 0) const { return ei_pset1<Packet>(m_other); }
  const Scalar m_other;
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_constant_op<Scalar> >
// FIXME replace this packet test by a safe one
{ enum { Cost = 1, PacketAccess = ei_packet_traits<Scalar>::Vectorizable, IsRepeatable = true }; };

template<typename Scalar> struct ei_scalar_identity_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_identity_op)
  template<typename Index>
  EIGEN_STRONG_INLINE const Scalar operator() (Index row, Index col) const { return row==col ? Scalar(1) : Scalar(0); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_identity_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::AddCost, PacketAccess = false, IsRepeatable = true }; };

template <typename Scalar, bool RandomAccess> struct ei_linspaced_op_impl;

// linear access for packet ops:
// 1) initialization
//   base = [low, ..., low] + ([step, ..., step] * [-size, ..., 0])
// 2) each step
//   base += [size*step, ..., size*step]
template <typename Scalar>
struct ei_linspaced_op_impl<Scalar,false>
{
  typedef typename ei_packet_traits<Scalar>::type Packet;

  ei_linspaced_op_impl(Scalar low, Scalar step) :
  m_low(low), m_step(step),
  m_packetStep(ei_pset1<Packet>(ei_packet_traits<Scalar>::size*step)),
  m_base(ei_padd(ei_pset1<Packet>(low),ei_pmul(ei_pset1<Packet>(step),ei_plset<Scalar>(-ei_packet_traits<Scalar>::size)))) {}

  template<typename Index>
  EIGEN_STRONG_INLINE const Scalar operator() (Index i) const { return m_low+i*m_step; }
  template<typename Index>
  EIGEN_STRONG_INLINE const Packet packetOp(Index) const { return m_base = ei_padd(m_base,m_packetStep); }

  const Scalar m_low;
  const Scalar m_step;
  const Packet m_packetStep;
  mutable Packet m_base;
};

// random access for packet ops:
// 1) each step
//   [low, ..., low] + ( [step, ..., step] * ( [i, ..., i] + [0, ..., size] ) )
template <typename Scalar>
struct ei_linspaced_op_impl<Scalar,true>
{
  typedef typename ei_packet_traits<Scalar>::type Packet;

  ei_linspaced_op_impl(Scalar low, Scalar step) :
  m_low(low), m_step(step),
  m_lowPacket(ei_pset1<Packet>(m_low)), m_stepPacket(ei_pset1<Packet>(m_step)), m_interPacket(ei_plset<Scalar>(0)) {}

  template<typename Index>
  EIGEN_STRONG_INLINE const Scalar operator() (Index i) const { return m_low+i*m_step; }
  template<typename Index>
  EIGEN_STRONG_INLINE const Packet packetOp(Index i) const
  { return ei_padd(m_lowPacket, ei_pmul(m_stepPacket, ei_padd(ei_pset1<Packet>(i),m_interPacket))); }

  const Scalar m_low;
  const Scalar m_step;
  const Packet m_lowPacket;
  const Packet m_stepPacket;
  const Packet m_interPacket;
};

// ----- Linspace functor ----------------------------------------------------------------

// Forward declaration (we default to random access which does not really give
// us a speed gain when using packet access but it allows to use the functor in
// nested expressions).
template <typename Scalar, bool RandomAccess = true> struct ei_linspaced_op;
template <typename Scalar, bool RandomAccess> struct ei_functor_traits< ei_linspaced_op<Scalar,RandomAccess> >
{ enum { Cost = 1, PacketAccess = ei_packet_traits<Scalar>::HasSetLinear, IsRepeatable = true }; };
template <typename Scalar, bool RandomAccess> struct ei_linspaced_op
{
  typedef typename ei_packet_traits<Scalar>::type Packet;
  ei_linspaced_op(Scalar low, Scalar high, int num_steps) : impl(low, (high-low)/(num_steps-1)) {}
  template<typename Index>
  EIGEN_STRONG_INLINE const Scalar operator() (Index i, Index = 0) const { return impl(i); }
  template<typename Index>
  EIGEN_STRONG_INLINE const Packet packetOp(Index i, Index = 0) const { return impl.packetOp(i); }
  // This proxy object handles the actual required temporaries, the different
  // implementations (random vs. sequential access) as well as the
  // correct piping to size 2/4 packet operations.
  const ei_linspaced_op_impl<Scalar,RandomAccess> impl;
};

// all functors allow linear access, except ei_scalar_identity_op. So we fix here a quick meta
// to indicate whether a functor allows linear access, just always answering 'yes' except for
// ei_scalar_identity_op.
// FIXME move this to ei_functor_traits adding a ei_functor_default
template<typename Functor> struct ei_functor_has_linear_access { enum { ret = 1 }; };
template<typename Scalar> struct ei_functor_has_linear_access<ei_scalar_identity_op<Scalar> > { enum { ret = 0 }; };

// in CwiseBinaryOp, we require the Lhs and Rhs to have the same scalar type, except for multiplication
// where we only require them to have the same _real_ scalar type so one may multiply, say, float by complex<float>.
// FIXME move this to ei_functor_traits adding a ei_functor_default
template<typename Functor> struct ei_functor_allows_mixing_real_and_complex { enum { ret = 0 }; };
template<typename LhsScalar,typename RhsScalar> struct ei_functor_allows_mixing_real_and_complex<ei_scalar_product_op<LhsScalar,RhsScalar> > { enum { ret = 1 }; };


/** \internal
  * \brief Template functor to add a scalar to a fixed other one
  * \sa class CwiseUnaryOp, Array::operator+
  */
/* If you wonder why doing the ei_pset1() in packetOp() is an optimization check ei_scalar_multiple_op */
template<typename Scalar>
struct ei_scalar_add_op {
  typedef typename ei_packet_traits<Scalar>::type Packet;
  // FIXME default copy constructors seems bugged with std::complex<>
  inline ei_scalar_add_op(const ei_scalar_add_op& other) : m_other(other.m_other) { }
  inline ei_scalar_add_op(const Scalar& other) : m_other(other) { }
  inline Scalar operator() (const Scalar& a) const { return a + m_other; }
  inline const Packet packetOp(const Packet& a) const
  { return ei_padd(a, ei_pset1<Packet>(m_other)); }
  const Scalar m_other;
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_add_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::AddCost, PacketAccess = ei_packet_traits<Scalar>::HasAdd }; };

/** \internal
  * \brief Template functor to compute the square root of a scalar
  * \sa class CwiseUnaryOp, Cwise::sqrt()
  */
template<typename Scalar> struct ei_scalar_sqrt_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_sqrt_op)
  inline const Scalar operator() (const Scalar& a) const { return ei_sqrt(a); }
  typedef typename ei_packet_traits<Scalar>::type Packet;
  inline Packet packetOp(const Packet& a) const { return ei_psqrt(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_sqrt_op<Scalar> >
{ enum {
    Cost = 5 * NumTraits<Scalar>::MulCost,
    PacketAccess = ei_packet_traits<Scalar>::HasSqrt
  };
};

/** \internal
  * \brief Template functor to compute the cosine of a scalar
  * \sa class CwiseUnaryOp, Cwise::cos()
  */
template<typename Scalar> struct ei_scalar_cos_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_cos_op)
  inline Scalar operator() (const Scalar& a) const { return ei_cos(a); }
  typedef typename ei_packet_traits<Scalar>::type Packet;
  inline Packet packetOp(const Packet& a) const { return ei_pcos(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_cos_op<Scalar> >
{
  enum {
    Cost = 5 * NumTraits<Scalar>::MulCost,
    PacketAccess = ei_packet_traits<Scalar>::HasCos
  };
};

/** \internal
  * \brief Template functor to compute the sine of a scalar
  * \sa class CwiseUnaryOp, Cwise::sin()
  */
template<typename Scalar> struct ei_scalar_sin_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_sin_op)
  inline const Scalar operator() (const Scalar& a) const { return ei_sin(a); }
  typedef typename ei_packet_traits<Scalar>::type Packet;
  inline Packet packetOp(const Packet& a) const { return ei_psin(a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_sin_op<Scalar> >
{
  enum {
    Cost = 5 * NumTraits<Scalar>::MulCost,
    PacketAccess = ei_packet_traits<Scalar>::HasSin
  };
};

/** \internal
  * \brief Template functor to raise a scalar to a power
  * \sa class CwiseUnaryOp, Cwise::pow
  */
template<typename Scalar>
struct ei_scalar_pow_op {
  // FIXME default copy constructors seems bugged with std::complex<>
  inline ei_scalar_pow_op(const ei_scalar_pow_op& other) : m_exponent(other.m_exponent) { }
  inline ei_scalar_pow_op(const Scalar& exponent) : m_exponent(exponent) {}
  inline Scalar operator() (const Scalar& a) const { return ei_pow(a, m_exponent); }
  const Scalar m_exponent;
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_pow_op<Scalar> >
{ enum { Cost = 5 * NumTraits<Scalar>::MulCost, PacketAccess = false }; };

/** \internal
  * \brief Template functor to compute the inverse of a scalar
  * \sa class CwiseUnaryOp, Cwise::inverse()
  */
template<typename Scalar>
struct ei_scalar_inverse_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_inverse_op)
  inline Scalar operator() (const Scalar& a) const { return Scalar(1)/a; }
  template<typename Packet>
  inline const Packet packetOp(const Packet& a) const
  { return ei_pdiv(ei_pset1<Packet>(Scalar(1)),a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_inverse_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasDiv }; };

/** \internal
  * \brief Template functor to compute the square of a scalar
  * \sa class CwiseUnaryOp, Cwise::square()
  */
template<typename Scalar>
struct ei_scalar_square_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_square_op)
  inline Scalar operator() (const Scalar& a) const { return a*a; }
  template<typename Packet>
  inline const Packet packetOp(const Packet& a) const
  { return ei_pmul(a,a); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_square_op<Scalar> >
{ enum { Cost = NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasMul }; };

/** \internal
  * \brief Template functor to compute the cube of a scalar
  * \sa class CwiseUnaryOp, Cwise::cube()
  */
template<typename Scalar>
struct ei_scalar_cube_op {
  EIGEN_EMPTY_STRUCT_CTOR(ei_scalar_cube_op)
  inline Scalar operator() (const Scalar& a) const { return a*a*a; }
  template<typename Packet>
  inline const Packet packetOp(const Packet& a) const
  { return ei_pmul(a,ei_pmul(a,a)); }
};
template<typename Scalar>
struct ei_functor_traits<ei_scalar_cube_op<Scalar> >
{ enum { Cost = 2*NumTraits<Scalar>::MulCost, PacketAccess = ei_packet_traits<Scalar>::HasMul }; };

// default functor traits for STL functors:

template<typename T>
struct ei_functor_traits<std::multiplies<T> >
{ enum { Cost = NumTraits<T>::MulCost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::divides<T> >
{ enum { Cost = NumTraits<T>::MulCost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::plus<T> >
{ enum { Cost = NumTraits<T>::AddCost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::minus<T> >
{ enum { Cost = NumTraits<T>::AddCost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::negate<T> >
{ enum { Cost = NumTraits<T>::AddCost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::logical_or<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::logical_and<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::logical_not<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::greater<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::less<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::greater_equal<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::less_equal<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::equal_to<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::not_equal_to<T> >
{ enum { Cost = 1, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::binder2nd<T> >
{ enum { Cost = ei_functor_traits<T>::Cost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::binder1st<T> >
{ enum { Cost = ei_functor_traits<T>::Cost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::unary_negate<T> >
{ enum { Cost = 1 + ei_functor_traits<T>::Cost, PacketAccess = false }; };

template<typename T>
struct ei_functor_traits<std::binary_negate<T> >
{ enum { Cost = 1 + ei_functor_traits<T>::Cost, PacketAccess = false }; };

#ifdef EIGEN_STDEXT_SUPPORT

template<typename T0,typename T1>
struct ei_functor_traits<std::project1st<T0,T1> >
{ enum { Cost = 0, PacketAccess = false }; };

template<typename T0,typename T1>
struct ei_functor_traits<std::project2nd<T0,T1> >
{ enum { Cost = 0, PacketAccess = false }; };

template<typename T0,typename T1>
struct ei_functor_traits<std::select2nd<std::pair<T0,T1> > >
{ enum { Cost = 0, PacketAccess = false }; };

template<typename T0,typename T1>
struct ei_functor_traits<std::select1st<std::pair<T0,T1> > >
{ enum { Cost = 0, PacketAccess = false }; };

template<typename T0,typename T1>
struct ei_functor_traits<std::unary_compose<T0,T1> >
{ enum { Cost = ei_functor_traits<T0>::Cost + ei_functor_traits<T1>::Cost, PacketAccess = false }; };

template<typename T0,typename T1,typename T2>
struct ei_functor_traits<std::binary_compose<T0,T1,T2> >
{ enum { Cost = ei_functor_traits<T0>::Cost + ei_functor_traits<T1>::Cost + ei_functor_traits<T2>::Cost, PacketAccess = false }; };

#endif // EIGEN_STDEXT_SUPPORT

// allow to add new functors and specializations of ei_functor_traits from outside Eigen.
// this macro is really needed because ei_functor_traits must be specialized after it is declared but before it is used...
#ifdef EIGEN_FUNCTORS_PLUGIN
#include EIGEN_FUNCTORS_PLUGIN
#endif

#endif // EIGEN_FUNCTORS_H
