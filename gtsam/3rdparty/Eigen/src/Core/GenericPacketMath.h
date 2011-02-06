// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
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

#ifndef EIGEN_GENERIC_PACKET_MATH_H
#define EIGEN_GENERIC_PACKET_MATH_H

/** \internal
  * \file GenericPacketMath.h
  *
  * Default implementation for types not supported by the vectorization.
  * In practice these functions are provided to make easier the writing
  * of generic vectorized code.
  */

#ifndef EIGEN_DEBUG_ALIGNED_LOAD
#define EIGEN_DEBUG_ALIGNED_LOAD
#endif

#ifndef EIGEN_DEBUG_UNALIGNED_LOAD
#define EIGEN_DEBUG_UNALIGNED_LOAD
#endif

#ifndef EIGEN_DEBUG_ALIGNED_STORE
#define EIGEN_DEBUG_ALIGNED_STORE
#endif

#ifndef EIGEN_DEBUG_UNALIGNED_STORE
#define EIGEN_DEBUG_UNALIGNED_STORE
#endif

struct ei_default_packet_traits
{
  enum {
    HasAdd    = 1,
    HasSub    = 1,
    HasMul    = 1,
    HasNegate = 1,
    HasAbs    = 1,
    HasAbs2   = 1,
    HasMin    = 1,
    HasMax    = 1,
    HasConj   = 1,
    HasSetLinear = 1,

    HasDiv    = 0,
    HasSqrt   = 0,
    HasExp    = 0,
    HasLog    = 0,
    HasPow    = 0,

    HasSin    = 0,
    HasCos    = 0,
    HasTan    = 0,
    HasASin   = 0,
    HasACos   = 0,
    HasATan   = 0
  };
};

template<typename T> struct ei_packet_traits : ei_default_packet_traits
{
  typedef T type;
  enum {
    Vectorizable = 0,
    size = 1,
    AlignedOnScalar = 0
  };
  enum {
    HasAdd    = 0,
    HasSub    = 0,
    HasMul    = 0,
    HasNegate = 0,
    HasAbs    = 0,
    HasAbs2   = 0,
    HasMin    = 0,
    HasMax    = 0,
    HasConj   = 0,
    HasSetLinear = 0
  };
};

/** \internal \returns a + b (coeff-wise) */
template<typename Packet> inline Packet
ei_padd(const Packet& a,
        const Packet& b) { return a+b; }

/** \internal \returns a - b (coeff-wise) */
template<typename Packet> inline Packet
ei_psub(const Packet& a,
        const Packet& b) { return a-b; }

/** \internal \returns -a (coeff-wise) */
template<typename Packet> inline Packet
ei_pnegate(const Packet& a) { return -a; }

/** \internal \returns conj(a) (coeff-wise) */
template<typename Packet> inline Packet
ei_pconj(const Packet& a) { return ei_conj(a); }

/** \internal \returns a * b (coeff-wise) */
template<typename Packet> inline Packet
ei_pmul(const Packet& a,
        const Packet& b) { return a*b; }

/** \internal \returns a / b (coeff-wise) */
template<typename Packet> inline Packet
ei_pdiv(const Packet& a,
        const Packet& b) { return a/b; }

/** \internal \returns the min of \a a and \a b  (coeff-wise) */
template<typename Packet> inline Packet
ei_pmin(const Packet& a,
        const Packet& b) { return std::min(a, b); }

/** \internal \returns the max of \a a and \a b  (coeff-wise) */
template<typename Packet> inline Packet
ei_pmax(const Packet& a,
        const Packet& b) { return std::max(a, b); }

/** \internal \returns the absolute value of \a a */
template<typename Packet> inline Packet
ei_pabs(const Packet& a) { return ei_abs(a); }

/** \internal \returns the bitwise and of \a a and \a b */
template<typename Packet> inline Packet
ei_pand(const Packet& a, const Packet& b) { return a & b; }

/** \internal \returns the bitwise or of \a a and \a b */
template<typename Packet> inline Packet
ei_por(const Packet& a, const Packet& b) { return a | b; }

/** \internal \returns the bitwise xor of \a a and \a b */
template<typename Packet> inline Packet
ei_pxor(const Packet& a, const Packet& b) { return a ^ b; }

/** \internal \returns the bitwise andnot of \a a and \a b */
template<typename Packet> inline Packet
ei_pandnot(const Packet& a, const Packet& b) { return a & (!b); }

/** \internal \returns a packet version of \a *from, from must be 16 bytes aligned */
template<typename Packet> inline Packet
ei_pload(const typename ei_unpacket_traits<Packet>::type* from) { return *from; }

/** \internal \returns a packet version of \a *from, (un-aligned load) */
template<typename Packet> inline Packet
ei_ploadu(const typename ei_unpacket_traits<Packet>::type* from) { return *from; }

/** \internal \returns a packet with elements of \a *from duplicated, e.g.: (from[0],from[0],from[1],from[1]) */
template<typename Packet> inline Packet
ei_ploaddup(const typename ei_unpacket_traits<Packet>::type* from) { return *from; }

/** \internal \returns a packet with constant coefficients \a a, e.g.: (a,a,a,a) */
template<typename Packet> inline Packet
ei_pset1(const typename ei_unpacket_traits<Packet>::type& a) { return a; }

/** \internal \brief Returns a packet with coefficients (a,a+1,...,a+packet_size-1). */
template<typename Scalar> inline typename ei_packet_traits<Scalar>::type
ei_plset(const Scalar& a) { return a; }

/** \internal copy the packet \a from to \a *to, \a to must be 16 bytes aligned */
template<typename Scalar, typename Packet> inline void ei_pstore(Scalar* to, const Packet& from)
{ (*to) = from; }

/** \internal copy the packet \a from to \a *to, (un-aligned store) */
template<typename Scalar, typename Packet> inline void ei_pstoreu(Scalar* to, const Packet& from)
{ (*to) = from; }

/** \internal tries to do cache prefetching of \a addr */
template<typename Scalar> inline void ei_prefetch(const Scalar* addr)
{
#if !defined(_MSC_VER)
__builtin_prefetch(addr);
#endif
}

/** \internal \returns the first element of a packet */
template<typename Packet> inline typename ei_unpacket_traits<Packet>::type ei_pfirst(const Packet& a)
{ return a; }

/** \internal \returns a packet where the element i contains the sum of the packet of \a vec[i] */
template<typename Packet> inline Packet
ei_preduxp(const Packet* vecs) { return vecs[0]; }

/** \internal \returns the sum of the elements of \a a*/
template<typename Packet> inline typename ei_unpacket_traits<Packet>::type ei_predux(const Packet& a)
{ return a; }

/** \internal \returns the product of the elements of \a a*/
template<typename Packet> inline typename ei_unpacket_traits<Packet>::type ei_predux_mul(const Packet& a)
{ return a; }

/** \internal \returns the min of the elements of \a a*/
template<typename Packet> inline typename ei_unpacket_traits<Packet>::type ei_predux_min(const Packet& a)
{ return a; }

/** \internal \returns the max of the elements of \a a*/
template<typename Packet> inline typename ei_unpacket_traits<Packet>::type ei_predux_max(const Packet& a)
{ return a; }

/** \internal \returns the reversed elements of \a a*/
template<typename Packet> inline Packet ei_preverse(const Packet& a)
{ return a; }

/**************************
* Special math functions
***************************/

/** \internal \returns the sin of \a a (coeff-wise) */
template<typename Packet> EIGEN_DECLARE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS
Packet ei_psin(const Packet& a) { return ei_sin(a); }

/** \internal \returns the cos of \a a (coeff-wise) */
template<typename Packet> EIGEN_DECLARE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS
Packet ei_pcos(const Packet& a) { return ei_cos(a); }

/** \internal \returns the exp of \a a (coeff-wise) */
template<typename Packet> EIGEN_DECLARE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS
Packet ei_pexp(const Packet& a) { return ei_exp(a); }

/** \internal \returns the log of \a a (coeff-wise) */
template<typename Packet> EIGEN_DECLARE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS
Packet ei_plog(const Packet& a) { return ei_log(a); }

/** \internal \returns the square-root of \a a (coeff-wise) */
template<typename Packet> EIGEN_DECLARE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS
Packet ei_psqrt(const Packet& a) { return ei_sqrt(a); }

/***************************************************************************
* The following functions might not have to be overwritten for vectorized types
***************************************************************************/

/** \internal \returns a * b + c (coeff-wise) */
template<typename Packet> inline Packet
ei_pmadd(const Packet&  a,
         const Packet&  b,
         const Packet&  c)
{ return ei_padd(ei_pmul(a, b),c); }

/** \internal \returns a packet version of \a *from.
  * \If LoadMode equals Aligned, \a from must be 16 bytes aligned */
template<typename Packet, int LoadMode>
inline Packet ei_ploadt(const typename ei_unpacket_traits<Packet>::type* from)
{
  if(LoadMode == Aligned)
    return ei_pload<Packet>(from);
  else
    return ei_ploadu<Packet>(from);
}

/** \internal copy the packet \a from to \a *to.
  * If StoreMode equals Aligned, \a to must be 16 bytes aligned */
template<typename Scalar, typename Packet, int LoadMode>
inline void ei_pstoret(Scalar* to, const Packet& from)
{
  if(LoadMode == Aligned)
    ei_pstore(to, from);
  else
    ei_pstoreu(to, from);
}

/** \internal default implementation of ei_palign() allowing partial specialization */
template<int Offset,typename PacketType>
struct ei_palign_impl
{
  // by default data are aligned, so there is nothing to be done :)
  inline static void run(PacketType&, const PacketType&) {}
};

/** \internal update \a first using the concatenation of the \a Offset last elements
  * of \a first and packet_size minus \a Offset first elements of \a second */
template<int Offset,typename PacketType>
inline void ei_palign(PacketType& first, const PacketType& second)
{
  ei_palign_impl<Offset,PacketType>::run(first,second);
}

/***************************************************************************
* Fast complex products (GCC generates a function call which is very slow)
***************************************************************************/

template<> inline std::complex<float> ei_pmul(const std::complex<float>& a, const std::complex<float>& b)
{ return std::complex<float>(ei_real(a)*ei_real(b) - ei_imag(a)*ei_imag(b), ei_imag(a)*ei_real(b) + ei_real(a)*ei_imag(b)); }

template<> inline std::complex<double> ei_pmul(const std::complex<double>& a, const std::complex<double>& b)
{ return std::complex<double>(ei_real(a)*ei_real(b) - ei_imag(a)*ei_imag(b), ei_imag(a)*ei_real(b) + ei_real(a)*ei_imag(b)); }

#endif // EIGEN_GENERIC_PACKET_MATH_H

