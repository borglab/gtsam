// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2010 Konstantinos Margaritis <markos@codex.gr>
// Heavily based on Gael's SSE version.
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

#ifndef EIGEN_PACKET_MATH_NEON_H
#define EIGEN_PACKET_MATH_NEON_H

#ifndef EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD
#define EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD 8
#endif

#ifndef EIGEN_TUNE_FOR_CPU_CACHE_SIZE
#define EIGEN_TUNE_FOR_CPU_CACHE_SIZE 4*192*192
#endif

// FIXME NEON has 16 quad registers, but since the current register allocator
// is so bad, it is much better to reduce it to 8
#ifndef EIGEN_ARCH_DEFAULT_NUMBER_OF_REGISTERS
#define EIGEN_ARCH_DEFAULT_NUMBER_OF_REGISTERS 8
#endif

typedef float32x4_t Packet4f;
typedef int32x4_t   Packet4i;

#define _EIGEN_DECLARE_CONST_Packet4f(NAME,X) \
  const Packet4f ei_p4f_##NAME = ei_pset1<Packet4f>(X)

#define _EIGEN_DECLARE_CONST_Packet4f_FROM_INT(NAME,X) \
  const Packet4f ei_p4f_##NAME = vreinterpretq_f32_u32(ei_pset1<int>(X))

#define _EIGEN_DECLARE_CONST_Packet4i(NAME,X) \
  const Packet4i ei_p4i_##NAME = ei_pset1<Packet4i>(X)

#ifndef __pld
#define __pld(x) asm volatile ( "   pld [%[addr]]\n" :: [addr] "r" (x) : "cc" );
#endif

template<> struct ei_packet_traits<float>  : ei_default_packet_traits
{
  typedef Packet4f type;
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    size = 4,
   
    HasDiv  = 1,
    // FIXME check the Has*
    HasSin  = 0,
    HasCos  = 0,
    HasLog  = 0,
    HasExp  = 0,
    HasSqrt = 0
  };
};
template<> struct ei_packet_traits<int>    : ei_default_packet_traits
{
  typedef Packet4i type;
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    size=4
    // FIXME check the Has*
  };
};

template<> struct ei_unpacket_traits<Packet4f> { typedef float  type; enum {size=4}; };
template<> struct ei_unpacket_traits<Packet4i> { typedef int    type; enum {size=4}; };

template<> EIGEN_STRONG_INLINE Packet4f ei_pset1<Packet4f>(const float&  from) { return vdupq_n_f32(from); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pset1<Packet4i>(const int&    from)   { return vdupq_n_s32(from); }

template<> EIGEN_STRONG_INLINE Packet4f ei_plset<float>(const float& a)
{
  Packet4f countdown = { 3, 2, 1, 0 };
  return vaddq_f32(ei_pset1<Packet4f>(a), countdown);
}
template<> EIGEN_STRONG_INLINE Packet4i ei_plset<int>(const int& a)
{
  Packet4i countdown = { 3, 2, 1, 0 };
  return vaddq_s32(ei_pset1<Packet4i>(a), countdown);
}

template<> EIGEN_STRONG_INLINE Packet4f ei_padd<Packet4f>(const Packet4f& a, const Packet4f& b) { return vaddq_f32(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_padd<Packet4i>(const Packet4i& a, const Packet4i& b) { return vaddq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_psub<Packet4f>(const Packet4f& a, const Packet4f& b) { return vsubq_f32(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_psub<Packet4i>(const Packet4i& a, const Packet4i& b) { return vsubq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pnegate(const Packet4f& a) { return vnegq_f32(a); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pnegate(const Packet4i& a) { return vnegq_s32(a); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pmul<Packet4f>(const Packet4f& a, const Packet4f& b) { return vmulq_f32(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pmul<Packet4i>(const Packet4i& a, const Packet4i& b) { return vmulq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pdiv<Packet4f>(const Packet4f& a, const Packet4f& b)
{
  Packet4f inv, restep, div;

  // NEON does not offer a divide instruction, we have to do a reciprocal approximation
  // However NEON in contrast to other SIMD engines (AltiVec/SSE), offers
  // a reciprocal estimate AND a reciprocal step -which saves a few instructions
  // vrecpeq_f32() returns an estimate to 1/b, which we will finetune with
  // Newton-Raphson and vrecpsq_f32()
  inv = vrecpeq_f32(b);

  // This returns a differential, by which we will have to multiply inv to get a better
  // approximation of 1/b.
  restep = vrecpsq_f32(b, inv);
  inv = vmulq_f32(restep, inv);

  // Finally, multiply a by 1/b and get the wanted result of the division.
  div = vmulq_f32(a, inv);

  return div;
}
template<> EIGEN_STRONG_INLINE Packet4i ei_pdiv<Packet4i>(const Packet4i& /*a*/, const Packet4i& /*b*/)
{ ei_assert(false && "packet integer division are not supported by NEON");
  return ei_pset1<Packet4i>(0);
}

// for some weird raisons, it has to be overloaded for packet of integers
template<> EIGEN_STRONG_INLINE Packet4i ei_pmadd(const Packet4i& a, const Packet4i& b, const Packet4i& c) { return ei_padd(ei_pmul(a,b), c); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pmin<Packet4f>(const Packet4f& a, const Packet4f& b) { return vminq_f32(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pmin<Packet4i>(const Packet4i& a, const Packet4i& b) { return vminq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pmax<Packet4f>(const Packet4f& a, const Packet4f& b) { return vmaxq_f32(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pmax<Packet4i>(const Packet4i& a, const Packet4i& b) { return vmaxq_s32(a,b); }

// Logical Operations are not supported for float, so we have to reinterpret casts using NEON intrinsics
template<> EIGEN_STRONG_INLINE Packet4f ei_pand<Packet4f>(const Packet4f& a, const Packet4f& b)
{
  return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a),vreinterpretq_u32_f32(b)));
}
template<> EIGEN_STRONG_INLINE Packet4i ei_pand<Packet4i>(const Packet4i& a, const Packet4i& b) { return vandq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_por<Packet4f>(const Packet4f& a, const Packet4f& b)
{
  return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a),vreinterpretq_u32_f32(b)));
}
template<> EIGEN_STRONG_INLINE Packet4i ei_por<Packet4i>(const Packet4i& a, const Packet4i& b) { return vorrq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pxor<Packet4f>(const Packet4f& a, const Packet4f& b)
{
  return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a),vreinterpretq_u32_f32(b)));
}
template<> EIGEN_STRONG_INLINE Packet4i ei_pxor<Packet4i>(const Packet4i& a, const Packet4i& b) { return veorq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pandnot<Packet4f>(const Packet4f& a, const Packet4f& b)
{
  return vreinterpretq_f32_u32(vbicq_u32(vreinterpretq_u32_f32(a),vreinterpretq_u32_f32(b)));
}
template<> EIGEN_STRONG_INLINE Packet4i ei_pandnot<Packet4i>(const Packet4i& a, const Packet4i& b) { return vbicq_s32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pload<float>(const float* from) { EIGEN_DEBUG_ALIGNED_LOAD return vld1q_f32(from); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pload<int>(const int*     from) { EIGEN_DEBUG_ALIGNED_LOAD return vld1q_s32(from); }

template<> EIGEN_STRONG_INLINE Packet4f ei_ploadu(const float* from) { EIGEN_DEBUG_UNALIGNED_LOAD return vld1q_f32(from); }
template<> EIGEN_STRONG_INLINE Packet4i ei_ploadu(const int* from)   { EIGEN_DEBUG_UNALIGNED_LOAD return vld1q_s32(from); }

template<> EIGEN_STRONG_INLINE Packet4f ei_ploaddup<Packet4f>(const float* from)
{
  float32x2_t lo, ho;
  lo = vdup_n_f32(*from);
  hi = vdup_n_f32(*from);
  return vcombine_f32(lo, hi);
}
template<> EIGEN_STRONG_INLINE Packet4i ei_ploaddup<Packet4i>(const float* from)
{
  int32x2_t lo, ho;
  lo = vdup_n_s32(*from);
  hi = vdup_n_s32(*from);
  return vcombine_s32(lo, hi);
}

template<> EIGEN_STRONG_INLINE void ei_pstore<float>(float*   to, const Packet4f& from) { EIGEN_DEBUG_ALIGNED_STORE vst1q_f32(to, from); }
template<> EIGEN_STRONG_INLINE void ei_pstore<int>(int*       to, const Packet4i& from) { EIGEN_DEBUG_ALIGNED_STORE vst1q_s32(to, from); }

template<> EIGEN_STRONG_INLINE void ei_pstoreu<float>(float*  to, const Packet4f& from) { EIGEN_DEBUG_UNALIGNED_STORE vst1q_f32(to, from); }
template<> EIGEN_STRONG_INLINE void ei_pstoreu<int>(int*      to, const Packet4i& from) { EIGEN_DEBUG_UNALIGNED_STORE vst1q_s32(to, from); }

template<> EIGEN_STRONG_INLINE void ei_prefetch<float>(const float* addr) { __pld(addr); }
template<> EIGEN_STRONG_INLINE void ei_prefetch<int>(const int*     addr) { __pld(addr); }

// FIXME only store the 2 first elements ?
template<> EIGEN_STRONG_INLINE float  ei_pfirst<Packet4f>(const Packet4f& a) { float EIGEN_ALIGN16 x[4]; vst1q_f32(x, a); return x[0]; }
template<> EIGEN_STRONG_INLINE int    ei_pfirst<Packet4i>(const Packet4i& a) { int   EIGEN_ALIGN16 x[4]; vst1q_s32(x, a); return x[0]; }

template<> EIGEN_STRONG_INLINE Packet4f ei_preverse(const Packet4f& a) {
  float32x2_t a_lo, a_hi;
  Packet4f a_r64;

  a_r64 = vrev64q_f32(a);
  a_lo = vget_low_f32(a_r64);
  a_hi = vget_high_f32(a_r64);
  return vcombine_f32(a_hi, a_lo);
}
template<> EIGEN_STRONG_INLINE Packet4i ei_preverse(const Packet4i& a) {
  int32x2_t a_lo, a_hi;
  Packet4i a_r64;

  a_r64 = vrev64q_s32(a);
  a_lo = vget_low_s32(a_r64);
  a_hi = vget_high_s32(a_r64);
  return vcombine_s32(a_hi, a_lo);
}
template<> EIGEN_STRONG_INLINE Packet4f ei_pabs(const Packet4f& a) { return vabsq_f32(a); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pabs(const Packet4i& a) { return vabsq_s32(a); }

template<> EIGEN_STRONG_INLINE float ei_predux<Packet4f>(const Packet4f& a)
{
  float32x2_t a_lo, a_hi, sum;
  float s[2];

  a_lo = vget_low_f32(a);
  a_hi = vget_high_f32(a);
  sum = vpadd_f32(a_lo, a_hi);
  sum = vpadd_f32(sum, sum);
  vst1_f32(s, sum);

  return s[0];
}

template<> EIGEN_STRONG_INLINE Packet4f ei_preduxp<Packet4f>(const Packet4f* vecs)
{
  float32x4x2_t vtrn1, vtrn2, res1, res2;
  Packet4f sum1, sum2, sum;

  // NEON zip performs interleaving of the supplied vectors.
  // We perform two interleaves in a row to acquire the transposed vector
  vtrn1 = vzipq_f32(vecs[0], vecs[2]);
  vtrn2 = vzipq_f32(vecs[1], vecs[3]);
  res1 = vzipq_f32(vtrn1.val[0], vtrn2.val[0]);
  res2 = vzipq_f32(vtrn1.val[1], vtrn2.val[1]);

  // Do the addition of the resulting vectors
  sum1 = vaddq_f32(res1.val[0], res1.val[1]);
  sum2 = vaddq_f32(res2.val[0], res2.val[1]);
  sum = vaddq_f32(sum1, sum2);

  return sum;
}

template<> EIGEN_STRONG_INLINE int ei_predux<Packet4i>(const Packet4i& a)
{
  int32x2_t a_lo, a_hi, sum;
  int32_t s[2];

  a_lo = vget_low_s32(a);
  a_hi = vget_high_s32(a);
  sum = vpadd_s32(a_lo, a_hi);
  sum = vpadd_s32(sum, sum);
  vst1_s32(s, sum);

  return s[0];
}

template<> EIGEN_STRONG_INLINE Packet4i ei_preduxp<Packet4i>(const Packet4i* vecs)
{
  int32x4x2_t vtrn1, vtrn2, res1, res2;
  Packet4i sum1, sum2, sum;

  // NEON zip performs interleaving of the supplied vectors.
  // We perform two interleaves in a row to acquire the transposed vector
  vtrn1 = vzipq_s32(vecs[0], vecs[2]);
  vtrn2 = vzipq_s32(vecs[1], vecs[3]);
  res1 = vzipq_s32(vtrn1.val[0], vtrn2.val[0]);
  res2 = vzipq_s32(vtrn1.val[1], vtrn2.val[1]);

  // Do the addition of the resulting vectors
  sum1 = vaddq_s32(res1.val[0], res1.val[1]);
  sum2 = vaddq_s32(res2.val[0], res2.val[1]);
  sum = vaddq_s32(sum1, sum2);

  return sum;
}

// Other reduction functions:
// mul
template<> EIGEN_STRONG_INLINE float ei_predux_mul<Packet4f>(const Packet4f& a)
{
  float32x2_t a_lo, a_hi, prod;
  float s[2];

  // Get a_lo = |a1|a2| and a_hi = |a3|a4|
  a_lo = vget_low_f32(a);
  a_hi = vget_high_f32(a);
  // Get the product of a_lo * a_hi -> |a1*a3|a2*a4|
  prod = vmul_f32(a_lo, a_hi);
  // Multiply prod with its swapped value |a2*a4|a1*a3|
  prod = vmul_f32(prod, vrev64_f32(prod));
  vst1_f32(s, prod);

  return s[0];
}
template<> EIGEN_STRONG_INLINE int ei_predux_mul<Packet4i>(const Packet4i& a)
{
  int32x2_t a_lo, a_hi, prod;
  int32_t s[2];

  // Get a_lo = |a1|a2| and a_hi = |a3|a4|
  a_lo = vget_low_s32(a);
  a_hi = vget_high_s32(a);
  // Get the product of a_lo * a_hi -> |a1*a3|a2*a4|
  prod = vmul_s32(a_lo, a_hi);
  // Multiply prod with its swapped value |a2*a4|a1*a3|
  prod = vmul_s32(prod, vrev64_s32(prod));
  vst1_s32(s, prod);

  return s[0];
}

// min
template<> EIGEN_STRONG_INLINE float ei_predux_min<Packet4f>(const Packet4f& a)
{
  float32x2_t a_lo, a_hi, min;
  float s[2];

  a_lo = vget_low_f32(a);
  a_hi = vget_high_f32(a);
  min = vpmin_f32(a_lo, a_hi);
  min = vpmin_f32(min, min);
  vst1_f32(s, min);

  return s[0];
}
template<> EIGEN_STRONG_INLINE int ei_predux_min<Packet4i>(const Packet4i& a)
{
  int32x2_t a_lo, a_hi, min;
  int32_t s[2];

  a_lo = vget_low_s32(a);
  a_hi = vget_high_s32(a);
  min = vpmin_s32(a_lo, a_hi);
  min = vpmin_s32(min, min);
  vst1_s32(s, min);

  return s[0];
}

// max
template<> EIGEN_STRONG_INLINE float ei_predux_max<Packet4f>(const Packet4f& a)
{
  float32x2_t a_lo, a_hi, max;
  float s[2];

  a_lo = vget_low_f32(a);
  a_hi = vget_high_f32(a);
  max = vpmax_f32(a_lo, a_hi);
  max = vpmax_f32(max, max);
  vst1_f32(s, max);

  return s[0];
}
template<> EIGEN_STRONG_INLINE int ei_predux_max<Packet4i>(const Packet4i& a)
{
  int32x2_t a_lo, a_hi, max;
  int32_t s[2];

  a_lo = vget_low_s32(a);
  a_hi = vget_high_s32(a);
  max = vpmax_s32(a_lo, a_hi);
  max = vpmax_s32(max, max);
  vst1_s32(s, max);

  return s[0];
}

template<int Offset>
struct ei_palign_impl<Offset,Packet4f>
{
  EIGEN_STRONG_INLINE static void run(Packet4f& first, const Packet4f& second)
  {
    if (Offset!=0)
      first = vextq_f32(first, second, Offset);
  }
};

template<int Offset>
struct ei_palign_impl<Offset,Packet4i>
{
  EIGEN_STRONG_INLINE static void run(Packet4i& first, const Packet4i& second)
  {
    if (Offset!=0)
      first = vextq_s32(first, second, Offset);
  }
};
#endif // EIGEN_PACKET_MATH_NEON_H
