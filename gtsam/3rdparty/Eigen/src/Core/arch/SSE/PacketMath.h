// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_PACKET_MATH_SSE_H
#define EIGEN_PACKET_MATH_SSE_H

#ifndef EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD
#define EIGEN_CACHEFRIENDLY_PRODUCT_THRESHOLD 8
#endif

#ifndef EIGEN_ARCH_DEFAULT_NUMBER_OF_REGISTERS
#define EIGEN_ARCH_DEFAULT_NUMBER_OF_REGISTERS (2*sizeof(void*))
#endif

typedef __m128  Packet4f;
typedef __m128i Packet4i;
typedef __m128d Packet2d;

template<> struct ei_is_arithmetic<__m128>  { enum { ret = true }; };
template<> struct ei_is_arithmetic<__m128i> { enum { ret = true }; };
template<> struct ei_is_arithmetic<__m128d> { enum { ret = true }; };

#define ei_vec4f_swizzle1(v,p,q,r,s) \
  (_mm_castsi128_ps(_mm_shuffle_epi32( _mm_castps_si128(v), ((s)<<6|(r)<<4|(q)<<2|(p)))))

#define ei_vec4i_swizzle1(v,p,q,r,s) \
  (_mm_shuffle_epi32( v, ((s)<<6|(r)<<4|(q)<<2|(p))))

#define ei_vec2d_swizzle1(v,p,q) \
  (_mm_castsi128_pd(_mm_shuffle_epi32( _mm_castpd_si128(v), ((q*2+1)<<6|(q*2)<<4|(p*2+1)<<2|(p*2)))))
  
#define ei_vec4f_swizzle2(a,b,p,q,r,s) \
  (_mm_shuffle_ps( (a), (b), ((s)<<6|(r)<<4|(q)<<2|(p))))

#define ei_vec4i_swizzle2(a,b,p,q,r,s) \
  (_mm_castps_si128( (_mm_shuffle_ps( _mm_castsi128_ps(a), _mm_castsi128_ps(b), ((s)<<6|(r)<<4|(q)<<2|(p))))))

#define _EIGEN_DECLARE_CONST_Packet4f(NAME,X) \
  const Packet4f ei_p4f_##NAME = ei_pset1<Packet4f>(X)

#define _EIGEN_DECLARE_CONST_Packet4f_FROM_INT(NAME,X) \
  const Packet4f ei_p4f_##NAME = _mm_castsi128_ps(ei_pset1<Packet4i>(X))

#define _EIGEN_DECLARE_CONST_Packet4i(NAME,X) \
  const Packet4i ei_p4i_##NAME = ei_pset1<Packet4i>(X)


template<> struct ei_packet_traits<float>  : ei_default_packet_traits
{
  typedef Packet4f type;
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    size=4,

    HasDiv    = 1,
    HasSin  = EIGEN_FAST_MATH,
    HasCos  = EIGEN_FAST_MATH,
    HasLog  = 1,
    HasExp  = 1,
    HasSqrt = 1
  };
};
template<> struct ei_packet_traits<double> : ei_default_packet_traits
{
  typedef Packet2d type;
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    size=2,

    HasDiv    = 1
  };
};
template<> struct ei_packet_traits<int>    : ei_default_packet_traits
{
  typedef Packet4i type;
  enum {
    // FIXME check the Has*
    Vectorizable = 1,
    AlignedOnScalar = 1,
    size=4
  };
};

template<> struct ei_unpacket_traits<Packet4f> { typedef float  type; enum {size=4}; };
template<> struct ei_unpacket_traits<Packet2d> { typedef double type; enum {size=2}; };
template<> struct ei_unpacket_traits<Packet4i> { typedef int    type; enum {size=4}; };

#ifdef __GNUC__
// Sometimes GCC implements _mm_set1_p* using multiple moves,
// that is inefficient :( (e.g., see ei_gemm_pack_rhs)
template<> EIGEN_STRONG_INLINE Packet4f ei_pset1<Packet4f>(const float&  from) {
  Packet4f res = _mm_set_ss(from);
  return ei_vec4f_swizzle1(res,0,0,0,0);
}
template<> EIGEN_STRONG_INLINE Packet2d ei_pset1<Packet2d>(const double&  from) {
  // NOTE the SSE3 intrinsic _mm_loaddup_pd is never faster but sometimes much slower
  Packet2d res = _mm_set_sd(from);
  return ei_vec2d_swizzle1(res, 0, 0);
}
#else
template<> EIGEN_STRONG_INLINE Packet4f ei_pset1<Packet4f>(const float&  from) { return _mm_set1_ps(from); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pset1<Packet2d>(const double& from) { return _mm_set1_pd(from); }
#endif
template<> EIGEN_STRONG_INLINE Packet4i ei_pset1<Packet4i>(const int&    from) { return _mm_set1_epi32(from); }

template<> EIGEN_STRONG_INLINE Packet4f ei_plset<float>(const float& a) { return _mm_add_ps(ei_pset1<Packet4f>(a), _mm_set_ps(3,2,1,0)); }
template<> EIGEN_STRONG_INLINE Packet2d ei_plset<double>(const double& a) { return _mm_add_pd(ei_pset1<Packet2d>(a),_mm_set_pd(1,0)); }
template<> EIGEN_STRONG_INLINE Packet4i ei_plset<int>(const int& a) { return _mm_add_epi32(ei_pset1<Packet4i>(a),_mm_set_epi32(3,2,1,0)); }

template<> EIGEN_STRONG_INLINE Packet4f ei_padd<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_add_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_padd<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_add_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_padd<Packet4i>(const Packet4i& a, const Packet4i& b) { return _mm_add_epi32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_psub<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_sub_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_psub<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_sub_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_psub<Packet4i>(const Packet4i& a, const Packet4i& b) { return _mm_sub_epi32(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pnegate(const Packet4f& a)
{
  const Packet4f mask = _mm_castsi128_ps(_mm_setr_epi32(0x80000000,0x80000000,0x80000000,0x80000000));
  return _mm_xor_ps(a,mask);
}
template<> EIGEN_STRONG_INLINE Packet2d ei_pnegate(const Packet2d& a)
{
  const Packet2d mask = _mm_castsi128_pd(_mm_setr_epi32(0x0,0x80000000,0x0,0x80000000));
  return _mm_xor_pd(a,mask);
}
template<> EIGEN_STRONG_INLINE Packet4i ei_pnegate(const Packet4i& a)
{
  return ei_psub(_mm_setr_epi32(0,0,0,0), a);
}

template<> EIGEN_STRONG_INLINE Packet4f ei_pmul<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_mul_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pmul<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_mul_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pmul<Packet4i>(const Packet4i& a, const Packet4i& b)
{
#ifdef EIGEN_VECTORIZE_SSE4_1
  return _mm_mullo_epi32(a,b);
#else
  // this version is slightly faster than 4 scalar products
  return ei_vec4i_swizzle1(
            ei_vec4i_swizzle2(
              _mm_mul_epu32(a,b),
              _mm_mul_epu32(ei_vec4i_swizzle1(a,1,0,3,2),
                            ei_vec4i_swizzle1(b,1,0,3,2)),
              0,2,0,2),
            0,2,1,3);
#endif
}

template<> EIGEN_STRONG_INLINE Packet4f ei_pdiv<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_div_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pdiv<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_div_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pdiv<Packet4i>(const Packet4i& /*a*/, const Packet4i& /*b*/)
{ ei_assert(false && "packet integer division are not supported by SSE");
  return ei_pset1<Packet4i>(0);
}

// for some weird raisons, it has to be overloaded for packet of integers
template<> EIGEN_STRONG_INLINE Packet4i ei_pmadd(const Packet4i& a, const Packet4i& b, const Packet4i& c) { return ei_padd(ei_pmul(a,b), c); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pmin<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_min_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pmin<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_min_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pmin<Packet4i>(const Packet4i& a, const Packet4i& b)
{
  // after some bench, this version *is* faster than a scalar implementation
  Packet4i mask = _mm_cmplt_epi32(a,b);
  return _mm_or_si128(_mm_and_si128(mask,a),_mm_andnot_si128(mask,b));
}

template<> EIGEN_STRONG_INLINE Packet4f ei_pmax<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_max_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pmax<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_max_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pmax<Packet4i>(const Packet4i& a, const Packet4i& b)
{
  // after some bench, this version *is* faster than a scalar implementation
  Packet4i mask = _mm_cmpgt_epi32(a,b);
  return _mm_or_si128(_mm_and_si128(mask,a),_mm_andnot_si128(mask,b));
}

template<> EIGEN_STRONG_INLINE Packet4f ei_pand<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_and_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pand<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_and_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pand<Packet4i>(const Packet4i& a, const Packet4i& b) { return _mm_and_si128(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_por<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_or_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_por<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_or_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_por<Packet4i>(const Packet4i& a, const Packet4i& b) { return _mm_or_si128(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pxor<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_xor_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pxor<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_xor_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pxor<Packet4i>(const Packet4i& a, const Packet4i& b) { return _mm_xor_si128(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pandnot<Packet4f>(const Packet4f& a, const Packet4f& b) { return _mm_andnot_ps(a,b); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pandnot<Packet2d>(const Packet2d& a, const Packet2d& b) { return _mm_andnot_pd(a,b); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pandnot<Packet4i>(const Packet4i& a, const Packet4i& b) { return _mm_andnot_si128(a,b); }

template<> EIGEN_STRONG_INLINE Packet4f ei_pload<Packet4f>(const float*   from) { EIGEN_DEBUG_ALIGNED_LOAD return _mm_load_ps(from); }
template<> EIGEN_STRONG_INLINE Packet2d ei_pload<Packet2d>(const double*  from) { EIGEN_DEBUG_ALIGNED_LOAD return _mm_load_pd(from); }
template<> EIGEN_STRONG_INLINE Packet4i ei_pload<Packet4i>(const int*     from) { EIGEN_DEBUG_ALIGNED_LOAD return _mm_load_si128(reinterpret_cast<const Packet4i*>(from)); }

#if defined(_MSC_VER)
  template<> EIGEN_STRONG_INLINE Packet4f ei_ploadu<Packet4f>(const float*  from) { EIGEN_DEBUG_UNALIGNED_LOAD return _mm_loadu_ps(from); }
  template<> EIGEN_STRONG_INLINE Packet2d ei_ploadu<Packet2d>(const double* from) { EIGEN_DEBUG_UNALIGNED_LOAD return _mm_loadu_pd(from); }
  template<> EIGEN_STRONG_INLINE Packet4i ei_ploadu<Packet4i>(const int*    from) { EIGEN_DEBUG_UNALIGNED_LOAD return _mm_loadu_si128(reinterpret_cast<const Packet4i*>(from)); }
#else
// Fast unaligned loads. Note that here we cannot directly use intrinsics: this would
// require pointer casting to incompatible pointer types and leads to invalid code
// because of the strict aliasing rule. The "dummy" stuff are required to enforce
// a correct instruction dependency.
// TODO: do the same for MSVC (ICC is compatible)
// NOTE: with the code below, MSVC's compiler crashes!
template<> EIGEN_STRONG_INLINE Packet4f ei_ploadu<Packet4f>(const float* from)
{
  EIGEN_DEBUG_UNALIGNED_LOAD
  __m128d res;
  res =  _mm_load_sd((const double*)(from)) ;
  res =  _mm_loadh_pd(res, (const double*)(from+2)) ;
  return _mm_castpd_ps(res);
}
template<> EIGEN_STRONG_INLINE Packet2d ei_ploadu<Packet2d>(const double* from)
{
  EIGEN_DEBUG_UNALIGNED_LOAD
  __m128d res;
  res = _mm_load_sd(from) ;
  res = _mm_loadh_pd(res,from+1);
  return res;
}
template<> EIGEN_STRONG_INLINE Packet4i ei_ploadu<Packet4i>(const int* from)
{
  EIGEN_DEBUG_UNALIGNED_LOAD
  __m128d res;
  res =  _mm_load_sd((const double*)(from)) ;
  res =  _mm_loadh_pd(res, (const double*)(from+2)) ;
  return _mm_castpd_si128(res);
}
#endif

template<> EIGEN_STRONG_INLINE Packet4f ei_ploaddup<Packet4f>(const float*   from)
{
  return ei_vec4f_swizzle1(_mm_castpd_ps(_mm_load_sd((const double*)from)), 0, 0, 1, 1);
}
template<> EIGEN_STRONG_INLINE Packet2d ei_ploaddup<Packet2d>(const double*  from)
{ return ei_pset1<Packet2d>(from[0]); }
template<> EIGEN_STRONG_INLINE Packet4i ei_ploaddup<Packet4i>(const int*     from)
{
  Packet4i tmp;
  tmp = _mm_loadl_epi64(reinterpret_cast<const Packet4i*>(from));
  return ei_vec4i_swizzle1(tmp, 0, 0, 1, 1);
}

template<> EIGEN_STRONG_INLINE void ei_pstore<float>(float*   to, const Packet4f& from) { EIGEN_DEBUG_ALIGNED_STORE _mm_store_ps(to, from); }
template<> EIGEN_STRONG_INLINE void ei_pstore<double>(double* to, const Packet2d& from) { EIGEN_DEBUG_ALIGNED_STORE _mm_store_pd(to, from); }
template<> EIGEN_STRONG_INLINE void ei_pstore<int>(int*       to, const Packet4i& from) { EIGEN_DEBUG_ALIGNED_STORE _mm_store_si128(reinterpret_cast<Packet4i*>(to), from); }

template<> EIGEN_STRONG_INLINE void ei_pstoreu<double>(double* to, const Packet2d& from) {
  EIGEN_DEBUG_UNALIGNED_STORE
  _mm_storel_pd((to), from);
  _mm_storeh_pd((to+1), from);
}
template<> EIGEN_STRONG_INLINE void ei_pstoreu<float>(float*  to, const Packet4f& from) { EIGEN_DEBUG_UNALIGNED_STORE ei_pstoreu((double*)to, _mm_castps_pd(from)); }
template<> EIGEN_STRONG_INLINE void ei_pstoreu<int>(int*      to, const Packet4i& from) { EIGEN_DEBUG_UNALIGNED_STORE ei_pstoreu((double*)to, _mm_castsi128_pd(from)); }

template<> EIGEN_STRONG_INLINE void ei_prefetch<float>(const float*   addr) { _mm_prefetch((const char*)(addr), _MM_HINT_T0); }
template<> EIGEN_STRONG_INLINE void ei_prefetch<double>(const double* addr) { _mm_prefetch((const char*)(addr), _MM_HINT_T0); }
template<> EIGEN_STRONG_INLINE void ei_prefetch<int>(const int*       addr) { _mm_prefetch((const char*)(addr), _MM_HINT_T0); }

#if defined(_MSC_VER) && (_MSC_VER <= 1500) && defined(_WIN64) && !defined(__INTEL_COMPILER)
// The temporary variable fixes an internal compilation error.
// Direct of the struct members fixed bug #62.
template<> EIGEN_STRONG_INLINE float  ei_pfirst<Packet4f>(const Packet4f& a) { return a.m128_f32[0]; }
template<> EIGEN_STRONG_INLINE double ei_pfirst<Packet2d>(const Packet2d& a) { return a.m128d_f64[0]; }
template<> EIGEN_STRONG_INLINE int    ei_pfirst<Packet4i>(const Packet4i& a) { int x = _mm_cvtsi128_si32(a); return x; }
#elif defined(_MSC_VER) && (_MSC_VER <= 1500) && !defined(__INTEL_COMPILER)
// The temporary variable fixes an internal compilation error.
template<> EIGEN_STRONG_INLINE float  ei_pfirst<Packet4f>(const Packet4f& a) { float x = _mm_cvtss_f32(a); return x; }
template<> EIGEN_STRONG_INLINE double ei_pfirst<Packet2d>(const Packet2d& a) { double x = _mm_cvtsd_f64(a); return x; }
template<> EIGEN_STRONG_INLINE int    ei_pfirst<Packet4i>(const Packet4i& a) { int x = _mm_cvtsi128_si32(a); return x; }
#else
template<> EIGEN_STRONG_INLINE float  ei_pfirst<Packet4f>(const Packet4f& a) { return _mm_cvtss_f32(a); }
template<> EIGEN_STRONG_INLINE double ei_pfirst<Packet2d>(const Packet2d& a) { return _mm_cvtsd_f64(a); }
template<> EIGEN_STRONG_INLINE int    ei_pfirst<Packet4i>(const Packet4i& a) { return _mm_cvtsi128_si32(a); }
#endif

template<> EIGEN_STRONG_INLINE Packet4f ei_preverse(const Packet4f& a)
{ return _mm_shuffle_ps(a,a,0x1B); }
template<> EIGEN_STRONG_INLINE Packet2d ei_preverse(const Packet2d& a)
{ return _mm_shuffle_pd(a,a,0x1); }
template<> EIGEN_STRONG_INLINE Packet4i ei_preverse(const Packet4i& a)
{ return _mm_shuffle_epi32(a,0x1B); }


template<> EIGEN_STRONG_INLINE Packet4f ei_pabs(const Packet4f& a)
{
  const Packet4f mask = _mm_castsi128_ps(_mm_setr_epi32(0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF));
  return _mm_and_ps(a,mask);
}
template<> EIGEN_STRONG_INLINE Packet2d ei_pabs(const Packet2d& a)
{
  const Packet2d mask = _mm_castsi128_pd(_mm_setr_epi32(0xFFFFFFFF,0x7FFFFFFF,0xFFFFFFFF,0x7FFFFFFF));
  return _mm_and_pd(a,mask);
}
template<> EIGEN_STRONG_INLINE Packet4i ei_pabs(const Packet4i& a)
{
  #ifdef EIGEN_VECTORIZE_SSSE3
  return _mm_abs_epi32(a);
  #else
  Packet4i aux = _mm_srai_epi32(a,31);
  return _mm_sub_epi32(_mm_xor_si128(a,aux),aux);
  #endif
}

EIGEN_STRONG_INLINE void ei_punpackp(Packet4f* vecs)
{
  vecs[1] = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(vecs[0]), 0x55));
  vecs[2] = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(vecs[0]), 0xAA));
  vecs[3] = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(vecs[0]), 0xFF));
  vecs[0] = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(vecs[0]), 0x00));
}

#ifdef EIGEN_VECTORIZE_SSE3
// TODO implement SSE2 versions as well as integer versions
template<> EIGEN_STRONG_INLINE Packet4f ei_preduxp<Packet4f>(const Packet4f* vecs)
{
  return _mm_hadd_ps(_mm_hadd_ps(vecs[0], vecs[1]),_mm_hadd_ps(vecs[2], vecs[3]));
}
template<> EIGEN_STRONG_INLINE Packet2d ei_preduxp<Packet2d>(const Packet2d* vecs)
{
  return _mm_hadd_pd(vecs[0], vecs[1]);
}
// SSSE3 version:
// EIGEN_STRONG_INLINE Packet4i ei_preduxp(const Packet4i* vecs)
// {
//   return _mm_hadd_epi32(_mm_hadd_epi32(vecs[0], vecs[1]),_mm_hadd_epi32(vecs[2], vecs[3]));
// }

template<> EIGEN_STRONG_INLINE float ei_predux<Packet4f>(const Packet4f& a)
{
  Packet4f tmp0 = _mm_hadd_ps(a,a);
  return ei_pfirst(_mm_hadd_ps(tmp0, tmp0));
}

template<> EIGEN_STRONG_INLINE double ei_predux<Packet2d>(const Packet2d& a) { return ei_pfirst(_mm_hadd_pd(a, a)); }

// SSSE3 version:
// EIGEN_STRONG_INLINE float ei_predux(const Packet4i& a)
// {
//   Packet4i tmp0 = _mm_hadd_epi32(a,a);
//   return ei_pfirst(_mm_hadd_epi32(tmp0, tmp0));
// }
#else
// SSE2 versions
template<> EIGEN_STRONG_INLINE float ei_predux<Packet4f>(const Packet4f& a)
{
  Packet4f tmp = _mm_add_ps(a, _mm_movehl_ps(a,a));
  return ei_pfirst(_mm_add_ss(tmp, _mm_shuffle_ps(tmp,tmp, 1)));
}
template<> EIGEN_STRONG_INLINE double ei_predux<Packet2d>(const Packet2d& a)
{
  return ei_pfirst(_mm_add_sd(a, _mm_unpackhi_pd(a,a)));
}

template<> EIGEN_STRONG_INLINE Packet4f ei_preduxp<Packet4f>(const Packet4f* vecs)
{
  Packet4f tmp0, tmp1, tmp2;
  tmp0 = _mm_unpacklo_ps(vecs[0], vecs[1]);
  tmp1 = _mm_unpackhi_ps(vecs[0], vecs[1]);
  tmp2 = _mm_unpackhi_ps(vecs[2], vecs[3]);
  tmp0 = _mm_add_ps(tmp0, tmp1);
  tmp1 = _mm_unpacklo_ps(vecs[2], vecs[3]);
  tmp1 = _mm_add_ps(tmp1, tmp2);
  tmp2 = _mm_movehl_ps(tmp1, tmp0);
  tmp0 = _mm_movelh_ps(tmp0, tmp1);
  return _mm_add_ps(tmp0, tmp2);
}

template<> EIGEN_STRONG_INLINE Packet2d ei_preduxp<Packet2d>(const Packet2d* vecs)
{
  return _mm_add_pd(_mm_unpacklo_pd(vecs[0], vecs[1]), _mm_unpackhi_pd(vecs[0], vecs[1]));
}
#endif  // SSE3

template<> EIGEN_STRONG_INLINE int ei_predux<Packet4i>(const Packet4i& a)
{
  Packet4i tmp = _mm_add_epi32(a, _mm_unpackhi_epi64(a,a));
  return ei_pfirst(tmp) + ei_pfirst(_mm_shuffle_epi32(tmp, 1));
}

template<> EIGEN_STRONG_INLINE Packet4i ei_preduxp<Packet4i>(const Packet4i* vecs)
{
  Packet4i tmp0, tmp1, tmp2;
  tmp0 = _mm_unpacklo_epi32(vecs[0], vecs[1]);
  tmp1 = _mm_unpackhi_epi32(vecs[0], vecs[1]);
  tmp2 = _mm_unpackhi_epi32(vecs[2], vecs[3]);
  tmp0 = _mm_add_epi32(tmp0, tmp1);
  tmp1 = _mm_unpacklo_epi32(vecs[2], vecs[3]);
  tmp1 = _mm_add_epi32(tmp1, tmp2);
  tmp2 = _mm_unpacklo_epi64(tmp0, tmp1);
  tmp0 = _mm_unpackhi_epi64(tmp0, tmp1);
  return _mm_add_epi32(tmp0, tmp2);
}

// Other reduction functions:

// mul
template<> EIGEN_STRONG_INLINE float ei_predux_mul<Packet4f>(const Packet4f& a)
{
  Packet4f tmp = _mm_mul_ps(a, _mm_movehl_ps(a,a));
  return ei_pfirst(_mm_mul_ss(tmp, _mm_shuffle_ps(tmp,tmp, 1)));
}
template<> EIGEN_STRONG_INLINE double ei_predux_mul<Packet2d>(const Packet2d& a)
{
  return ei_pfirst(_mm_mul_sd(a, _mm_unpackhi_pd(a,a)));
}
template<> EIGEN_STRONG_INLINE int ei_predux_mul<Packet4i>(const Packet4i& a)
{
  // after some experiments, it is seems this is the fastest way to implement it
  // for GCC (eg., reusing ei_pmul is very slow !)
  // TODO try to call _mm_mul_epu32 directly
  EIGEN_ALIGN16 int aux[4];
  ei_pstore(aux, a);
  return  (aux[0] * aux[1]) * (aux[2] * aux[3]);;
}

// min
template<> EIGEN_STRONG_INLINE float ei_predux_min<Packet4f>(const Packet4f& a)
{
  Packet4f tmp = _mm_min_ps(a, _mm_movehl_ps(a,a));
  return ei_pfirst(_mm_min_ss(tmp, _mm_shuffle_ps(tmp,tmp, 1)));
}
template<> EIGEN_STRONG_INLINE double ei_predux_min<Packet2d>(const Packet2d& a)
{
  return ei_pfirst(_mm_min_sd(a, _mm_unpackhi_pd(a,a)));
}
template<> EIGEN_STRONG_INLINE int ei_predux_min<Packet4i>(const Packet4i& a)
{
  // after some experiments, it is seems this is the fastest way to implement it
  // for GCC (eg., it does not like using std::min after the ei_pstore !!)
  EIGEN_ALIGN16 int aux[4];
  ei_pstore(aux, a);
  register int aux0 = aux[0]<aux[1] ? aux[0] : aux[1];
  register int aux2 = aux[2]<aux[3] ? aux[2] : aux[3];
  return aux0<aux2 ? aux0 : aux2;
}

// max
template<> EIGEN_STRONG_INLINE float ei_predux_max<Packet4f>(const Packet4f& a)
{
  Packet4f tmp = _mm_max_ps(a, _mm_movehl_ps(a,a));
  return ei_pfirst(_mm_max_ss(tmp, _mm_shuffle_ps(tmp,tmp, 1)));
}
template<> EIGEN_STRONG_INLINE double ei_predux_max<Packet2d>(const Packet2d& a)
{
  return ei_pfirst(_mm_max_sd(a, _mm_unpackhi_pd(a,a)));
}
template<> EIGEN_STRONG_INLINE int ei_predux_max<Packet4i>(const Packet4i& a)
{
  // after some experiments, it is seems this is the fastest way to implement it
  // for GCC (eg., it does not like using std::min after the ei_pstore !!)
  EIGEN_ALIGN16 int aux[4];
  ei_pstore(aux, a);
  register int aux0 = aux[0]>aux[1] ? aux[0] : aux[1];
  register int aux2 = aux[2]>aux[3] ? aux[2] : aux[3];
  return aux0>aux2 ? aux0 : aux2;
}

#if (defined __GNUC__)
// template <> EIGEN_STRONG_INLINE Packet4f ei_pmadd(const Packet4f&  a, const Packet4f&  b, const Packet4f&  c)
// {
//   Packet4f res = b;
//   asm("mulps %[a], %[b] \n\taddps %[c], %[b]" : [b] "+x" (res) : [a] "x" (a), [c] "x" (c));
//   return res;
// }
// EIGEN_STRONG_INLINE Packet4i _mm_alignr_epi8(const Packet4i&  a, const Packet4i&  b, const int i)
// {
//   Packet4i res = a;
//   asm("palignr %[i], %[a], %[b] " : [b] "+x" (res) : [a] "x" (a), [i] "i" (i));
//   return res;
// }
#endif

#ifdef EIGEN_VECTORIZE_SSSE3
// SSSE3 versions
template<int Offset>
struct ei_palign_impl<Offset,Packet4f>
{
  EIGEN_STRONG_INLINE static void run(Packet4f& first, const Packet4f& second)
  {
    if (Offset!=0)
      first = _mm_castsi128_ps(_mm_alignr_epi8(_mm_castps_si128(second), _mm_castps_si128(first), Offset*4));
  }
};

template<int Offset>
struct ei_palign_impl<Offset,Packet4i>
{
  EIGEN_STRONG_INLINE static void run(Packet4i& first, const Packet4i& second)
  {
    if (Offset!=0)
      first = _mm_alignr_epi8(second,first, Offset*4);
  }
};

template<int Offset>
struct ei_palign_impl<Offset,Packet2d>
{
  EIGEN_STRONG_INLINE static void run(Packet2d& first, const Packet2d& second)
  {
    if (Offset==1)
      first = _mm_castsi128_pd(_mm_alignr_epi8(_mm_castpd_si128(second), _mm_castpd_si128(first), 8));
  }
};
#else
// SSE2 versions
template<int Offset>
struct ei_palign_impl<Offset,Packet4f>
{
  EIGEN_STRONG_INLINE static void run(Packet4f& first, const Packet4f& second)
  {
    if (Offset==1)
    {
      first = _mm_move_ss(first,second);
      first = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(first),0x39));
    }
    else if (Offset==2)
    {
      first = _mm_movehl_ps(first,first);
      first = _mm_movelh_ps(first,second);
    }
    else if (Offset==3)
    {
      first = _mm_move_ss(first,second);
      first = _mm_shuffle_ps(first,second,0x93);
    }
  }
};

template<int Offset>
struct ei_palign_impl<Offset,Packet4i>
{
  EIGEN_STRONG_INLINE static void run(Packet4i& first, const Packet4i& second)
  {
    if (Offset==1)
    {
      first = _mm_castps_si128(_mm_move_ss(_mm_castsi128_ps(first),_mm_castsi128_ps(second)));
      first = _mm_shuffle_epi32(first,0x39);
    }
    else if (Offset==2)
    {
      first = _mm_castps_si128(_mm_movehl_ps(_mm_castsi128_ps(first),_mm_castsi128_ps(first)));
      first = _mm_castps_si128(_mm_movelh_ps(_mm_castsi128_ps(first),_mm_castsi128_ps(second)));
    }
    else if (Offset==3)
    {
      first = _mm_castps_si128(_mm_move_ss(_mm_castsi128_ps(first),_mm_castsi128_ps(second)));
      first = _mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(first),_mm_castsi128_ps(second),0x93));
    }
  }
};

template<int Offset>
struct ei_palign_impl<Offset,Packet2d>
{
  EIGEN_STRONG_INLINE static void run(Packet2d& first, const Packet2d& second)
  {
    if (Offset==1)
    {
      first = _mm_castps_pd(_mm_movehl_ps(_mm_castpd_ps(first),_mm_castpd_ps(first)));
      first = _mm_castps_pd(_mm_movelh_ps(_mm_castpd_ps(first),_mm_castpd_ps(second)));
    }
  }
};
#endif

#endif // EIGEN_PACKET_MATH_SSE_H
