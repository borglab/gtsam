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

#ifndef EIGEN_COMPLEX_SSE_H
#define EIGEN_COMPLEX_SSE_H

//---------- float ----------
struct Packet2cf
{
  EIGEN_STRONG_INLINE Packet2cf() {}
  EIGEN_STRONG_INLINE explicit Packet2cf(const __m128& a) : v(a) {}
  __m128  v;
};

template<> struct ei_packet_traits<std::complex<float> >  : ei_default_packet_traits
{
  typedef Packet2cf type;
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 1,
    size = 2,

    HasAdd    = 1,
    HasSub    = 1,
    HasMul    = 1,
    HasDiv    = 1,
    HasNegate = 1,
    HasAbs    = 0,
    HasAbs2   = 0,
    HasMin    = 0,
    HasMax    = 0,
    HasSetLinear = 0
  };
};

template<> struct ei_unpacket_traits<Packet2cf> { typedef std::complex<float> type; enum {size=2}; };

template<> EIGEN_STRONG_INLINE Packet2cf ei_padd<Packet2cf>(const Packet2cf& a, const Packet2cf& b) { return Packet2cf(_mm_add_ps(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet2cf ei_psub<Packet2cf>(const Packet2cf& a, const Packet2cf& b) { return Packet2cf(_mm_sub_ps(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet2cf ei_pnegate(const Packet2cf& a)
{
  const __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(0x80000000,0x80000000,0x80000000,0x80000000));
  return Packet2cf(_mm_xor_ps(a.v,mask));
}
template<> EIGEN_STRONG_INLINE Packet2cf ei_pconj(const Packet2cf& a)
{
  const __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(0x00000000,0x80000000,0x00000000,0x80000000));
  return Packet2cf(_mm_xor_ps(a.v,mask));
}

template<> EIGEN_STRONG_INLINE Packet2cf ei_pmul<Packet2cf>(const Packet2cf& a, const Packet2cf& b)
{
  // TODO optimize it for SSE3 and 4
  #ifdef EIGEN_VECTORIZE_SSE3
  return Packet2cf(_mm_addsub_ps(_mm_mul_ps(_mm_moveldup_ps(a.v), b.v),
                                 _mm_mul_ps(_mm_movehdup_ps(a.v),
                                            ei_vec4f_swizzle1(b.v, 1, 0, 3, 2))));
//   return Packet2cf(_mm_addsub_ps(_mm_mul_ps(ei_vec4f_swizzle1(a.v, 0, 0, 2, 2), b.v),
//                                  _mm_mul_ps(ei_vec4f_swizzle1(a.v, 1, 1, 3, 3),
//                                             ei_vec4f_swizzle1(b.v, 1, 0, 3, 2))));
  #else
  const __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(0x80000000,0x00000000,0x80000000,0x00000000));
  return Packet2cf(_mm_add_ps(_mm_mul_ps(ei_vec4f_swizzle1(a.v, 0, 0, 2, 2), b.v),
                              _mm_xor_ps(_mm_mul_ps(ei_vec4f_swizzle1(a.v, 1, 1, 3, 3),
                                                    ei_vec4f_swizzle1(b.v, 1, 0, 3, 2)), mask)));
  #endif
}

template<> EIGEN_STRONG_INLINE Packet2cf ei_pand   <Packet2cf>(const Packet2cf& a, const Packet2cf& b) { return Packet2cf(_mm_and_ps(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet2cf ei_por    <Packet2cf>(const Packet2cf& a, const Packet2cf& b) { return Packet2cf(_mm_or_ps(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet2cf ei_pxor   <Packet2cf>(const Packet2cf& a, const Packet2cf& b) { return Packet2cf(_mm_xor_ps(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet2cf ei_pandnot<Packet2cf>(const Packet2cf& a, const Packet2cf& b) { return Packet2cf(_mm_andnot_ps(a.v,b.v)); }

template<> EIGEN_STRONG_INLINE Packet2cf ei_pload <Packet2cf>(const std::complex<float>* from) { EIGEN_DEBUG_ALIGNED_LOAD return Packet2cf(ei_pload<Packet4f>(&ei_real_ref(*from))); }
template<> EIGEN_STRONG_INLINE Packet2cf ei_ploadu<Packet2cf>(const std::complex<float>* from) { EIGEN_DEBUG_UNALIGNED_LOAD return Packet2cf(ei_ploadu<Packet4f>(&ei_real_ref(*from))); }

template<> EIGEN_STRONG_INLINE void ei_pstore <std::complex<float> >(std::complex<float> *   to, const Packet2cf& from) { EIGEN_DEBUG_ALIGNED_STORE ei_pstore(&ei_real_ref(*to), from.v); }
template<> EIGEN_STRONG_INLINE void ei_pstoreu<std::complex<float> >(std::complex<float> *   to, const Packet2cf& from) { EIGEN_DEBUG_UNALIGNED_STORE ei_pstoreu(&ei_real_ref(*to), from.v); }

template<> EIGEN_STRONG_INLINE void ei_prefetch<std::complex<float> >(const std::complex<float> *   addr) { _mm_prefetch((const char*)(addr), _MM_HINT_T0); }

template<> EIGEN_STRONG_INLINE Packet2cf ei_pset1<Packet2cf>(const std::complex<float>&  from)
{
  Packet2cf res;
  res.v = _mm_loadl_pi(res.v, (const __m64*)&from);
  return Packet2cf(_mm_movelh_ps(res.v,res.v));
}

template<> EIGEN_STRONG_INLINE std::complex<float>  ei_pfirst<Packet2cf>(const Packet2cf& a)
{
  std::complex<float> res;
  _mm_storel_pi((__m64*)&res, a.v);
  return res;
}

template<> EIGEN_STRONG_INLINE Packet2cf ei_preverse(const Packet2cf& a) { return Packet2cf(_mm_castpd_ps(ei_preverse(_mm_castps_pd(a.v)))); }

template<> EIGEN_STRONG_INLINE std::complex<float> ei_predux<Packet2cf>(const Packet2cf& a)
{
  return ei_pfirst(Packet2cf(_mm_add_ps(a.v, _mm_movehl_ps(a.v,a.v))));
}

template<> EIGEN_STRONG_INLINE Packet2cf ei_preduxp<Packet2cf>(const Packet2cf* vecs)
{
  return Packet2cf(_mm_add_ps(_mm_movelh_ps(vecs[0].v,vecs[1].v), _mm_movehl_ps(vecs[1].v,vecs[0].v)));
}

template<> EIGEN_STRONG_INLINE std::complex<float> ei_predux_mul<Packet2cf>(const Packet2cf& a)
{
  return ei_pfirst(ei_pmul(a, Packet2cf(_mm_movehl_ps(a.v,a.v))));
}

template<int Offset>
struct ei_palign_impl<Offset,Packet2cf>
{
  EIGEN_STRONG_INLINE static void run(Packet2cf& first, const Packet2cf& second)
  {
    if (Offset==1)
    {
      first.v = _mm_movehl_ps(first.v, first.v);
      first.v = _mm_movelh_ps(first.v, second.v);
    }
  }
};

template<> struct ei_conj_helper<Packet2cf, Packet2cf, false,true>
{
  EIGEN_STRONG_INLINE Packet2cf pmadd(const Packet2cf& x, const Packet2cf& y, const Packet2cf& c) const
  { return ei_padd(pmul(x,y),c); }

  EIGEN_STRONG_INLINE Packet2cf pmul(const Packet2cf& a, const Packet2cf& b) const
  {
    #ifdef EIGEN_VECTORIZE_SSE3
    return ei_pmul(a, ei_pconj(b));
    #else
    const __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(0x00000000,0x80000000,0x00000000,0x80000000));
    return Packet2cf(_mm_add_ps(_mm_xor_ps(_mm_mul_ps(ei_vec4f_swizzle1(a.v, 0, 0, 2, 2), b.v), mask),
                                _mm_mul_ps(ei_vec4f_swizzle1(a.v, 1, 1, 3, 3),
                                           ei_vec4f_swizzle1(b.v, 1, 0, 3, 2))));
    #endif
  }
};

template<> struct ei_conj_helper<Packet2cf, Packet2cf, true,false>
{
  EIGEN_STRONG_INLINE Packet2cf pmadd(const Packet2cf& x, const Packet2cf& y, const Packet2cf& c) const
  { return ei_padd(pmul(x,y),c); }

  EIGEN_STRONG_INLINE Packet2cf pmul(const Packet2cf& a, const Packet2cf& b) const
  {
    #ifdef EIGEN_VECTORIZE_SSE3
    return ei_pmul(ei_pconj(a), b);
    #else
    const __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(0x00000000,0x80000000,0x00000000,0x80000000));
    return Packet2cf(_mm_add_ps(_mm_mul_ps(ei_vec4f_swizzle1(a.v, 0, 0, 2, 2), b.v),
                                _mm_xor_ps(_mm_mul_ps(ei_vec4f_swizzle1(a.v, 1, 1, 3, 3),
                                                      ei_vec4f_swizzle1(b.v, 1, 0, 3, 2)), mask)));
    #endif
  }
};

template<> struct ei_conj_helper<Packet2cf, Packet2cf, true,true>
{
  EIGEN_STRONG_INLINE Packet2cf pmadd(const Packet2cf& x, const Packet2cf& y, const Packet2cf& c) const
  { return ei_padd(pmul(x,y),c); }

  EIGEN_STRONG_INLINE Packet2cf pmul(const Packet2cf& a, const Packet2cf& b) const
  {
    #ifdef EIGEN_VECTORIZE_SSE3
    return ei_pconj(ei_pmul(a, b));
    #else
    const __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(0x00000000,0x80000000,0x00000000,0x80000000));
    return Packet2cf(_mm_sub_ps(_mm_xor_ps(_mm_mul_ps(ei_vec4f_swizzle1(a.v, 0, 0, 2, 2), b.v), mask),
                                _mm_mul_ps(ei_vec4f_swizzle1(a.v, 1, 1, 3, 3),
                                           ei_vec4f_swizzle1(b.v, 1, 0, 3, 2))));
    #endif
  }
};

template<> struct ei_conj_helper<Packet4f, Packet2cf, false,false>
{
  EIGEN_STRONG_INLINE Packet2cf pmadd(const Packet4f& x, const Packet2cf& y, const Packet2cf& c) const
  { return ei_padd(c, pmul(x,y)); }

  EIGEN_STRONG_INLINE Packet2cf pmul(const Packet4f& x, const Packet2cf& y) const
  { return Packet2cf(ei_pmul(x, y.v)); }
};

template<> struct ei_conj_helper<Packet2cf, Packet4f, false,false>
{
  EIGEN_STRONG_INLINE Packet2cf pmadd(const Packet2cf& x, const Packet4f& y, const Packet2cf& c) const
  { return ei_padd(c, pmul(x,y)); }

  EIGEN_STRONG_INLINE Packet2cf pmul(const Packet2cf& x, const Packet4f& y) const
  { return Packet2cf(ei_pmul(x.v, y)); }
};

template<> EIGEN_STRONG_INLINE Packet2cf ei_pdiv<Packet2cf>(const Packet2cf& a, const Packet2cf& b)
{
  // TODO optimize it for SSE3 and 4
  Packet2cf res = ei_conj_helper<Packet2cf,Packet2cf,false,true>().pmul(a,b);
  __m128 s = _mm_mul_ps(b.v,b.v);
  return Packet2cf(_mm_div_ps(res.v,_mm_add_ps(s,_mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(s), 0xb1)))));
}

EIGEN_STRONG_INLINE Packet2cf ei_pcplxflip/*<Packet2cf>*/(const Packet2cf& x)
{
  return Packet2cf(ei_vec4f_swizzle1(x.v, 1, 0, 3, 2));
}


//---------- double ----------
struct Packet1cd
{
  EIGEN_STRONG_INLINE Packet1cd() {}
  EIGEN_STRONG_INLINE explicit Packet1cd(const __m128d& a) : v(a) {}
  __m128d  v;
};

template<> struct ei_packet_traits<std::complex<double> >  : ei_default_packet_traits
{
  typedef Packet1cd type;
  enum {
    Vectorizable = 1,
    AlignedOnScalar = 0,
    size = 1,

    HasAdd    = 1,
    HasSub    = 1,
    HasMul    = 1,
    HasDiv    = 1,
    HasNegate = 1,
    HasAbs    = 0,
    HasAbs2   = 0,
    HasMin    = 0,
    HasMax    = 0,
    HasSetLinear = 0
  };
};

template<> struct ei_unpacket_traits<Packet1cd> { typedef std::complex<double> type; enum {size=1}; };

template<> EIGEN_STRONG_INLINE Packet1cd ei_padd<Packet1cd>(const Packet1cd& a, const Packet1cd& b) { return Packet1cd(_mm_add_pd(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_psub<Packet1cd>(const Packet1cd& a, const Packet1cd& b) { return Packet1cd(_mm_sub_pd(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_pnegate(const Packet1cd& a) { return Packet1cd(ei_pnegate(a.v)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_pconj(const Packet1cd& a)
{
  const __m128d mask = _mm_castsi128_pd(_mm_set_epi32(0x80000000,0x0,0x0,0x0));
  return Packet1cd(_mm_xor_pd(a.v,mask));
}

template<> EIGEN_STRONG_INLINE Packet1cd ei_pmul<Packet1cd>(const Packet1cd& a, const Packet1cd& b)
{
  // TODO optimize it for SSE3 and 4
  #ifdef EIGEN_VECTORIZE_SSE3
  return Packet1cd(_mm_addsub_pd(_mm_mul_pd(ei_vec2d_swizzle1(a.v, 0, 0), b.v),
                                 _mm_mul_pd(ei_vec2d_swizzle1(a.v, 1, 1),
                                            ei_vec2d_swizzle1(b.v, 1, 0))));
  #else
  const __m128d mask = _mm_castsi128_pd(_mm_set_epi32(0x0,0x0,0x80000000,0x0));
  return Packet1cd(_mm_add_pd(_mm_mul_pd(ei_vec2d_swizzle1(a.v, 0, 0), b.v),
                              _mm_xor_pd(_mm_mul_pd(ei_vec2d_swizzle1(a.v, 1, 1),
                                                    ei_vec2d_swizzle1(b.v, 1, 0)), mask)));
  #endif
}

template<> EIGEN_STRONG_INLINE Packet1cd ei_pand   <Packet1cd>(const Packet1cd& a, const Packet1cd& b) { return Packet1cd(_mm_and_pd(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_por    <Packet1cd>(const Packet1cd& a, const Packet1cd& b) { return Packet1cd(_mm_or_pd(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_pxor   <Packet1cd>(const Packet1cd& a, const Packet1cd& b) { return Packet1cd(_mm_xor_pd(a.v,b.v)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_pandnot<Packet1cd>(const Packet1cd& a, const Packet1cd& b) { return Packet1cd(_mm_andnot_pd(a.v,b.v)); }

// FIXME force unaligned load, this is a temporary fix
template<> EIGEN_STRONG_INLINE Packet1cd ei_pload <Packet1cd>(const std::complex<double>* from)
{ EIGEN_DEBUG_ALIGNED_LOAD return Packet1cd(ei_pload<Packet2d>((const double*)from)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_ploadu<Packet1cd>(const std::complex<double>* from)
{ EIGEN_DEBUG_UNALIGNED_LOAD return Packet1cd(ei_ploadu<Packet2d>((const double*)from)); }
template<> EIGEN_STRONG_INLINE Packet1cd ei_pset1<Packet1cd>(const std::complex<double>&  from)
{ /* here we really have to use unaligned loads :( */ return ei_ploadu<Packet1cd>(&from); }

// FIXME force unaligned store, this is a temporary fix
template<> EIGEN_STRONG_INLINE void ei_pstore <std::complex<double> >(std::complex<double> *   to, const Packet1cd& from) { EIGEN_DEBUG_ALIGNED_STORE ei_pstore((double*)to, from.v); }
template<> EIGEN_STRONG_INLINE void ei_pstoreu<std::complex<double> >(std::complex<double> *   to, const Packet1cd& from) { EIGEN_DEBUG_UNALIGNED_STORE ei_pstoreu((double*)to, from.v); }

template<> EIGEN_STRONG_INLINE void ei_prefetch<std::complex<double> >(const std::complex<double> *   addr) { _mm_prefetch((const char*)(addr), _MM_HINT_T0); }

template<> EIGEN_STRONG_INLINE std::complex<double>  ei_pfirst<Packet1cd>(const Packet1cd& a)
{
  EIGEN_ALIGN16 double res[2];
  _mm_store_pd(res, a.v);
  return std::complex<double>(res[0],res[1]);
}

template<> EIGEN_STRONG_INLINE Packet1cd ei_preverse(const Packet1cd& a) { return a; }

template<> EIGEN_STRONG_INLINE std::complex<double> ei_predux<Packet1cd>(const Packet1cd& a)
{
  return ei_pfirst(a);
}

template<> EIGEN_STRONG_INLINE Packet1cd ei_preduxp<Packet1cd>(const Packet1cd* vecs)
{
  return vecs[0];
}

template<> EIGEN_STRONG_INLINE std::complex<double> ei_predux_mul<Packet1cd>(const Packet1cd& a)
{
  return ei_pfirst(a);
}

template<int Offset>
struct ei_palign_impl<Offset,Packet1cd>
{
  EIGEN_STRONG_INLINE static void run(Packet1cd& /*first*/, const Packet1cd& /*second*/)
  {
    // FIXME is it sure we never have to align a Packet1cd?
    // Even though a std::complex<double> has 16 bytes, it is not necessarily aligned on a 16 bytes boundary...
  }
};

template<> struct ei_conj_helper<Packet1cd, Packet1cd, false,true>
{
  EIGEN_STRONG_INLINE Packet1cd pmadd(const Packet1cd& x, const Packet1cd& y, const Packet1cd& c) const
  { return ei_padd(pmul(x,y),c); }

  EIGEN_STRONG_INLINE Packet1cd pmul(const Packet1cd& a, const Packet1cd& b) const
  {
    #ifdef EIGEN_VECTORIZE_SSE3
    return ei_pmul(a, ei_pconj(b));
    #else
    const __m128d mask = _mm_castsi128_pd(_mm_set_epi32(0x80000000,0x0,0x0,0x0));
    return Packet1cd(_mm_add_pd(_mm_xor_pd(_mm_mul_pd(ei_vec2d_swizzle1(a.v, 0, 0), b.v), mask),
                                _mm_mul_pd(ei_vec2d_swizzle1(a.v, 1, 1),
                                           ei_vec2d_swizzle1(b.v, 1, 0))));
    #endif
  }
};

template<> struct ei_conj_helper<Packet1cd, Packet1cd, true,false>
{
  EIGEN_STRONG_INLINE Packet1cd pmadd(const Packet1cd& x, const Packet1cd& y, const Packet1cd& c) const
  { return ei_padd(pmul(x,y),c); }

  EIGEN_STRONG_INLINE Packet1cd pmul(const Packet1cd& a, const Packet1cd& b) const
  {
    #ifdef EIGEN_VECTORIZE_SSE3
    return ei_pmul(ei_pconj(a), b);
    #else
    const __m128d mask = _mm_castsi128_pd(_mm_set_epi32(0x80000000,0x0,0x0,0x0));
    return Packet1cd(_mm_add_pd(_mm_mul_pd(ei_vec2d_swizzle1(a.v, 0, 0), b.v),
                                _mm_xor_pd(_mm_mul_pd(ei_vec2d_swizzle1(a.v, 1, 1),
                                                      ei_vec2d_swizzle1(b.v, 1, 0)), mask)));
    #endif
  }
};

template<> struct ei_conj_helper<Packet1cd, Packet1cd, true,true>
{
  EIGEN_STRONG_INLINE Packet1cd pmadd(const Packet1cd& x, const Packet1cd& y, const Packet1cd& c) const
  { return ei_padd(pmul(x,y),c); }

  EIGEN_STRONG_INLINE Packet1cd pmul(const Packet1cd& a, const Packet1cd& b) const
  {
    #ifdef EIGEN_VECTORIZE_SSE3
    return ei_pconj(ei_pmul(a, b));
    #else
    const __m128d mask = _mm_castsi128_pd(_mm_set_epi32(0x80000000,0x0,0x0,0x0));
    return Packet1cd(_mm_sub_pd(_mm_xor_pd(_mm_mul_pd(ei_vec2d_swizzle1(a.v, 0, 0), b.v), mask),
                                _mm_mul_pd(ei_vec2d_swizzle1(a.v, 1, 1),
                                           ei_vec2d_swizzle1(b.v, 1, 0))));
    #endif
  }
};

template<> struct ei_conj_helper<Packet2d, Packet1cd, false,false>
{
  EIGEN_STRONG_INLINE Packet1cd pmadd(const Packet2d& x, const Packet1cd& y, const Packet1cd& c) const
  { return ei_padd(c, pmul(x,y)); }

  EIGEN_STRONG_INLINE Packet1cd pmul(const Packet2d& x, const Packet1cd& y) const
  { return Packet1cd(ei_pmul(x, y.v)); }
};

template<> struct ei_conj_helper<Packet1cd, Packet2d, false,false>
{
  EIGEN_STRONG_INLINE Packet1cd pmadd(const Packet1cd& x, const Packet2d& y, const Packet1cd& c) const
  { return ei_padd(c, pmul(x,y)); }

  EIGEN_STRONG_INLINE Packet1cd pmul(const Packet1cd& x, const Packet2d& y) const
  { return Packet1cd(ei_pmul(x.v, y)); }
};

template<> EIGEN_STRONG_INLINE Packet1cd ei_pdiv<Packet1cd>(const Packet1cd& a, const Packet1cd& b)
{
  // TODO optimize it for SSE3 and 4
  Packet1cd res = ei_conj_helper<Packet1cd,Packet1cd,false,true>().pmul(a,b);
  __m128d s = _mm_mul_pd(b.v,b.v);
  return Packet1cd(_mm_div_pd(res.v, _mm_add_pd(s,_mm_shuffle_pd(s, s, 0x1))));
}

EIGEN_STRONG_INLINE Packet1cd ei_pcplxflip/*<Packet1cd>*/(const Packet1cd& x)
{
  return Packet1cd(ei_preverse(x.v));
}

#endif // EIGEN_COMPLEX_SSE_H
