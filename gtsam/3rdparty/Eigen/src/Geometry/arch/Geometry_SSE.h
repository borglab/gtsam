// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Rohit Garg <rpg.314@gmail.com>
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

#ifndef EIGEN_GEOMETRY_SSE_H
#define EIGEN_GEOMETRY_SSE_H

template<class Derived, class OtherDerived>
struct ei_quat_product<Architecture::SSE, Derived, OtherDerived, float, Aligned>
{
  inline static Quaternion<float> run(const QuaternionBase<Derived>& _a, const QuaternionBase<OtherDerived>& _b)
  {
    const __m128 mask = _mm_castsi128_ps(_mm_setr_epi32(0,0,0,0x80000000));
    Quaternion<float> res;
    __m128 a = _a.coeffs().template packet<Aligned>(0);
    __m128 b = _b.coeffs().template packet<Aligned>(0);
    __m128 flip1 = _mm_xor_ps(_mm_mul_ps(ei_vec4f_swizzle1(a,1,2,0,2),
                                         ei_vec4f_swizzle1(b,2,0,1,2)),mask);
    __m128 flip2 = _mm_xor_ps(_mm_mul_ps(ei_vec4f_swizzle1(a,3,3,3,1),
                                         ei_vec4f_swizzle1(b,0,1,2,1)),mask);
    ei_pstore(&res.x(),
              _mm_add_ps(_mm_sub_ps(_mm_mul_ps(a,ei_vec4f_swizzle1(b,3,3,3,3)),
                                    _mm_mul_ps(ei_vec4f_swizzle1(a,2,0,1,0),
                                               ei_vec4f_swizzle1(b,1,2,0,0))),
                         _mm_add_ps(flip1,flip2)));
    return res;
  }
};

template<typename VectorLhs,typename VectorRhs>
struct ei_cross3_impl<Architecture::SSE,VectorLhs,VectorRhs,float,true>
{
  inline static typename ei_plain_matrix_type<VectorLhs>::type
  run(const VectorLhs& lhs, const VectorRhs& rhs)
  {
    __m128 a = lhs.template packet<VectorLhs::Flags&AlignedBit ? Aligned : Unaligned>(0);
    __m128 b = rhs.template packet<VectorRhs::Flags&AlignedBit ? Aligned : Unaligned>(0);
    __m128 mul1=_mm_mul_ps(ei_vec4f_swizzle1(a,1,2,0,3),ei_vec4f_swizzle1(b,2,0,1,3));
    __m128 mul2=_mm_mul_ps(ei_vec4f_swizzle1(a,2,0,1,3),ei_vec4f_swizzle1(b,1,2,0,3));
    typename ei_plain_matrix_type<VectorLhs>::type res;
    ei_pstore(&res.x(),_mm_sub_ps(mul1,mul2));
    return res;
  }
};




template<class Derived, class OtherDerived>
struct ei_quat_product<Architecture::SSE, Derived, OtherDerived, double, Aligned>
{
  inline static Quaternion<double> run(const QuaternionBase<Derived>& _a, const QuaternionBase<OtherDerived>& _b)
  {
  const Packet2d mask = _mm_castsi128_pd(_mm_set_epi32(0x0,0x0,0x80000000,0x0));

  Quaternion<double> res;

  const double* a = _a.coeffs().data();
  Packet2d b_xy = _b.coeffs().template packet<Aligned>(0);
  Packet2d b_zw = _b.coeffs().template packet<Aligned>(2);
  Packet2d a_xx = ei_pset1<Packet2d>(a[0]);
  Packet2d a_yy = ei_pset1<Packet2d>(a[1]);
  Packet2d a_zz = ei_pset1<Packet2d>(a[2]);
  Packet2d a_ww = ei_pset1<Packet2d>(a[3]);

  // two temporaries:
  Packet2d t1, t2;

  /*
   * t1 = ww*xy + yy*zw
   * t2 = zz*xy - xx*zw
   * res.xy = t1 +/- swap(t2)
   */
  t1 = ei_padd(ei_pmul(a_ww, b_xy), ei_pmul(a_yy, b_zw));
  t2 = ei_psub(ei_pmul(a_zz, b_xy), ei_pmul(a_xx, b_zw));
#ifdef __SSE3__
  EIGEN_UNUSED_VARIABLE(mask)
  ei_pstore(&res.x(), _mm_addsub_pd(t1, ei_preverse(t2)));
#else
  ei_pstore(&res.x(), ei_padd(t1, ei_pxor(mask,ei_preverse(t2))));
#endif
  
  /*
   * t1 = ww*zw - yy*xy
   * t2 = zz*zw + xx*xy
   * res.zw = t1 -/+ swap(t2) = swap( swap(t1) +/- t2)
   */
  t1 = ei_psub(ei_pmul(a_ww, b_zw), ei_pmul(a_yy, b_xy));
  t2 = ei_padd(ei_pmul(a_zz, b_zw), ei_pmul(a_xx, b_xy));
#ifdef __SSE3__
  EIGEN_UNUSED_VARIABLE(mask)
  ei_pstore(&res.z(), ei_preverse(_mm_addsub_pd(ei_preverse(t1), t2)));
#else
  ei_pstore(&res.z(), ei_psub(t1, ei_pxor(mask,ei_preverse(t2))));
#endif

  return res;
}
};


#endif // EIGEN_GEOMETRY_SSE_H
