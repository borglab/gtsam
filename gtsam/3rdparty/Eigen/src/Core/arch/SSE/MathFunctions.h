// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2007 Julien Pommier
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

/* The sin, cos, exp, and log functions of this file come from
 * Julien Pommier's sse math library: http://gruntthepeon.free.fr/ssemath/
 */

#ifndef EIGEN_MATH_FUNCTIONS_SSE_H
#define EIGEN_MATH_FUNCTIONS_SSE_H

template<> EIGEN_DEFINE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS EIGEN_UNUSED
Packet4f ei_plog<Packet4f>(const Packet4f& _x)
{
  Packet4f x = _x;
  _EIGEN_DECLARE_CONST_Packet4f(1 , 1.0f);
  _EIGEN_DECLARE_CONST_Packet4f(half, 0.5f);
  _EIGEN_DECLARE_CONST_Packet4i(0x7f, 0x7f);

  _EIGEN_DECLARE_CONST_Packet4f_FROM_INT(inv_mant_mask, ~0x7f800000);

  /* the smallest non denormalized float number */
  _EIGEN_DECLARE_CONST_Packet4f_FROM_INT(min_norm_pos,  0x00800000);

  /* natural logarithm computed for 4 simultaneous float
    return NaN for x <= 0
  */
  _EIGEN_DECLARE_CONST_Packet4f(cephes_SQRTHF, 0.707106781186547524f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p0, 7.0376836292E-2f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p1, - 1.1514610310E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p2, 1.1676998740E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p3, - 1.2420140846E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p4, + 1.4249322787E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p5, - 1.6668057665E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p6, + 2.0000714765E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p7, - 2.4999993993E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_p8, + 3.3333331174E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_q1, -2.12194440e-4f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_log_q2, 0.693359375f);


  Packet4i emm0;

  Packet4f invalid_mask = _mm_cmple_ps(x, _mm_setzero_ps());

  x = ei_pmax(x, ei_p4f_min_norm_pos);  /* cut off denormalized stuff */
  emm0 = _mm_srli_epi32(_mm_castps_si128(x), 23);

  /* keep only the fractional part */
  x = _mm_and_ps(x, ei_p4f_inv_mant_mask);
  x = _mm_or_ps(x, ei_p4f_half);

  emm0 = _mm_sub_epi32(emm0, ei_p4i_0x7f);
  Packet4f e = ei_padd(_mm_cvtepi32_ps(emm0), ei_p4f_1);

  /* part2:
     if( x < SQRTHF ) {
       e -= 1;
       x = x + x - 1.0;
     } else { x = x - 1.0; }
  */
  Packet4f mask = _mm_cmplt_ps(x, ei_p4f_cephes_SQRTHF);
  Packet4f tmp = _mm_and_ps(x, mask);
  x = ei_psub(x, ei_p4f_1);
  e = ei_psub(e, _mm_and_ps(ei_p4f_1, mask));
  x = ei_padd(x, tmp);

  Packet4f x2 = ei_pmul(x,x);
  Packet4f x3 = ei_pmul(x2,x);

  Packet4f y, y1, y2;
  y  = ei_pmadd(ei_p4f_cephes_log_p0, x, ei_p4f_cephes_log_p1);
  y1 = ei_pmadd(ei_p4f_cephes_log_p3, x, ei_p4f_cephes_log_p4);
  y2 = ei_pmadd(ei_p4f_cephes_log_p6, x, ei_p4f_cephes_log_p7);
  y  = ei_pmadd(y , x, ei_p4f_cephes_log_p2);
  y1 = ei_pmadd(y1, x, ei_p4f_cephes_log_p5);
  y2 = ei_pmadd(y2, x, ei_p4f_cephes_log_p8);
  y = ei_pmadd(y, x3, y1);
  y = ei_pmadd(y, x3, y2);
  y = ei_pmul(y, x3);

  y1 = ei_pmul(e, ei_p4f_cephes_log_q1);
  tmp = ei_pmul(x2, ei_p4f_half);
  y = ei_padd(y, y1);
  x = ei_psub(x, tmp);
  y2 = ei_pmul(e, ei_p4f_cephes_log_q2);
  x = ei_padd(x, y);
  x = ei_padd(x, y2);
  return _mm_or_ps(x, invalid_mask); // negative arg will be NAN
}

template<> EIGEN_DEFINE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS EIGEN_UNUSED
Packet4f ei_pexp<Packet4f>(const Packet4f& _x)
{
  Packet4f x = _x;
  _EIGEN_DECLARE_CONST_Packet4f(1 , 1.0f);
  _EIGEN_DECLARE_CONST_Packet4f(half, 0.5f);
  _EIGEN_DECLARE_CONST_Packet4i(0x7f, 0x7f);


  _EIGEN_DECLARE_CONST_Packet4f(exp_hi, 88.3762626647949f);
  _EIGEN_DECLARE_CONST_Packet4f(exp_lo, -88.3762626647949f);

  _EIGEN_DECLARE_CONST_Packet4f(cephes_LOG2EF, 1.44269504088896341f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_C1, 0.693359375f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_C2, -2.12194440e-4f);

  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_p0, 1.9875691500E-4f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_p1, 1.3981999507E-3f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_p2, 8.3334519073E-3f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_p3, 4.1665795894E-2f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_p4, 1.6666665459E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_exp_p5, 5.0000001201E-1f);

  Packet4f tmp = _mm_setzero_ps(), fx;
  Packet4i emm0;

  // clamp x
  x = ei_pmax(ei_pmin(x, ei_p4f_exp_hi), ei_p4f_exp_lo);

  /* express exp(x) as exp(g + n*log(2)) */
  fx = ei_pmadd(x, ei_p4f_cephes_LOG2EF, ei_p4f_half);

  /* how to perform a floorf with SSE: just below */
  emm0 = _mm_cvttps_epi32(fx);
  tmp  = _mm_cvtepi32_ps(emm0);
  /* if greater, substract 1 */
  Packet4f mask = _mm_cmpgt_ps(tmp, fx);
  mask = _mm_and_ps(mask, ei_p4f_1);
  fx = ei_psub(tmp, mask);

  tmp = ei_pmul(fx, ei_p4f_cephes_exp_C1);
  Packet4f z = ei_pmul(fx, ei_p4f_cephes_exp_C2);
  x = ei_psub(x, tmp);
  x = ei_psub(x, z);

  z = ei_pmul(x,x);

  Packet4f y = ei_p4f_cephes_exp_p0;
  y = ei_pmadd(y, x, ei_p4f_cephes_exp_p1);
  y = ei_pmadd(y, x, ei_p4f_cephes_exp_p2);
  y = ei_pmadd(y, x, ei_p4f_cephes_exp_p3);
  y = ei_pmadd(y, x, ei_p4f_cephes_exp_p4);
  y = ei_pmadd(y, x, ei_p4f_cephes_exp_p5);
  y = ei_pmadd(y, z, x);
  y = ei_padd(y, ei_p4f_1);

  /* build 2^n */
  emm0 = _mm_cvttps_epi32(fx);
  emm0 = _mm_add_epi32(emm0, ei_p4i_0x7f);
  emm0 = _mm_slli_epi32(emm0, 23);
  return ei_pmul(y, _mm_castsi128_ps(emm0));
}

/* evaluation of 4 sines at onces, using SSE2 intrinsics.

   The code is the exact rewriting of the cephes sinf function.
   Precision is excellent as long as x < 8192 (I did not bother to
   take into account the special handling they have for greater values
   -- it does not return garbage for arguments over 8192, though, but
   the extra precision is missing).

   Note that it is such that sinf((float)M_PI) = 8.74e-8, which is the
   surprising but correct result.
*/

template<> EIGEN_DEFINE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS EIGEN_UNUSED
Packet4f ei_psin<Packet4f>(const Packet4f& _x)
{
  Packet4f x = _x;
  _EIGEN_DECLARE_CONST_Packet4f(1 , 1.0f);
  _EIGEN_DECLARE_CONST_Packet4f(half, 0.5f);

  _EIGEN_DECLARE_CONST_Packet4i(1, 1);
  _EIGEN_DECLARE_CONST_Packet4i(not1, ~1);
  _EIGEN_DECLARE_CONST_Packet4i(2, 2);
  _EIGEN_DECLARE_CONST_Packet4i(4, 4);

  _EIGEN_DECLARE_CONST_Packet4f_FROM_INT(sign_mask, 0x80000000);

  _EIGEN_DECLARE_CONST_Packet4f(minus_cephes_DP1,-0.78515625f);
  _EIGEN_DECLARE_CONST_Packet4f(minus_cephes_DP2, -2.4187564849853515625e-4f);
  _EIGEN_DECLARE_CONST_Packet4f(minus_cephes_DP3, -3.77489497744594108e-8f);
  _EIGEN_DECLARE_CONST_Packet4f(sincof_p0, -1.9515295891E-4f);
  _EIGEN_DECLARE_CONST_Packet4f(sincof_p1,  8.3321608736E-3f);
  _EIGEN_DECLARE_CONST_Packet4f(sincof_p2, -1.6666654611E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(coscof_p0,  2.443315711809948E-005f);
  _EIGEN_DECLARE_CONST_Packet4f(coscof_p1, -1.388731625493765E-003f);
  _EIGEN_DECLARE_CONST_Packet4f(coscof_p2,  4.166664568298827E-002f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_FOPI, 1.27323954473516f); // 4 / M_PI

  Packet4f xmm1, xmm2 = _mm_setzero_ps(), xmm3, sign_bit, y;

  Packet4i emm0, emm2;
  sign_bit = x;
  /* take the absolute value */
  x = ei_pabs(x);

  /* take the modulo */

  /* extract the sign bit (upper one) */
  sign_bit = _mm_and_ps(sign_bit, ei_p4f_sign_mask);

  /* scale by 4/Pi */
  y = ei_pmul(x, ei_p4f_cephes_FOPI);

  /* store the integer part of y in mm0 */
  emm2 = _mm_cvttps_epi32(y);
  /* j=(j+1) & (~1) (see the cephes sources) */
  emm2 = _mm_add_epi32(emm2, ei_p4i_1);
  emm2 = _mm_and_si128(emm2, ei_p4i_not1);
  y = _mm_cvtepi32_ps(emm2);
  /* get the swap sign flag */
  emm0 = _mm_and_si128(emm2, ei_p4i_4);
  emm0 = _mm_slli_epi32(emm0, 29);
  /* get the polynom selection mask
     there is one polynom for 0 <= x <= Pi/4
     and another one for Pi/4<x<=Pi/2

     Both branches will be computed.
  */
  emm2 = _mm_and_si128(emm2, ei_p4i_2);
  emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());

  Packet4f swap_sign_bit = _mm_castsi128_ps(emm0);
  Packet4f poly_mask = _mm_castsi128_ps(emm2);
  sign_bit = _mm_xor_ps(sign_bit, swap_sign_bit);

  /* The magic pass: "Extended precision modular arithmetic"
     x = ((x - y * DP1) - y * DP2) - y * DP3; */
  xmm1 = ei_pmul(y, ei_p4f_minus_cephes_DP1);
  xmm2 = ei_pmul(y, ei_p4f_minus_cephes_DP2);
  xmm3 = ei_pmul(y, ei_p4f_minus_cephes_DP3);
  x = ei_padd(x, xmm1);
  x = ei_padd(x, xmm2);
  x = ei_padd(x, xmm3);

  /* Evaluate the first polynom  (0 <= x <= Pi/4) */
  y = ei_p4f_coscof_p0;
  Packet4f z = _mm_mul_ps(x,x);

  y = ei_pmadd(y, z, ei_p4f_coscof_p1);
  y = ei_pmadd(y, z, ei_p4f_coscof_p2);
  y = ei_pmul(y, z);
  y = ei_pmul(y, z);
  Packet4f tmp = ei_pmul(z, ei_p4f_half);
  y = ei_psub(y, tmp);
  y = ei_padd(y, ei_p4f_1);

  /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

  Packet4f y2 = ei_p4f_sincof_p0;
  y2 = ei_pmadd(y2, z, ei_p4f_sincof_p1);
  y2 = ei_pmadd(y2, z, ei_p4f_sincof_p2);
  y2 = ei_pmul(y2, z);
  y2 = ei_pmul(y2, x);
  y2 = ei_padd(y2, x);

  /* select the correct result from the two polynoms */
  y2 = _mm_and_ps(poly_mask, y2);
  y = _mm_andnot_ps(poly_mask, y);
  y = _mm_or_ps(y,y2);
  /* update the sign */
  return _mm_xor_ps(y, sign_bit);
}

/* almost the same as ei_psin */
template<> EIGEN_DEFINE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS EIGEN_UNUSED
Packet4f ei_pcos<Packet4f>(const Packet4f& _x)
{
  Packet4f x = _x;
  _EIGEN_DECLARE_CONST_Packet4f(1 , 1.0f);
  _EIGEN_DECLARE_CONST_Packet4f(half, 0.5f);

  _EIGEN_DECLARE_CONST_Packet4i(1, 1);
  _EIGEN_DECLARE_CONST_Packet4i(not1, ~1);
  _EIGEN_DECLARE_CONST_Packet4i(2, 2);
  _EIGEN_DECLARE_CONST_Packet4i(4, 4);

  _EIGEN_DECLARE_CONST_Packet4f(minus_cephes_DP1,-0.78515625f);
  _EIGEN_DECLARE_CONST_Packet4f(minus_cephes_DP2, -2.4187564849853515625e-4f);
  _EIGEN_DECLARE_CONST_Packet4f(minus_cephes_DP3, -3.77489497744594108e-8f);
  _EIGEN_DECLARE_CONST_Packet4f(sincof_p0, -1.9515295891E-4f);
  _EIGEN_DECLARE_CONST_Packet4f(sincof_p1,  8.3321608736E-3f);
  _EIGEN_DECLARE_CONST_Packet4f(sincof_p2, -1.6666654611E-1f);
  _EIGEN_DECLARE_CONST_Packet4f(coscof_p0,  2.443315711809948E-005f);
  _EIGEN_DECLARE_CONST_Packet4f(coscof_p1, -1.388731625493765E-003f);
  _EIGEN_DECLARE_CONST_Packet4f(coscof_p2,  4.166664568298827E-002f);
  _EIGEN_DECLARE_CONST_Packet4f(cephes_FOPI, 1.27323954473516f); // 4 / M_PI

  Packet4f xmm1, xmm2 = _mm_setzero_ps(), xmm3, y;
  Packet4i emm0, emm2;

  x = ei_pabs(x);

  /* scale by 4/Pi */
  y = ei_pmul(x, ei_p4f_cephes_FOPI);

  /* get the integer part of y */
  emm2 = _mm_cvttps_epi32(y);
  /* j=(j+1) & (~1) (see the cephes sources) */
  emm2 = _mm_add_epi32(emm2, ei_p4i_1);
  emm2 = _mm_and_si128(emm2, ei_p4i_not1);
  y = _mm_cvtepi32_ps(emm2);

  emm2 = _mm_sub_epi32(emm2, ei_p4i_2);

  /* get the swap sign flag */
  emm0 = _mm_andnot_si128(emm2, ei_p4i_4);
  emm0 = _mm_slli_epi32(emm0, 29);
  /* get the polynom selection mask */
  emm2 = _mm_and_si128(emm2, ei_p4i_2);
  emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());

  Packet4f sign_bit = _mm_castsi128_ps(emm0);
  Packet4f poly_mask = _mm_castsi128_ps(emm2);

  /* The magic pass: "Extended precision modular arithmetic"
     x = ((x - y * DP1) - y * DP2) - y * DP3; */
  xmm1 = ei_pmul(y, ei_p4f_minus_cephes_DP1);
  xmm2 = ei_pmul(y, ei_p4f_minus_cephes_DP2);
  xmm3 = ei_pmul(y, ei_p4f_minus_cephes_DP3);
  x = ei_padd(x, xmm1);
  x = ei_padd(x, xmm2);
  x = ei_padd(x, xmm3);

  /* Evaluate the first polynom  (0 <= x <= Pi/4) */
  y = ei_p4f_coscof_p0;
  Packet4f z = ei_pmul(x,x);

  y = ei_pmadd(y,z,ei_p4f_coscof_p1);
  y = ei_pmadd(y,z,ei_p4f_coscof_p2);
  y = ei_pmul(y, z);
  y = ei_pmul(y, z);
  Packet4f tmp = _mm_mul_ps(z, ei_p4f_half);
  y = ei_psub(y, tmp);
  y = ei_padd(y, ei_p4f_1);

  /* Evaluate the second polynom  (Pi/4 <= x <= 0) */
  Packet4f y2 = ei_p4f_sincof_p0;
  y2 = ei_pmadd(y2, z, ei_p4f_sincof_p1);
  y2 = ei_pmadd(y2, z, ei_p4f_sincof_p2);
  y2 = ei_pmul(y2, z);
  y2 = ei_pmadd(y2, x, x);

  /* select the correct result from the two polynoms */
  y2 = _mm_and_ps(poly_mask, y2);
  y  = _mm_andnot_ps(poly_mask, y);
  y  = _mm_or_ps(y,y2);

  /* update the sign */
  return _mm_xor_ps(y, sign_bit);
}

// This is based on Quake3's fast inverse square root.
// For detail see here: http://www.beyond3d.com/content/articles/8/
template<> EIGEN_DEFINE_FUNCTION_ALLOWING_MULTIPLE_DEFINITIONS EIGEN_UNUSED
Packet4f ei_psqrt<Packet4f>(const Packet4f& _x)
{
  Packet4f half = ei_pmul(_x, ei_pset1<Packet4f>(.5f));

  /* select only the inverse sqrt of non-zero inputs */
  Packet4f non_zero_mask = _mm_cmpgt_ps(_x, ei_pset1<Packet4f>(std::numeric_limits<float>::epsilon()));
  Packet4f x = _mm_and_ps(non_zero_mask, _mm_rsqrt_ps(_x));

  x = ei_pmul(x, ei_psub(ei_pset1<Packet4f>(1.5f), ei_pmul(half, ei_pmul(x,x))));
  return ei_pmul(_x,x);
}

#endif // EIGEN_MATH_FUNCTIONS_SSE_H
