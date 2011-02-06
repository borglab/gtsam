// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
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

#ifndef EIGEN_MATHFUNCTIONS_H
#define EIGEN_MATHFUNCTIONS_H

/** \internal \struct ei_global_math_functions_filtering_base
  *
  * What it does:
  * Defines a typedef 'type' as follows:
  * - if type T has a member typedef Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl, then
  *   ei_global_math_functions_filtering_base<T>::type is a typedef for it.
  * - otherwise, ei_global_math_functions_filtering_base<T>::type is a typedef for T.
  *
  * How it's used:
  * To allow to defined the global math functions (like ei_sin...) in certain cases, like the Array expressions.
  * When you do ei_sin(array1+array2), the object array1+array2 has a complicated expression type, all what you want to know
  * is that it inherits ArrayBase. So we implement a partial specialization of ei_sin_impl for ArrayBase<Derived>.
  * So we must make sure to use ei_sin_impl<ArrayBase<Derived> > and not ei_sin_impl<Derived>, otherwise our partial specialization
  * won't be used. How does ei_sin know that? That's exactly what ei_global_math_functions_filtering_base tells it.
  *
  * How it's implemented:
  * SFINAE in the style of enable_if. Highly susceptible of breaking compilers. With GCC, it sure does work, but if you replace
  * the typename dummy by an integer template parameter, it doesn't work anymore!
  */

template<typename T, typename dummy = void>
struct ei_global_math_functions_filtering_base
{
  typedef T type;
};

template<typename T> struct ei_always_void { typedef void type; };

template<typename T>
struct ei_global_math_functions_filtering_base
  <T,
   typename ei_always_void<typename T::Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl>::type
  >
{
  typedef typename T::Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl type;
};

#define EIGEN_MATHFUNC_IMPL(func, scalar) ei_##func##_impl<typename ei_global_math_functions_filtering_base<scalar>::type>
#define EIGEN_MATHFUNC_RETVAL(func, scalar) typename ei_##func##_retval<typename ei_global_math_functions_filtering_base<scalar>::type>::type


/****************************************************************************
* Implementation of ei_real                                                 *
****************************************************************************/

template<typename Scalar>
struct ei_real_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return x;
  }
};

template<typename RealScalar>
struct ei_real_impl<std::complex<RealScalar> >
{
  static inline RealScalar run(const std::complex<RealScalar>& x)
  {
    return std::real(x);
  }
};

template<typename Scalar>
struct ei_real_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(real, Scalar) ei_real(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(real, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_imag                                                 *
****************************************************************************/

template<typename Scalar>
struct ei_imag_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar&)
  {
    return RealScalar(0);
  }
};

template<typename RealScalar>
struct ei_imag_impl<std::complex<RealScalar> >
{
  static inline RealScalar run(const std::complex<RealScalar>& x)
  {
    return std::imag(x);
  }
};

template<typename Scalar>
struct ei_imag_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(imag, Scalar) ei_imag(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(imag, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_real_ref                                             *
****************************************************************************/

template<typename Scalar>
struct ei_real_ref_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar& run(Scalar& x)
  {
    return reinterpret_cast<RealScalar*>(&x)[0];
  }
  static inline const RealScalar& run(const Scalar& x)
  {
    return reinterpret_cast<const RealScalar*>(&x)[0];
  }
};

template<typename Scalar>
struct ei_real_ref_retval
{
  typedef typename NumTraits<Scalar>::Real & type;
};

template<typename Scalar>
inline typename ei_makeconst< EIGEN_MATHFUNC_RETVAL(real_ref, Scalar) >::type ei_real_ref(const Scalar& x)
{
  return ei_real_ref_impl<Scalar>::run(x);
}

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(real_ref, Scalar) ei_real_ref(Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(real_ref, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_imag_ref                                             *
****************************************************************************/

template<typename Scalar, bool IsComplex>
struct ei_imag_ref_default_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar& run(Scalar& x)
  {
    return reinterpret_cast<RealScalar*>(&x)[1];
  }
  static inline const RealScalar& run(const Scalar& x)
  {
    return reinterpret_cast<RealScalar*>(&x)[1];
  }
};

template<typename Scalar>
struct ei_imag_ref_default_impl<Scalar, false>
{
  static inline Scalar run(Scalar&)
  {
    return Scalar(0);
  }
  static inline const Scalar run(const Scalar&)
  {
    return Scalar(0);
  }
};

template<typename Scalar>
struct ei_imag_ref_impl : ei_imag_ref_default_impl<Scalar, NumTraits<Scalar>::IsComplex> {};

template<typename Scalar>
struct ei_imag_ref_retval
{
  typedef typename NumTraits<Scalar>::Real & type;
};

template<typename Scalar>
inline typename ei_makeconst< EIGEN_MATHFUNC_RETVAL(imag_ref, Scalar) >::type ei_imag_ref(const Scalar& x)
{
  return ei_imag_ref_impl<Scalar>::run(x);
}

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(imag_ref, Scalar) ei_imag_ref(Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(imag_ref, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_conj                                                 *
****************************************************************************/

template<typename Scalar>
struct ei_conj_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return x;
  }
};

template<typename RealScalar>
struct ei_conj_impl<std::complex<RealScalar> >
{
  static inline std::complex<RealScalar> run(const std::complex<RealScalar>& x)
  {
    return std::conj(x);
  }
};

template<typename Scalar>
struct ei_conj_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(conj, Scalar) ei_conj(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(conj, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_abs                                                  *
****************************************************************************/

template<typename Scalar>
struct ei_abs_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return std::abs(x);
  }
};

template<typename Scalar>
struct ei_abs_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(abs, Scalar) ei_abs(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(abs, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_abs2                                                 *
****************************************************************************/

template<typename Scalar>
struct ei_abs2_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return x*x;
  }
};

template<typename RealScalar>
struct ei_abs2_impl<std::complex<RealScalar> >
{
  static inline RealScalar run(const std::complex<RealScalar>& x)
  {
    return std::norm(x);
  }
};

template<typename Scalar>
struct ei_abs2_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(abs2, Scalar) ei_abs2(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(abs2, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_norm1                                                *
****************************************************************************/

template<typename Scalar, bool IsComplex>
struct ei_norm1_default_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return ei_abs(ei_real(x)) + ei_abs(ei_imag(x));
  }
};

template<typename Scalar>
struct ei_norm1_default_impl<Scalar, false>
{
  static inline Scalar run(const Scalar& x)
  {
    return ei_abs(x);
  }
};

template<typename Scalar>
struct ei_norm1_impl : ei_norm1_default_impl<Scalar, NumTraits<Scalar>::IsComplex> {};

template<typename Scalar>
struct ei_norm1_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(norm1, Scalar) ei_norm1(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(norm1, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_hypot                                                *
****************************************************************************/

template<typename Scalar>
struct ei_hypot_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x, const Scalar& y)
  {
    RealScalar _x = ei_abs(x);
    RealScalar _y = ei_abs(y);
    RealScalar p = std::max(_x, _y);
    RealScalar q = std::min(_x, _y);
    RealScalar qp = q/p;
    return p * ei_sqrt(RealScalar(1) + qp*qp);
  }
};

template<typename Scalar>
struct ei_hypot_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(hypot, Scalar) ei_hypot(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(hypot, Scalar)::run(x, y);
}

/****************************************************************************
* Implementation of ei_cast                                                 *
****************************************************************************/

template<typename OldType, typename NewType>
struct ei_cast_impl
{
  static inline NewType run(const OldType& x)
  {
    return static_cast<NewType>(x);
  }
};

// here, for once, we're plainly returning NewType: we don't want ei_cast to do weird things.

template<typename OldType, typename NewType>
inline NewType ei_cast(const OldType& x)
{
  return ei_cast_impl<OldType, NewType>::run(x);
}

/****************************************************************************
* Implementation of ei_sqrt                                                 *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct ei_sqrt_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::sqrt(x);
  }
};

template<typename Scalar>
struct ei_sqrt_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct ei_sqrt_impl : ei_sqrt_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_sqrt_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(sqrt, Scalar) ei_sqrt(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(sqrt, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_exp                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct ei_exp_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::exp(x);
  }
};

template<typename Scalar>
struct ei_exp_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct ei_exp_impl : ei_exp_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_exp_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(exp, Scalar) ei_exp(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(exp, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_cos                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct ei_cos_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::cos(x);
  }
};

template<typename Scalar>
struct ei_cos_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct ei_cos_impl : ei_cos_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_cos_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(cos, Scalar) ei_cos(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(cos, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_sin                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct ei_sin_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::sin(x);
  }
};

template<typename Scalar>
struct ei_sin_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct ei_sin_impl : ei_sin_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_sin_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(sin, Scalar) ei_sin(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(sin, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_log                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct ei_log_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::log(x);
  }
};

template<typename Scalar>
struct ei_log_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct ei_log_impl : ei_log_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_log_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(log, Scalar) ei_log(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(log, Scalar)::run(x);
}

/****************************************************************************
* Implementation of ei_atan2                                                *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct ei_atan2_default_impl
{
  typedef Scalar retval;
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return std::atan2(x, y);
  }
};

template<typename Scalar>
struct ei_atan2_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&, const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct ei_atan2_impl : ei_atan2_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_atan2_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(atan2, Scalar) ei_atan2(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(atan2, Scalar)::run(x, y);
}

/****************************************************************************
* Implementation of ei_pow                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct ei_pow_default_impl
{
  typedef Scalar retval;
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return std::pow(x, y);
  }
};

template<typename Scalar>
struct ei_pow_default_impl<Scalar, true>
{
  static inline Scalar run(Scalar x, Scalar y)
  {
    Scalar res = 1;
    ei_assert(!NumTraits<Scalar>::IsSigned || y >= 0);
    if(y & 1) res *= x;
    y >>= 1;
    while(y)
    {
      x *= x;
      if(y&1) res *= x;
      y >>= 1;
    }
    return res;
  }
};

template<typename Scalar>
struct ei_pow_impl : ei_pow_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_pow_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(pow, Scalar) ei_pow(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(pow, Scalar)::run(x, y);
}

/****************************************************************************
* Implementation of ei_random                                               *
****************************************************************************/

template<typename Scalar,
         bool IsComplex,
         bool IsInteger>
struct ei_random_default_impl {};

template<typename Scalar>
struct ei_random_impl : ei_random_default_impl<Scalar, NumTraits<Scalar>::IsComplex, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct ei_random_retval
{
  typedef Scalar type;
};

template<typename Scalar> inline EIGEN_MATHFUNC_RETVAL(random, Scalar) ei_random(const Scalar& x, const Scalar& y);
template<typename Scalar> inline EIGEN_MATHFUNC_RETVAL(random, Scalar) ei_random();

template<typename Scalar>
struct ei_random_default_impl<Scalar, false, false>
{
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return x + (y-x) * Scalar(std::rand()) / float(RAND_MAX);
  }
  static inline Scalar run()
  {
    return run(Scalar(NumTraits<Scalar>::IsSigned ? -1 : 0), Scalar(1));
  }
};

template<typename Scalar>
struct ei_random_default_impl<Scalar, false, true>
{
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return x + Scalar((y-x+1) * (std::rand() / (RAND_MAX + typename NumTraits<Scalar>::NonInteger(1))));
  }
  static inline Scalar run()
  {
    return run(Scalar(NumTraits<Scalar>::IsSigned ? -10 : 0), Scalar(10));
  }
};

template<typename Scalar>
struct ei_random_default_impl<Scalar, true, false>
{
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return Scalar(ei_random(ei_real(x), ei_real(y)),
                  ei_random(ei_imag(x), ei_imag(y)));
  }
  static inline Scalar run()
  {
    typedef typename NumTraits<Scalar>::Real RealScalar;
    return Scalar(ei_random<RealScalar>(), ei_random<RealScalar>());
  }
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(random, Scalar) ei_random(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(random, Scalar)::run(x, y);
}

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(random, Scalar) ei_random()
{
  return EIGEN_MATHFUNC_IMPL(random, Scalar)::run();
}

/****************************************************************************
* Implementation of fuzzy comparisons                                       *
****************************************************************************/

template<typename Scalar,
         bool IsComplex,
         bool IsInteger>
struct ei_scalar_fuzzy_default_impl {};

template<typename Scalar>
struct ei_scalar_fuzzy_default_impl<Scalar, false, false>
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  template<typename OtherScalar>
  static inline bool isMuchSmallerThan(const Scalar& x, const OtherScalar& y, const RealScalar& prec)
  {
    return ei_abs(x) <= ei_abs(y) * prec;
  }
  static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar& prec)
  {
    return ei_abs(x - y) <= std::min(ei_abs(x), ei_abs(y)) * prec;
  }
  static inline bool isApproxOrLessThan(const Scalar& x, const Scalar& y, const RealScalar& prec)
  {
    return x <= y || isApprox(x, y, prec);
  }
};

template<typename Scalar>
struct ei_scalar_fuzzy_default_impl<Scalar, false, true>
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  template<typename OtherScalar>
  static inline bool isMuchSmallerThan(const Scalar& x, const Scalar&, const RealScalar&)
  {
    return x == Scalar(0);
  }
  static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar&)
  {
    return x == y;
  }
  static inline bool isApproxOrLessThan(const Scalar& x, const Scalar& y, const RealScalar&)
  {
    return x <= y;
  }
};

template<typename Scalar>
struct ei_scalar_fuzzy_default_impl<Scalar, true, false>
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  template<typename OtherScalar>
  static inline bool isMuchSmallerThan(const Scalar& x, const OtherScalar& y, const RealScalar& prec)
  {
    return ei_abs2(x) <= ei_abs2(y) * prec * prec;
  }
  static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar& prec)
  {
    return ei_abs2(x - y) <= std::min(ei_abs2(x), ei_abs2(y)) * prec * prec;
  }
};

template<typename Scalar>
struct ei_scalar_fuzzy_impl : ei_scalar_fuzzy_default_impl<Scalar, NumTraits<Scalar>::IsComplex, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar, typename OtherScalar>
inline bool ei_isMuchSmallerThan(const Scalar& x, const OtherScalar& y,
                                   typename NumTraits<Scalar>::Real precision = NumTraits<Scalar>::dummy_precision())
{
  return ei_scalar_fuzzy_impl<Scalar>::template isMuchSmallerThan<OtherScalar>(x, y, precision);
}

template<typename Scalar>
inline bool ei_isApprox(const Scalar& x, const Scalar& y,
                          typename NumTraits<Scalar>::Real precision = NumTraits<Scalar>::dummy_precision())
{
  return ei_scalar_fuzzy_impl<Scalar>::isApprox(x, y, precision);
}

template<typename Scalar>
inline bool ei_isApproxOrLessThan(const Scalar& x, const Scalar& y,
                                    typename NumTraits<Scalar>::Real precision = NumTraits<Scalar>::dummy_precision())
{
  return ei_scalar_fuzzy_impl<Scalar>::isApproxOrLessThan(x, y, precision);
}

/******************************************
***  The special case of the  bool type ***
******************************************/

template<> struct ei_random_impl<bool>
{
  static inline bool run()
  {
    return ei_random<int>(0,1)==0 ? false : true;
  }
};

template<> struct ei_scalar_fuzzy_impl<bool>
{
  static inline bool isApprox(bool x, bool y, bool)
  {
    return x == y;
  }
};

#endif // EIGEN_MATHFUNCTIONS_H
