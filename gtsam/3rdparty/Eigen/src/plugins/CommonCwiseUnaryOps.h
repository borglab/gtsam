// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
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

// This file is a base class plugin containing common coefficient wise functions.

#ifndef EIGEN_PARSED_BY_DOXYGEN

/** \internal Represents a scalar multiple of an expression */
typedef CwiseUnaryOp<ei_scalar_multiple_op<Scalar>, Derived> ScalarMultipleReturnType;
/** \internal Represents a quotient of an expression by a scalar*/
typedef CwiseUnaryOp<ei_scalar_quotient1_op<Scalar>, Derived> ScalarQuotient1ReturnType;
/** \internal the return type of conjugate() */
typedef typename ei_meta_if<NumTraits<Scalar>::IsComplex,
                    const CwiseUnaryOp<ei_scalar_conjugate_op<Scalar>, Derived>,
                    const Derived&
                  >::ret ConjugateReturnType;
/** \internal the return type of real() const */
typedef typename ei_meta_if<NumTraits<Scalar>::IsComplex,
                    const CwiseUnaryOp<ei_scalar_real_op<Scalar>, Derived>,
                    const Derived&
                  >::ret RealReturnType;
/** \internal the return type of real() */
typedef typename ei_meta_if<NumTraits<Scalar>::IsComplex,
                    CwiseUnaryView<ei_scalar_real_ref_op<Scalar>, Derived>,
                    Derived&
                  >::ret NonConstRealReturnType;
/** \internal the return type of imag() const */
typedef CwiseUnaryOp<ei_scalar_imag_op<Scalar>, Derived> ImagReturnType;
/** \internal the return type of imag() */
typedef CwiseUnaryView<ei_scalar_imag_ref_op<Scalar>, Derived> NonConstImagReturnType;

#endif // not EIGEN_PARSED_BY_DOXYGEN

/** \returns an expression of the opposite of \c *this
  */
inline const CwiseUnaryOp<ei_scalar_opposite_op<typename ei_traits<Derived>::Scalar>,Derived>
operator-() const { return derived(); }


/** \returns an expression of \c *this scaled by the scalar factor \a scalar */
inline const ScalarMultipleReturnType
operator*(const Scalar& scalar) const
{
  return CwiseUnaryOp<ei_scalar_multiple_op<Scalar>, Derived>
    (derived(), ei_scalar_multiple_op<Scalar>(scalar));
}

#ifdef EIGEN_PARSED_BY_DOXYGEN
const ScalarMultipleReturnType operator*(const RealScalar& scalar) const;
#endif

/** \returns an expression of \c *this divided by the scalar value \a scalar */
inline const CwiseUnaryOp<ei_scalar_quotient1_op<typename ei_traits<Derived>::Scalar>, Derived>
operator/(const Scalar& scalar) const
{
  return CwiseUnaryOp<ei_scalar_quotient1_op<Scalar>, Derived>
    (derived(), ei_scalar_quotient1_op<Scalar>(scalar));
}

/** Overloaded for efficient real matrix times complex scalar value */
inline const CwiseUnaryOp<ei_scalar_multiple2_op<Scalar,std::complex<Scalar> >, Derived>
operator*(const std::complex<Scalar>& scalar) const
{
  return CwiseUnaryOp<ei_scalar_multiple2_op<Scalar,std::complex<Scalar> >, Derived>
    (*static_cast<const Derived*>(this), ei_scalar_multiple2_op<Scalar,std::complex<Scalar> >(scalar));
}

inline friend const ScalarMultipleReturnType
operator*(const Scalar& scalar, const StorageBaseType& matrix)
{ return matrix*scalar; }

inline friend const CwiseUnaryOp<ei_scalar_multiple2_op<Scalar,std::complex<Scalar> >, Derived>
operator*(const std::complex<Scalar>& scalar, const StorageBaseType& matrix)
{ return matrix*scalar; }

/** \returns an expression of *this with the \a Scalar type casted to
  * \a NewScalar.
  *
  * The template parameter \a NewScalar is the type we are casting the scalars to.
  *
  * \sa class CwiseUnaryOp
  */
template<typename NewType>
typename ei_cast_return_type<Derived,const CwiseUnaryOp<ei_scalar_cast_op<typename ei_traits<Derived>::Scalar, NewType>, Derived> >::type
cast() const
{
  return derived();
}

/** \returns an expression of the complex conjugate of \c *this.
  *
  * \sa adjoint() */
inline ConjugateReturnType
conjugate() const
{
  return ConjugateReturnType(derived());
}

/** \returns a read-only expression of the real part of \c *this.
  *
  * \sa imag() */
inline RealReturnType
real() const { return derived(); }

/** \returns an read-only expression of the imaginary part of \c *this.
  *
  * \sa real() */
inline const ImagReturnType
imag() const { return derived(); }

/** \returns an expression of a custom coefficient-wise unary operator \a func of *this
  *
  * The template parameter \a CustomUnaryOp is the type of the functor
  * of the custom unary operator.
  *
  * Example:
  * \include class_CwiseUnaryOp.cpp
  * Output: \verbinclude class_CwiseUnaryOp.out
  *
  * \sa class CwiseUnaryOp, class CwiseBinaryOp
  */
template<typename CustomUnaryOp>
inline const CwiseUnaryOp<CustomUnaryOp, Derived>
unaryExpr(const CustomUnaryOp& func = CustomUnaryOp()) const
{
  return CwiseUnaryOp<CustomUnaryOp, Derived>(derived(), func);
}

/** \returns an expression of a custom coefficient-wise unary operator \a func of *this
  *
  * The template parameter \a CustomUnaryOp is the type of the functor
  * of the custom unary operator.
  *
  * Example:
  * \include class_CwiseUnaryOp.cpp
  * Output: \verbinclude class_CwiseUnaryOp.out
  *
  * \sa class CwiseUnaryOp, class CwiseBinaryOp
  */
template<typename CustomViewOp>
inline const CwiseUnaryView<CustomViewOp, Derived>
unaryViewExpr(const CustomViewOp& func = CustomViewOp()) const
{
  return CwiseUnaryView<CustomViewOp, Derived>(derived(), func);
}

/** \returns a non const expression of the real part of \c *this.
  *
  * \sa imag() */
inline NonConstRealReturnType
real() { return derived(); }

/** \returns a non const expression of the imaginary part of \c *this.
  *
  * \sa real() */
inline NonConstImagReturnType
imag() { return derived(); }
