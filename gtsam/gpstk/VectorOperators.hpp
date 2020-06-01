#pragma ident "$Id$"



/**
 * @file VectorOperators.hpp
 * Vector operators, including arithmetic, trig, cross, RMS, etc
 */

#ifndef GPSTK_VECTOR_OPERATORS_HPP
#define GPSTK_VECTOR_OPERATORS_HPP

//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//  
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

namespace gpstk
{

 /** @addtogroup VectorGroup */
   //@{

#define VecBaseNewUnaryOperator(func) \
      /** performs func on each element of x, returning a new vector */ \
   template <class T, class BaseClass> \
   Vector<T> func(const ConstVectorBase<T, BaseClass>& x) \
      { \
         BaseClass toReturn(x.size()); \
         size_t i; for (i=0; i < x.size(); i++) toReturn[i] = func(x[i]); \
         return toReturn; \
      }

//   VecBaseNewUnaryOperator(-)
   VecBaseNewUnaryOperator(abs)
      VecBaseNewUnaryOperator(acos)
      VecBaseNewUnaryOperator(asin)
      VecBaseNewUnaryOperator(atan)
      VecBaseNewUnaryOperator(cos)
      VecBaseNewUnaryOperator(cosh)
      VecBaseNewUnaryOperator(exp)
      VecBaseNewUnaryOperator(log)
      VecBaseNewUnaryOperator(log10)
      VecBaseNewUnaryOperator(sinh)
      VecBaseNewUnaryOperator(sin)
      VecBaseNewUnaryOperator(sqrt)
      VecBaseNewUnaryOperator(tan)
      VecBaseNewUnaryOperator(tanh)

#define VecBaseNewBinaryOperator(func, retval) \
/** returns a retval with each element the result of l[i] func r[i] */ \
template <class T, class BaseClass, class BaseClass2> \
retval operator func(const ConstVectorBase<T, BaseClass>& l, \
            const ConstVectorBase<T, BaseClass2>& r) \
{ \
   if (l.size() != r.size()) \
   { \
      VectorException e("Unequal lengths vectors"); \
      GPSTK_THROW(e); \
   } \
   retval toReturn(l.size()); \
   size_t i; \
   for (i=0; i < l.size(); i++) toReturn[i] = l[i] func r[i]; \
   return toReturn; \
} \
/** returns a retval with each element the result of l[i] func (scalar)r */ \
template <class T, class BaseClass> \
retval operator func(const ConstVectorBase<T, BaseClass>& l, const T r) \
{ \
   retval toReturn(l.size()); \
   size_t i; \
   for (i=0; i < l.size(); i++) toReturn[i] = l[i] func r; \
   return toReturn; \
} \
/** returns a retval with each element the result of (scalar)l func r[i] */ \
template <class T, class BaseClass> \
retval operator func(const T l, const ConstVectorBase<T, BaseClass>& r) \
{ \
   retval toReturn(r.size()); \
   size_t i; \
   for (i=0; i < r.size(); i++) toReturn[i] = l func r[i]; \
   return toReturn; \
} 

      VecBaseNewBinaryOperator(*, Vector<T>)
      VecBaseNewBinaryOperator(/, Vector<T>)
      VecBaseNewBinaryOperator(%, Vector<T>)
      VecBaseNewBinaryOperator(+, Vector<T>)
      VecBaseNewBinaryOperator(-, Vector<T>)
      VecBaseNewBinaryOperator(^, Vector<T>)
      VecBaseNewBinaryOperator(&, Vector<T>)
      VecBaseNewBinaryOperator(|, Vector<T>)

      VecBaseNewBinaryOperator(==, Vector<bool>)
      VecBaseNewBinaryOperator(<, Vector<bool>)
      VecBaseNewBinaryOperator(>, Vector<bool>)
      VecBaseNewBinaryOperator(!=, Vector<bool>)
      VecBaseNewBinaryOperator(<=, Vector<bool>)
      VecBaseNewBinaryOperator(>=, Vector<bool>)

#define VecBaseNewBinaryTranscendentalOperator(func, retval) \
/** performs func between each element of l and r, returning a retval */ \
   template <class T, class BaseClass, class BaseClass2> \
   retval func(const ConstVectorBase<T, BaseClass>& l, \
               const ConstVectorBase<T, BaseClass2>& r) \
   { \
      retval toReturn(l.size()); \
      size_t i; \
      for (i=0; i < l.size(); i++) toReturn[i] = func(l[i], r[i]); \
      return toReturn; \
   } \
/** performs func between each element of l and (scalar)r, returning a retval */ \
template <class T, class BaseClass> \
retval func(const ConstVectorBase<T, BaseClass>& l, const T r) \
{ \
   retval toReturn(l.size()); \
   size_t i; \
   for (i=0; i < l.size(); i++) toReturn[i] = func(l[i], r); \
   return toReturn; \
} \
/** performs func between (scalar)l and each element of r, returning a retval */ \
template <class T, class BaseClass> \
retval func(const T l, const ConstVectorBase<T, BaseClass>& r) \
{ \
   retval toReturn(r.size()); \
   size_t i; \
   for (i=0; i < r.size(); i++) toReturn[i] = func(l, r[i]); \
   return toReturn; \
} 

      VecBaseNewBinaryTranscendentalOperator(atan, Vector<T>)
      VecBaseNewBinaryTranscendentalOperator(pow, Vector<T>)

/** finds the cross product between l and r */
      template <class T, class BaseClass, class BaseClass2> 
   Vector<T> cross(const ConstVectorBase<T, BaseClass>& l, 
                const ConstVectorBase<T, BaseClass2>& r) throw(VectorException)
{ 
   if ((l.size() != 3) && (r.size() != 3))
   {
      VectorException e("Cross product requires vectors of size 3");
      GPSTK_THROW(e);
   }
   BaseClass toReturn(3);
   toReturn[0] = l[1] * r[2] - l[2] * r[1];
   toReturn[1] = l[2] * r[0] - l[0] * r[2];
   toReturn[2] = l[0] * r[1] - l[1] * r[0];
   return toReturn;
} 

/** returns a new vector with the normalized version of l */
template <class T, class BaseClass>
Vector<T> normalize(const ConstVectorBase<T, BaseClass>& l) 
{ return l / norm(l); } 

/** returns the root-sum-square of the elements of l */
template <class T, class BaseClass>
T RSS(const ConstVectorBase<T, BaseClass>& l) 
{ return norm(l); } 

/** returns the root-mean-square of the elements of l */
template <class T, class BaseClass>
T RMS(const ConstVectorBase<T, BaseClass>& l) 
{ return norm(l)/SQRT(T(l.size())); } 

   //@}
 
}  // namespace

#endif


