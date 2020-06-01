#pragma ident "$Id$"



/**
 * @file VectorBaseOperators.hpp
 * Vector base class operators, including I/O, min(), dot(), comparisons, etc
 */

#ifndef GPSTK_VECTOR_BASE_OPERATORS_HPP
#define GPSTK_VECTOR_BASE_OPERATORS_HPP

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

#include <fstream> // for copyfmt
#include <vector>
#include <iomanip>

// to solve the conflict with windows.h
#if defined(min)
#undef min
#endif

#if defined(max)
#undef max
#endif

namespace gpstk
{

 /** @addtogroup VectorGroup */
 //@{
 
/** Output operator for ConstVectorBase objects */
   template <class T, class E>
   std::ostream& operator<<(std::ostream& s, const ConstVectorBase<T, E>& a) 
   {
      std::ofstream savefmt;
      savefmt.copyfmt(s);
      size_t i;
      for (i=0; i< a.size(); i++) {
         s << std::setw(1) << ' ';
         s.copyfmt(savefmt);
         s << a[i];
      }
      return s;
   }

/** Returns the sum of the elements of the vector */
   template <class T, class BaseClass>
   inline T sum(const ConstVectorBase<T, BaseClass>& l)
   { 
      T total(0);
      size_t i;
      for (i = 0; i < l.size(); i++)
         total += l[i];
      return total;
   }

/** Return the element with smallest absolute value in the vector */
   template <class T, class BaseClass>
   inline T minabs(const ConstVectorBase<T, BaseClass>& l) throw (VectorException)
   { 
      if (l.size() == 0)
      {
         VectorException e("Can't find the minabs of an empty vector");
         GPSTK_THROW(e);
      }
      T min = l[0];
      size_t i;
      for (i = 1; i < l.size(); i++)
         if (ABS(l[i]) < ABS(min)) 
            min = l[i];
      return min;
   }

/** Returns the smallest element of the vector */
   template <class T, class BaseClass>
   inline T min(const ConstVectorBase<T, BaseClass>& l) throw (VectorException)
   { 
      if (l.size() == 0)
      {
         VectorException e("Can't find the min of an empty vector");
         GPSTK_THROW(e);
      }
      T min = l[0];
      size_t i;
      for (i = 1; i < l.size(); i++)
         if (l[i] < min) 
            min = l[i];
      return min;
   }

/** Return the element with largest absolute value in the vector */
   template <class T, class BaseClass>
   inline T maxabs(const ConstVectorBase<T, BaseClass>& l)
   {
      if (l.size() == 0)
      {
         VectorException e("Can't find the maxabs of an empty vector");
         GPSTK_THROW(e);
      }
      T max = l[0];
      size_t i;
      for (i = 1; i < l.size(); i++)
         if (ABS(l[i]) > ABS(max)) 
            max = l[i];
      return max;
   }

/** Returns the largest element of the vector */
   template <class T, class BaseClass>
   inline T max(const ConstVectorBase<T, BaseClass>& l)
   {
      if (l.size() == 0)
      {
         VectorException e("Can't find the max of an empty vector");
         GPSTK_THROW(e);
      }
      T max = l[0];
      size_t i;
      for (i = 1; i < l.size(); i++)
         if (l[i] > max) 
            max = l[i];
      return max;
   }

/** returns the dot product of the two vectors */
   template <class T, class BaseClass, class BaseClass2> 
   inline T dot(const ConstVectorBase<T, BaseClass>& l, 
         const ConstVectorBase<T, BaseClass2>& r) 
   {
      T sum(0);
      size_t i,n=(l.size() > r.size() ? r.size() : l.size());
      for (i = 0; i < n; i++)
      {
         sum += l[i] * r[i];
      }
      return sum;
   } 

/** returns the dot product of a vector and a scalar */
   template <class T, class BaseClass> 
   inline T dot(const ConstVectorBase<T, BaseClass>& l, const T r) 
   {
      T sum(0);
      size_t i;
      for (i = 0; i < l.size(); i++)
      {
         sum += l[i] * r;
      }
      return sum;
   }

/** returns the dot product of a scalar and a vector */
   template <class T, class BaseClass> 
   inline T dot(const T l, const ConstVectorBase<T, BaseClass>& r) 
   {
      T sum(0);
      size_t i;
      for (i = 0; i < r.size(); i++)
      {
         sum += l * r[i];
      }
      return sum;
   }

/** returns the norm of the vector */
   template <class T, class BaseClass> 
   inline T norm(const ConstVectorBase<T, BaseClass>& v) 
   {
      T mag=T(0);
      if(v.size()==0) return mag;
      mag = ABS(v(0));
      for(size_t i=1; i<v.size(); i++) {
         if(mag > ABS(v(i)))
            mag *= SQRT(T(1)+(v(i)/mag)*(v(i)/mag));
         else if(ABS(v(i)) > mag)
            mag = ABS(v(i))*SQRT(T(1)+(mag/v(i))*(mag/v(i)));
         else
            mag *= SQRT(T(2));
      }
      return mag;
   } 

/** return the Minkowski product of two vectors of length 4. */
   template <class T, class BaseClass, class BaseClass2> 
   inline T Minkowski(const ConstVectorBase<T, BaseClass>& v, 
         const ConstVectorBase<T, BaseClass2>& w) 
   {
      if (v.size()<4 || w.size()<4)
      {
         VectorException e("Minkowski requires vector length 4");
         GPSTK_THROW(e);
      }
      return (v(0)*w(0)+v(1)*w(1)+v(2)*w(2)-v(3)*w(3));
   }

/** finds the cosine between the two vectors */
   template <class T, class BaseClass1, class BaseClass2>
   inline T cosVec(const ConstVectorBase<T, BaseClass1>& a,
                const ConstVectorBase<T, BaseClass2>& b)
   {
      T na=norm(a), nb=norm(b), c(0);
      size_t i,n=(b.size() > a.size() ? a.size() : b.size());
      for(i=0; i<n; i++) c += (a(i)/na)*(b(i)/nb);
      return c;
   }

// shortwire equality operators - compares each individual
// element in the vector but returns one 'true' or 'false'
// for the whole comparison.  note this only compares
// the smaller of the size of the two vectors
#define VecShortwireComparisonOperator(func, op) \
/** Performs op on each element of l and r, returning false if any fail */ \
template <class T, class BaseClass, class BaseClass2>  \
inline bool func(const ConstVectorBase<T, BaseClass>& l,  \
       const ConstVectorBase<T, BaseClass2>& r)  \
{  \
   size_t len = (l.size() < r.size()) ? l.size() : r.size(); \
   size_t i; \
   for(i = 0; i < len; i++) \
      if ( !(l[i] op r[i]) ) \
         return false; \
   return true; \
}  \
/** Performs op on each element of l to r, returning false if any fail */ \
template <class T, class BaseClass>  \
inline bool func(const ConstVectorBase<T, BaseClass>& l, const T r)  \
{ \
   size_t len = l.size(); \
   size_t i; \
   for(i = 0; i < len; i++) \
      if ( !(l[i] op r) ) \
         return false; \
   return true; \
} \
/** Performs op on each element of r to l, returning false if any fail */ \
template <class T, class BaseClass>  \
inline bool func(const T l, const ConstVectorBase<T, BaseClass>& r)  \
{  \
   size_t len = r.size(); \
   size_t i; \
   for(i = 0; i < len; i++) \
      if ( !(l op r[i]) ) \
         return false; \
   return true; \
}

VecShortwireComparisonOperator(eq, ==)
   VecShortwireComparisonOperator(ne, !=)
   VecShortwireComparisonOperator(lt, <)
   VecShortwireComparisonOperator(gt, >)
   VecShortwireComparisonOperator(ge, >=)
   VecShortwireComparisonOperator(le, <=)

 //@}

}  // namespace gpstk
 
#endif // GPSTK_VECTOR_BASE_OPERATORS_HPP
