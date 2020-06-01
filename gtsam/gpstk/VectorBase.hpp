#pragma ident "$Id$"



/**
 * @file VectorBase.hpp
 * Base Vector class
 */
 
#ifndef GPSTK_VECTOR_BASE_HPP
#define GPSTK_VECTOR_BASE_HPP

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

#include <valarray>
#include "Exception.hpp"

#include "MathBase.hpp"

namespace gpstk
{
/** @addtogroup VectorGroup */
   //@{
 
/// An exception thrown when there's a problem with a vector
/// @ingroup exceptiongroup
NEW_EXCEPTION_CLASS(VectorException, gpstk::Exception);

/*
 * There were two overriding philosophies to the vector and matrix classes:
 *
 * The concept of "const" and "reference" (i.e. changable) vector
 * and matrix types both exist, so that any Const* type could not be altered
 * while Ref* types can.  This allowed one to add the slice classes that 
 * let you use a subset of a vector and modify (or not if it's const) the
 * original vector.  Furthermore, it allowed slice and non-slice classes
 * to interoperate through the ConstVectorBase or ConstMatrixBase classes, so, for 
 * example, operator* only needs to be written in terms of ConstVectorBase to
 * work correctly with Vector, VectorSlice and ConstVectorSlice.
 * 
 * Remember that a slice MUST refer to a vector or matrix; you cannot have
 * a slice independent of a base vector or matrix.
 *
 * In the future:
 *
 * - Change the math operators to template expressions.
 * - Add general slices and diagonal matrix slices.
 * - Make range checking more consistent.
 * - Make operator= and copy constructors consistent between const and
 *   non-const versions.
 * - Reevaluate the need for default slice constructors...?
 * - find a way for LUD and SVD to use the template type of the parameters
 *   rather than specified when the object is created.
 * - come up with a policy for when zeroize() will be used before results
 *   are returned.
 *
 * @warning MSVC cant deal with cmath header.  
 * Changes to accomidate this may break complex!
 */

/**
 * A base class for a vector that does not allow modification of the internal
 * vector.  BaseClass is the base class that implements the vector.
 */
   template <class T, class BaseClass>
   class ConstVectorBase
   {
   public:
         /// Constructor
      explicit ConstVectorBase() {}

         /// Returns the size of the base class.
      size_t size() const
         { return static_cast<const BaseClass*>(this)->size(); }
         /// returns the element at index i
      T operator[] (size_t i) const 
         { return constVectorRef(i); }
         /// returns the element at index i
      T operator() (size_t i) const 
         { return constVectorRef(i); }

   protected:
         /// returns the element at index i by calling the base class's operator[]
      inline T constVectorRef(size_t i) const
         throw(VectorException)
         {
            const BaseClass& b = static_cast<const BaseClass&>(*this);
#ifdef RANGECHECK
            if (i >= b.size())
            {
               VectorException e("Invalid ConstVectorBase index");
               GPSTK_THROW(e);
            }
#endif
            return b[i];
         }
   };

      /// a class to hold the static members of RefVectorBase. Static members
      /// in template classes have to be initialized on a PER TEMPLATE
      /// basis - this gets around that problem.
   class RefVectorBaseHelper
   {
   public:
         /// used with zeroize(), any number below this value will become 0.
         /// this variable can be assigned any value.
      static double zeroTolerance;
   };

/**
 * A vector base class that allows modification of the internal representation.
 */
   template <class T, class BaseClass>
   class RefVectorBase : public ConstVectorBase<T, BaseClass>,
                      public RefVectorBaseHelper
   {
   public:
         /// constructor
      explicit RefVectorBase() {}
         /// returns a modifiable version of the element at index i.
      T& operator[] (size_t i) 
         { return vecRef(i); }
         /// returns a modifiable version of the element at index i.
      T& operator() (size_t i) 
         { return vecRef(i); }
         /// Any value in the vector with absolute value below
         /// zeroTolerance is set to zero.
      BaseClass& zeroize()
         {
            BaseClass& me = static_cast<BaseClass&>(*this); 
            size_t i;
            for (i = 0; i < me.size(); i++)
               if (ABS(me[i]) < zeroTolerance)
                  me[i] = T(0);
            return me;
         }

#define VecBaseArrayAssignMacroDontCheckRange(func) \
   BaseClass& me = static_cast<BaseClass&>(*this); \
   size_t i; for (i=0; i < me.size(); i++) { \
      me[i] func x[i]; \
   } \
   return me;

#ifdef RANGECHECK
#define VecBaseArrayAssignMacro(func) \
   BaseClass& me = static_cast<BaseClass&>(*this); \
   if (x.size() != me.size()) \
      { \
         VectorException e("Unequal lengths for vectors"); \
         GPSTK_THROW(e); \
      } \
   size_t i; for (i=0; i < me.size(); i++) me[i] func x[i]; \
   return me;
#else
#define VecBaseArrayAssignMacro(func) \
VecBaseArrayAssignMacroDontCheckRange(func)
#endif

#define VecBaseAtomicAssignMacro(func) \
   BaseClass& me = static_cast<BaseClass&>(*this); \
   size_t i; for (i=0; i < me.size(); i++) me[i] func x; \
   return me;

#define VecBaseNewAssignOperator(funcName, op) \
            /** Performs op on (*this).size() elements of (*this) from x */ \
   template <class E> BaseClass& funcName(const ConstVectorBase<T, E>& x) \
      { VecBaseArrayAssignMacro(op) } \
            /** Performs op on (*this).size() elements of (*this) from x */ \
   BaseClass& funcName(const std::valarray<T>& x) \
      { VecBaseArrayAssignMacro(op) } \
            /** Performs op on (*this).size() elements of (*this) from x */ \
   BaseClass& funcName(const T* x) \
      { VecBaseArrayAssignMacroDontCheckRange(op) } \
            /** Performs op on (*this).size() elements of (*this) from x */ \
   BaseClass& funcName(T x) \
      { VecBaseAtomicAssignMacro(op) }

         /** 
          * Remember that operator= is NOT inherited. Derived classes can
          * use assignFrom to initialize values from a copy constructor or
          * their own operator= rather than explicitly copying them. 
          */
      VecBaseNewAssignOperator(assignFrom, =);
      VecBaseNewAssignOperator(operator+=, +=);
      VecBaseNewAssignOperator(operator-=, -=);
      VecBaseNewAssignOperator(operator*=, *=);
      VecBaseNewAssignOperator(operator/=, /=);

      //   // unary minus: multiplies each element of this vector by -1.
      //BaseClass& operator-()
      //   {
      //      const T x=T(-1);
      //      VecBaseAtomicAssignMacro(*=);
      //   }
      // unary minus must not return an l-value

      /// unary minus: multiplies each element in this matrix by -1.
      const BaseClass operator-() const
      {
         const T x=T(-1);
         BaseClass me = static_cast<BaseClass>(*this);
         size_t i;
         for (i=0; i < me.size(); i++) me(i) *= x;
         return me;
      }

   protected:
         /// Returns a modifiable object at index i.
      inline T& vecRef(size_t i) 
         throw(VectorException)
         {
            BaseClass& b = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
            if (i >= b.size())
            {
               VectorException e("Invalid VectorBase index");
               GPSTK_THROW(e);
            }
#endif
            return b[i]; 
         }
   };

/**
 * A base class that represents a subset of a vector.
 */
   template <class BaseClass>
   class VectorSliceBase
   {
   public:
         /// constructor
      explicit VectorSliceBase() {}

         /// the number of elements in the slice.
      size_t size() const
         { return static_cast<const BaseClass*>(this)->size(); }
         /// the start index in the BaseClass vector for this slice.
      size_t start() const
         { return static_cast<const BaseClass*>(this)->start(); }
         /// How many elements separate the i'th element from the i+1'th element.
      size_t stride() const
         { return static_cast<const BaseClass*>(this)->stride(); }

   protected:
         /// Given the size of the source vector, checks that the slice is valid.
      inline void vecSliceCheck(size_t sourceSize) const
         throw(VectorException)
         {
#ifdef RANGECHECK
               // sanity checks...
            if ( (start() >= sourceSize) ||
                 ((start() + (size() - 1) * stride()) >= sourceSize) )
            {
               VectorException e("Invalid range for slice");
               GPSTK_THROW(e);
            }
#endif
         }
   };

/** 
 * A vector slice base class that doesn't allow modification of the 
 * internal elements. 
 */
   template <class T, class BaseClass>
   class ConstVectorSliceBase : public VectorSliceBase<BaseClass>,
                             public ConstVectorBase<T, BaseClass>
   {
public:
   explicit ConstVectorSliceBase() {}
};

/** 
 * A vector slice base class that does allow modification of the 
 * internal elements. 
 */
template <class T, class BaseClass>
class RefVectorSliceBase : public VectorSliceBase<BaseClass>,
                        public RefVectorBase<T, BaseClass>
{
public:
   explicit RefVectorSliceBase() {}
};

//@}

}  // namespace gpstk

#include "VectorBaseOperators.hpp"

#endif //GPSTK_VECTOR_BASE_HPP
