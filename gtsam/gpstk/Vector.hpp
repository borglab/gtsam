#pragma ident "$Id$"



/**
 * @file Vector.hpp
 * Classes for Vector, both constant and modifiable
 */

#ifndef GPSTK_VECTOR_HPP
#define GPSTK_VECTOR_HPP

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

#include <limits>
#include <vector>
#include "VectorBase.hpp"

namespace gpstk
{
 /** @addtogroup VectorGroup */
   //@{

// forward declaration
   template <class T> class VectorSlice;

/**
 * This class pretty much duplicates std::valarray<T> except it's fully
 * STL container compliant.  Remember that operators +=, -=, *= and /=
 * are provided by RefVectorBase.
 * 
 * @sa matvectest.cpp for examples
 */
   template <class T>
   class Vector : public RefVectorBase<T, Vector<T> >
   {
   public:
         /// STL value type
      typedef T value_type;
         /// STL reference type
      typedef T& reference;
         /// STL const reference type
      typedef const T& const_reference;
         /// STL iterator type
      typedef T* iterator;
         /// STL const iterator type
      typedef const T* const_iterator;

         /// Default constructor
      Vector() : v(NULL), s(0) {}
         /// Constructor given an initial size.
      Vector(size_t siz) : s(siz)
            //: v(new T[siz]), s(siz)
         {
            v = new T[siz];
            if(!v) {
               VectorException e("Vector(size_t) failed to allocate");
               GPSTK_THROW(e);
            }
         }
         /**
          * Constructor given an initial size and default value for all elements.
          */
      Vector(size_t siz, const T defaultValue) : s(siz)
            //: v(new T[siz]), s(siz)
         {
            v = new T[siz];
            if(!v) {
               VectorException e("Vector<T>(size_t, const T) failed to allocate");
               GPSTK_THROW(e);
            }
            this->assignFrom(defaultValue);
         }
         /**
          * Copy constructor from a ConstVectorBase type.
          */
      template <class E>
      Vector(const ConstVectorBase<T, E>& r) : s(r.size())
            //: v(new T[r.size()]), s(r.size())
         {
            v = new T[r.size()];
            if(!v) {
               VectorException e("Vector<T>(ConstVectorBase) failed to allocate");
               GPSTK_THROW(e);
            }
            this->assignFrom(r);
         }
         /**
          * Copy constructor.
          */
      Vector(const Vector& r) : s(r.s)
            //: v(new T[r.s]), s(r.s)
         {
            v = new T[r.s];
            if(!v) {
               VectorException e("Vector(Vector) failed to allocate");
               GPSTK_THROW(e);
            }
            this->assignFrom(r);
         }
         /**
          * Valarray constructor
          */
      Vector(const std::valarray<T>& r) : s(r.size())
            //: v(new T[r.size()]), s(r.size())
         {
            v = new T[r.size()];
            if(!v) {
               VectorException e("Vector(valarray) failed to allocate");
               GPSTK_THROW(e);
            }
            this->assignFrom(r);
         }

         /// subvector constructor
      template <class E>
      Vector(const ConstVectorBase<T, E>& vec,
             size_t top,
             size_t num) : v(NULL),s(0)
         {
               // sanity checks...
            if ( top >= vec.size() || 
                 top + num > vec.size())
            {
               VectorException e("Invalid dimensions or size for Vector(VectorBase)");
               GPSTK_THROW(e);
            }
         
            v = new T[num];
            if(!v) {
               VectorException e("Vector(subvector) failed to allocate");
               GPSTK_THROW(e);
            }
            size_t i;
            for(i = 0; i < num; i++)
               v[i] = vec(top+i);
            s = num;
         }
   
         /// Destructor
      ~Vector()
         { if (v) delete [] v; }

         /// STL iterator begin
      iterator begin() { return v; }
         /// STL const iterator begin
      const_iterator begin() const { return v; }
         /// STL iterator end
      iterator end() { return v + s; }
         /// STL const iterator end
      const_iterator end() const { return v + s; }
         /// STL front
      value_type front() { return v[s-1]; }
         /// STL const front
      const_reference front() const { return v[s-1];}
         /// STL empty
      bool empty() const { return size() == 0; }
         /// STL size
      size_t size() const {return s; }
         /// STL max_size
      size_t max_size() const { return std::numeric_limits<size_t>().max(); }

         /// Non-const operator []
      T& operator[] (size_t i) 
         { return v[i]; }
         /// Const operator []
      T operator[] (size_t i) const
         { return v[i]; }
         /// Non-const operator ()
      T& operator() (size_t i) 
         { return v[i]; }
         /// Const operator ()
      T operator() (size_t i) const
         { return v[i]; }

         /// Like valarray, lets you do vec[slice] to get a VectorSlice.
      VectorSlice<T> operator[] (const std::slice& sli)
         { return VectorSlice<T>(*this, sli); }

         /// *this will be resized if it isn't as large as x.
      Vector& operator=(const Vector& x)
         { resize(x.s); return this->assignFrom(x); }

         /// *this will be resized if it isn't as large as x.
      template <class E>
      Vector& operator=(const ConstVectorBase<T, E>& x)
         { resize(x.size()); return this->assignFrom(x); }

         /// *this will be resized if it isn't as large as x.
      Vector& operator=(const std::valarray<T>& x)
         { resize(x.size()); return this->assignFrom(x); }
         /// Only (*this).size() elements will be assigned.
      Vector& operator=(const T x)
         { return this->assignFrom(x); }
         /// Only (*this).size() elements will be assigned.
      Vector& operator=(const T* x)
         { return this->assignFrom(x); }

      /// *this will be cleared and resized as necessary
      inline Vector& operator=(const std::vector<T>& x)
      {
          size_t i;
          size_t vs = x.size();
          (*this).resize(vs);

          for (i = 0; i < vs; i++) 
              (*this)[i] = x[i];

          return (*this); 
      }

         /// Resizes the vector.  if index > size, the vector will be
         /// erased and the contents destroyed.
      Vector& resize(const size_t index)
         { 
            if (index > s)
            {
               if (v)
                  delete [] v;
               v = new T[index];
               if(!v) {
                  VectorException e("Vector.resize(size_t) failed to allocate");
                  GPSTK_THROW(e);
               }
            }
            s = index;
            return *this;
         }

         /// resize with new default value
      Vector& resize(const size_t index, const T defaultValue)
         {
            resize(index);
            size_t i;
            for(i = 0; i < s; i++)
               v[i] = defaultValue;
            return *this;
         }

        // convert the gpstk vector to std vector
      std::vector<T> toStdVector()
      {
          std::vector<T> v;
          for(size_t i = 0; i < s; i++)
              v.push_back(v[i] );
          return v;
      }

   inline Vector& operator<<(const Vector& b)
   {
       size_t i;
       size_t vs = this->size();
       size_t bs = b.size();
       size_t rows = vs + bs;
       Vector<T> toReturn(rows);

       for (i = 0; i < vs; i++)
           toReturn[i] = (*this)[i];

       for (i = 0; i < bs; i++)
           toReturn[i+vs] = b[i];

       (*this) = toReturn;

       return (*this);
   }

   /// Returns the concatenation of this Vector and a scalar of type T
   inline Vector& operator<<(const T &b) 
   {
       return (*this) << Vector(1,b);
   }

    /// Returns the concatenation of this Vector and Vector b
    inline Vector operator&&(const Vector &b) 
    {
        size_t i;
        size_t vs = this->size();
        size_t bs = b.size();
        size_t rows = vs + bs;
        Vector<T> toReturn(rows);

        for (i = 0; i < vs; i++)
            toReturn[i] = (*this)[i];

        for (i = 0; i < bs; i++)
            toReturn[i+vs] = b[i];

        return toReturn;
    }

    /// Returns the concatenation of this Vector and a scalar of type T
    inline Vector operator&&(const T &b) 
    {
        size_t i;
        size_t vs = this->size();
        size_t rows = vs + 1;
        Vector<T> toReturn(rows);

        for (i = 0; i < vs; i++)
            toReturn[i] = (*this)[i];

        toReturn[rows - 1] = b;

        return toReturn;
    }

   private:

         // a good optimizer will remove this function call
         // if RANGECHECK isn't defined.  remember that
         // range checking affects EVERY operation
      inline bool rangeCheck(const size_t index) const
         {
#ifdef RANGECHECK
            return (index < s);
#else
            return true;
#endif
         }
   
         /// The vector
      T* v;
         /// The size of the vector.
      size_t s;
   };
   // end class Vector<T>

/**
 * A slice of Vector<T> that can be modified.  
 * @warning Remember that (VectorSlice = VectorSlice) will
 * assign elements to the VectorSlice, not copy the VectorSlice internal data!
 */
   template <class T>
   class VectorSlice : public RefVectorSliceBase<T, VectorSlice<T> >
   {
   public:
         /// Default constructor
      VectorSlice()
            : v(NULL), s(std::slice(0,0,0))
         { }

         /// Makes a slice of the whole vector
      VectorSlice(Vector<T>& vv)
            : v(&vv), s(std::slice(0,vv.size(),1))
         { }
            
         /// Makes a slice of the vector with the given std::slice.
      VectorSlice(Vector<T>& vv, const std::slice& ss)
            : v(&vv), s(ss)
         { vecSliceCheck(vv.size()); }

         /// Assign the elements of this slice from another vector.
      template <class V>
      VectorSlice& operator=(const ConstVectorBase<T, V>& x)
         { return this->assignFrom(x); }

         /// Assign the elements of this slice from a valarray.
      VectorSlice& operator=(const std::valarray<T>& x)
         { return this->assignFrom(x); }

         /// Assign all the elements of this slice to x.
      VectorSlice& operator=(const T x)
         { return this->assignFrom(x); }

         /// Assign (*this).size() elements from x to (*this).
      VectorSlice& operator=(const T* x)
         { return this->assignFrom(x); }

         /// Returns the modifiable i'th element of the slice.
      T& operator[] (size_t i) 
         { return (*v)[start() + i * stride()]; }
         /// Returns the const i'th element of the slice.
      T operator[] (size_t i) const
         { return (*v)[start() + i * stride()]; }
         /// Returns the modifiable i'th element of the slice.
      T& operator() (size_t i) 
         { return (*v)[start() + i * stride()]; }
         /// Returns the const i'th element of the slice.
      T operator() (size_t i) const
         { return (*v)[start() + i * stride()]; }

         /// returns the number of elements in the slice
      inline size_t size() const { return s.size(); }
         /// returns the index in the vector of the first element.
      inline size_t start() const { return s.start(); }
         /// returns the number of elements to skip between (*this)[i] and 
         /// (*this)[i+1]
      inline size_t stride() const { return s.stride(); }
   private:
         /// the vector used as a source for the slice
      Vector<T>* v;
         /// the slice specification.
      std::slice s;
   };

/**
 * A Vector<T> slice that doesn't allow modification. 
 */
   template <class T>
   class ConstVectorSlice : public ConstVectorSliceBase<T, ConstVectorSlice<T> >
   {
   public:
         /// default constructor
      ConstVectorSlice()
            : v(NULL), s(std::slice(0,0,0))
         { }

         /// Makes a slice of the whole vector
      ConstVectorSlice(const Vector<T>& vv)
            : v(&vv), s(std::slice(0,vv.size(),1))
         { }
            
         /// Uses the given slice and vector.
      ConstVectorSlice(const Vector<T>& vv, const std::slice& ss)
            : v(&vv), s(ss)
         { vecSliceCheck(vv.size()); }

         /// Returns a const version of the i'th slice element.
      T operator[] (size_t i) const
         { return (*v)[start() + i * stride()]; }
         /// Returns a const version of the i'th slice element.
      T operator() (size_t i) const
         { return (*v)[start() + i * stride()]; }

         /// returns the number of elements in the slice
      inline size_t size() const { return s.size(); }
         /// returns the index in the vector of the first element.
      inline size_t start() const { return s.start(); }
         /// returns the number of elements to skip between (*this)[i] and 
         /// (*this)[i+1]
      inline size_t stride() const { return s.stride(); }

   private:
         /// Vectortor used as a source for this slice.
      const Vector<T>* v;
         /// the slice specification.
      std::slice s;
   };

   //@}

}  // namespace

#include "VectorOperators.hpp"

#endif
