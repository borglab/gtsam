#pragma ident "$Id$"



/**
 * @file MatrixBase.hpp
 * Base classes (const and ref) for Matrix
 */
 
#ifndef GPSTK_MATRIX_BASE_HPP
#define GPSTK_MATRIX_BASE_HPP

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

#include "Vector.hpp"

namespace gpstk
{
 /** @addtogroup VectorGroup */
   //@{

/// @ingroup VectorGroup
/// Thrown when there are problems with the matrix operations
   NEW_EXCEPTION_CLASS(MatrixException, Exception);
/// @ingroup VectorGroup
/// Thrown when an operation can't be performed on a singular matrix.
   NEW_EXCEPTION_CLASS(SingularMatrixException, MatrixException);

/**
 * A matrix base class for a non-modifiable matrix. There is no operator[]
 * for base matrix classes.
 */
   template <class T, class BaseClass>
   class ConstMatrixBase
   {
   public:
         /// default constructor
      explicit ConstMatrixBase() {}

         /// the rows()*cols() size of the matrix.
      size_t size() const
         { return static_cast<const BaseClass*>(this)->size(); }
         /// the number of columns in the matrix
      size_t cols() const
         { return static_cast<const BaseClass*>(this)->cols(); }
         /// the number of rows in the matrix
      size_t rows() const
         { return static_cast<const BaseClass*>(this)->rows(); }
         /// returns a const version of the (i,j)'th element in the matrix,
         /// valid for 0...rows()-1, 0...cols()-1.
      T operator() (size_t i, size_t j) const 
         { return constMatrixRef(i, j); }
   
         /// returns true if this is a square matrix (false for a null matrix).
      inline bool isSquare() const 
         { return ((rows() == cols()) && (rows() != 0)); }
         /// returns true if this is an upper triangular matrix.
      inline bool isUT() const
         {
            if (!isSquare())
               return false;
            size_t i, j;
            for (i = 1; i < rows(); i++)
               for (j = 0; j < i; j++)
                  if ((*this)(i,j) != T(0))
                     return false;
            return true;
         }
         /// returns true if this is a lower triangular matrix.
      inline bool isLT() const
         {
            if (!isSquare())
               return false;
            size_t i, j;
            for (i = 0; i < rows(); i++)
               for (j = i+1; j < cols(); j++)
                  if ((*this)(i,j) != T(0))
                     return false;
            return true;
         }

         /// returns true if this is a diagonal matrix
      inline bool isDiagonal() const
         {
            if (!isSquare())
               return false;
            size_t i, j;
            for (i = 0; i < rows(); i++)
               for (j = 0; j < cols(); j++)
                  if (i != j)
                     if ((*this)(i,j) != T(0))
                        return false;
            return true;
         }
         /// returns true if this is a symmetrical matrix (across the primary
         /// diagonal)
      inline bool isSymmetric() const
         {
            if (!isSquare())
               return false;
            size_t i,j;
            for (i = 0; i < rows(); i++)
               for (j = i + 1; j < cols(); j++)
                  if ((*this)(i,j) != (*this)(j,i))
                     return false;
            return true;
         }

         /// copies out column c into a vector starting with row r
      Vector<T> colCopy(size_t c, size_t r = 0) const
         throw(MatrixException)
         { 
#ifdef RANGECHECK
            if ((c >= cols()) || (r >= rows()))
            {
               MatrixException e("Invalid ConstMatrixBase index for colCopy");
               GPSTK_THROW(e);
            }
#endif
            Vector<T> toReturn(rows() - r);
            size_t i;
            for (i = r; i < rows(); i++)
               toReturn(i - r) = (*this)(i, c);
            return toReturn;
         }

         /// copies out row r into a vector starting with column c
      Vector<T> rowCopy(size_t r, size_t c = 0) const
         throw(MatrixException)
         { 
#ifdef RANGECHECK
            if ((c >= cols()) || (r >= rows()))
            {
               MatrixException e("Invalid ConstMatrixBase index for rowCopy");
               GPSTK_THROW(e);
            }
#endif
            Vector<T> toReturn(cols() - c);
            size_t i;
            for (i = c; i < cols(); i++)
               toReturn(i - c) = (*this)(r, i);
            return toReturn;
         }

   protected:
         /// returns the const (i,j) element from the matrix
      inline T constMatrixRef(size_t i, size_t j) const
         throw(MatrixException)
         {
            const BaseClass& b = static_cast<const BaseClass&>(*this);
#ifdef RANGECHECK
            if ((i >= b.rows()) || (j > b.cols()))
            {
               MatrixException e("Invalid ConstMatrixBase index for ref");
               GPSTK_THROW(e);
            }
#endif
            return b(i,j);
         }
   };

/**
 * A matrix base class that allows assignment of the internal matrix.
 * There is no operator[] for base matrix classes.
 */
   template <class T, class BaseClass>
   class RefMatrixBase : public ConstMatrixBase<T, BaseClass>
   {
   public:
         /// default constructor
      explicit RefMatrixBase() {}

         /// returns a reference to the (i,j) element of the matrix.
      T& operator() (size_t i, size_t j) 
         { return static_cast<BaseClass*>(this)->operator()(i,j); }

         /// returns the rows()*cols() size of the matrix
      size_t size() const
         { return static_cast<const BaseClass*>(this)->size(); }
         /// returns the number of columns in the matrix
      size_t cols() const
         { return static_cast<const BaseClass*>(this)->cols(); }
         /// returns the number of rows in the matrix
      size_t rows() const
         { return static_cast<const BaseClass*>(this)->rows(); }
         /// any value with absolute value below
         /// RefVectorBaseHelper::zeroTolerance is set to 0.
      BaseClass& zeroize()
         {
            BaseClass& me = static_cast<BaseClass&>(*this);
            size_t i, j;
            for (i=0; i < me.rows(); i++)
               for (j=0; j < me.cols(); j++)
                  if (ABS(me(i,j)) < RefVectorBaseHelper::zeroTolerance)
                     me(i,j) = T(0);
            return me;
         }
         /// any value in row r with absolute value below 
         /// RefVectorBaseHelper::zeroTolerance is set to 0.
      BaseClass& zeroizeRow(size_t r)
         {
            BaseClass& me = static_cast<BaseClass&>(*this);
            size_t j;
            for (j=0; j < me.cols(); j++)
               if (ABS(me(r,j)) < RefVectorBaseHelper::zeroTolerance)
                  me(r,j) = T(0);
            return me;
         }
         /// any value in column c with absolute value below 
         /// RefVectorBaseHelper::zeroTolerance is set to 0.
      BaseClass& zeroizeCol(size_t c)
         {
            BaseClass& me = static_cast<BaseClass&>(*this);
            size_t i;
            for (i=0; i < me.rows(); i++)
               if (ABS(me(i,c)) < RefVectorBaseHelper::zeroTolerance)
                  me(i,c) = T(0);
            return me;
         }


              
         /// remember that operator= isn't inherited.  use assignFrom in
         /// derived classes' copy constructors and operator=.
      //MatBaseNewAssignOperator(assignFrom, =);
      //MatBaseNewAssignOperator(operator+=, +=);
      //MatBaseNewAssignOperator(operator-=, -=);
//-----------------------------------------------------------------------
//      MatBaseNewAssignOperator(assignFrom, =);
/** performs = on each element of this matrix with each element of x */
   template <class E> BaseClass& assignFrom(const ConstMatrixBase<T, E>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacro(=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.rows() != me.rows() || x.cols() != me.cols()) {
            MatrixException e("Invalid dimensions for Matrix assignFrom(Matrix)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) = x(i,j);
         return me;
      }
/** performs = on each element of this matrix with each element of x */
   template <class E> BaseClass& assignFrom(const ConstVectorBase<T, E>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacroVecSource(=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.size() != me.rows() * me.cols()) {
            MatrixException e("Invalid dimensions for Matrix assignFrom(Vector)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) = x[i*me.cols()+j];
         return me;
      }
/** performs = on each element of this matrix with each element of x */
   BaseClass& assignFrom(const std::valarray<T>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacroVecSource(=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.size() != me.rows() * me.cols()) {
            MatrixException e("Invalid dimensions for Matrix assignFrom(valarray)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) = x[i*me.cols()+j];
         return me;
      }
/** performs = on each element of this matrix with each element of x */
   BaseClass& assignFrom(const T* x)
      {
         //MatBaseArrayAssignMacroVecSource(=);
         BaseClass& me = static_cast<BaseClass&>(*this);
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) = x[i*me.cols()+j];          // no way to RANGECHECK on x[..]!
         return me;
      }
/** performs = on each element of this matrix with x */
   BaseClass& assignFrom(T x)
      {
         //MatBaseAtomicAssignMacro(=);
         BaseClass& me = static_cast<BaseClass&>(*this);
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) = x;
         return me;
      }

//------------------------------------------------------------------------------------
//      MatBaseNewAssignOperator(operator+=, +=);
/** performs += on each element of this matrix with each element of x */
   template <class E> BaseClass& operator+=(const ConstMatrixBase<T, E>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacro(+=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.rows() != me.rows() || x.cols() != me.cols()) {
            MatrixException e("Invalid dimensions for Matrix operator+=(Matrix)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) += x(i,j);
         return me;
      }
/** performs += on each element of this matrix with each element of x */
   template <class E> BaseClass& operator+=(const ConstVectorBase<T, E>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacroVecSource(+=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.size() != me.rows() * me.cols()) {
            MatrixException e("Invalid dimensions for Matrix operator+=(Vector)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) += x[i*me.cols()+j];
         return me;
      }
/** performs += on each element of this matrix with each element of x */
   BaseClass& operator+=(const std::valarray<T>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacroVecSource(+=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.size() != me.rows() * me.cols()) {
            MatrixException e("Invalid dimensions for Matrix operator+=(valarray)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) += x[i*me.cols()+j];
         return me;
      }
/** performs += on each element of this matrix with each element of x */
   BaseClass& operator+=(const T* x)
      {
         //MatBaseArrayAssignMacroVecSource(+=);
         BaseClass& me = static_cast<BaseClass&>(*this);
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) += x[i*me.cols()+j];          // no way to RANGECHECK on x[..]!
         return me;
      }
/** performs += on each element of this matrix with x */
   BaseClass& operator+=(T x)
      {
         //MatBaseAtomicAssignMacro(+=);
         BaseClass& me = static_cast<BaseClass&>(*this);
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) += x;
         return me;
      }

//------------------------------------------------------------------------------------
//#define MatBaseNewAssignOperator(operator-=, -=)
/** performs -= on each element of this matrix with each element of x */
   template <class E> BaseClass& operator-=(const ConstMatrixBase<T, E>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacro(-=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.rows() != me.rows() || x.cols() != me.cols()) {
            MatrixException e("Invalid dimensions for Matrix operator-=(Matrix)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) -= x(i,j);
         return me;
      }
/** performs -= on each element of this matrix with each element of x */
   template <class E> BaseClass& operator-=(const ConstVectorBase<T, E>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacroVecSource(-=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.size() != me.rows() * me.cols()) {
            MatrixException e("Invalid dimensions for Matrix operator-=(Vector)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) -= x[i*me.cols()+j];
         return me;
      }
/** performs -= on each element of this matrix with each element of x */
   BaseClass& operator-=(const std::valarray<T>& x)
      throw(MatrixException)
      {
         //MatBaseArrayAssignMacroVecSource(-=);
         BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
         if(x.size() != me.rows() * me.cols()) {
            MatrixException e("Invalid dimensions for Matrix operator-=(valarray)");
            GPSTK_THROW(e);
         }
#endif
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) -= x[i*me.cols()+j];
         return me;
      }
/** performs -= on each element of this matrix with each element of x */
   BaseClass& operator-=(const T* x)
      {
         //MatBaseArrayAssignMacroVecSource(-=);
         BaseClass& me = static_cast<BaseClass&>(*this);
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) -= x[i*me.cols()+j];          // no way to RANGECHECK on x[..]!
         return me;
      }
/** performs -= on each element of this matrix with x */
   BaseClass& operator-=(T x)
      {
         //MatBaseAtomicAssignMacro(-=);
         BaseClass& me = static_cast<BaseClass&>(*this);
         size_t i,j;
         for (i=0; i < me.rows(); i++)
            for (j=0; j < me.cols(); j++)
               me(i,j) -= x;
         return me;
      }
   

         /// multiplies each element in this matrix by x.
      BaseClass& operator*=(const T x)
         {
            //MatBaseAtomicAssignMacro(*=);
            BaseClass& me = static_cast<BaseClass&>(*this);
            size_t i,j;
            for (i=0; i < me.rows(); i++)
               for (j=0; j < me.cols(); j++)
                  me(i,j) *= x;
            return me;
         }

         /// divides each element in this matrix by x.
      BaseClass& operator/=(const T x)
         {
            //MatBaseAtomicAssignMacro(/=);
            BaseClass& me = static_cast<BaseClass&>(*this);
            size_t i,j;
            for (i=0; i < me.rows(); i++)
               for (j=0; j < me.cols(); j++)
                  me(i,j) /= x;
            return me;
         }

         /// unary minus: multiplies each element in this matrix by -1.
      const BaseClass operator-()
         {
            const T x=T(-1);
            BaseClass me = static_cast<BaseClass>(*this);
            size_t i,j;
            for (i=0; i < me.rows(); i++)
               for (j=0; j < me.cols(); j++)
                  me(i,j) *= x;
            return me;
         }

         /// swaps rows row1 and row2 in this matrix.
      BaseClass& swapRows(size_t row1, size_t row2) 
         throw(MatrixException)
         {
            BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
            if ( (row1 >= me.rows()) || (row2 >= me.rows()) )
            {
               MatrixException e("Invalid rows for swapRows");
               GPSTK_THROW(e);
            }
#endif
            size_t i;
            T temp;
            for (i = 0; i < me.cols(); i++)
            {
               temp = me(row1, i);
               me(row1,i) = me(row2,i);
               me(row2,i) = temp;
            }
            return me;
         }

         /// swaps columns col1 and col2 in this matrix.
      BaseClass& swapCols(size_t col1, size_t col2) 
         throw(MatrixException)
         {
            BaseClass& me = static_cast<BaseClass&>(*this);
#ifdef RANGECHECK
            if ( (col1 >= me.cols()) || (col2 >= me.cols()) )
            {
               MatrixException e("Invalid columns for swapCols");
               GPSTK_THROW(e);
            }
#endif
            size_t i;
            T temp;
            for (i = 0; i < me.rows(); i++)
            {
               temp = me(i, col1);
               me(i, col1) = me(i, col2);
               me(i, col2) = temp;
            }
            return me;
         }
   };

/**
 * Base class for defining a slice of a matrix.
 */
   template <class T, class BaseClass>
   class MatrixSliceBase
   {
         /// returns the number of rows in this slice
      size_t rowSize() const
         { return static_cast<const BaseClass*>(this)->rowSize(); }
         /// returns the starting row in the base matrix of this slice
      size_t rowStart() const
         { return static_cast<const BaseClass*>(this)->rowStart(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t rowStride() const
         { return static_cast<const BaseClass*>(this)->rowStride(); }
         /// returns the number of columns in this slice
      size_t colSize() const
         { return static_cast<const BaseClass*>(this)->colSize(); }
         /// returns the starting row in the base matrix of this slice
      size_t colStart() const
         { return static_cast<const BaseClass*>(this)->colStart(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t colStride() const
         { return static_cast<const BaseClass*>(this)->colStride(); }
   protected:
         /// checks this slice against the source matrix row and column size
         /// to see if it's a valid slice.
      inline void matSliceCheck(size_t sourceRowSize, 
                                size_t sourceColSize) const
         throw(MatrixException)
         {
//#ifdef RANGECHECK
            if (rowSize() > 0)
            {
               if ( (rowStart() >= sourceRowSize) || 
                    ((rowStart() + (rowSize()-1) * rowStride()) >= sourceRowSize))
               {
                  MatrixException e("Invalid row range for slice");
                  GPSTK_THROW(e);
               }
            }
            if (colSize() > 0)
            {
               if ( (colStart() >= sourceColSize) ||
                    ((colStart() + (colSize()-1) * colStride()) >= sourceColSize))
               {
                  MatrixException e("Invalid col range for slice");
                  GPSTK_THROW(e);
               }
            }
//#endif
         }
   };

/// Base class for an unmodifiable matrix slice
   template <class T, class BaseClass>
   class ConstMatrixSliceBase : public MatrixSliceBase<T, BaseClass>,
                             public ConstMatrixBase<T, BaseClass>
   {
   public:
      explicit ConstMatrixSliceBase() {}
   };

/// Base class for a modifiable matrix slice
   template <class T, class BaseClass>
   class RefMatrixSliceBase : public MatrixSliceBase<T, BaseClass>,
                           public RefMatrixBase<T, BaseClass>
   {
   public:
      explicit RefMatrixSliceBase() {}
   };

   //@}

}  // namespace

#include "MatrixBaseOperators.hpp"

#endif
