#pragma ident "$Id$"



/**
 * @file Matrix.hpp
 * Basic Matrix algorithms
 */
 
#ifndef GPSTK_MATRIX_HPP
#define GPSTK_MATRIX_HPP

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
#include "MatrixBase.hpp"

namespace gpstk
{

   /** @defgroup VectorGroup Vector and Matrix mathematics */
   //@{
 
// forward declarations
   template <class T> class MatrixRowSlice;
   template <class T> class ConstMatrixRowSlice;
   template <class T> class MatrixColSlice;
   template <class T> class ConstMatrixColSlice;


/**
 * An implementation of a matrix class using Vector<T> as its internal basis.
 * This class is STL compliant with the iterator proceeding in row major order.
 * Operators +=, -=, *= and /= are implemented in RefMatrixBase.
 * 
 * @sa matvectest.cpp for examples
 */
   template <class T>
   class Matrix : public RefMatrixBase<T, Matrix<T> >
   {
   public:
         /// STL value_type
      typedef typename Vector<T>::value_type value_type;
         /// STL reference type
      typedef typename Vector<T>::reference reference;
         /// STL const reference type
      typedef typename Vector<T>::const_reference const_reference;
         /// STL iterator type
      typedef typename Vector<T>::iterator iterator;
         /// STL const iterator type
      typedef typename Vector<T>::const_iterator const_iterator;  

         /// default constructor
      Matrix();
         /// constructor given an initial size
      Matrix(size_t rows, size_t cols);
         /// constructor for an initial size and value
      Matrix(size_t rows, size_t cols, T initialValue);
         /// copies out the contents of vec to initialize the matrix
      Matrix(size_t rows, size_t cols, const T* vec);
         /// copies out the contents of vec to initialize the matrix
      template <class BaseClass>
      Matrix(size_t rows, size_t cols, const ConstVectorBase<T, BaseClass>& vec)
         : v(rows*cols), r(rows), c(cols), s(rows * cols)
      { assignFrom(vec); }

         /// constructor for a ConstMatrixBase object
      template <class BaseClass>
      Matrix(const ConstMatrixBase<T, BaseClass>& mat) 
            : v(mat.size()), r(mat.rows()), c(mat.cols()), s(mat.size())
         {
            size_t i,j;
            for(i = 0; i < r; i++)
               for(j = 0; j < c; j++)
                  (*this)(i,j) = mat(i, j);
         }

         /// submatrix constructor
      template <class BaseClass>
      Matrix(const ConstMatrixBase<T, BaseClass>& mat, size_t topRow, 
          size_t topCol, size_t numRows, size_t numCols) 
            : v((size_t)0), r(0), c(0), s(0)
         {
               // sanity checks...
            if ( (topCol > mat.cols()) || 
                 (topRow > mat.rows()) ||
                 ((topRow + numRows) > mat.rows()) ||
                 ((topCol + numCols) > mat.cols()) )
            {
               MatrixException e("Invalid dimensions or size for Matrix(MatrixBase)");
               GPSTK_THROW(e);
            }
         
               // seems ok - make the valarray and copy column by column
            r = numRows;
            c = numCols;
            s = r * c;
            v.resize(r * c);
            size_t i, j;
            for(i = 0; i < r; i++)
               for(j = 0; j < c; j++)
                  (*this)(i,j) = mat(topRow + i, topCol + j);
         }

         /// STL begin
      iterator begin() { return v.begin(); }
         /// STL const begin
      const_iterator begin() const { return v.begin(); }
         /// STL end
      iterator end() { return v.end(); }
         /// STL const end
      const_iterator end() const { return v.end(); }
         /// STL front
      value_type front() { return v.front(); }
         /// STL const front
      const_reference front() const { return v.front();}
         /// STL empty
      bool empty() const { return s == 0; }
         /// STL size
      size_t size() const {return s; }
         /// STL max size
      size_t max_size() const { return s; }

         /// The number of rows in the matrix
      inline size_t rows() const { return r; }
         /// The number of columns in the matrix
      inline size_t cols() const { return c; }
         /// A reference slice of a row with a given std::slice
      inline MatrixRowSlice<T> rowRef(size_t rowNum, const std::slice& s);
         /// A reference slice of a row with a starting column (i.e. sub-row)
      inline MatrixRowSlice<T> rowRef(size_t rowNum, size_t colNum = 0);
         /// A const reference slice of a row with a given std::slice
      inline ConstMatrixRowSlice<T> row(size_t rowNum, const std::slice& s) const;
         /// A const reference slice of a row with a starting column (i.e. sub-row)
      inline ConstMatrixRowSlice<T> row(size_t rowNum, size_t colNum = 0) const;

         /// A reference column with a given slice
      inline MatrixColSlice<T> colRef(size_t colNum, const std::slice& s);
         /// A reference column with a starting row number (i.e. sub-column)
      inline MatrixColSlice<T> colRef(size_t colNum, size_t rowNum = 0);
         /// A const reference column with a given slice
      inline ConstMatrixColSlice<T> col(size_t colNum, const std::slice& s) const;
         /// A const reference column with a starting row number (i.e. sub-column)
      inline ConstMatrixColSlice<T> col(size_t colNum, size_t rowNum = 0) const;

         /// Non-const matrix operator(row,col)
      inline T& operator() (size_t rowNum, size_t colNum)
         { return v(rowNum + colNum * r); }
         /// Const matrix operator(row,col)
      inline T operator() (size_t rowNum, size_t colNum) const
         { return v(rowNum + colNum * r); }
         /// operator[] that returns a row slice
      inline MatrixRowSlice<T> operator[] (size_t row)
         { return rowRef(row); }
         /// const operator[] that returns a const row slice
      inline ConstMatrixRowSlice<T> operator[] (size_t rowNum) const 
         { return row(rowNum);}

         /// Resizes the matrix to rows*cols.
         /// @warning YOUR DATA MAY NOT BE RETAINED!!!
      inline Matrix& resize(size_t rows, size_t cols);

      inline Matrix& resize(size_t rows, size_t cols, 
                         const T initialValue);

         /**
          * Assigns this matrix to a T* in column major order.
          * @warning be careful that array is as large as the matrix is!
          */
      inline Matrix& operator=(const T* array)
         { return this->assignFrom(array); }
         /// Assigns the contents of this matrix to those in array in column
         /// major order.
      inline Matrix& operator=(const std::valarray<T> array)
         { return this->assignFrom(array); }
         /// Assigns all elements of the matrix to \c t.
      inline Matrix& operator=(const T t)
         { return this->assignFrom(t); }
         /// Copies the other matrix.
      inline Matrix& operator=(const Matrix& mat)
         { v = mat.v; r = mat.r; c = mat.c; s = mat.s; return *this; }
         /// Copies from any matrix.
      template <class BaseClass>
      inline Matrix& operator=(const ConstMatrixBase<T, BaseClass>& mat)
         { 
            v.resize(mat.size()); 
            r=mat.rows(); 
            c=mat.cols(); 
            s=mat.size();
            return this->assignFrom(mat);
         }
         /// Copies from any vector.
      template <class BaseClass>
      inline Matrix& operator=(const ConstVectorBase<T, BaseClass>& mat)
         { return this->assignFrom(mat); }

   private:
         /// the matrix stored in column major order
      Vector<T> v;
      size_t r,  ///< the number of rows
         c,  ///< the number of columns
         s;  ///< the overall size
   };

/**
 * An assignable slice of a matrix.
 */
   template <class T>
   class MatrixSlice : public RefMatrixSliceBase<T, MatrixSlice<T> >
   {
   public:
         /// default constructor
      MatrixSlice() : m(NULL), rSlice(std::slice(0,0,0)), 
         cSlice(std::slice(0,0,0)), s(0)
         {}

         /// Makes a slice of the whole matrix.
      MatrixSlice(Matrix<T>& mat)
            : m(&mat), rSlice(std::slice(0, mat.rows(), 1)),
              cSlice(std::slice(0,mat.cols(), 1)), s(mat.size())
         {
            this->matSliceCheck(mat.rows(), mat.cols());
         }

         /// Makes a partial slice of a matrix.
      MatrixSlice(Matrix<T>& mat, const std::slice& rowSlice,
               const std::slice& colSlice)
            : m(&mat), rSlice(rowSlice), cSlice(colSlice),
              s(rSlice.size() * cSlice.size())
         {
            this->matSliceCheck(mat.rows(), mat.cols());
         }

         /// Submatrix slice.
      MatrixSlice(Matrix<T>& mat, size_t topRow, size_t topCol, 
               size_t numRows, size_t numCols)
            : m(&mat), rSlice(std::slice(topRow, numRows, 1)),
              cSlice(std::slice(topCol, numCols, 1)),
              s(rSlice.size() * cSlice.size())
         {
            this->matSliceCheck(mat.rows(), mat.cols());
         }
      
         /// Copies from x to (*this).
      template <class V>
      MatrixSlice& operator=(const ConstMatrixBase<T, V>& x)
         { return this->assignFrom(x); }

         /// Copies from x to (*this).
      template <class V>
      MatrixSlice& operator=(const ConstVectorBase<T, V>& x)
         { return this->assignFrom(x); }

         /// Copies from x to (*this).
      MatrixSlice& operator=(const std::valarray<T>& x)
         { return this->assignFrom(x); }
         /// Copies from x to (*this).
      MatrixSlice& operator=(const T x)
         { return this->assignFrom(x); }
         /// Copies from x to (*this).
      MatrixSlice& operator=(const T* x)
         { return this->assignFrom(x); }

         /// returns the size of this slice
      size_t size() const { return s; }
         /// returns the number of columns in the slice
      size_t cols() const { return colSize(); }
         /// returns the number of rows in the slice
      size_t rows() const { return rowSize(); }
         /// returns the (i,j) element of the slice.
      T& operator() (size_t i, size_t j)
         { return (*m)(i * rowStride() + rowStart(), 
                       j * colStride() + colStart()); }
         /// returns the (i,j) element of the slice, const version.
      T operator() (size_t i, size_t j) const
         { return (*m)(i * rowStride() + rowStart(), 
                       j * colStride() + colStart()); }


         /// returns the number of rows in this slice
      size_t rowSize() const { return rSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t rowStart() const{ return rSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t rowStride() const { return rSlice.stride(); }
         /// returns the number of columns in this slice
      size_t colSize() const { return cSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t colStart() const { return cSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t colStride() const { return cSlice.stride(); }

   private:
         /// The matrix this slice refers to
      Matrix<T>* m;
      std::slice rSlice, ///< a row slice
         cSlice; ///< a column slice
      size_t s; ///< the overall size
   };

/**
 * An unmodifiable matrix slice.
 */
   template <class T>
   class ConstMatrixSlice : public ConstMatrixSliceBase<T, ConstMatrixSlice<T> >
   {
   public:
         /// default constructor
      ConstMatrixSlice(void) : m(NULL), rSlice(std::slice(0,0,0)), 
         cSlice(std::slice(0,0,0)), s(0)
         {}

         /// makes a const slice of the whole matrix
      ConstMatrixSlice(const Matrix<T>& mat)
            : m(&mat), rSlice(std::slice(0, mat.rows(), 1)),
              cSlice(std::slice(0,mat.cols(), 1)), s(mat.size())
         {
            this->matSliceCheck(mat.rows(), mat.cols());
         }

         /// makes a slice given std::slices for rows and columns
      ConstMatrixSlice(const Matrix<T>& mat, const std::slice& rowSlice,
               const std::slice& colSlice)
            : m(&mat), rSlice(rowSlice), cSlice(colSlice),
              s(rSlice.size() * cSlice.size())
         {
            this->matSliceCheck(mat.rows(), mat.cols());
         }

         /// submatrix slice
      ConstMatrixSlice(const Matrix<T>& mat, size_t topRow, size_t topCol, 
               size_t numRows, size_t numCols)
            : m(&mat), rSlice(std::slice(topRow, numRows, 1)),
              cSlice(std::slice(topCol, numCols, 1)),
              s(rSlice.size() * cSlice.size())
         {
            this->matSliceCheck(mat.rows(), mat.cols());
         }

         /// the size of the slice
      size_t size() const { return s; }
         /// the number of columns in the slice
      size_t cols() const { return colSize(); }
         /// the number of rows in the slice
      size_t rows() const { return rowSize(); }
         /// the (i,j) element of the slice, const.
      T operator() (size_t i, size_t j) const 
         { return (*m)(i * rowStride() + rowStart(), 
                       j * colStride() + colStart()); }

         /// returns the number of rows in this slice
      size_t rowSize() const { return rSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t rowStart() const{ return rSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t rowStride() const { return rSlice.stride(); }
         /// returns the number of columns in this slice
      size_t colSize() const { return cSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t colStart() const { return cSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t colStride() const { return cSlice.stride(); }
   private:
         /// the matrix this slice refers to.
      const Matrix<T>* m;
      std::slice rSlice, ///< the row slice
         cSlice; ///< the column slice
      size_t s; ///< the size of the slice
   };

/**
 * an assignable single column slice of a matrix
 */
   template <class T>
   class MatrixColSlice : public RefMatrixSliceBase<T, MatrixColSlice<T> >
   {
   public:
         /// default constructor
      MatrixColSlice() : m(NULL), c(0), rSlice(std::slice(0,0,0)) {}
         /// makes a slice of the column \c col from matrix \c mat.
      MatrixColSlice(Matrix<T>& mat, size_t col)
            : m(&mat), c(col), rSlice(std::slice(0,mat.rows(),1))
         { 
            this->matSliceCheck(mat.rows(), mat.cols()); 
         }
         /// makes a slice of the column from the matrix using \c s to
         /// further slice the column.
      MatrixColSlice(Matrix<T>& mat, size_t col, const std::slice& s)
            : m(&mat), c(col), rSlice(s)
         { 
               // decide if the input is reasonable
            this->matSliceCheck(mat.rows(), mat.cols());
         }

         /// assigns this column to x
      template <class V>
      MatrixColSlice& operator=(const ConstMatrixBase<T, V>& x)
         { return this->assignFrom(x); }

         /// assigns this column to x
      template <class V>
      MatrixColSlice& operator=(const ConstVectorBase<T, V>& x)
         { return this->assignFrom(x); }
         /// assigns this column to x
      MatrixColSlice& operator=(const std::valarray<T>& x)
         { return this->assignFrom(x); }
         /// assigns this column to x
      MatrixColSlice& operator=(const T x)
         { return this->assignFrom(x); }
         /// assigns this column to x
      MatrixColSlice& operator=(const T* x)
         { return this->assignFrom(x); }

         /// returns the i'th element of the column, non-const
      T& operator[] (size_t i) 
         { return (*m)(rowStart() + i * rowStride(), c); }
         /// returns the i'th element of the column, non-const
      T& operator() (size_t i) 
         { return (*m)(rowStart() + i * rowStride(), c); }
         /// returns the i'th element of the column, const
      T operator[] (size_t i) const
         { return (*m)(rowStart() + i * rowStride(), c); }
         /// returns the i'th element of the column, const
      T operator() (size_t i) const
         { return (*m)(rowStart() + i * rowStride(), c); }

         /// returns the (i,j) element, non-const
      T& operator() (size_t i, size_t j) 
         { return (*m)(rowStart() + i * rowStride(), j + c); }
         /// returns the (i,j) element, non-const
      T operator() (size_t i, size_t j) const
         { return (*m)(rowStart() + i * rowStride(), j + c); }

         /// returns the number of rows in the slice
      size_t rows() const {return size();}
         /// returns the number of columns in the slice
      size_t cols() const {return 1;}
         /// returns the size of the slice
      size_t size() const {return rowSize();}

         /// returns the number of rows in this slice
      size_t rowSize() const { return rSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t rowStart() const{ return rSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t rowStride() const { return rSlice.stride(); }
         /// returns the number of columns in this slice
      size_t colSize() const { return 1; }
         /// returns the starting row in the base matrix of this slice
      size_t colStart() const { return c; }
         /// returns the number of elements between the i'th and i+1'th row
      size_t colStride() const { return 1; }

   private:
         /// the matrix this slice refers to.
      Matrix<T>* m;
         /// the column this slice is for
      size_t c;
         /// slice down the rows
      std::slice rSlice;

   };

/**
 * a constant slice of a single column from a matrix.
 */
   template <class T>
   class ConstMatrixColSlice : public ConstMatrixSliceBase<T, ConstMatrixColSlice<T> >
   {
   public:
         /// default constructor
      ConstMatrixColSlice() 
            : m(NULL), c(0), rSlice(std::slice(0,0,0)) 
         {}

         /// constructor taking a slice of column \c col from the matrix.
      ConstMatrixColSlice(const Matrix<T>& mat, size_t col)
            : m(&mat), c(col), rSlice(std::slice(0,mat.rows(),1))
         { this->matSliceCheck(mat.rows(), mat.cols()); }

         /// constructor taking a slice of column \c col from the matrix,
         /// slicing the column by \c s.
      ConstMatrixColSlice(const Matrix<T>& mat, size_t col, 
                       const std::slice& s)
            : m(&mat), c(col), rSlice(s)
         { 
               // decide if the input is reasonable
            this->matSliceCheck(mat.rows(), mat.cols());
         }

         /// returns the i'th element of the column slice
      T operator[] (size_t i) const
         { return (*m)(rowStart() + i * rowStride(), c); }
         /// returns the i'th element of the column slice
      T operator() (size_t i) const
         { return (*m)(rowStart() + i * rowStride(), c); }

         /// returns the (i,j) element of the column slice
      T operator() (size_t i, size_t j) const
         { return (*m)(rowStart() + i * rowStride(), j + c); }

         /// returns the size of the slice in rows
      size_t rows() const {return rowSize();}
         /// returns the size of the slice in columns
      size_t cols() const {return 1;}
         /// returns the overall size of the slice
      size_t size() const {return rowSize();}

         /// returns the number of rows in this slice
      size_t rowSize() const { return rSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t rowStart() const{ return rSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t rowStride() const { return rSlice.stride(); }
         /// returns the number of columns in this slice
      size_t colSize() const { return 1; }
         /// returns the starting row in the base matrix of this slice
      size_t colStart() const { return c; }
         /// returns the number of elements between the i'th and i+1'th row
      size_t colStride() const { return 1; }
   private:
         /// the matrix this slice refers to
      const Matrix<T>* m;
         /// the column this slice refers to
      size_t c;
         /// a slice down the rows
      std::slice rSlice;
   };

/**
 * an assignable single row slice of a matrix
 */
   template <class T>
   class MatrixRowSlice : public RefMatrixSliceBase<T, MatrixRowSlice<T> >
   {
   public:
         /// default constructor
      MatrixRowSlice() 
            : m(NULL), r(0), cSlice(std::slice(0,0,0)) 
         {}
         /// makes a slice of row \c row from the matrix.
      MatrixRowSlice(Matrix<T>& mat, size_t row)
            : m(&mat), r(row), cSlice(std::slice(0,mat.cols(),1))
         { this->matSliceCheck(mat.rows(), mat.cols()); }

         /// makes a slice of row \c row from the matrix, slicing it by \c s.
      MatrixRowSlice(Matrix<T>& mat, size_t row, 
                  const std::slice& s)
            : m(&mat), r(row), cSlice(s)
         { 
               // decide if the input is reasonable
            this->matSliceCheck(mat.rows(), mat.cols());
         }   

         /// assigns this row to x.
      template <class V>
      MatrixRowSlice& operator=(const ConstMatrixBase<T, V>& x)
         { return this->assignFrom(x); }
         /// assigns this row to x.
      template <class V>
      MatrixRowSlice& operator=(const ConstVectorBase<T, V>& x)
         { return this->assignFrom(x); }
         /// assigns this row to x.
      MatrixRowSlice& operator=(const std::valarray<T>& x)
         { return this->assignFrom(x); }
         /// assigns this row to x.
      MatrixRowSlice& operator=(const T x)
         { return this->assignFrom(x); }
         /// assigns this row to x.
      MatrixRowSlice& operator=(const T* x)
         { return this->assignFrom(x); }

         /// returns the j'th element of the slice, non-const
      T& operator[] (size_t j)
         { return (*m)(r, colStart() + j * colStride()); }
         /// returns the j'th element of the slice, non-const
      T& operator() (size_t j)
         { return (*m)(r, colStart() + j * colStride()); }
         /// returns the j'th element of the slice, const
      T operator[] (size_t j) const
         { return (*m)(r, colStart() + j * colStride()); }
         /// returns the j'th element of the slice, const
      T operator() (size_t j) const
         { return (*m)(r, colStart() + j * colStride()); }
         /// returns the (i,j) element of the slice, non-const
      T& operator() (size_t i, size_t j) 
         { return (*m)(i + r, colStart() + j * colStride()); }
         /// returns the (i,j) element of the slice, const
      T operator() (size_t i, size_t j) const
         { return (*m)(i + r, colStart() + j * colStride()); }

         /// returns the number of rows in the row slice
      size_t rows() const {return 1;}
         /// returns the number of columns in the row slice
      size_t cols() const {return colSize();}
         /// returns the size of the slice
      size_t size() const {return colSize();}

         /// returns the number of rows in this slice
      size_t rowSize() const { return 1; }
         /// returns the starting row in the base matrix of this slice
      size_t rowStart() const{ return r; }
         /// returns the number of elements between the i'th and i+1'th row
      size_t rowStride() const { return 1; }
         /// returns the number of columns in this slice
      size_t colSize() const { return cSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t colStart() const { return cSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t colStride() const { return cSlice.stride(); }

   private:
         /// the matrix this slice refers to.
      Matrix<T>* m;
         /// the row of the slice
      size_t r;
         /// the column slice of the row.
      std::slice cSlice;
   };

/**
 * an unmodifiable row slice of a matrix.
 */
   template <class T>
   class ConstMatrixRowSlice : public ConstMatrixSliceBase<T, ConstMatrixRowSlice<T> >
   {
   public:
         /// default constructor
      ConstMatrixRowSlice() 
            : m(NULL), r(0), cSlice(std::slice(0,0,0)) 
         {}
         /// makes a const row slice from the matrix
      ConstMatrixRowSlice(const Matrix<T>& mat, size_t row)
            : m(&mat), r(row), cSlice(std::slice(0,mat.cols(),1))
         { this->matSliceCheck(mat.rows(), mat.cols()); }

         /// makes a const row slice from the matrix, slicing that row by \c s.
      ConstMatrixRowSlice(const Matrix<T>& mat, size_t row, 
                       const std::slice& s)
            : m(&mat), r(row), cSlice(s)
         { 
               // decide if the input is reasonable
            this->matSliceCheck(mat.rows(), mat.cols());
         }   

         /// returns the i'th element of the slice
      T operator[] (size_t i) const
         { return (*m)(r, colStart() + i * colStride()); }
         /// returns the i'th element of the slice
      T operator() (size_t i) const
         { return (*m)(r, colStart() + i * colStride()); }

         /// returns the (i,j) element of the slice
      T operator() (size_t i, size_t j) const
         { return (*m)(i + r, colStart() + j * colStride()); }

         /// returns the number of rows in the slice
      size_t rows() const {return 1;}
         /// returns the number of columns in the slice
      size_t cols() const {return colSize();}
         /// returns the overall size of the slice
      size_t size() const {return colSize();}

         /// returns the number of rows in this slice
      size_t rowSize() const { return 1; }
         /// returns the starting row in the base matrix of this slice
      size_t rowStart() const{ return r; }
         /// returns the number of elements between the i'th and i+1'th row
      size_t rowStride() const { return 1; }
         /// returns the number of columns in this slice
      size_t colSize() const { return cSlice.size(); }
         /// returns the starting row in the base matrix of this slice
      size_t colStart() const { return cSlice.start(); }
         /// returns the number of elements between the i'th and i+1'th row
      size_t colStride() const { return cSlice.stride(); }
   private:
         /// the matrix this slice refers to
      const Matrix<T>* m;
         /// the row of the slice
      size_t r;
         /// the slice of the row's columns
      std::slice cSlice;
   };

   //@}

}  // namespace

#include "MatrixImplementation.hpp"
#include "MatrixOperators.hpp"
#include "MatrixFunctors.hpp"

#endif
