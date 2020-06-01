#pragma ident "$Id$"

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

/**
 * @file MatrixOperators.hpp
 * Matrix operators (arithmetic, transpose(), inverse(), etc)
 */

#ifndef GPSTK_MATRIX_OPERATORS_HPP
#define GPSTK_MATRIX_OPERATORS_HPP

#include <limits>
#include "MiscMath.hpp"
#include "MatrixFunctors.hpp"

namespace gpstk
{
 /** @addtogroup VectorGroup */
   //@{
 
/** 
 * Returns the top to bottom concatenation of Matrices l and r only if they have the
 * same number of columns.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator&&(const ConstMatrixBase<T, BaseClass1>& l, 
                            const ConstMatrixBase<T, BaseClass2>& r) 
   throw(MatrixException)
   {
      if (l.cols() != r.cols())
      {
         MatrixException e("Incompatible dimensions for Matrix && Matrix");
         GPSTK_THROW(e);
      }

      size_t rows = l.rows() + r.rows();
      size_t cols = l.cols();
      Matrix<T> toReturn(rows, cols);

      for (rows = 0; rows < l.rows(); rows++)
         for (cols = 0; cols < l.cols(); cols++)
            toReturn(rows, cols) = l(rows, cols);

      for (rows = 0; rows < r.rows(); rows++)
         for (cols = 0; cols < l.cols(); cols++)
            toReturn(rows + l.rows(), cols) = r(rows, cols);

      return toReturn;
   }

/** 
 * Returns the top to bottom concatenation of Matrix t and Vector b
 * only if they have the same number of columns.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator&&(const ConstMatrixBase<T, BaseClass1>& t, 
                            const ConstVectorBase<T, BaseClass2>& b) 
   throw(MatrixException)
   {
      if (t.cols() != b.size())
      {
         MatrixException e("Incompatible dimensions for Matrix && Vector");
         GPSTK_THROW(e);
      }

      size_t rows = t.rows() + 1;
      size_t cols = t.cols();
      Matrix<T> toReturn(rows, cols);

      for (rows = 0; rows < t.rows(); rows++)
         for (cols = 0; cols < t.cols(); cols++)
            toReturn(rows, cols) = t(rows, cols);

      for (cols = 0; cols < t.cols(); cols++)
         toReturn(t.rows(), cols) = b(cols);

      return toReturn;
   }

/** 
 * Returns the top to bottom concatenation of Vector t and Matrix b
 * only if they have the same number of columns.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator&&(const ConstVectorBase<T, BaseClass1>& t, 
                            const ConstMatrixBase<T, BaseClass2>& b) 
   throw(MatrixException)
   {
      if (t.size() != b.cols())
      {
         MatrixException e("Incompatible dimensions for Vector && Matrix");
         GPSTK_THROW(e);
      }

      size_t rows = 1 + b.rows();
      size_t cols = b.cols();
      Matrix<T> toReturn(rows, cols);

      for (cols = 0; cols < b.cols(); cols++)
         toReturn(0, cols) = t(cols);

      for (rows = 1; rows < b.rows()+1; rows++)
         for (cols = 0; cols < b.cols(); cols++)
            toReturn(rows, cols) = b(rows, cols);

      return toReturn;
   }

/** 
 * Returns the left to right concatenation of l and r only if they have the
 * same number of rows.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator||(const ConstMatrixBase<T, BaseClass1>& l,
                            const ConstMatrixBase<T, BaseClass2>& r)  
      throw(MatrixException)
   {
      if (l.rows() != r.rows())
      {
         MatrixException e("Incompatible dimensions for Matrix || Matrix");
         GPSTK_THROW(e);
      }

      size_t rows = l.rows();
      size_t cols = l.cols() + r.cols();
      Matrix<T> toReturn(rows, cols);

      for (cols = 0; cols < l.cols(); cols++)
         for (rows = 0; rows < l.rows(); rows++)
            toReturn(rows, cols) = l(rows, cols);

      for (cols = 0; cols < r.cols(); cols++)
         for (rows = 0; rows < l.rows(); rows++)
            toReturn(rows, cols + l.cols()) = r(rows,cols);

      return toReturn;
   }

/** 
 * Returns the left to right concatenation of Matrix l and Vector r
 * only if they have the same number of rows.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator||(const ConstMatrixBase<T, BaseClass1>& l,
                            const ConstVectorBase<T, BaseClass2>& r)
      throw(MatrixException)
   {
      if (l.rows() != r.size())
      {
         MatrixException e("Incompatible dimensions for Matrix || Vector");
         GPSTK_THROW(e);
      }

      size_t rows = l.rows();
      size_t cols = l.cols() + 1;
      Matrix<T> toReturn(rows, cols);

      for (cols = 0; cols < l.cols(); cols++)
         for (rows = 0; rows < l.rows(); rows++)
            toReturn(rows, cols) = l(rows, cols);

      for (rows = 0; rows < l.rows(); rows++)
         toReturn(rows, l.cols()) = r(rows);

      return toReturn;
   }

/** 
 * Returns the left to right concatenation of Vector l and Matrix r
 * only if they have the same number of rows.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator||(const ConstVectorBase<T, BaseClass1>& l,
                            const ConstMatrixBase<T, BaseClass2>& r)
      throw(MatrixException)
   {
      if (l.size() != r.rows())
      {
         MatrixException e("Incompatible dimensions for Vector || Matrix");
         GPSTK_THROW(e);
      }

      size_t rows = r.rows();
      size_t cols = r.cols() + 1;
      Matrix<T> toReturn(rows, cols);

      for (rows = 0; rows < r.rows(); rows++)
         toReturn(rows, 0) = l(rows);

      for (cols = 1; cols < r.cols()+1; cols++)
         for (rows = 0; rows < r.rows(); rows++)
            toReturn(rows, cols) = r(rows, cols);

      return toReturn;
   }

   /** 
 * Returns the left to right concatenation of Vector l and Vector r
 * only if they have the same number of rows.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator||(const ConstVectorBase<T, BaseClass1>& l,
                            const ConstVectorBase<T, BaseClass2>& r)
      throw(MatrixException)
   {
      if (l.size() != r.size())
      {
         MatrixException e("Incompatible dimensions for Vector || Vector");
         GPSTK_THROW(e);
      }

      size_t rows = r.size();
      Matrix<T> toReturn(rows, 2);

      for (rows = 0; rows < r.size(); rows++)
      {
        toReturn(rows, 0) = l(rows);
        toReturn(rows, 1) = r(rows);
      }

      return toReturn;
   }

/** 
 * Returns the minor matrix of l at element (row, col).  A minor matrix is the
 * same matrix as \c l but with row \c row and col \c col removed.
 */
   template <class T, class BaseClass>
   inline Matrix<T> minorMatrix(const ConstMatrixBase<T, BaseClass>& l,
                          size_t row, size_t col) 
      throw(MatrixException)
   {
      if ((row >= l.rows()) || (col >= l.cols()))
      {
         MatrixException e("Invalid row or column for minorMatrix()");
         GPSTK_THROW(e);
      }
         // handle special cases
      if (row == 0)
      {
         if (col == 0)
         {
            return Matrix<T>(l,1,1,l.rows()-1,l.cols()-1);  
         }
         else if (col == (l.cols() - 1))
         {
            return Matrix<T>(l,1,0,l.rows()-1,l.cols()-1);
         }
         else
         {
            return Matrix<T>(l,1,0,l.rows()-1,col) ||
               Matrix<T>(l,1,col+1,l.rows()-1,l.cols()-col-1);
         }
      }
      else if (row == (l.rows() - 1))
      {
         if (col == 0)
         {
            return Matrix<T>(l,0,1,l.rows()-1,l.cols()-1);
         }
         else if (col == (l.cols() - 1))
         {
            return Matrix<T>(l,0,0,l.rows()-1,l.cols()-1);
         }
         else
         {
            return Matrix<T>(l,0,0,l.rows()-1,col) ||
               Matrix<T>(l,0,col+1,l.rows()-1,l.cols()-col-1);
         }
      }
      else if (col == 0)
      {
         return Matrix<T>(l,0,1,row,l.cols()-1) &&
            Matrix<T>(l,row+1,1,l.rows()-row-1,l.cols()-1);
      }
      else if (col == (l.cols() - 1))
      {
         return Matrix<T>(l,0,0,row,l.cols()-1) &&
            Matrix<T>(l,row+1,0,l.rows()-row-1,l.cols()-1);
      }
      else
      {
         return (Matrix<T>(l, 0, 0, row, col) || 
                 Matrix<T>(l, 0, col + 1, row, l.cols()-col-1)) &&
            (Matrix<T>(l, row + 1, 0, l.rows()-row-1, col) ||
             Matrix<T>(l, row + 1, col + 1, l.rows()-row-1, l.cols()-col-1));
      }
   }

/**
 * Returns a matrix that is \c m transposed.
 */
   template <class T, class BaseClass>
   inline Matrix<T> transpose(const ConstMatrixBase<T, BaseClass>& m)
   {
      Matrix<T> temp(m.cols(), m.rows());
      size_t i, j;
      for (i = 0; i < m.rows(); i++)
         for (j = 0; j < m.cols(); j++)
            temp(j,i) = m(i,j);
      return temp;
   }
 
/**
 * Uses an LU Decomposition to calculate the determinate of m.
 */
   template <class T, class BaseClass>
   inline T det(const ConstMatrixBase<T, BaseClass>& m) 
      throw(MatrixException)
   {
      try
      {
         LUDecomp<T> LU;
         LU(m);
         return LU.det();
      }
      catch(MatrixException& e)
      {
         e.addText("in det()");
         GPSTK_RETHROW(e);
      }
   }

/**
 * returns the condition number of the matrix
 */
   template <class T, class BaseClass>
   inline T condNum(const ConstMatrixBase<T, BaseClass>& m, T& big, T& small) 
      throw ()
   {
      SVD<T> svd;
      svd(m);
      // SVD will not always sort singular values in descending order
      svd.sort(true);
      big = svd.S(0);
      small = svd.S(svd.S.size()-1);
      if(small <= std::numeric_limits<T>::epsilon())
         return T(0);
      return big/small;
   }

/**
 * returns the condition number of the matrix, doesnt require big or small..
 */
   template <class T, class BaseClass>
   inline T condNum(const ConstMatrixBase<T, BaseClass>& m) 
      throw ()
   {
      T big, small;
      return condNum(m, big, small);
   }

/**
 * Returns a new \c dim * \c dim matrix that's an identity matrix.
 */
   template <class T>
   inline Matrix<T> ident(size_t dim)
      throw(MatrixException)
   {
      if (dim == 0)
      {
         MatrixException e("Invalid (0) dimension for ident()");
         GPSTK_THROW(e);
      }
      Matrix<T> toReturn(dim, dim, T(0));
      size_t i;
      for (i = 0; i < toReturn.rows(); i++)
         toReturn(i,i) = T(1);
      return toReturn;
   }

/**
 * Returns the diagonal matrix  of \c m .
 */
   template <class T, class BaseClass>
   inline Matrix<T> diag(const ConstMatrixBase<T, BaseClass>& m)
      throw(MatrixException)
   {
      if ( (m.rows() != m.cols()) || (m.cols() < 1) )
      {
         MatrixException e("invalid matrix dimensions for ident()");
         GPSTK_THROW(e);
      }

      const size_t dim = m.rows();

      Matrix<T> temp(dim, dim, T(0));
      for (size_t i = 0; i < dim; i++)
         temp(i,i) = m(i,i);

      return temp;
   }

/**
 * Block diagonal concatenation of matrix input.
 */
   template <class T, class BaseClass>
   inline Matrix<T> blkdiag(const ConstMatrixBase<T, BaseClass>& m1,
                            const ConstMatrixBase<T, BaseClass>& m2)
      throw(MatrixException)
   {
      if ( (m1.rows() != m1.cols()) || (m1.cols() < 1) ||
           (m2.rows() != m2.cols()) || (m2.cols() < 1) )
      {
         MatrixException e("Invalid matrix dimensions of input.");
         GPSTK_THROW(e);
      }

      const size_t dim1 = m1.rows();
      const size_t dim2 = m2.rows();

      Matrix<T> temp(dim1+dim2, dim1+dim2, T(0));
      for (size_t i = 0; i < dim1; i++)
      {
         for(size_t j = 0; j < dim1; j++)
         {
            temp(i,j) = m1(i,j);
         }
      }
      for (size_t i = 0; i < dim2; i++)
      {
         for(size_t j = 0; j < dim2; j++)
         {
            temp(i+dim1,j+dim1) = m2(i,j);
         }
      }

      return temp;
   }

   template <class T, class BaseClass>
   inline Matrix<T> blkdiag(const ConstMatrixBase<T, BaseClass>& m1,
                            const ConstMatrixBase<T, BaseClass>& m2,
                            const ConstMatrixBase<T, BaseClass>& m3)
      throw(MatrixException)
   { return blkdiag( blkdiag(m1,m2), m3 ); }

   template <class T, class BaseClass>
   inline Matrix<T> blkdiag(const ConstMatrixBase<T, BaseClass>& m1,
                            const ConstMatrixBase<T, BaseClass>& m2,
                            const ConstMatrixBase<T, BaseClass>& m3,
                            const ConstMatrixBase<T, BaseClass>& m4)
      throw(MatrixException)
   { return blkdiag( blkdiag(m1,m2,m3), m4 ); }


/**
 * Return a rotation matrix [dimensioned 3x3, inverse() = transpose()]
 * for the rotation through \c angle radians about \c axis number (= 1, 2 or 3).
 */
   template <class T>
   inline Matrix<T> rotation(T angle, int axis)
      throw(MatrixException)
   {
      if (axis < 1 || axis > 3)
      {
         MatrixException e("Invalid axis (must be 1,2, or 3)");
         GPSTK_THROW(e);
      }
      Matrix<T> toReturn(3,3,T(0));
      int i1 = axis-1;
      int i2 = (i1+1) % 3;
      int i3 = (i2+1) % 3;
      toReturn(i1,i1) = 1.0;
      toReturn(i2,i2) = toReturn(i3,i3) = ::cos(angle);
      toReturn(i3,i2) = -(toReturn(i2,i3) = ::sin(angle));

      return toReturn;
   }

/**
 * Inverts the matrix M by Gaussian elimination. Throws on non-square
 * and singular matricies.
 */
   template <class T, class BaseClass>
   inline Matrix<T> inverse(const ConstMatrixBase<T, BaseClass>& m)
      throw (MatrixException)
   {
      if ((m.rows() != m.cols()) || (m.cols() == 0))
      {
         MatrixException e("inverse() requires non-trivial square matrix");
         GPSTK_THROW(e);
      }

      Matrix<T> toReturn(m.rows(), m.cols() * 2);

      size_t r, t, j;
      T temp;

         // set the left half to m
      {
         MatrixSlice<T> ms(toReturn, 0, 0, m.rows(), m.cols());
         ms = m;
      }

         // set the right half to identity
      {
         MatrixSlice<T> ms(toReturn, 0, m.cols(), m.rows(), m.cols());
         ident(ms);
      }

      for (r = 0; r < m.rows(); r++)
      {
            // if m(r,r) is zero, find another row
            // to add to it...
         if (toReturn(r,r) == 0)
         {
            t = r+1;
            while ( (t < m.rows()) && (toReturn(t,r) == 0) )
               t++;

            if (t == m.rows())
            {
               SingularMatrixException e("Singular matrix");
               GPSTK_THROW(e);
            }

            for (j = r; j < toReturn.cols(); j++)
               toReturn(r,j) += (toReturn(t,j) / toReturn(t,r));
         }

            // scale this row's (r,r)'th element to 1
         temp = toReturn(r,r);
         for (j = r; j < toReturn.cols(); j++)
            toReturn(r,j) /= temp;

            // do the elimination
         for (t = 0; t < m.rows(); t++)
         {
            if (t != r)
            {
               temp = toReturn(t,r);
               for (j = r; j < toReturn.cols(); j++)
                  toReturn(t,j) -= temp/toReturn(r,r) * toReturn(r,j);
            }
         }
      }
         // return the right hand side square matrix
      return Matrix<T>(toReturn, 0, m.cols(), m.rows(), m.cols());

   }  // end inverse

/**
 * Inverts the matrix M by LU decomposition. Throws on non-square
 * and singular matricies.
 */
   template <class T, class BaseClass>
   inline Matrix<T> inverseLUD(const ConstMatrixBase<T, BaseClass>& m)
      throw (MatrixException)
   {
      if ((m.rows() != m.cols()) || (m.cols() == 0)) {
         MatrixException e("inverseLUD() requires non-trivial square matrix");
         GPSTK_THROW(e);
      }

      size_t i,j,N=m.rows();
      Matrix<T> inv(m);
      Vector<T> V(N);
      LUDecomp<T> LU;
      LU(m);
      for(j=0; j<N; j++) {    // loop over columns
         V = T(0);
         V(j) = T(1);
         LU.backSub(V);
         for(i=0; i<N; i++) inv(i,j)=V(i);
      }
      return inv;

   }  // end inverseLUD

/**
 * Inverts the matrix M by LU decomposition, and returns determinant as well
 * Throws on non-square and singular matricies.
 */
   template <class T, class BaseClass>
   inline Matrix<T> inverseLUD(const ConstMatrixBase<T, BaseClass>& m, T& determ)
      throw (MatrixException)
   {
      if ((m.rows() != m.cols()) || (m.cols() == 0)) {
         MatrixException e("inverseLUD() requires non-trivial square matrix");
         GPSTK_THROW(e);
      }

      size_t i,j,N=m.rows();
      Matrix<T> inv(m);
      Vector<T> V(N);
      LUDecomp<T> LU;
      LU(m);
      // compute determinant
      determ = T(LU.parity);
      for(i = 0; i < m.rows(); i++) determ *= LU.LU(i,i);
      // compute inverse
      for(j=0; j<N; j++) {    // loop over columns
         V = T(0);
         V(j) = T(1);
         LU.backSub(V);
         for(i=0; i<N; i++) inv(i,j)=V(i);
      }
      return inv;

   }  // end inverseLUD

/**
 * Inverts the square matrix M by SVD, editing the singular values
 * using tolerance tol. Throws only on input of the zero matrix.
 */
   template <class T, class BaseClass>
   inline Matrix<T> inverseSVD(const ConstMatrixBase<T, BaseClass>& m,
         const T tol=T(1.e-8)) throw (MatrixException)
   {
      if ((m.rows() != m.cols()) || (m.cols() == 0)) {
         MatrixException e("inverseSVD() requires non-trivial square matrix");
         GPSTK_THROW(e);
      }

      size_t i,j,N=m.rows();
      Matrix<T> inv(m);
      SVD<T> svd;
      svd(m);
      // SVD will not always sort singular values in descending order
      svd.sort(true);
      if(svd.S(0) == T(0)) {
         MatrixException e("Input is the zero matrix");
         GPSTK_THROW(e);
      }
      // edit singular values TD input tolerance, output edited SVs
      for(i=1; i<N; i++) if(svd.S(i) < tol*svd.S(0)) svd.S(i)=T(0);
      // back substitution
      Vector<T> V(N);
      for(j=0; j<N; j++) {    //loop over columns
         V = T(0);
         V(j) = T(1);
         svd.backSub(V);
         for(i=0; i<N; i++) inv(i,j)=V(i);
      }
      return inv;

   }  // end inverseSVD

/**
 * Invert the square matrix M by SVD, editing the singular values with tolerance tol,
 * and return the largest and smallest singular values (before any editing).
 * Throws only on input of the zero matrix.
 */
   template <class T, class BaseClass>
   inline Matrix<T> inverseSVD(const ConstMatrixBase<T, BaseClass>& m,
      T& big, T& small, const T tol=T(1.e-8)) throw (MatrixException)
   {
      if ((m.rows() != m.cols()) || (m.cols() == 0)) {
         MatrixException e("inverseSVD() requires non-trivial square matrix");
         GPSTK_THROW(e);
      }

      size_t i,j,N=m.rows();
      Matrix<T> inv(m);
      SVD<T> svd;
      svd(m);
      // SVD will not always sort singular values in descending order
      svd.sort(true);
      if(svd.S(0) == T(0)) {
         MatrixException e("Input is the zero matrix");
         GPSTK_THROW(e);
      }

      // compute condition number = big/small
      big = svd.S(0);
      small = svd.S(svd.S.size()-1);

      // edit singular values using input tolerance, output edited SVs
      for(i=1; i<N; i++) if(svd.S(i) < tol*svd.S(0)) svd.S(i)=T(0);

      // back substitution
      Vector<T> V(N);
      for(j=0; j<N; j++) {    //loop over columns
         V = T(0);
         V(j) = T(1);
         svd.backSub(V);
         for(i=0; i<N; i++) inv(i,j)=V(i);
      }
      return inv;

   }  // end inverseSVD

/**
 * Invert the square matrix M by SVD, editing the singular values
 * using tolerance tol, and return the singular values
 * (before any editing). Throws only on input of the zero matrix.
 */
   template <class T, class BaseClass>
   inline Matrix<T> inverseSVD(const ConstMatrixBase<T, BaseClass>& m,
      Vector<T>& sv, const T tol=T(1.e-8)) throw (MatrixException)
   {
      if ((m.rows() != m.cols()) || (m.cols() == 0)) {
         MatrixException e("inverseSVD() requires non-trivial square matrix");
         GPSTK_THROW(e);
      }

      size_t i,j,N=m.rows();
      Matrix<T> inv(m);
      SVD<T> svd;
      svd(m);
      // SVD will not always sort singular values in descending order
      svd.sort(true);
      if(svd.S(0) == T(0)) {
         MatrixException e("Input is the zero matrix");
         GPSTK_THROW(e);
      }

      // save the singular values
      sv = Vector<T>(N);
      for(i=0; i<N; i++) sv(i) = svd.S(i);

      // edit singular values using input tolerance, output edited SVs
      for(i=1; i<N; i++) if(svd.S(i) < tol*svd.S(0)) svd.S(i)=T(0);

      // back substitution
      Vector<T> V(N);
      for(j=0; j<N; j++) {    //loop over columns
         V = T(0);
         V(j) = T(1);
         svd.backSub(V);
         for(i=0; i<N; i++) inv(i,j)=V(i);
      }
      return inv;

   }  // end inverseSVD

   /**
    * Inverts the square symetrix positive definite matrix M using Cholesky-Crout
    * algorithm. Very fast and useful when M comes from using a Least Mean-Square 
    * (LMS) or Weighted Least Mean-Square (WLMS) method.
    */
   template <class T, class BaseClass>
   inline Matrix<T> inverseChol(const ConstMatrixBase<T, BaseClass>& m)
       throw (MatrixException)
   {
       int N = m.rows(), i, j, k;
       double sum;
       Matrix<T> LI(N,N, 0.0);      // Here we will first store L^-1, and later m^-1

       // Let's call CholeskyCrout class to decompose matrix "m" in L*LT
       gpstk::CholeskyCrout<double> CC;
       CC(m);

       // Let's find the inverse of L (the LI from above)
       for(i=0; i<N; i++) {
           LI(i,i) = 1.0 / CC.L(i,i);
           for(j=0; j<i; j++) {
               sum = 0.0;
               for(k=i; k>=0; k-- ) sum += CC.L(i,k)*LI(k,j);
               LI(i,j) = -sum*LI(i,i);
           }
       }

       // Now, let's remember that m^-1 = transpose(LI)*LI
       LI = transpose(LI) * LI;
       return LI;

   }  // end inverseChol


/**
 *  Matrix * Matrix : row by column multiplication of two matricies.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator* (const ConstMatrixBase<T, BaseClass1>& l, 
                            const ConstMatrixBase<T, BaseClass2>& r)
      throw (MatrixException)
   {
      if (l.cols() != r.rows())
      {
         MatrixException e("Incompatible dimensions for Matrix * Matrix");
         GPSTK_THROW(e);
      }
   
      Matrix<T> toReturn(l.rows(), r.cols(), T(0));
      size_t i, j, k;
      for (i = 0; i < toReturn.rows(); i++)
         for (j = 0; j < toReturn.cols(); j++)
            for (k = 0; k < l.cols(); k++)
               toReturn(i,j) += l(i,k) * r(k,j);

      return toReturn;
   }

/**
 * Matrix times vector multiplication, returning a vector.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Vector<T> operator* (const ConstMatrixBase<T, BaseClass1>& m, 
                            const ConstVectorBase<T, BaseClass2>& v)
      throw (MatrixException)
   {
      if (v.size() != m.cols())
      {
         gpstk::MatrixException e("Incompatible dimensions for Vector * Matrix");
         GPSTK_THROW(e);
      }
   
      Vector<T> toReturn(m.rows());
      size_t i, j;
      for (i = 0; i < m.rows(); i++) 
      {
         toReturn[i] = 0;
         for (j = 0; j < m.cols(); j++)
            toReturn[i] += m(i, j) * v[j];
      }
      return toReturn;
   }
/**
 * Vector times matrix multiplication, returning a vector.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Vector<T> operator* (const ConstVectorBase<T, BaseClass1>& v, 
                            const ConstMatrixBase<T, BaseClass2>& m)
      throw (gpstk::MatrixException)
   {
      if (v.size() != m.rows())
      {
         gpstk::MatrixException e("Incompatible dimensions for Vector * Matrix");
         GPSTK_THROW(e);
      }
   
      Vector<T> toReturn(m.cols());
      size_t i, j;
      for (i = 0; i < m.cols(); i++) 
      {
         toReturn[i] = 0;
         for (j = 0; j < m.rows(); j++)
            toReturn[i] += m(j,i) * v[j];
      }
      return toReturn;
   }

/**
 * Compute sum of two matricies.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator+ (const ConstMatrixBase<T, BaseClass1>& l,
                            const ConstMatrixBase<T, BaseClass2>& r)
      throw (MatrixException)
   {
      if (l.cols() != r.cols() || l.rows() != r.rows())
      {
         MatrixException e("Incompatible dimensions for Matrix + Matrix");
         GPSTK_THROW(e);
      }

      Matrix<T> toReturn(l.rows(), r.cols(), T(0));
      size_t i, j;
      for (i = 0; i < toReturn.rows(); i++)
         for (j = 0; j < toReturn.cols(); j++)
            toReturn(i,j) = l(i,j) + r(i,j);

      return toReturn;
   }

/**
 * Compute difference of two matricies.
 */
   template <class T, class BaseClass1, class BaseClass2>
   inline Matrix<T> operator- (const ConstMatrixBase<T, BaseClass1>& l,
                            const ConstMatrixBase<T, BaseClass2>& r)
      throw (MatrixException)
   {
      if (l.cols() != r.cols() || l.rows() != r.rows())
      {
         MatrixException e("Incompatible dimensions for Matrix - Matrix");
         GPSTK_THROW(e);
      }

      Matrix<T> toReturn(l.rows(), r.cols(), T(0));
      size_t i, j;
      for (i = 0; i < toReturn.rows(); i++)
         for (j = 0; j < toReturn.cols(); j++)
            toReturn(i,j) = l(i,j) - r(i,j);

      return toReturn;
   }

/**
 * Compute the outer product of two vectors.
 */
   template <class T, class BaseClass>
   inline Matrix<T> outer(const ConstVectorBase<T, BaseClass>& v,
                        const ConstVectorBase<T, BaseClass>& w)
      throw (MatrixException)
   {
      if(v.size()*w.size() == 0) {
         MatrixException e("Zero length vector(s)");
         GPSTK_THROW(e);
      }
      Matrix<T> M(v.size(),w.size(),T(0));
      for(size_t i=0; i<v.size(); i++)
         for(size_t j=0; j<w.size(); j++)
            M(i,j) = v(i)*w(j);
      return M;
   }

/// Multiplies all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator* (const ConstMatrixBase<T, BaseClass>& m, const T d)
   {
      Matrix<T> temp(m);
      return temp *= d;
   }

/// Multiplies all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator* (const T d, const ConstMatrixBase<T, BaseClass>& m)
   {
      Matrix<T> temp(m);
      return temp *= d;
   }

/// Divides all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator/ (const ConstMatrixBase<T, BaseClass>& m, const T d)
   {
      Matrix<T> temp(m);
      return temp /= d;
   }

/// Divides all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator/ (const T d, const ConstMatrixBase<T, BaseClass>& m)
   {
      Matrix<T> temp(m);
      return temp /= d;
   }

/// Adds all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator+ (const ConstMatrixBase<T, BaseClass>& m, const T d)
   {
      Matrix<T> temp(m);
      return temp += d;
   }

/// Adds all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator+ (const T d, const ConstMatrixBase<T, BaseClass>& m)
   {
      Matrix<T> temp(m);
      return temp += d;
   }

/// Subtracts all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator- (const ConstMatrixBase<T, BaseClass>& m, const T d)
   {
      Matrix<T> temp(m);
      return temp -= d;
   }

/// Subtracts all the elements of m by d.
   template <class T, class BaseClass>
   inline Matrix<T> operator- (const T d, const ConstMatrixBase<T, BaseClass>& m)
   {
      Matrix<T> temp(m);
      return temp -= d;
   }

   //@}
 
}  // namespace

#endif
