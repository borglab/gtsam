/// @file MatrixFunctors.hpp
/// Matrix function operators (SVD, LUD, etc)
 
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

#ifndef GPSTK_MATRIX_FUNCTORS_HPP
#define GPSTK_MATRIX_FUNCTORS_HPP

#include <cmath>
#include <limits>

namespace gpstk
{

 /** @addtogroup VectorGroup */
   //@{

/**
 * Class SVD: A function object for the singular value decomposition of a matrix.
 * Given a matrix A [m,n], the SVD of A = U*S*transpose(V), where U is [m,m],
 * V is [n,n], and S is [m,n] (like A). Both U and V are unitary [meaning
 * transpose(U)*U = unity = transpose(V)*V] and the columns of U[resp,V]
 * are orthonormal vectors spanning the space A*transpose(A) [transpose(A)*A].
 * Note that U*transpose(U)=1 and V*transpose(V)=1 are not true in general,
 * but may be. S[m,n] is 'diagonal' in the sense that only diagonal elements
 * are non-zero (even when m != n); the min(m,n) diagonal elements are called
 * the singular values of A, often referred to as S[i]. The singular values
 * may be sorted, as the SVD is invariant under a consistent re-ordering of
 * {singular values / columns of U / columns of V}.
 * The condition number of A is the ratio
 *    cn = fabs(largest S[i])/fabs(smallest S[i]).
 * Note that inverse(A) = V*inverse(S)*UT where inverse(S) is diagonal with
 * elements equal to the inverse of elements of S, and with dimension [n,m].
 * The matrix A is non-singular matrix if and only if all of its singular
 * values are non-zero. If some of the singular values are zero, the
 * 'generalized inverse' of A may be formed by editing the singular values
 * in this way: if the ratio of S[i] to S[0] (where S[0] is the largest
 * singular value) is bigger than some tolerance (1.e-7 is good), then 1/S[i]
 * is set to zero in the inverse. In this way the 'generalized inverse' of
 * ANY matrix is guaranteed to exist.
 * The SVD algorithm never fails.
 *
 * Ref: Bulirsch and Stoer, "Introduction to Numerical Analysis,"
 * NY, Springer-Verlag, 1980.
 *
 * @code
 * Matrix<double> m(and is assigned some value);
 * SVD<double> d;
 * d(m);
 * cout << d.U << endl << d.V << endl << d.S << endl;
 * @endcode
 */
   template <class T>
   class SVD
   {
   public:
      SVD() : iterationMax(30) {}

         // Singular Value Decomposition
      template <class BaseClass>
      bool operator() (const ConstMatrixBase<T, BaseClass>& mat)
         throw (MatrixException)
         {
            const T eps=T(8)*std::numeric_limits<T>::epsilon();
            bool flip=false;
            U = mat;
            if(mat.rows() > mat.cols()) {
               flip = true;
               U = transpose(mat);
            }

            size_t n(U.cols()), m(U.rows());
            size_t i, j, k, l, nm, jj;
            T anorm(0), scale(0), g(0), s, f, h, c, x, y, z;

            V = Matrix<T>(n, n, T(0));
            S = Vector<T>(n, T(1));
            Vector<T> B(n, T(1));

            for (i = 0; i < n; i++) {
               l = i + 1;
               B[i] = scale * g;
               g = s = scale = T(0);
               if (i < m) {
                  for(k = i; k < m; k++) scale += ABS(U(k, i));
                  if (scale) {
                     for(k = i; k < m; k++) {
                        U(k, i) /= scale;
                        s += U(k, i) * U(k, i);
                     }
                     f = U(i, i);
                     g = -SIGN(SQRT(s),f);
                     h = f * g - s;
                     U(i,i) = f - g;
                     for(j = l; j < n; j++) {
                        for(s = T(0), k = i; k < m; k++) s += U(k, i) * U(k, j);
                        f = s / h;
                        for(k = i; k < m; k++) U(k, j) += f * U(k, i);
                     }
                     for(k = i; k < m; k++) U(k, i) *= scale;
                  } // if (scale)
               }  // if (i < m)
               S[i] = scale * g;
               g = s = scale = T(0);
               if ( (i < m) && (i != n-1) ) {
                  for(k = l; k < n; k++) scale += ABS(U(i, k));
                  if (scale) {
                     for(k = l; k < n; k++) {
                        U(i, k) /= scale;
                        s += U(i, k) * U(i, k);
                     }
                     f = U(i, l);
                     g = -SIGN(SQRT(s),f);
                     h = f * g - s;
                     U(i, l) = f - g;
                     for(k = l; k < n; k++) B[k] = U(i, k) / h;
                     for(j = l; j < m; j++) {
                        for(s = T(0), k = l; k < n; k++) s += U(j, k) * U(i, k);
                        for(k = l; k < n; k++) U(j, k) += s * B[k];
                     }
                     for(k = l; k < n; k++) U(i, k) *= scale;
                  }
               }
               if(ABS(S[i])+ABS(B[i]) > anorm) anorm=ABS(S[i])+ABS(B[i]);
            }
            for(i = n - 1; ; i--) {
               if (i < n - 1) {
                  if (g) {
                     for(j = l; j < n; j++) V(j, i) = (U(i, j) / U(i, l)) / g;
                     for(j = l; j < n; j++) {
                        for(s = T(0), k = l; k < n; k++) s += U(i, k) * V(k, j);
                        for(k = l; k < n; k++) V(k, j) += s * V(k, i);
                     }
                  }
                  for(j = l; j < n; j++) V(j, i) = V(i, j) = T(0);
               }
               V(i,i) =T(1);
               g = B[i];
               l = i;
               if(i==0) break;
            }
            for(i = ( (m-1 < n-1) ? m-1 : n-1); ; i--) {
               l = i+1;
               g = S[i];
               for(j=l; j<n; j++) U(i, j) = T(0);
               if (g) {
                  g = T(1) / g;
                  for(j = l; j < n; j++) {
                     for(s = T(0), k = l; k < m; k++) s += U(k,i) * U(k,j);
                     f = (s / U(i,i)) * g;
                     for(k=i; k<m; k++) U(k,j) += f * U(k,i);
                  }
                  for(j = i; j < m; j++) U(j,i) *= g;
               } 
               else {
                  for(j=i; j<m; j++) U(j,i) = T(0);
               }
               ++U(i,i);
               if(i==0) break;
            }
            for(k = n - 1; ; k--) {
               size_t its;
               for(its = 1; its <= iterationMax; its++) {
                  bool flag = true;
                  for(l = k; ; l--) {
                     nm = l - 1;
                     if (ABS(B[l])/MAX(ABS(B[l]),anorm) < eps) {
                        flag = false;
                        break;
                     }
                     if (l == 0) { // should never happen
                        MatrixException e("SVD algorithm has nm==-1");
                        GPSTK_THROW(e);
                     }
                     if (ABS(S[nm])/MAX(ABS(S[nm]),anorm) < eps) break;
                     if(l == 0) break; // since l is unsigned...
                  }
                  if (flag) {
                     c = T(0);
                     s = T(1);
                     for(i = l; i <= k; i++) {
                        f = s * B[i];
                        B[i] = c * B[i];
                        if (ABS(f)/MAX(ABS(f),anorm) < eps) break;
                        g = S[i];
                        h = RSS(f,g);
                        S[i] = h;
                        h = T(1) / h;
                        c = g * h;
                        s = -f * h;
                        for(j = 0; j < m; j++) {
                           y = U(j, nm);
                           z = U(j,i);
                           U(j, nm) = y * c + z * s;
                           U(j,i) = z * c - y * s;
                        }
                     }
                  }
                  z = S[k];
                  if (l == k) {
                     if (z < T(0)) {
                        S[k] = -z;
                        for(j = 0; j < n; j++) V(j,k) = -V(j,k);
                     }
                     break;
                  }
            
                  if (its == iterationMax) {
                     MatrixException e("SVD algorithm did not converge");
                     GPSTK_THROW(e);
                  }
                  x = S[l];
                  if(k == 0) { // should never happen
                     MatrixException e("SVD algorithm has k==0");
                     GPSTK_THROW(e);
                  }
                  nm = k - 1;
                  y = S[nm];
                  g = B[nm];
                  h = B[k];
                  f = ( (y-z) * (y+z) + (g-h) * (g+h)) / (T(2) * h * y);
                  g = RSS(f,T(1));
                  f = ( (x-z) * (x+z) + h * ((y/(f + SIGN(g,f))) - h)) / x;
                  c = s = 1.0;
                  for(j = l; j <= nm; j++) {
                     i = j + 1;
                     g = B[i];
                     y = S[i];
                     h = s * g;
                     g = c * g;
                     z = RSS(f, h);
                     B[j] = z;
                     c = f / z;
                     s = h / z;
                     f = x * c + g * s;
                     g = g * c - x * s;
                     h = y * s;
                     y *= c;
                     for(jj = 0; jj < n; jj++) {
                        x = V(jj, j);
                        z = V(jj, i);
                        V(jj, j) = x * c + z * s;
                        V(jj, i) = z * c - x * s;
                     }
                     z = RSS(f, h);
                     S[j] = z;
                     if (z) {
                        z = T(1) / z;
                        c = f * z;
                        s = h * z;
                     }
                     f = c * g + s * y;
                     x = c * y - s * g;
                     for(jj = 0; jj < m; jj++) {
                        y = U(jj, j);
                        z = U(jj, i);
                        U(jj, j) = y * c + z * s;
                        U(jj, i) = z * c - y * s;
                     }
                  }
                  B[l] = T(0);
                  B[k] = f;
                  S[k] = x;
               }
               if(k==0) break;   // since k is unsigned...
            }
               // if U is not square - last n-m columns of U are zero - remove
            if(U.cols() > U.rows()) {
               for(i=1; i<S.size(); i++) {   // sort in descending order
                  T sv=S[i],svj;
                  int kk = i-1;  // not size_t ~ unsigned
                  while(kk >= 0) {
                     svj = S[kk];
                     if(sv < svj) break;
                     S[kk+1] = svj;
                     // exchange columns kk and kk+1 in U and V
                     U.swapCols(kk,kk+1);
                     V.swapCols(kk,kk+1);
                     kk = kk - 1;
                  }
                  S[kk+1] = sv;
               }
               Matrix<T> Temp(U);
               U = Matrix<T>(Temp,0,0,Temp.rows(),Temp.rows());
               S.resize(Temp.rows());
            }

            if(flip) {
               Matrix<T> Temp(U);
               U = V;
               V = Temp;
            }

            return true;

         }  // end SVD::operator() - the SVD algorithm
   
         // Backsubstitution using SVD.
         // Solve A*x=b for vector x where A [mxn] has been SVD'ed and is given by
         // U,W,V (*this); that is A[mxn] = U[mxm]*W[mxn]*VT[nxn]. b has dimension m,
         // x dimension n. Singular values are NOT edited, except that if s.v. == 0,
         // 1/0 is replaced by 0. Result is returned as b.
      template <class BaseClass>
      void backSub(RefVectorBase<T, BaseClass>& b) const 
         throw(MatrixException)
      {
         if(b.size() != U.rows())
         {
             MatrixException e("SVD::BackSub called with unequal dimensions");
             GPSTK_THROW(e);
         }
   
         size_t i, n=V.cols(), m=U.rows();
         Matrix<T> W(n,m,T(0));     // build the 'inverse singular values' matrix
         for(i=0; i<S.size(); i++) W(i,i)=(S(i)==T(0)?T(0):T(1)/S(i));
         Vector<T> Y;
         Y = V*W*transpose(U)*b;
         //b = Y;
         // this fails because operator= is not defined for the base class
         // (op= not inherited); use assignFrom instead
         b.assignFrom(Y);

      }  // end SVD::backSub

         /// sort singular values
      void sort(bool descending)
         throw(MatrixException)
      {
         size_t i;
         int j;         // j must be allowed to go negative
         for(i=1; i<S.size(); i++) {
            T sv=S(i),svj;
            j = i - 1;
            while(j >= 0) {
               svj = S(j);
               if(descending && sv < svj) break;
               if(!descending && sv > svj) break;
               S(j+1) = svj;
               // exchange columns j and j+1 in U and V
               U.swapCols(j,j+1);
               V.swapCols(j,j+1);
               j = j - 1;
            }
            S(j+1) = sv;
         }
      }  // end SVD::sort

         /// compute determinant from SVD
      inline T det(void)
         throw(MatrixException)
      {
         T d(1);
         for(size_t i=0; i<S.size(); i++) d *= S(i);
         return d;
      }  // end SVD::det

         /// Matrix U
      Matrix<T> U;
         /// Vector of singular values
      Vector<T> S;
         /// Matrix V (not transpose(V))
      Matrix<T> V;

   private:
      const size_t iterationMax;
   
      T SIGN(T a, T b)
         { 
            if (b >= T(0))
               return ABS(a);
            else
               return -ABS(a);
         }

   }; // end class SVD

// Performs the lower/upper triangular decomposition of a matrix PA = LU.
// The results are put into the matricies L, U, and P (pivot), and sign
// (representing even (positive) or odd (negative) row swaps.
   template <class T>
   class LUDecomp
   {
   public:
      LUDecomp() {}        // why is there no constructor from ConstMatrixBase?

         /// Does the decomposition.
      template <class BaseClass>
      void operator() (const ConstMatrixBase<T, BaseClass>& m)
         throw (MatrixException)
         {
            if(!m.isSquare() || m.rows()<=1) {
               MatrixException e("LUDecomp requires a square, non-trivial matrix");
               GPSTK_THROW(e);
            }

            size_t N=m.rows(),i,j,k,imax;
            T big,t,d;
            Vector<T> V(N,T(0));

            LU = m;
            Pivot = Vector<int>(N);
            parity = 1;

            for(i=0; i<N; i++) {    // get scale of each row
               big = T(0);
               for(j=0; j<N; j++) {
                  t = ABS(LU(i,j));
                  if(t > big) big=t;
               }
               if(big <= T(0)) {    // m is singular
                  //LU *= T(0);
                  SingularMatrixException e("singular matrix!");
                  GPSTK_THROW(e);
               }
               V(i) = T(1)/big;
            }

            for(j=0; j<N; j++) {    // loop over columns
               for(i=0; i<j; i++) {
                  t = LU(i,j);
                  for(k=0; k<i; k++) t -= LU(i,k)*LU(k,j);
                  LU(i,j) = t;
               }
               big = T(0);          // find largest pivot
               for(i=j; i<N; i++) {
                  t = LU(i,j);
                  for(k=0; k<j; k++) t -= LU(i,k)*LU(k,j);
                  LU(i,j) = t;
                  d = V(i)*ABS(t);
                  if(d >= big) {
                     big = d;
                     imax = i;
                  }
               }
               if(j != imax) {
                  LU.swapRows(imax,j);
                  V(imax) = V(j);
                  parity = -parity;
               }
               Pivot(j) = imax;

               t = LU(j,j);
               if(t == 0.0) {       // m is singular
                  //LU *= T(0);
                  SingularMatrixException e("singular matrix!");
                  GPSTK_THROW(e);
               }
               if(j != N-1) {
                  d = T(1)/t;
                  for(i=j+1; i<N; i++) LU(i,j) *= d;
               }
            }
         }  // end LUDecomp()

         // Compute inverse(m)*v, where *this is LUD(m), via back substitution
         // Solution overwrites input Vector v
      template <class BaseClass2>
      void backSub(RefVectorBase<T, BaseClass2>& v) const
         throw (MatrixException)
      {
         if(LU.rows() != v.size()) {
            MatrixException e("Vector size does not match dimension of LUDecomp");
            GPSTK_THROW(e);
         }

         bool first=true;
         size_t N=LU.rows(),i,j,ii(0);
         T sum;

         // un-pivot
         for(i=0; i<N; i++) {
            sum = v(Pivot(i));
            v(Pivot(i)) = v(i);
            if(first && sum != T(0)) {
               ii = i;
               first = false;
            }
            else for(j=ii; j<i; j++) sum -= LU(i,j)*v(j);
            v(i) = sum;
         }
         // back substitution
         for(i=N-1; ; i--) {
            sum = v(i);
            for(j=i+1; j<N; j++) sum -= LU(i,j)*v(j);
            v(i) = sum / LU(i,i);
            if(i == 0) break;       // b/c i is unsigned
         }
      }  // end LUD::backSub

         /// compute determinant from LUD
      inline T det(void)
         throw(MatrixException)
      {
         T d(parity);
         for(size_t i=0; i<LU.rows(); i++) d *= LU(i,i);
         return d;
      }

         // The matrix in LU-decomposed form: L and U together;
         // all diagonal elements of L are implied 1.
         Matrix<T> LU;
         /// The pivot array
         Vector<int> Pivot;
         /// Parity
         int parity;

   }; // end class LUDecomp


   // Compute Cholesky decomposition (triangular square root) of the given matrix,
   // which must be square and positive definite. That is, if M is square and positive
   // definite, then M = C * transpose(C) where C is the Cholesky decomposition. This
   // routine computes both the upper (U) and lower (L) triangular C's;
   // thus M = L * transpose(L) = U * transpose(U).
   //
   // NB Note that M = U*UT NOT UT*U; if M = U'T*U' is desired, use U'=transpose(L);
   // then note the similarity to LU decomposition: M = L*U' = L*LT = U'T*U'.
   //
   // Positive definite (PD) implies positive eigenvalues, and the reverse.
   //
   // Note that the result is not unique, as any orthogonal transformation R of UT,
   // including a change in sign, will preserve the result m = U*UT = U*RT*R*UT.
   //
   // @code
   // Matrix<double> M(i,i);     // square and PD
   ///Cholesky<double> Ch;
   // Ch(M);
   // cout << Ch.U << endl << Ch.L << endl;
   // // note that M = Ch.L * transpose(Ch.L);
   // // note that M = Ch.U * transpose(Ch.U);
   // // and that if Matrix<double> Up(transpose(Ch.L)); then
   // // M = transpose(Up) * Up;
   // @endcode
   template <class T>
   class Cholesky
   {
   public:
      Cholesky() {}

         /// @todo potential complex number problem!
      template <class BaseClass>
      void operator() (const ConstMatrixBase<T, BaseClass>& m)
         throw (MatrixException)
      {
         if(!m.isSquare()) {
            MatrixException e("Cholesky requires a square matrix");
            GPSTK_THROW(e);
         }

         size_t N=m.rows(),i,j,k;
         double d;
         Matrix<T> P(m);
         U = Matrix<T>(m.rows(),m.cols(),T(0));

         for(j=N-1; ; j--) {
            if(P(j,j) <= T(0)) {
               MatrixException e("Cholesky fails - eigenvalue <= 0");
               GPSTK_THROW(e);
            }
            U(j,j) = SQRT(P(j,j));
            d = T(1)/U(j,j);
            if(j > 0) {
               for(k=0; k<j; k++) U(k,j)=d*P(k,j);
               for(k=0; k<j; k++)
                  for(i=0; i<=k; i++)
                     P(i,k) -= U(k,j)*U(i,j);
            }
            if(j==0) break;      // since j is unsigned
         }

         P = m;
         L = Matrix<T>(m.rows(),m.cols(),T(0));
         for(j=0; j<=N-1; j++) {
            if(P(j,j) <= T(0)) {
               MatrixException e("Cholesky fails - eigenvalue <= 0");
               GPSTK_THROW(e);
            }
            L(j,j) = SQRT(P(j,j));
            d = T(1)/L(j,j);
            if(j < N-1) {
               for(k=j+1; k<N; k++) L(k,j)=d*P(k,j);
               for(k=j+1; k<N; k++) {
                  for(i=k; i<N; i++) {
                     P(i,k) -= L(i,j)*L(k,j);
                  }
               }
            }
         }

      }  // end Cholesky::operator()

      // Use backsubstition to solve the equation A*x=b where *this Cholesky
      // has been applied to A, i.e. A = L*transpose(L). The algorithm is in
      // two steps: since A*x=L*LT*x=b, first solve L*y=b for y, then solve
      // LT*x=y for x. x is returned as b.
      template <class BaseClass2>
      void backSub(RefVectorBase<T, BaseClass2>& b) const
         throw (MatrixException)
      {
         if (L.rows() != b.size())
         {
            MatrixException e("Vector size does not match dimension of Cholesky");
            GPSTK_THROW(e);
         }
         size_t i,j,N=L.rows();
      
         Vector<T> y(b.size());
         y(0) = b(0)/L(0,0);
         for(i=1; i<N; i++) {
            y(i) = b(i);
            for(j=0; j<i; j++) y(i)-=L(i,j)*y(j);
            y(i) /= L(i,i);
         }
         // b is now x
         b(N-1) = y(N-1)/L(N-1,N-1);
         for(i=N-1; ; i--) {
            b(i) = y(i);
            for(j=i+1; j<N; j++) b(i)-=L(j,i)*b(j);
            b(i) /= L(i,i);
            if(i==0) break;
         }

      }  // end Cholesky::backSub

         /// Lower triangular and Upper triangular Cholesky decompositions
      Matrix<T> L, U;

   }; // end class Cholesky

   // Compute the Cholesky decomposition using the Cholesky-Crout algorithm,
   // which is very fast; if A is the given matrix we will get L, where A = L*LT. 
   // A must be symetric and positive definite. This is the usual case when
   // A comes from applying a Least Mean-Square (LMS) or Weighted Least 
   // Mean-Square (WLMS) method.
   //
   // NB. U here is NOT the same as U in class Cholesky (but L is the same);
   // here m = transpose(U)*U, but there m = U * transpose(U); see doc for Cholesky.
   template <class T> 
   class CholeskyCrout : public Cholesky<T>
   {
   public:
       template <class BaseClass>
       void operator() (const ConstMatrixBase<T, BaseClass>& m) throw(MatrixException)
       {
           if(!m.isSquare()) {
               MatrixException e("CholeskyCrout requires a square matrix");
               GPSTK_THROW(e);
           }

           int N = m.rows(), i, j, k;
           double sum;
           (*this).L = Matrix<T>(N,N, 0.0);

           for(j=0; j<N; j++) {
               sum = m(j,j);
               for(k=0; k<j; k++ ) sum -= (*this).L(j,k)*(*this).L(j,k);
               if(sum > 0.0) {
                   (*this).L(j,j) = SQRT(sum);
               } else {
                   MatrixException e("CholeskyCrout fails - eigenvalue <= 0");
                   GPSTK_THROW(e);          
               }

               for(i=j+1; i<N; i++){
                   sum = m(i,j);
                   for(k=0; k<j; k++) sum -= (*this).L(i,k)*(*this).L(j,k);
                   (*this).L(i,j) = sum/(*this).L(j,j);
               }
           }

           (*this).U = transpose((*this).L);
       }

   }; // end class CholeskyCrout


   // The Householder transformation is simply an orthogonal transformation
   // designed to make the elements below the diagonal zero. It applies to any
   // matrix.
   template <class T>
   class Householder
   {
   public:
      Householder() {}

      // Explicitly perform the transformation, one column at a time, without
      // actually constructing the transformation matrix. Let y be column k of the
      // input matrix. y can be zeroed below the diagonal as follows:
      // let sum=sign(y(k))*sqrt(y*y), and define vector u(k)=y(k)+sum,
      // u(j)=y(j) (j.gt.k). This defines the transformation matrix as (1-bu*u),
      // with b=2/u*u=1/sum*u(k). Redefine y(k)=u(k) and apply the transformation to
      // elements of the input matrix below and to the right of the (k,k) element.
      // This algorithm for each column k=0,n-1 in turn is equivalent to a single
      // orthogonal transformation which triangularizes the matrix.
      template <class BaseClass>
      inline void operator() (const ConstMatrixBase<T, BaseClass>& m)
         throw (MatrixException)
         {
            A = m;
            size_t i,j,k;
            Vector<T> v(A.rows());
            T sum,alpha;

            // loop over cols
            const T EPS(1.e-200);
            for(j=0; (j<A.cols()-1 && j<A.rows()-1); j++) {
               sum = T(0);
               // loop over rows at diagonal and below
               for(i=j; i<A.rows(); i++) {
                  v(i) = A(i,j);
                  A(i,j) = T(0);       // this is optional - below the diag is trash
                  sum += v(i)*v(i);
               }

               if(sum < EPS) continue;             // already zero below diagonal

               sum = SQRT(sum);
               if(v(j) > T(0)) sum = -sum;
               A(j,j) = sum;
               v(j) = v(j) - sum;
               sum = 1/(sum*v(j));

               // loop over columns beyond j
               for(k=j+1; k<A.cols(); k++) {
                  alpha = T(0);
                  // loop over rows at and below j
                  for(i=j; i<A.rows(); i++) {
                     alpha += A(i,k)*v(i);
                  }
                  alpha *= sum;
                  // modify column k at and below j
                  for(i=j; i<A.rows(); i++) {
                     A(i,k) += alpha*v(i);
                  }
               }  // end loop over cols > j

            }  // end loop over cols

         }  // end Householder::operator()
      
         /// The upper triangular transformed matrix.
      Matrix<T> A;

   }; // end class Householder

   //@}

}  // namespace

#endif
