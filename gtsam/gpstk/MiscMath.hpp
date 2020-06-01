/**
 * @file MiscMath.hpp
 * Miscellaneous mathematical algorithms
 */

#ifndef GPSTK_MISCMATH_HPP
#define GPSTK_MISCMATH_HPP

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

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

#include <cstring>   // for size_t
#include <vector>
#include "MathBase.hpp"
#include "Exception.hpp"

namespace gpstk
{
   /** @defgroup math Mathematical algorithms */
   //@{

   /// This is a straightforward version of Lagrange Interpolation.
   /// Y must have size at least as large as X, and X.size() must be >= 2;
   /// x should lie within the range of X.
   template <class T>
   T SimpleLagrangeInterpolation(const std::vector<T>& X, const std::vector<T>& Y,
      const T x) throw(Exception)
   {
      if(Y.size() < X.size()) {
         GPSTK_THROW(Exception("Input vectors must be of same size"));
      }
      size_t i,j;
      T Yx(0);
      for(i=0; i<X.size(); i++) {
         if(x==X[i]) return Y[i];

         T Li(1);
         for(j=0; j<X.size(); j++)
             if(i!=j)  Li *= (x-X[j])/(X[i]-X[j]);

         Yx += Li*Y[i];
      }
      return Yx;
   }  // end T LagrangeInterpolation(const vector, const vector, const T)

   /// Lagrange interpolation on data (X[i],Y[i]), i=0,N-1 to compute Y(x).
   /// Also return an estimate of the estimation error in 'err'.
   /// This routine assumes that N=X.size() is even and that x is centered on the
   /// interval, that is X[N/2-1] <= x <= X[N/2].
   /// NB This routine will work for N as small as 4, however tests with satellite
   /// ephemerides have shown that N=4 yields m-level errors, N=6 cm-level,
   /// N=8 ~0.1mm level and N=10 ~numerical noise errors; best to use N>=8.
   template <class T>
   T LagrangeInterpolation(const std::vector<T>& X, const std::vector<T>& Y,
      const T& x, T& err) throw(Exception)
   {
      if(Y.size() < X.size() || X.size() < 4) {
         GPSTK_THROW(Exception("Input vectors must be of same length, at least 4"));
      }

      std::size_t i,j,k;
      T y,del;
      std::vector<T> D,Q;

      err = T(0);
      k = X.size()/2;
      if(x == X[k]) return Y[k];
      if(x == X[k-1]) return Y[k-1];
      if(ABS(x-X[k-1]) < ABS(x-X[k])) k=k-1;
      for(i=0; i<X.size(); i++) {
         Q.push_back(Y[i]);
         D.push_back(Y[i]);
      }
      y = Y[k--];
      for(j=1; j<X.size(); j++) {
         for(i=0; i<X.size()-j; i++) {
            del = (Q[i+1]-D[i])/(X[i]-X[i+j]);
            D[i] = (X[i+j]-x)*del;
            Q[i] = (X[i]-x)*del;
         }
         err = (2*(k+1) < X.size()-j ? Q[k+1] : D[k--]);    // NOT 2*k
         y += err;
      }
      return y;
   }  // end T LagrangeInterpolation(vector, vector, const T, T&)

   // The following is a
   // Straightforward implementation of Lagrange polynomial and its derivative
   // { all sums are over index=0,N-1; Xi is short for X[i]; Lp is dL/dx;
   //   y(x) is the function being approximated. }
   // y(x) = SUM[Li(x)*Yi]
   // Li(x) = PROD(j!=i)[x-Xj] / PROD(j!=i)[Xi-Xj]
   // dy(x)/dx = SUM[Lpi(x)*Yi]
   // Lpi(x) = SUM(k!=i){PROD(j!=i,j!=k)[x-Xj]} / PROD(j!=i)[Xi-Xj]
   // Define Pi = PROD(j!=i)[x-Xj], Di = PROD(j!=i)[Xi-Xj],
   // Qij = PROD(k!=i,k!=j)[x-Xk] and Si = SUM(j!=i)Qij.
   // then Li(x) = Pi/Di, and Lpi(x) = Si/Di.
   // Qij is symmetric, there are only N(N+1)/2 - N of them, so store them
   // in a vector of length N(N+1)/2, where Qij==Q[i+j*(j+1)/2] (ignore i=j).

   /// Perform Lagrange interpolation on the data (X[i],Y[i]), i=1,N (N=X.size()),
   /// returning the value of Y(x) and dY(x)/dX.
   /// Assumes that x is between X[k-1] and X[k], where k=N/2 and N > 2;
   /// Warning: for use with the precise (SP3) ephemeris only when velocity is not
   /// available; estimates of velocity, and especially clock drift, not as accurate.
   template <class T>
   void LagrangeInterpolation(const std::vector<T>& X, const std::vector<T>& Y,
      const T& x, T& y, T& dydx) throw(Exception)
   {
      if(Y.size() < X.size() || X.size() < 4) {
         GPSTK_THROW(Exception("Input vectors must be of same length, at least 4"));
      }

      std::size_t i,j,k,N=X.size(),M;
      M = (N*(N+1))/2;
      std::vector<T> P(N,T(1)),Q(M,T(1)),D(N,T(1));
      for(i=0; i<N; i++) {
         for(j=0; j<N; j++) {
            if(i != j) {
               P[i] *= x-X[j];
               D[i] *= X[i]-X[j];
               if(i < j) {
                  for(k=0; k<N; k++) {
                     if(k == i || k == j) continue;
                     Q[i+(j*(j+1))/2] *= (x-X[k]);
                  }
               }
            }
         }
      }
      y = dydx = T(0);
      for(i=0; i<N; i++) {
         y += Y[i]*(P[i]/D[i]);
         T S(0);
         for(k=0; k<N; k++) if(i != k) {
            if(k<i) S += Q[k+(i*(i+1))/2]/D[i];
            else    S += Q[i+(k*(k+1))/2]/D[i];
         }
         dydx += Y[i]*S;
      }
   }  // end void LagrangeInterpolation(vector, vector, const T, T&, T&)


      /// Returns the second derivative of Lagrange interpolation.
   template <class T>
   T LagrangeInterpolating2ndDerivative(const std::vector<T>& pos,
                                        const std::vector<T>& val,
                                        const T desiredPos)
   {
      int degree(pos.size());
      int i,j,m,n;

         // First, compute interpolation factors
      typedef std::vector< T > vectorType;
      std::vector< vectorType > delta(degree, vectorType(degree, 0.0));

      for(i=0; i < degree; ++i) {
         for(j=0; j < degree; ++j) {
            if(j != i) {
               delta[i][j] = ((desiredPos - pos[j])/(pos[i] - pos[j]));
            }
         }
      }

      double retVal(0.0);
      for(i=0; i < degree; ++i) {
         double sum(0.0);

         for(m=0; m < degree; ++m) {
            if(m != i) {
               double weight1(1.0/(pos[i]-pos[m]));
               double sum2(0.0);

               for(j=0; j < degree; ++j) {
                  if((j != i) && (j != m)) {
                     double weight2(1.0/(pos[i]-pos[j]));
                     for(n=0; n < degree; ++n) {
                        if((n != j) && (n != m) && (n != i)) {
                           weight2 *= delta[i][n];
                        }
                     }
                     sum2 += weight2;
                  }
               }
               sum += sum2*weight1;
            }
         }
         retVal += val[i] * sum;
      }

      return retVal;

   }  // End of 'lagrangeInterpolating2ndDerivative()'

#define tswap(x,y) { T tmp; tmp = x; x = y; y = tmp; }

   /// Perform the root sum square of aa, bb and cc
   template <class T>
   T RSS (T aa, T bb, T cc)
   {
      T a(ABS(aa)), b(ABS(bb)), c(ABS(cc));
      if(a < b) tswap(a,b);
      if(a < c) tswap(a,c);
      if(a == T(0)) return T(0);
      return a * SQRT(1 + (b/a)*(b/a) + (c/a)*(c/a));
   }

   /// Perform the root sum square of aa, bb
   template <class T>
   T RSS (T aa, T bb)
   {
      return RSS(aa,bb,T(0));
   }

   /// Perform the root sum square of aa, bb, cc and dd
   template <class T>
   T RSS (T aa, T bb, T cc, T dd)
   {
      T a(ABS(aa)), b(ABS(bb)), c(ABS(cc)), d(ABS(dd));
      // For numerical reason, let's just put the biggest in "a" (we are not sorting)
      if(a < b) tswap(a,b);
      if(a < c) tswap(a,c);
      if(a < d) tswap(a,d);
      if(a == T(0)) return T(0);
      return a * SQRT(1 + (b/a)*(b/a) + (c/a)*(c/a) + (d/a)*(d/a));
   }

#undef tswap

   inline double Round(double x)
   {
      return double(std::floor(x+0.5));
   }

   //@}

}  // namespace gpstk

#endif
