#pragma ident "$Id$"



/**
 * @file PolyFit.hpp
 * Least squares fit using a polynomial model.
 */
 
#ifndef GPSTK_POLYFIT_HPP
#define GPSTK_POLYFIT_HPP

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






#include "Matrix.hpp"

namespace gpstk
{
   /** @addtogroup math */
   //@{

/**
 * Compute a polynomial fit of the form sum[X(i)*t**i] = d, that is solve for
 * coefficients X given a set of data pairs (t,d). The dimension of X is n, the
 * degree of the polynomial.
 * @code
 * unsigned int i,n=4;
 * double dat[17]={...}, times[17]={...};
 * PolyFit<double> PF(n);
 * for(i=0; i<17; i++)
 *    PF.Add(dat[i],times[i]);
 * 
 * cout << "Solution vector: " << PF.Solution() << endl;
 * cout << "Covariance matrix: " << PF.Covariance() << endl;
 * for(i=0; i<17; i++)
 *    cout << times[i] << " " << dat[i] << " " << PF.Evaluate(times[i]) << endl;
 * @endcode
 */
   template <class T>
   class PolyFit
   {
   public:
         /// Empty constructor
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
      PolyFit() : n_(0), Inverted(false), Singular(true), Npts(0) {}
#pragma clang diagnostic pop
         /// Constructor given an initial size.
      PolyFit(unsigned int n) : n_(n), Inverted(false), Singular(true), Npts(0)
      {
         InfData.resize(n_,T(0));
         X.resize(n_);
         InfMatrix.resize(n_,n_,T(0));
         Cov.resize(n_,n_);
      }

         /** Reset the estimation, i.e. restart with new data, with new dimension.
          * Default dimension is 0, meaning do not change dimension.
          */
      void Reset(unsigned int n=0)
      {
         if(n != 0 && n_ != n) {
            InfData.resize(n,T(0));
            X.resize(n);
            InfMatrix.resize(n,n,T(0));
            Cov.resize(n,n);
            n_ = n;
         }
         Npts = 0;
         InfData = T(0);
         X = T(0);
         InfMatrix = T(0);
         Cov = T(0);
         Singular = true;
         Inverted = false;
      }

         /// Add a single (optional: weighted) datum to the estimation.
      void Add(T d, T t, T w=T(1))
      {
         //             n_-1
         // Equation is sum[t^i * X(i)] = d     OR    P * X = d
         //             i=0                          1xn_ n_  1
         Vector<T> P(n_);
         T tt=T(1);
         for(size_t i=0; i<n_; i++) { P(i)=tt; tt *= t; }
         Npts++;
         Matrix<T> PP;
         PP = outer(P,P);
         PP *= w;
         InfMatrix += PP;
         P *= d*w;
         InfData += P;
         Inverted = false;
      }

         /// Add a gpstk::Vector of data to the estimation.
      void Add(const Vector<T>& d, const Vector<T>& t)
      {
         size_t m=d.size();
         if(t.size() < m) m=t.size();

         Matrix<T> P(m,n_);
         Vector<T> D(d);
         D.resize(m);

         for(size_t j=0; j<m; j++) {
            T tt=T(1);
            for(size_t i=0; i<n_; i++) { P(j,i)=tt; tt *= t(j); }
         }
         Npts += m;
         Matrix<T> PTP,PT;
         PT = transpose(P);
         PTP = PT*P;
         InfMatrix += PTP;
         InfData += PT*D;
         Inverted = false;
      }

         /// Add a std::vector of data to the estimation.
      void Add(const std::vector<T>& d, const std::vector<T>& t)
      {
         size_t m=d.size();
         if(t.size() < m) m=t.size();

         Matrix<T> P(m,n_);
         Vector<T> D;
         D.resize(m);
         for(int i=0; i<m; i++) D(i)=d[i];

         for(size_t j=0; j<m; j++) {
            T tt=T(1);
            for(size_t i=0; i<n_; i++) { P(j,i)=tt; tt *= t[j]; }
         }
         Npts += m;
         Matrix<T> PTP,PT;
         PT = transpose(P);
         PTP = PT*P;
         InfMatrix += PTP;
         InfData += PT*D;
         Inverted = false;
      }

         /// Evaluate the polynomial at the given time; singular problems return zero.
      T Evaluate(T t)
      {
         if(n_ <= 0) { Singular=true; return T(0); }
         Solve();
         if(Singular) return T(0);

         T d,tn;
         d = X(0);
         tn = t;
         for(size_t i=1; i<X.size(); i++) {
            d += X(i)*tn;
            tn *= t;
         }
         return d;
      }

         /// Evaluate the polynomial at a Vector of times;
         /// singular problems return an empty vector.
      Vector<T> Evaluate(const Vector<T>& Vt)
      {
         Vector<T> R;
         if(n_ <= 0) { Singular=true; return R; }
         Solve();
         if(Singular) return R;

         T tn;
         R = Vector<T>(Vt.size());
         for(size_t j=0; j<Vt.size(); j++) {
            R(j) = X(0);
            tn = Vt(j);
            for(size_t i=1; i<X.size(); i++) {
               R(j) += X(i)*tn;
               tn *= Vt(j);
            }
         }
         return R;
      }

         /// is the problem singular?
      inline bool isSingular(void) { Solve(); return Singular; }
         /// get the solution vector (coefficients)
      inline Vector<T> Solution(void) { Solve(); return X; }
         /// get the covariance matrix
      inline Matrix<T> Covariance(void) { Solve(); return Cov; }
         /// get the degree of the polynomial
      inline unsigned int Degree(void) const { return n_; }
         /// get the number of data points processed
      inline unsigned int N(void) const { return Npts; }

   private:
         /// Invert the equation
      inline void Solve(void)
      {
         if(Inverted) return;
         try { Cov=inverse(InfMatrix); }
         catch (gpstk::Exception& e) { Singular=true; return; }
         Singular = false;
         X = Cov * InfData;
         Inverted = true;
      }

         /// degree of polynomial to be fit (dimension of state).
      unsigned int n_;
         /// number of data points added to the estimation so far.
      unsigned int Npts;
         /// information matrix = inverse(Cov) = sum[transpose(P)*P], P=partials.
      Matrix<T> InfMatrix;
         /// information data = inverse(Cov)*X = sum[transpose(P)*data].
      Vector<T> InfData;
         /// flag indicating problem has been inverted.
      bool Inverted;
         /// flag indicating problem is singular.
      bool Singular;
         /// State vector (array of polynomial coefficients) of size n_.
      Vector<T> X;
         /// Covariance matrix
      Matrix<T> Cov;

   }; // end class PolyFit

   //@}

}  // namespace gpstk

#endif
