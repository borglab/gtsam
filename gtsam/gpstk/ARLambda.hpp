#pragma ident "$Id$"

/**
 * @file ARLambda.hpp
 * 
 */

#ifndef GPSTK_ARLAMBDA_HPP
#define GPSTK_ARLAMBDA_HPP

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
//  Wei Yan - Chinese Academy of Sciences . 2011
//
//============================================================================

#include "ARBase.hpp"

namespace gpstk
{
      /** This class resolve integer ambiguity by the LAMBDA method.
       *
       * The algorithm was described by:
       *
       *   P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
       *   a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
       *   1995
       */
   class ARLambda : public ARBase
   {
   public:
      
         /// Default constructor
      ARLambda() : squaredRatio(0.0) {}      
      

         /// Integer Ambiguity Resolution method
      virtual Vector<double> resolveIntegerAmbiguity( 
                                                const Vector<double>& ambFloat, 
                                                const Matrix<double>& ambCov )
         throw(ARException);
      

         /// Destractor
      virtual ~ARLambda(){}


      bool isFixedSuccessfully(double threshhold = 3.0)
      { return (squaredRatio>threshhold)?true:false; }
      

      double squaredRatio;

   protected:

      double sign(double x)
      { return (x<=0.0)?-1.0:1.0; }


      double round(double x)
      { return double(std::floor(x+0.5));}


      void swap(double& a,double& b)
      { double t(a); a = b; b = t; }


         // Q = L'*diag(D)*L
      int factorize( const Matrix<double>& Q,
                     Matrix<double>& L, 
                     Vector<double>& D );


         /// integer gauss transformation
      void gauss(Matrix<double>& L, Matrix<double>& Z, int i, int j );


         /// permutations
      void permute( Matrix<double>& L, 
                    Vector<double>& D, 
                    int j, 
                    double del, 
                    Matrix<double>& Z);


         /// lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L)
      void reduction( Matrix<double>& L, 
                      Vector<double>& D, 
                      Matrix<double>& Z );


         // modified lambda (mlambda) search
      virtual int search( const Matrix<double>& L, 
                          const Vector<double>& D, 
                          const Vector<double>& zs, 
                          Matrix<double>& zn, 
                          Vector<double>& s, 
                          const int& m = 2 );

         // lambda/mlambda integer least-square estimation
         // a     Float parameters (n x 1)
         // Q     Covariance matrix of float parameters (n x n)
         // F     Fixed solutions (n x m)
         // s     Sum of squared residulas of fixed solutions (1 x m)
         // m     Number of fixed solutions
         //      status (0:ok,other:error)
      int lambda( const Vector<double>& a, 
                  const Matrix<double>& Q, 
                  Matrix<double>& F, 
                  Vector<double>& s, 
                  const int& m = 2 );
      
   protected:

      
         
   };   // End of class 'ARLambda'
   
}   // End of namespace gpstk


#endif  //GPSTK_ARLAMBDA_HPP

