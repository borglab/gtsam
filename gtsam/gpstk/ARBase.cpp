#pragma ident "$Id$"

/**
 * @file ARBase.cpp
 * 
 */
 
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
      // This method to get the convert matrix from SD ambiguity to DD 
      // ambiguity
   Matrix<double> ARBase::sd2ddMatrix(const size_t& n, const size_t& i)
      throw(ARException)
   {
      if( i >= n)
      {
         Exception e("The reference index CAN NOT greater than toltal number.");
         GPSTK_THROW(e);
      }

      Matrix<double> sdMat(n-1, n, 0.0);

      for(size_t j = 0; j< sdMat.rows(); j++)
      {
         sdMat(j,i) = -1.0;
         if(j<i) sdMat(j,j) = 1.0;  
         else    sdMat(j,j+1) = 1.0;    
      }

      return sdMat;

   }  // End of method 'ARBase::sd2ddMatrix()'
   
      // Compute float ambiguity by selection of the parameter weight method
   void ARBase::computeFloatAmbiguity(const Matrix<double>& A, 
                                      const Matrix<double>& B, 
                                      const Matrix<double>& W,
                                      const Vector<double>& y, 
                                      Vector<double>& ambFloat,
                                      Matrix<double>& ambCov,
                                      double smFactor)
      throw(ARException)
   {
      try
      {
            // check the smooth factor
         if( (smFactor< 0.001) || (smFactor>0.0001))
         {
            smFactor = 0.001;
         }

         Matrix<double> I = ident<double>(A.rows());
         Matrix<double> AT = transpose(A);
         Matrix<double> ATW = AT*W;

         Matrix<double> Px = diag(ATW*A);

         Matrix<double> Ja = I - A * inverseSVD(ATW * A + smFactor*Px) * ATW;

         Matrix<double> BTWJ = transpose(B) * W * Ja;

         ambFloat= inverseSVD(BTWJ * B) * BTWJ * y;
         ambCov = inverseSVD(BTWJ * B);
      }
      catch (...)
      {
         ARException e("Failed to compute float ambiguity.");
         GPSTK_THROW(e);
      }
   
   }  // End of method 'ARLambda::computeFloatAmbiguity()'

}   // End of namespace gpstk

