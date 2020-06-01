#pragma ident "$Id$"

/**
 * @file ARSimple.cpp
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

#include "ARSimple.hpp"


namespace gpstk
{
   Vector<double> ARSimple::resolveIntegerAmbiguity( 
                                                 const Vector<double>& ambFloat, 
                                                 const Matrix<double>& ambCov )
      throw(ARException)
   {
         // check input
      if( ambFloat.size()!=ambCov.rows() || ambFloat.size()!=ambCov.cols() )
      {
         ARException e("The dimension of input does not match.");
         GPSTK_THROW(e);
      }

      const size_t n = ambFloat.size();

      Vector<double> ambFixed(n,0.0);
      for(size_t i = 0; i < ambFloat.size(); i++)
      {
         const double threshold = 3.0 * std::sqrt(ambCov(i,i));

         double lowerValue = ambFloat(i) - threshold;
         double upperValue = ambFloat(i) + threshold;

         if( std::abs(upperValue-lowerValue) <= 1.0 )
         {
            ambFixed(i) = double( std::floor(ambFloat(i)+0.5) );
         }
         else
         {
            ambFixed(i) = ambFloat(i);       // keep it with float value
         }
      }

      return ambFixed;

   }  // End of method 'ARSimple::resolveIntegerAmbiguity()'

   
}   // End of namespace gpstk

