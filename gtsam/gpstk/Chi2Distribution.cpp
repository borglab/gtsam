#pragma ident "$Id$"

/**
 * @file Chi2Distribution.cpp
 * This class implements the Chi-square distribution.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008
//
//============================================================================


#include "Chi2Distribution.hpp"


using namespace std;

namespace gpstk
{


      /* Computes the probability density function
       *
       * @param x    Value
       */
   double Chi2Distribution::pdf(double x)
   {

      if( x <= 0.0 )
      {
         return (0.0);
      }

         // Compute ndf/2
      double khalf( static_cast<double>(ndf) / 2.0 );

         // Compute the natural logarithm of terms
      double t1( -0.69314718055994529 * khalf );
      double t2( -lngamma( khalf ) );
      double t3( ( khalf - 1.0 ) * log(x) );
      double t4( - x / 2.0 );

      return ( exp( t1 + t2 + t3 + t4 ) );

   }  // End of method 'Chi2Distribution::pdf()'



      /* Computes the cumulative distribution function
       *
       * @param x    Value
       */
   double Chi2Distribution::cdf(double x)
   {


      if( x <= 0.0 )
      {
         return (0.0);
      }

         // Compute ndf/2
      double khalf( static_cast<double>(ndf) / 2.0 );

      return ( gammaP( khalf, (x/2.0) ) );

   }  // End of method 'Chi2Distribution::cdf()'



      /* Set the number of degrees of freedom.
       *
       * @param n       Degrees of freedom
       *
       * \warning "n" must be > 0.0, otherwise n = |n|.
       */
   Chi2Distribution& Chi2Distribution::setNDF(int n)
      throw(InvalidParameter)
   {

      if( n == 0 )
      {
         InvalidParameter e( "Invalid value for NDF." );
         GPSTK_THROW(e);
      }

      if( n < 0 )
      {
         ndf = -n;
      }
      else
      {
         ndf = n;
      }

      return (*this);

   }  // End of method 'Chi2Distribution::setNDF()'



}  // End of namespace gpstk
