#pragma ident "$Id$"

/**
 * @file FIRDifferentiator5thOrder.cpp
 * Class of Finite Impulsive Response (FIR) Differentiator filters of 5th order.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2011
//
//============================================================================


#include "FIRDifferentiator5thOrder.hpp"



namespace gpstk
{


      /* Return result.
       *
       * @param input      Input data.
       */
   double FIRDifferentiator5thOrder::Compute( double input )
   {

         // Be default, the filter is invalid
      valid = false;

         // Default result
      double result( 0.0 );


         // Check if there are enough stored data values
      if( X.size() == 10 )
      {

            // Compute the result. This implements the equation:
            // y[k] = k1*x[k+5] - k2*x[k+4] + k3*x[k+3] - k4*x[k+2] + k5*x[k+1]
            //      - k5*x[k-1] + k4*x[k-2] - k3*x[k-3] + k2*x[k-4] - k1*x[k-5]

         result = k1*(input - X[9]) + k2*(X[8] - X[0]) + k3*(X[1] - X[7])
                                    + k4*(X[6] - X[2]) + k5*(X[3] - X[5]);

            // Filter result is valid
         valid = true;
      }

         // Update filter state

         // Insert input in input vector
      X.push_front( input );

         // Delete old inputs
      if(X.size() > 10) X.pop_back();

         // Return result
      return result;

   }  // End of constructor 'FIRDifferentiator5thOrder::Compute()'



      // Resets filter, cleaning its internal state.
   void FIRDifferentiator5thOrder::Reset(void)
   {
         // Clear stored input values
      X.clear();

         // Filter state is invalid
      valid = false;

         // Return
      return;

   }  // End of constructor 'FIRDifferentiator5thOrder::Reset()'



      /* Set the sampling period, in seconds.
       *
       * @param[in] period      Sampling period, in seconds.
       *
       * @warning Only values higher that zero are allowed. Other values will
       * be ignored.
       *
       * @warning This operation resets the filter.
       */
   FIRDifferentiator5thOrder& FIRDifferentiator5thOrder::setT( double period )
   {
         // Check period
      if( period > 0.0 )
      {
         T = period;
         Init();
         Reset();
      }

      return (*this);

   }  // End of method 'FIRDifferentiator5thOrder::setT()'



      // Initialization method
   void FIRDifferentiator5thOrder::Init( void )
   {

         // Generate filter parameters
      k1 = 1.0/(1260.0*T);
      k2 = k1*(12.5);
      k3 = k1*(75.0);
      k4 = k1*(300.0);
      k5 = k1*(1050.0);

         // Filter parameters are set. Let's return
      return;

   }  // End of method 'FIRDifferentiator5thOrder::Init()'


}  // End of namespace gpstk
