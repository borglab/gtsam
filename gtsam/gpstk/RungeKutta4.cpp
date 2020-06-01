#pragma ident "$Id$"

/**
 * @file RungeKutta4.hpp
 * Implementation of a Runge Kutta integrator.
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
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================


#include "RungeKutta4.hpp"


void gpstk::RungeKutta4::integrateTo ( double nextTime,
                                       double stepSize )
{
   if ( stepSize == 0.0 )
      stepSize = nextTime - currentTime;

   bool done = false;

   while (!done)
   {

         // Time steps
      double ctPlusDeltaT = currentTime + stepSize;
      double ctPlusHalfDeltaT = currentTime + (stepSize * 0.5);

         // k1
      k1 = stepSize * derivative(currentTime, currentState, k1);
      tempy = currentState + (0.5 * k1);

         // k2
      k2 = stepSize * derivative(ctPlusHalfDeltaT, tempy, k2);
      tempy = currentState + (0.5 * k2);

         // k3
      k3 = stepSize * derivative(ctPlusHalfDeltaT, tempy, k3);

         // k4
      k4 = stepSize * derivative(ctPlusDeltaT, tempy, k4);
      currentState += (k1 + 2.0 * (k2 + k3) + k4) / 6.0;

         // If we are within teps of the goal time, we are done.
      if (fabs(currentTime + stepSize - nextTime) < teps)
         done = true;

         // If we are about to overstep, change the stepsize appropriately
         // to hit our target final time.
      if( stepSize > 0.0 )
      {
         if( (currentTime + stepSize) > nextTime )
            stepSize = (nextTime - currentTime);
      }
      else
      {
         if ( (currentTime + stepSize) < nextTime )
            stepSize = (nextTime - currentTime);
      }
      
      currentTime += stepSize;
   }

   currentTime = nextTime;

}  // End of method 'gpstk::RungeKutta4::integrateTo( nextTime, stepSize )'


void gpstk::RungeKutta4::integrateTo ( double nextTime,
                                       Matrix<double>& error,
                                       double stepSize )
{

   double deltaT = nextTime - currentTime;

      // Save the current state and time for the second step.
   double savedTime = currentTime;
   gpstk::Matrix<double> savedState = currentState;

      // First, take the integration using two steps.
   integrateTo(currentTime + (deltaT * 0.5), stepSize);
   integrateTo(nextTime, stepSize);

      // Save the results.
   gpstk::Matrix<double> twoStepState = currentState;

      // Restore the original state.
   currentTime = savedTime;
   currentState = savedState;

      // Now, take the integration using only one step.
   integrateTo(nextTime, stepSize);
   gpstk::Matrix<double> oneStepState = currentState;

   error = oneStepState - twoStepState;

   currentState = twoStepState + (twoStepState - oneStepState) / 15.0;

}  // End of method 'gpstk::RungeKutta4::integrateTo(nextTime, error, stepSize)'

