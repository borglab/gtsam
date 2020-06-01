#pragma ident "$Id$"

/**
 * @file RK4VehicleModel.cpp
 * This class implements a simple model of a vehicle based on the Runge-Kutta-4
 * numerical integration algorithm.
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


#include "RK4VehicleModel.hpp"


namespace gpstk
{


      /* Set acceleration
       *
       * @param acceleration     Accelerations matrix (3D).
       */
   void RK4VehicleModel::setAcceleration( const Matrix<double>& acceleration )
   {

         // Set accelerations
      ax = acceleration(0,0);
      ay = acceleration(1,0);
      az = acceleration(2,0);

      return;

   }  // End of method 'setAcceleration()'



      /* Set acceleration
       *
       * @param accelerationX     Acceleration X component.
       * @param accelerationY     Acceleration Y component.
       * @param accelerationZ     Acceleration Z component.
       */
   void RK4VehicleModel::setAcceleration( double accelerationX,
                                          double accelerationY,
                                          double accelerationZ )
   {

         // Set accelerations
      ax = accelerationX;
      ay = accelerationY;
      az = accelerationZ;

      return;

   }  // End of method 'setAcceleration()'



      /* Implements "derivative()". It is based on accelerations.
       *
       * @param time          Time step.
       * @param inState       Internal state matrix.
       * @param inStateDot    Derivative of internal state matrix.
       */
   Matrix<double>& RK4VehicleModel::derivative( long double time,
                                                const Matrix<double>& inState,
                                                Matrix<double>& inStateDot )
   {

         // Let's insert data related to X coordinates
      inStateDot(0,0) = inState(1,0);     // Set X'  = Vx
      inStateDot(1,0) = ax;               // Set Vx' = Ax

         // Let's insert data related to Y coordinates
      inStateDot(2,0) = inState(3,0);     // Set Y'  = Vy
      inStateDot(3,0) = ay;               // Set Vy' = Ay

         // Let's insert data related to Z coordinates
      inStateDot(4,0) = inState(5,0);     // Set Z'  = Vz
      inStateDot(5,0) = az;               // Set Vz' = Az

      return inStateDot;

   }  // End of method 'RK4VehicleModel::derivative()'


}  // End of namespace gpstk
