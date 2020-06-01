#pragma ident "$Id$"

/**
 * @file RK4VehicleModel.hpp
 * This class implements a simple model of a vehicle based on the Runge-Kutta-4
 * numerical integration algorithm.
 */

#ifndef GPSTK_RK4VEHICLEMODEL_HPP
#define GPSTK_RK4VEHICLEMODEL_HPP

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


#include "RungeKutta4.hpp"


namespace gpstk
{

      /** @defgroup vehicles Tools to model vehicles */
      //@{

      /// This class implements a simple kinematic model of a vehicle.
   class RK4VehicleModel : public RungeKutta4
   {
   public:


         /** Common constructor.
          *
          * @param initialState     Initial state matrix.
          * @param initialTime      Initial time.
          * @param timeEpsilon      Time epsilon for Runge-Kutta algorithm.
          */
      RK4VehicleModel( const Matrix<double>& initialState,
                       double initialTime = 0.0,
                       double timeEpsilon = 1e-18 )
         : RungeKutta4(initialState, initialTime, timeEpsilon)
      {};


         /** Set acceleration
          *
          * @param acceleration     Accelerations matrix (3D).
          */
      void setAcceleration( const Matrix<double>& acceleration );


         /** Set acceleration
          *
          * @param accelerationX     Acceleration X component.
          * @param accelerationY     Acceleration Y component.
          * @param accelerationZ     Acceleration Z component.
          */
      void setAcceleration( double accelerationX = 0.0,
                            double accelerationY = 0.0,
                            double accelerationZ = 0.0 );


   protected:


      double ax;   //< acceleration in X component
      double ay;   //< acceleration in Y component
      double az;   //< acceleration in Z component


   private:


         /** Implements "derivative()". It is based on accelerations.
          *
          * @param time          Time step.
          * @param inState       Internal state matrix.
          * @param inStateDot    Derivative of internal state matrix.
          */
      virtual Matrix<double>& derivative( long double time,
                                          const Matrix<double>& inState,
                                          Matrix<double>& inStateDot );


   };  // End of class "RK4VehicleModel"

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_RK4VEHICLEMODEL_HPP
