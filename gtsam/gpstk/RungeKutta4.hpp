#pragma ident "$Id$"

/**
 * @file RungeKutta4.hpp
 * RungeKutta4 class. Provides several versions of the Runge Kutta integration
 * The algorithms are based on Ch.16 of "Numerical Recipes in C" but the
 * implementation is entirely independent, i.e., this isn't the source
 * distributed with the text.
 */

#ifndef GPSTK_RUNGEKUTTA4_HPP
#define GPSTK_RUNGEKUTTA4_HPP



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


#include "Matrix.hpp"

namespace gpstk
{

      /** @addtogroup math */
      //@{

      /** The RungeKutta4 class provides a collection of integration routines
       * that work on a Matrix of doubles.  Integrations use a fixed step-size.
       */
   class RungeKutta4 
   {
   public:
         /** Constructor.
          * @param initalState a reference to the original Matrix to work on 
          *  that is copied to an internal Matrix.
          * @param initialTime the time at which to begin integrations 
          * @param timeEpsilon how close the final internal timestep must match
          *  the specified final time of an integration
          */
      RungeKutta4(const Matrix<double>& initialState,
                  double initialTime=0,
                  double timeEpsilon=1e-18)
         : currentTime(initialTime), currentState(initialState),
           teps(timeEpsilon), M(initialState.rows()), N(initialState.cols()),
           k1(M,N), k2(M,N), k3(M,N), k4(M,N), yn(M,N), tempy(M,N)
      { }

         /** The classic Runge Kutta 4th Order Integration Algorithm.
          * This routine integrates using a Runge Kutta 4th order algorithm 
          * with a fixed step from the internal time to \a nextTime. 
          * @param nextTime the time to integrate to
          * @param stepSize the amount time between internal integration steps
          */
      virtual void integrateTo ( double nextTime,
                                 double stepSize = 0 );

         /** The classic Runge Kutta 4th-5th Order Integration Algorithm.
          *  This function integrates by applying a 4th order Runge Kutta
          *  algorithm multiple times. This provides two benefits. First, an
          *  estimate of the truncation error is returned. Second, the multiple
          *  4th order estimates are combined to produce the 5th order estimate.
          * @param nextTime the time to integrate to
          * @param error the Matrix of estimated integration error 
          *  (one for each element)
          * @param stepSize the amount time between internal integration steps
          */
      virtual void integrateTo ( double nextTime,
                                 Matrix<double>& error,
                                 double stepSize = 0 );

         /** This is the function to be integrated. 
          * @param time the time at which to evaluate the derivative
          * @param inState the Matrix to evaluate the derivative of at /a time.
          * @param inStateDot the derivative of /a inState evaluated at /a time.
          * @return a reference to /a inStateDot
          */
      virtual gpstk::Matrix<double>& derivative( long double time,
                                       const gpstk::Matrix<double>& inState,
                                       gpstk::Matrix<double>& inStateDot) = 0;

         /// Return the currnet time of the system.
      double getTime(void) 
      { return currentTime; }

         /// Return the current state of the system.
      const Matrix<double>& getState(void) 
      { return currentState; }


   protected:


         /// Current time of the system
      double currentTime;

         /// State of the system at the current time
      gpstk::Matrix<double> currentState;

      double teps;   //< Precision for time calculations and comparisons
      int M;         //< Number of rows in the state
      int N;         //< Number of columns in the state


   private:


         /// Disallow copy constructor
      RungeKutta4(const RungeKutta4& cloneDonor);

         /// Disallow the assignment operator
      RungeKutta4& operator= (const RungeKutta4& right);

         /** These values are only used in the integrateTo method(s).
          *  Declaring them here keeps them from being constructed every
          *  time the integrateTo method(s) are called.
          */
      Matrix<double> k1, k2, k3, k4, yn, tempy;


   }; // End of class 'RungeKutta4'

   //@}

}  // End of namespace gpstk

#endif   // GPSTK_RUNGEKUTTA4_HPP


