#pragma ident "$Id$"

/**
 * @file FIRDifferentiator5thOrder.hpp
 * Class of Finite Impulsive Response (FIR) Differentiator filters of 5th order.
 */

#ifndef GPSTK_FIRDIFFERENTIATOR5THORDER_HPP
#define GPSTK_FIRDIFFERENTIATOR5THORDER_HPP

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



#include <cmath>
#include <deque>

#include "FilterBase.hpp"



namespace gpstk
{

      /** @addtogroup GPSsolutions */
      /// @ingroup math

      //@{


      /** This class implements a Finite Impulsive Response (FIR)
       *  Differentiator filter of 5th order designed according to central
       *  difference approximation.
       *
       * Further information about this filter and its use in GNSS may be found
       * at:
       *
       *    Kennedy, S. L. (2002) Acceleration Estimation from GPS Carrier
       *    Phases for Airborne Gravimetry. Master’s thesis, University of
       *    Calgary. URL http://www.geomatics.ucalgary.ca/
       *
       * This filter shows constant, linear phase response over the passband.
       * It is very important to note that the CURRENT derivative corresponds
       * to FIVE EPOCHS BEFORE.
       *
       * A typical way to use this class follows:
       *
       * @code
       *      // Set up function parameters
       *   const double Tf(15.0);         // Function period
       *   const double omega( TWO_PI/Tf );
       *
       *      // Set filter parameters
       *   const double Ts(0.1);          // Sampling period
       *
       *      // Declare and configure an FIRDifferentiator5thOrder object
       *   FIRDifferentiator5thOrder firDiff( Ts );
       *
       *      // Let's iterate and print function values and derivatives
       *   for(int i = 0; i < 200; ++i)
       *   {
       *      double t( static_cast<double>(i*Ts) );
       *
       *      double y( sin(omega*t) );            // Function is sine
       *      double dy( omega*cos(omega*t) );     // Sine derivative
       *
       *      double dy2( firDiff.Compute(y) );
       *
       *         // Print results
       *      cout << t    << "  "
       *           << y    << "  "
       *           << dy   << "  "
       *           << dy2  << endl;
       *   }
       *
       * @endcode
       *
       */
   class FIRDifferentiator5thOrder : public FilterBase
   {
   public:


         /// Default constructor.
      FIRDifferentiator5thOrder()
         : T(1.0)
      { Init(); };


         /** Common constructor.
          *
          * @param[in] period      Sampling period, in seconds.
          */
      FIRDifferentiator5thOrder( double period )
      { setT(period); Init(); };


         /** Return result.
          *
          * @param input      Input data.
          */
      virtual double Compute( double input );


         /// Resets filter, cleaning its internal state.
      virtual void Reset(void);


         /// Get the sampling period, in seconds.
      virtual double getT( void ) const
      { return T; };


         /** Set the sampling period, in seconds.
          *
          * @param[in] period      Sampling period, in seconds.
          *
          * @warning Only values higher that zero are allowed. Other values will
          * be ignored.
          *
          * @warning This operation resets the filter.
          */
      virtual FIRDifferentiator5thOrder& setT( double period );


         /// Destructor
      virtual ~FIRDifferentiator5thOrder() {};


   private:


         /// Sampling period, in seconds.
      double T;


         /// Vector storing input
      std::deque<double> X;


         /// General filter parameters.
      double k1;
      double k2;
      double k3;
      double k4;
      double k5;

         /// Initialization method
      void Init( void );


   }; // End of class 'FIRDifferentiator5thOrder'


      //@}

}  // End of namespace gpstk

#endif   // GPSTK_FIRDIFFERENTIATOR5THORDER_HPP
