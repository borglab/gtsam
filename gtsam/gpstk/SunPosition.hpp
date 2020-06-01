#pragma ident "$Id$"

/**
 * @file SunPosition.hpp
 * Returns the approximate position of the Sun at the given epoch in the
 * ECEF system.
 */

#ifndef SUNPOSITION_HPP
#define SUNPOSITION_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007
//
//============================================================================


#include <cmath>
#include <string>

#include "CommonTime.hpp"
#include "Triple.hpp"
#include "GNSSconstants.hpp"
#include "AstronomicalFunctions.hpp"



namespace gpstk
{

      /** @addtogroup ephemcalc */
      //@{

      /** This class computes the approximate position of the Sun at the
       * given epoch in the ECEF system. It yields best results between
       * March 1st 1900 and February 28th 2100.
       *
       * This is a C++ implementation version based on the FORTRAN version
       * originally written by P.T. Wallace, Starlink Project. The FORTRAN
       * version of Starlink project was available under the GPL license.
       *
       * Errors in position in the period 1950-2050 are:
       *
       * \li Maximum: 13*10^-5 AU (19200 km).
       * \li RMS: 5*10^-5 AU (7600 km).
       *
       * More information may be found in http://starlink.jach.hawaii.edu/
       */
   class SunPosition
   {
   public:

         /// Default constructor
      SunPosition() throw() {}

         /// Destructor
      virtual ~SunPosition() {}


         /** Returns the position of Sun ECEF coordinates (meters) at the
          *  indicated time.
          *
          * @param[in]  t the time to look up
          *
          * @return  the position of the Sun at time (as a Triple)
          *
          * @throw InvalidRequest If the request can not be completed for any
          *    reason, this is thrown. The text may have additional
          *    information as to why the request failed.
          *
          * @warning This method yields an approximate result, given that
          *    pole movement is not taken into account, neither precession
          *    nor nutation.
          */
      Triple getPosition(const CommonTime& t) const 
         throw(InvalidRequest);


         /** Function to compute Sun position in CIS system (coordinates
          *  in meters)
          *
          * @param t Epoch
          */
      Triple getPositionCIS(const CommonTime& t) const 
         throw(InvalidRequest);


         /** Determine the earliest time for which this object can
          *  successfully determine the position for the Sun.
          *
          * @return The initial time
          *
          * @throw InvalidRequest This is thrown if the object has no data.
          */
      CommonTime getInitialTime() const throw(InvalidRequest)
      { return initialTime; }


         /** Determine the latest time for which this object can
          *  successfully determine the position for the Sun.
          *
          * @return The final time
          *
          * @throw InvalidRequest This is thrown if the object has no data.
          */
      CommonTime getFinalTime() const throw(InvalidRequest)
      { return finalTime; }


   private:

         /// Time of the first valid time
      static const CommonTime initialTime;

         /// Time of the last valid time
      static const CommonTime finalTime;

   }; // end class SunPosition


      //@}

} // namespace gpstk
#endif  // SUNPOSITION_HPP
