#pragma ident "$Id$"

/**
 * @file AstronomicalFunctions.hpp
 * Useful functions used in astronomical computations.
 */

#ifndef ASTRONOMICALFUNCTIONS_HPP
#define ASTRONOMICALFUNCTIONS_HPP

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


namespace gpstk
{
      /** @addtogroup ephemcalc */
      //@{


      /// Astronomical Unit value (AU), in meters
   const double AU_CONST = 1.49597870e11;

      /// Mean Earth-Moon barycenter (EMB) distance (AU)
   const double MeanEarthMoonBary = 3.12e-5;

      /// Ratio of mass Sun to Earth
   const double MU_SUN = 332946.0;

      /// Ratio of mass Moon to Earth
   const double MU_MOON = 0.01230002;

      /// Earth gravity acceleration on surface (m/s^2)
   const double EarthGrav = 9.80665;

      /// Degrees to radians
   const double D2R = 0.0174532925199432957692369;

      /// Arcseconds to radians
   const double DAS2R = 4.848136811095359935899141e-6;

      /// Seconds of time to radians
   const double DS2R = 7.272205216643039903848712e-5;

      /// Julian epoch of B1950
   const double B1950 = 1949.9997904423;

      /// Earth equatorial radius in AU ( 6378.137 km / 149597870 km)
   const double ERADAU = 4.2635212653763e-5;


      /** Function to change from CIS to CTS(ECEF) coordinate system
       * (coordinates in meters)
       * @param posCis    Coordinates in CIS system (in meters).
       * @param t         Epoch
       *
       * @return Triple in CTS(ECEF) coordinate system.
       */
   Triple CIS2CTS(const Triple posCIS,
                  const CommonTime& t);


      /** Function to convert from UTC to sidereal time
       * @param t         Epoch
       *
       * @return sidereal time in hours.
       */
   double UTC2SID(const CommonTime& t);

      //@}

} // namespace gpstk
#endif  // ASTRONOMICALFUNCTIONS_HPP
