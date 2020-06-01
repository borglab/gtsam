#pragma ident "$Id$"



/**
 * @file convhelp.hpp
 * Conversion of units (deg F to C, meters to cycles, etc)
 */

#ifndef GPSTK_CONVHELP_HPP
#define GPSTK_CONVHELP_HPP

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

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================






#include "EllipsoidModel.hpp"

namespace gpstk
{
      /** @defgroup geodeticgroup Geodetic coordinates and geoids */
      //@{

      /**
       * Convert a phase and frequency to meters
       * @param phase Phase in cycles (radians?)
       * @param freq Frequency in Hertz
       * @param ellipsoid geodetic parameters (for c)
       * @return Range in meters
       * @see meters2cycles
       */
   inline double cycles2meters(double phase, double freq, EllipsoidModel& ellipsoid)
   {
      return ellipsoid.c()/freq * phase;
   }
   
      /**
       * Convert a range and frequency to cycles
       * @param range Distance in meters
       * @param freq Frequency in Hertz
       * @param ellipsoid geodetic parameters (for c)
       * @return Phase in cycles (radians?)
       * @see cycles2meters
       */
   inline double meters2cycles(double range, double freq, EllipsoidModel& ellipsoid)
   {
      return freq/ellipsoid.c() * range;
   }
   
      /**
       * Convert a temperature from Celsius to Fahrenheit
       * @param c Temperature in degrees Celsius
       * @return Temperature in degrees Fahrenheit
       * @see far2cel
       */
   inline double cel2far(double c)
   {
      return 9.0 / 5.0 * c + 32;
   }
   
      /**
       * Convert a temperature from Fahrenheit to Celsius
       * @param f Temperature in degrees Fahrenheit
       * @return Temperature in degrees Celsius
       * @see cel2far
       */
   inline double far2cel(double f)
   {
      return 5.0 / 9.0 * (f - 32);
   }
   
      /**
       * Convert pressure from millibars to inches of mercury
       * @param mb Pressure in millibars
       * @return Pressure in inches of mercury
       * @see hg2mb
       */
   inline double mb2hg(double mb)
   {
      return mb / 33.8638815789;
   }
   
      /**
       * Convert pressure from inches of mercury to millibars
       * @param hg Pressure in inches of mercury
       * @return Pressure in millibars
       * @see mb2hg
       */
   inline double hg2mb(double hg)
   {
      return hg * 33.8638815789;
   }

   //@}

} // namespace

#endif
