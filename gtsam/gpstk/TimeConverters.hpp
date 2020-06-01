/// @file TimeConverters.hpp

#ifndef GPSTK_TIMECONVERTERS_HPP
#define GPSTK_TIMECONVERTERS_HPP

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

namespace gpstk
{
      /** Fundamental routine to convert from "Julian day" (= JD + 0.5)
       *  to calendar day.
       * @param jd long integer "Julian day" = JD+0.5
       * @param iyear reference to integer year
       * @param imonth reference to integer month (January == 1)
       * @param iday reference to integer day of month
       *  (1st day of month == 1)
       * @note range of applicability of this routine is from 0JD (4713BC)
       *  to approx 3442448JD (4713AD).
       * Algorithm references: Sinnott, R. W. "Bits and Bytes,"
       *  Sky & Telescope Magazine, Vol 82, p. 183, August 1991, and
       *  The Astronomical Almanac, published by the U.S. Naval Observatory.
       */
   void convertJDtoCalendar( long jd,
                             int& iyear,
                             int& imonth,
                             int& iday );

      /** Fundamental routine to convert from calendar day to "Julian day"
       *  (= JD + 0.5)
       * @param iyear reference to integer year
       * @param imonth reference to integer month (January == 1)
       * @param iday reference to integer day of month
       *  (1st day of month == 1)
       * @return jd long integer "Julian day" = JD+0.5
       * @note range of applicability of this routine is from 0JD (4713BC)
       * to approx 3442448JD (4713AD).
       * Algorithm references: Sinnott, R. W. "Bits and Bytes,"
       *  Sky & Telescope Magazine, Vol 82, p. 183, August 1991, and
       *  The Astronomical Almanac, published by the U.S. Naval Observatory.
       */
   long convertCalendarToJD( int iyear,
                             int imonth,
                             int iday ) ;

      /** Fundamental routine to convert seconds of day to H:M:S
       * @param sod seconds of day (input)
       * @param hh reference to integer hour (0 <= hh < 24) (output)
       * @param mm reference to integer minutes (0 <= mm < 60) (output)
       * @param sec reference to double seconds (0 <= sec < 60.0) (output)
       */
   void convertSODtoTime( double sod,
                          int& hh,
                          int& mm,
                          double& sec );

      /** Fundamental routine to convert H:M:S to seconds of day
       * @param hh integer hour (0 <= hh < 24) (input)
       * @param mm integer minutes (0 <= mm < 60) (input)
       * @param sec double seconds (0 <= sec < 60.0) (input)
       * @return sod seconds of day (input)
       */
   double convertTimeToSOD( int hh,
                            int mm,
                            double sec ) ;

} // namespace

#endif // GPSTK_TIMECONVERTERS_HPP
