#pragma ident "$Id$"



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

#include "TimeConverters.hpp"
#include "TimeConstants.hpp"
#include <math.h>

namespace gpstk
{

      // These two routines convert 'integer JD' and calendar time; they were
      // derived from Sinnott, R. W. "Bits and Bytes" Sky & Telescope Magazine,
      // Vol 82, p. 183, August 1991, and The Astronomical Almanac, published
      // by the U.S. Naval Observatory.
      // NB range of applicability of this routine is from 0JD (4713BC)
      // to approx 3442448JD (4713AD).
   void convertJDtoCalendar( long jd,
                             int& iyear,
                             int& imonth,
                             int& iday )
   {
      long L, M, N, P, Q;
      if(jd > 2299160)    // after Oct 4, 1582
      {
         L = jd + 68569;
         M = (4 * L) / 146097;
         L = L - ((146097 * M + 3) / 4);
         N = (4000 * (L + 1)) / 1461001;
         L = L - ((1461 * N) / 4) + 31;
         P = (80 * L) / 2447;
         iday = int(L - (2447 * P) / 80);
         L = P / 11;
         imonth = int(P + 2 - 12 * L);
         iyear = int(100 * (M - 49) + N + L);
      }
      else
      {
         P = jd + 1402;
         Q = (P - 1) / 1461;
         L = P - 1461 * Q;
         M = (L - 1) / 365 - L / 1461;
         N = L - 365 * M + 30;
         P = (80 * N) / 2447;
         iday = int(N - (2447 * P) / 80);
         N = P / 11;
         imonth = int(P + 2 - 12 * N);
         iyear = int(4 * Q + M + N - 4716);
         if(iyear <= 0)
         {
            --iyear;
         }
      }
         // catch century/non-400 non-leap years
      if(iyear > 1599 &&
         !(iyear % 100) &&
         (iyear % 400) &&
         imonth == 2 &&
         iday == 29)
      {
         imonth = 3;
         iday = 1;
      }
   }

   long convertCalendarToJD( int yy,
                             int mm,
                             int dd )
   {
      if(yy == 0)
         --yy;         // there is no year 0

      if(yy < 0)
         ++yy;

      long jd;
      double y = static_cast<double>( yy ),
         m = static_cast<double>( mm ),
         d = static_cast<double>( dd );
#pragma unused(d)
       
         // In the conversion from the Julian Calendar to the Gregorian
         // Calendar the day after October 4, 1582 was October 15, 1582.
         //
         // if the date is before October 15, 1582
      if(yy < 1582 || (yy == 1582 && (mm < 10 || (mm == 10 && dd < 15))))
      {
         jd = 1729777 + dd + 367 * yy
            - static_cast<long>(7 * ( y + 5001 +
                                      static_cast<long>((m - 9) / 7)) / 4)
            + static_cast<long>(275 * m / 9);
      }
      else   // after Oct 4, 1582
      {
        jd = 1721029 + dd + 367 * yy
           - static_cast<long>(7 * (y + static_cast<long>((m + 9) / 12)) / 4)
           - static_cast<long>(3 * (static_cast<long>((y + (m - 9) / 7) / 100)
                                    + 1) / 4)
           + static_cast<long>(275 * m / 9);

            // catch century/non-400 non-leap years
         if( (! (yy % 100) &&
              (yy % 400) &&
              mm > 2 &&
              mm < 9)      ||
             (!((yy - 1) % 100) &&
              ((yy - 1) % 400) &&
              mm == 1))
         {
            --jd;
         }
      }
      return jd;
   }

   void convertSODtoTime( double sod,
                          int& hh,
                          int& mm,
                          double& sec )
   {
         // Get us to within one day.
      if (sod < 0)
      {
         sod += (1 +
                 static_cast<unsigned long>(sod / SEC_PER_DAY)) * SEC_PER_DAY ;
      }
      else if (sod >= SEC_PER_DAY)
      {
         sod -= static_cast<unsigned long>(sod / SEC_PER_DAY) * SEC_PER_DAY ;
      }

      double temp;               // variable to hold the integer part of sod
      sod = modf(sod, &temp);    // sod holds the fraction, temp the integer
      long seconds = static_cast<long>(temp); // get temp into a real integer

      hh = seconds / 3600 ;
      mm = (seconds % 3600) / 60 ;
      sec = double(seconds % 60) + sod ;

   }

   double convertTimeToSOD( int hh,
                            int mm,
                            double sec )
   {
      return (sec + 60. * (mm + 60. * hh));
   }

} // namespace
