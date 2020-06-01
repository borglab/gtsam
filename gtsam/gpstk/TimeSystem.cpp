/// TimeSystem.cpp

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

#include <cmath>
#include "TimeSystem.hpp"
#include "TimeConverters.hpp"
#include "Exception.hpp"

using namespace std;

namespace gpstk
{
   // Static initialization of const std::strings for asString().
   // Must parallel enum Systems in TimeSystem.hpp.
   // NB: DO NOT use std::map here; on some systems initialization fails.
   const string TimeSystem::Strings[count] =
     {
       string("UNK"),
       string("Any"),
       string("GPS"),
       string("GLO"),
       string("GAL"),
       string("QZS"),
       string("BDT"),
       string("UTC"),
       string("TAI"),
       string("TT"),
       string("TRT"),
     };

   void TimeSystem::setTimeSystem(const Systems& sys)
   {
      if(sys < 0 || sys >= count)
         system = Unknown;
      else
         system = sys;
   }

   void TimeSystem::fromString(const string str)

   {
      system = Unknown;
      for(int i=0; i<count; i++) {
         if(Strings[i] == str) {
            system = static_cast<Systems>(i);
            break;
         }
      }
   }

   ostream& operator<<(ostream os, const TimeSystem& ts)
   {
      return os << ts.asString();
   }

   // NB. The table 'leaps' must be modified when a new leap second is announced.
   // Return the number of leap seconds between UTC and TAI, that is the
   // difference in time scales UTC-TAI at an epoch defined by (year, month, day).
   // NB. Input day in a floating quantity and thus any epoch may be represented;
   // this is relevant the period 1960 to 1972, when UTC-TAI was not integral.
   // NB. GPS = TAI - 19sec and so GPS-UTC = getLeapSeconds()-19.
   double TimeSystem::getLeapSeconds(const int& year,
                                     const int& month,
                                     const double& day)
   {
      // Leap second data --------------------------------------------------------
      // number of changes before leap seconds (1960-1971) - this should never change.
      static const int NPRE=14;

      // epoch year, epoch month(1-12), delta t(sec), rate (sec/day) for [1960,1972).
      static const struct {
         int year, month;
         double delt, rate;
      } preleap[NPRE] = {
         { 1960,  1,  1.4178180, 0.0012960 },
         { 1961,  1,  1.4228180, 0.0012960 },
         { 1961,  8,  1.3728180, 0.0012960 },
         { 1962,  1,  1.8458580, 0.0011232 },
         { 1963, 11,  1.9458580, 0.0011232 },
         { 1964,  1,  3.2401300, 0.0012960 },
         { 1964,  4,  3.3401300, 0.0012960 },
         { 1964,  9,  3.4401300, 0.0012960 },
         { 1965,  1,  3.5401300, 0.0012960 },
         { 1965,  3,  3.6401300, 0.0012960 },
         { 1965,  7,  3.7401300, 0.0012960 },
         { 1965,  9,  3.8401300, 0.0012960 },
         { 1966,  1,  4.3131700, 0.0025920 },
         { 1968,  2,  4.2131700, 0.0025920 }
      };

      // Leap seconds history
      // ***** This table must be updated for new leap seconds **************
      static const struct {
         int year, month, nleap;
      } leaps[] = {
         { 1972,  1, 10 },
         { 1972,  7, 11 },
         { 1973,  1, 12 },
         { 1974,  1, 13 },
         { 1975,  1, 14 },
         { 1976,  1, 15 },
         { 1977,  1, 16 },
         { 1978,  1, 17 },
         { 1979,  1, 18 },
         { 1980,  1, 19 },
         { 1981,  7, 20 },
         { 1982,  7, 21 },
         { 1983,  7, 22 },
         { 1985,  7, 23 },
         { 1988,  1, 24 },
         { 1990,  1, 25 },
         { 1991,  1, 26 },
         { 1992,  7, 27 },
         { 1993,  7, 28 },
         { 1994,  7, 29 },
         { 1996,  1, 30 },
         { 1997,  7, 31 },
         { 1999,  1, 32 },
         { 2006,  1, 33 },
         { 2009,  1, 34 },
         { 2012,  7, 35 }, // leave the last comma!
         // add new entry here, of the form:
         // { year, month(1-12), leap_sec }, // leave the last comma!
      };

      // the number of leaps (do not change this)
      static const int NLEAPS = sizeof(leaps)/sizeof(leaps[0]);

      // last year in leaps
      static const int MAXYEAR = leaps[NLEAPS-1].year;
#pragma unused(MAXYEAR)
       
      // END static data -----------------------------------------------------

      // search for the input year, month
      if(year < 1960)                        // pre-1960 no deltas
         ;
      else if(month < 1 || month > 12)       // blunder, should never happen - throw?
         ;
      else if(year < 1972) {                 // [1960-1972) pre-leap
         for(int i=NPRE-1; i>=0; i--) {
            if(preleap[i].year > year ||
               (preleap[i].year == year && preleap[i].month > month)) continue;

            // found last record with < rec.year >= year and rec.month >= month
            // watch out - cannot use CommonTime here
            int iday(day);
            double dday(iday-int(day));
            if(iday == 0) { iday = 1; dday = 1.0-dday; }
            long JD0 = convertCalendarToJD(year,month,iday);
            long JD = convertCalendarToJD(preleap[i].year,preleap[i].month,1);
            return (preleap[i].delt + (double(JD0-JD)+dday)*preleap[i].rate);
         }
      }
      else {                                    // [1972- leap seconds
         for(int i=NLEAPS-1; i>=0; i--) {
            if(leaps[i].year > year ||
               (leaps[i].year == year && leaps[i].month > month)) continue;
            return double(leaps[i].nleap);
         }
      }

      return 0.0;
   }

   // Compute the conversion (in seconds) from one time system (inTS) to another
   // (outTS), given the year and month of the time to be converted.
   // Result is to be added to the first time (inTS) to yield the converted (outTS),
   // that is t(outTS) = t(inTS) + correction(inTS,outTS).
   // NB. the caller must not forget to change to outTS after adding this correction.
   // @param TimeSystem inTS, input system
   // @param TimeSystem outTS, output system
   // @param int year, year of the time to be converted.
   // @param int month, month (1-12) of the time to be converted.
   // @return double dt, correction (sec) to be added to t(in) to yield t(out).
   // @throw if input system(s) are invalid or Unknown.
   double TimeSystem::Correction(const TimeSystem& inTS, const TimeSystem& outTS,
                                 const int& year, const int& month, const double& day)
   {
      double dt(0.0);

      // identity
      if(inTS == outTS)
         return dt;

      // cannot convert unknowns
      if(inTS == Unknown || outTS == Unknown) {
         Exception e("Cannot compute correction for TimeSystem::Unknown");
         GPSTK_THROW(e);
      }

      // compute TT-TDB here; ref Astronomical Almanac B7
      double TDBmTT(0.0);
      if(inTS == TDB || outTS == TDB) {
         int iday = int(day);
         long jday = convertCalendarToJD(year, month, iday) ;
         double frac(day-iday);
         double TJ2000(jday-2451545.5+frac);     // t-J2000
         //       0.0001657 sec * sin(357.53 + 0.98560028 * TJ2000 deg)
         frac = ::fmod(0.017201969994578 * TJ2000, 6.2831853071796);
         TDBmTT = 0.0001657 * ::sin(6.240075674 + frac);
         //        0.000022 sec * sin(246.11 + 0.90251792 * TJ2000 deg)
         frac = ::fmod(0.015751909262251 * TJ2000, 6.2831853071796);
         TDBmTT += 0.000022  * ::sin(4.295429822 + frac);
      }

      // -----------------------------------------------------------
      // conversions: first convert inTS->TAI ...
      // TAI = GPS + 19s
      // TAI = UTC + getLeapSeconds()
      // TAI = TT - 32.184s
      if(inTS == GPS ||       // GPS -> TAI
         inTS == GAL)         // GAL -> TAI
         dt = 19.;
      else if(inTS == UTC |   // UTC -> TAI
              inTS == BDT |   // BDT -> TAI           // TD is this right?
              inTS == GLO)    // GLO -> TAI
         dt = getLeapSeconds(year, month, day);
      //else if(inTS == BDT)    // BDT -> TAI         // RINEX 3.02 seems to say this
      //   dt = 34.;
      else if(inTS == TAI)    // TAI
         ;
      else if(inTS == TT)     // TT -> TAI
         dt = -32.184;
      else if(inTS == TDB)    // TDB -> TAI
         dt = -32.184 + TDBmTT;
      else {                              // other
         Exception e("Invalid input TimeSystem " + inTS.asString());
         GPSTK_THROW(e);
      }

      // -----------------------------------------------------------
      // ... then convert TAI->outTS
      // GPS = TAI - 19s
      // UTC = TAI - getLeapSeconds()
      // TT = TAI + 32.184s
      if(outTS == GPS ||      // TAI -> GPS
         outTS == GAL)        // TAI -> GAL
         dt -= 19.;
      else if(outTS == UTC |  // TAI -> UTC
              outTS == BDT |  // TAI -> BDT
              outTS == GLO)   // TAI -> GLO
         dt -= getLeapSeconds(year, month, day);
      //else if(outTS == BDT)   // TAI -> BDT
      //   dt -= 34.;
      else if(outTS == TAI)   // TAI
         ;
      else if(outTS == TT)    // TAI -> TT
         dt += 32.184;
      else if(outTS == TDB)   // TAI -> TDB
         dt += 32.184 - TDBmTT;
      else if(outTS == GAL)   // TD
         dt = 0.0;
      else {                              // other
         Exception e("Invalid output TimeSystem " + outTS.asString());
         GPSTK_THROW(e);
      }

      return dt;
   }

}   // end namespace
