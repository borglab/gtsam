/// @file TimeSystem.hpp

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

#ifndef GPSTK_TIMESYSTEM_HPP
#define GPSTK_TIMESYSTEM_HPP

#include <iostream>
#include <string>

namespace gpstk
{
   /// This class encapsulates time systems, including std::string I/O. This is an
   /// example of a 'smart enum' class.
   class TimeSystem
   {
   public:

      /// list of time systems supported by this class
      enum Systems
      {
         // add new systems BEFORE count, then
         // *** add to Strings[] in TimeSystem.cpp and make parallel to this enum. ***

         // Unknown MUST BE FIRST, and must = 0
         Unknown = 0, ///< unknown time frame; for legacy code compatibility
         Any,         ///< wildcard; allows comparison with any other type
         GPS,         ///< GPS system time
         GLO,         ///< GLONASS system time
         GAL,         ///< Galileo system time
         QZS,         ///< QZSS system Time
         BDT,         ///< BeiDou system Time
         UTC,         ///< Coordinated Universal Time (e.g., from NTP)
         TAI,         ///< International Atomic Time
         TT,          ///< Terrestrial time (used in IERS conventions)
         TDB,         ///< Barycentric dynamical time (JPL ephemeris); very near TT
         // count MUST BE LAST
         count        ///< the number of systems - not a system
      };

      /// Constructor, including empty constructor
      TimeSystem(Systems sys = Unknown)
      {
         if(sys < 0 || sys >= count)
            system = Unknown;
         else
            system = sys;
      }

      /// constructor from int
      TimeSystem(int i)
      {
         if(i < 0 || i >= count)
            system = Unknown;
         else
            system = static_cast<Systems>(i);
      }

      // (copy constructor and operator= are defined by compiler)

      /// set the time system
      void setTimeSystem(const Systems& sys);

      /// get the time system
      Systems getTimeSystem() const
      { return system; }

      /// Return a std::string for each system (these strings are const and static).
      /// @return the std::string
      std::string asString() const
      { return Strings[system]; }

      /// define system based on input string
      /// @param str input string, expected to match output string for given system
      void fromString(const std::string str);

      /// boolean operator==
      bool operator==(const TimeSystem& right) const
      { return system == right.system; }

      /// boolean operator< (used by STL to sort)
      bool operator<(const TimeSystem& right) const
      { return system < right.system; }

      // the rest follow from Boolean algebra...
      /// boolean operator!=
      bool operator!=(const TimeSystem& right) const
      { return !operator==(right); }

      /// boolean operator>=
      bool operator>=(const TimeSystem& right) const
      { return !operator<(right); }

      /// boolean operator<=
      bool operator<=(const TimeSystem& right) const
      { return (operator<(right) || operator==(right)); }

      /// boolean operator>
      bool operator>(const TimeSystem& right) const
      { return (!operator<(right) && !operator==(right)); }

      /// Return the number of leap seconds between UTC and TAI, that is the
      /// difference in time scales UTC-TAI, at an epoch defined by year/month/day.
      /// NB. Input day in a floating quantity and thus any epoch may be represented;
      /// this is relevant the period 1960 to 1972, when UTC-TAI was not integral.
      /// NB. GPS = TAI-19sec and so GPS-UTC = getLeapSeconds()-19 == dtLS.
      /// NB. GLO = UTC = GPS - dtLS. but not incl. RINEX::TIME SYSTEM CORR::GPUT
      /// NB. GLO is actually UTC(SU) Moscow
      /// NB. GAL = GPS = UTC + dtLS this does not incl. RINEX::TIME SYSTEM CORR::GAUT
      /// NB. BDT = GPS - 15 but this does not include RINEX::TIME SYSTEM CORR::BDUT
      /// NB. BDT is actually UTC(NTSC) China
      /// @param  yr, mon, day give the day of interest
      static double getLeapSeconds(const int& yr, const int& mon, const double& day);

      /// Compute the conversion (in seconds) from one time system (inTS) to another
      /// (outTS), given the year and month of the time to be converted.
      /// Result is to be added to the first time (inTS) to yield the second (outTS),
      /// that is t(outTS) = t(inTS) + correction(inTS,outTS).
      /// NB. caller must not forget to change to outTS after adding this correction.
      /// @param TimeSystem inTS, input system
      /// @param TimeSystem outTS, output system
      /// @param int year, year of the time to be converted.
      /// @param int month, month (1-12) of the time to be converted.
      /// @return double dt, correction (sec) to be added to t(in) to yield t(out).
      /// @throw if input system(s) are invalid or Unknown.
      static double Correction(const TimeSystem& inTS, const TimeSystem& outTS,
                               const int& year, const int& month, const double& day);

   private:

      /// time system (= element of Systems enum) for this object
      Systems system;

      /// set of string labels for elements of Systems
      static const std::string Strings[];

   };   // end class TimeSystem

   /// Write name (asString()) of a TimeSystem to an output stream.
   /// @param os The output stream
   /// @param ts The TimeSystem to be written
   /// @return reference to the output stream
   std::ostream& operator<<(std::ostream& os, const TimeSystem& ts);

}   // end namespace

#endif // GPSTK_TIMESYSTEM_HPP
