#pragma ident "$Id$"

#ifndef GPSTK_GPSWEEK_HPP
#define GPSTK_GPSWEEK_HPP

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

#include "TimeTag.hpp"

namespace gpstk
{
      /** This class is an abstract base class inherited by other
       * GPS-time-related classes. It is abstract because it does not fulfill
       * the abstract methods imposed by the TimeTag class.
       * It handles the week portion of the GPS TimeTag classes.
       *
       * All of the GPS time classes can be boiled down to just two basic
       * types: GPSWeekZcount and GPSWeekSecond.
       * GPSWeekZcount consists of an integer week and integer zcount.
       * GPSWeekSecond consists of an integer week and double second.
       *
       * The GPSWeek class inherits from TimeTag and handles the epoch and
       * 10-bit week special cases:
       *  getEpoch() returns week / 1024,
       *  getWeek10() returns week % 1024,
       *  getEpochWeek10(int& e, int& w) e = getEpoch(); w = getWeek10();
       *  setEpoch(int e) sets week = (week & 0x3FF) | (e << 10);
       *  setWeek10(int w) sets week = (week & ~0x3FF) | (w & 0x3FF);
       *  setEpochWeek10(int e, int w) calls setEpoch(e); setWeek(w);
       *
       * GPSWeekZcount inherits from GPSWeek and covers all Zcount-related
       * special cases:
       *  getZcount29() returns (getWeek10() << 19) | getZcount()
       *  getZcount32() returns (getWeek() << 19) | getZcount()
       *
       *  setZcount29(int z) sets week = (z >> 19) & 0x3FF;
       *                          zcount = z & 0x7FFFF;
       *  setZcount32(int z) sets week = z >> 19; zcount = z & 07FFFF;
       *
       * GPSWeekSecond inherits from GPSWeek.  I don't know of any special
       * cases related to second-of-week.
       *
       */
   class GPSWeek : public TimeTag
   {
   public:
         /// This is a 10-bit mask used in the several special Epoch and
         /// 10-bit Week methods.
      static const int bits10 = 0x3FF;
         /// This is the greatest week value for which a conversion to or
         /// from CommonTime would work.
      static const int MAX_WEEK;

         /// Constructor.
      GPSWeek( int w = 0,
               TimeSystem ts = TimeSystem::Unknown )
            : week(w)
      { timeSystem = ts; }

         /// Virtual Destructor.
      virtual ~GPSWeek()
      {}

         /// Assignment Operator.
      GPSWeek& operator=(const GPSWeek& right);

         /// @name Comparison Operators.
         //@{
      inline bool operator==(const GPSWeek& right) const
      {
        /// Any (wildcard) type exception allowed, otherwise must be same time systems
         if ((timeSystem != TimeSystem::Any &&
              right.timeSystem != TimeSystem::Any) &&
             timeSystem != right.timeSystem)
            return false;

         return week == right.week;
      }

      inline bool operator!=(const GPSWeek& right) const
      {
        /// Any (wildcard) type exception allowed, otherwise must be same time systems
         if ((timeSystem != TimeSystem::Any &&
              right.timeSystem != TimeSystem::Any) &&
             timeSystem != right.timeSystem)
         {
            gpstk::InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
            GPSTK_THROW(ir);
         }

         return week != right.week;
      }

      inline bool operator<(const GPSWeek& right) const
      {
        /// Any (wildcard) type exception allowed, otherwise must be same time systems
         if ((timeSystem != TimeSystem::Any &&
              right.timeSystem != TimeSystem::Any) &&
             timeSystem != right.timeSystem)
         {
            gpstk::InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
            GPSTK_THROW(ir);
         }

         return week < right.week;
      }

      inline bool operator<=(const GPSWeek& right) const
      {
        /// Any (wildcard) type exception allowed, otherwise must be same time systems
         if ((timeSystem != TimeSystem::Any &&
              right.timeSystem != TimeSystem::Any) &&
             timeSystem != right.timeSystem)
    {
            gpstk::InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
            GPSTK_THROW(ir);
         }

         return week <= right.week;
      }

      inline bool operator>(const GPSWeek& right) const
      {
        /// Any (wildcard) type exception allowed, otherwise must be same time systems
         if ((timeSystem != TimeSystem::Any &&
              right.timeSystem != TimeSystem::Any) &&
             timeSystem != right.timeSystem)
    {
            gpstk::InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
            GPSTK_THROW(ir);
         }

         return week > right.week;
      }

      inline bool operator>=(const GPSWeek& right) const
      {
        /// Any (wildcard) type exception allowed, otherwise must be same time systems
         if ((timeSystem != TimeSystem::Any &&
              right.timeSystem != TimeSystem::Any) &&
             timeSystem != right.timeSystem)
    {
            gpstk::InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
            GPSTK_THROW(ir);
         }

         return week >= right.week;
      }
         //@}

         /// @name Special Epoch and 10-bit Week Methods.
         /// @todo Should the "set" methods return a reference?
         //@{
      inline virtual unsigned int getEpoch() const
      {
         return week >> 10;
      }

      inline virtual unsigned int getWeek10() const
      {
         return week & bits10;
      }

      inline virtual void getEpochWeek10(unsigned int& e,
                                         unsigned int& w) const
      {
         e = getEpoch();
         w = getWeek10();
      }

      inline virtual void setEpoch(unsigned int e)
      {
         week &= bits10;
         week |= e << 10;
      }

      inline virtual void setWeek10(unsigned int w)
      {
         week &= ~bits10;
         week |= w & bits10;
      }

      inline virtual void setEpochWeek10(unsigned int e,
                                         unsigned int w)
      {
         setEpoch(e);
         setWeek10(w);
      }
         //@}

         /// This function formats this time to a string.  The exceptions
         /// thrown would only be due to problems parsing the fmt string.
      virtual std::string printf( const std::string& fmt ) const;

         /// This function works similarly to printf.  Instead of filling
         /// the format with data, it fills with error messages.
      virtual std::string printError( const std::string& fmt ) const;

         /**
          * Set this object using the information provided in \a info.
          * @param info the IdToValue object to which this object shall be set.
          * @return true if this object was successfully set using the
          *  data in \a info, false if not.
          */
      virtual bool setFromInfo( const IdToValue& info );

         /// Return a string containing the characters that this class
         /// understands when printing times.
      inline virtual std::string getPrintChars() const
      {
         return "EFGP";
      }

         /// Return a string containing the default format to use in printing.
      inline virtual std::string getDefaultFormat() const
      {
         return "%04F";
      }

      virtual bool isValid() const
      {
         return (week >= 0 && week <= MAX_WEEK);
      }

      inline virtual void reset()
      {
         week = 0;
      }

         /// Force this interface on this classes descendants.
      virtual unsigned int getDayOfWeek() const = 0;

      int week;
   };

} // namespace

#endif // GPSTK_GPSTIME_HPP
