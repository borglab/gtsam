/// @file Epoch.cpp
/// gpstk::Epoch - encapsulates date and time-of-day

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

#include <iostream>
#include <iomanip>
#include <string>
#include <ctime>

#include "gpstkplatform.h"
#include "Epoch.hpp"

#include "TimeConstants.hpp"
#include "TimeString.hpp"

namespace gpstk
{
   using namespace std;
   using namespace gpstk::StringUtils;

      // One nanosecond tolerance.
   const double Epoch::ONE_NSEC_TOLERANCE = 1e-9;
      // One microsecond tolerance.
   const double Epoch::ONE_USEC_TOLERANCE = 1e-6;
      // One millisecond tolerance.
   const double Epoch::ONE_MSEC_TOLERANCE = 1e-3;
      // One second tolerance.
   const double Epoch::ONE_SEC_TOLERANCE = 1;
      // One minute tolerance.
   const double Epoch::ONE_MIN_TOLERANCE = 60;
      // One hour tolerance.
   const double Epoch::ONE_HOUR_TOLERANCE = 3600;
   
      // Tolerance for time equality.
#ifdef _WIN32
   double Epoch::EPOCH_TOLERANCE = ONE_USEC_TOLERANCE;
#else
   double Epoch::EPOCH_TOLERANCE = ONE_NSEC_TOLERANCE;
#endif

      /// Earliest representable Epoch.
   const Epoch BEGINNING_OF_TIME(CommonTime::BEGINNING_OF_TIME);
      /// Latest Representable Epoch.
   const Epoch END_OF_TIME(CommonTime::END_OF_TIME);

   std::string Epoch::PRINT_FORMAT("%02m/%02d/%04Y %02H:%02M:%02S");

   Epoch& Epoch::setTolerance(double tol)
      throw()
   {
      tolerance = tol;
      return *this;
   }
   
      // Default constructor; initializes to current system time.
   Epoch::Epoch(const TimeTag& tt)
      throw(Epoch::EpochException)
         : tolerance(EPOCH_TOLERANCE)
   {
      set(tt);
   }
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
   Epoch::Epoch(const CommonTime& ct)
      throw()
         : tolerance(EPOCH_TOLERANCE),
           core(ct)
   {}
#pragma clang diagnostic pop
      /** 
       * TimeTag + Year Constructor.
       * Set the current time using the given year as a hint.
       */
   Epoch::Epoch(const WeekSecond& tt,
                short year)
      throw(Epoch::EpochException)
         : tolerance(EPOCH_TOLERANCE)
   {
      set(tt, year);
   }

      // GPS Zcount constructor.
      // @param z GPSZcount object to set to
      // @param f Time frame (see #TimeFrame)
   Epoch::Epoch(const GPSZcount& z)
      throw()
         : tolerance(EPOCH_TOLERANCE)
   {
      set(z);
   }

   Epoch::operator GPSZcount() const
      throw(Epoch::EpochException)
   {
      try
      {
            // this wants a rounded zcount
         Epoch e = *this + 0.75;
         GPSWeekZcount wz = get<GPSWeekZcount>();
         return GPSZcount(wz.week, wz.zcount);
      }
      catch (gpstk::InvalidParameter& ip)
      {
         Epoch::EpochException ee(ip);
         GPSTK_THROW(ee);
      }
   }

      // Copy constructor
   Epoch::Epoch(const Epoch& right)
      throw()
         : core(right.core),
           tolerance(right.tolerance)
   {}

      // Assignment operator.
   Epoch& Epoch::operator=(const Epoch& right)
      throw()
   {
      core = right.core;
      tolerance = right.tolerance;
      return *this;
   }

      // Epoch difference function.
      // @param right Epoch to subtract from this one.
      // @return difference in seconds.
   double Epoch::operator-(const Epoch& right) const
      throw()
   {
      return core - right.core;
   }

      // Add seconds to this time.
      // @param seconds Number of seconds to increase this time by.
      // @return The new time incremented by \c seconds.
   Epoch Epoch::operator+(double seconds) const
      throw(Epoch::EpochException)
   {
      return Epoch(*this).addSeconds(seconds);
   }

      // Subtract seconds from this time.
      // @param seconds Number of seconds to decrease this time by.
      // @return The new time decremented by \c seconds.
   Epoch Epoch::operator-(double seconds) const
      throw(Epoch::EpochException)
   {
      return Epoch(*this).addSeconds(-seconds);
   }

      // Add seconds to this time.
      // @param seconds Number of seconds to increase this time by.
   Epoch& Epoch::operator+=(double seconds)
      throw(Epoch::EpochException)
   {
      return addSeconds(seconds);
   }
   
      // Subtract seconds from this time.
      // @param sec Number of seconds to decrease this time by.
   Epoch& Epoch::operator-=(double seconds)
      throw(Epoch::EpochException)
   {
      return addSeconds(-seconds);
   }

      // Add seconds to this object.
      // @param seconds Number of seconds to add
   Epoch& Epoch::addSeconds(double seconds)
      throw(Epoch::EpochException)
   {
      try
      {
         core.addSeconds(seconds);
         return *this;
      }
      catch( InvalidRequest& ir )
      {
         Epoch::EpochException ee(ir);
         GPSTK_THROW(ee);
      }
   }

      // Add (integer) seconds to this object.
      // @param seconds Number of seconds to add.
   Epoch& Epoch::addSeconds(long seconds)
      throw(Epoch::EpochException)
   {
      try
      {
         core.addSeconds(seconds);
         return *this ;
      }
      catch( InvalidRequest& ir )
      {
         Epoch::EpochException ee(ir);
         GPSTK_THROW(ee);
      }
   }

      // Add (integer) milliseconds to this object.
      // @param msec Number of milliseconds to add.
   Epoch& Epoch::addMilliSeconds(long msec)
      throw(Epoch::EpochException)
   {
      try
      {
         core.addMilliseconds(msec);
         return *this;
      }
      catch( InvalidRequest& ir )
      {
         Epoch::EpochException ee(ir);
         GPSTK_THROW(ee);
      }
   }

      // Add (integer) microseconds to this object.
      // @param usec Number of microseconds to add.
   Epoch& Epoch::addMicroSeconds(long usec)
      throw(Epoch::EpochException)
   {
      try
      {
         long ms = usec / 1000;     // whole milliseconds
         usec -= ms * 1000;         // leftover microseconds
         core.addMilliseconds(ms);
            // us * 1ms/1000us * 1s/1000ms
         core.addSeconds(static_cast<double>(usec) * 1e-6);
         return *this;
      }
      catch( InvalidRequest& ir )
      {
         Epoch::EpochException ee(ir);
         GPSTK_THROW(ee);
      }
   }

      // Equality operator.
   bool Epoch::operator==(const Epoch &right) const 
      throw()
   {
      // use the smaller of the two tolerances for comparison
      return (ABS(operator-(right)) <=
         ((tolerance > right.tolerance) ? right.tolerance : tolerance));
   }

      // Inequality operator.
   bool Epoch::operator!=(const Epoch &right) const 
      throw()
   {
      return !(operator==(right));
   }

      // Comparison operator (less-than).
   bool Epoch::operator<(const Epoch &right) const 
      throw()
   {
      return (operator-(right) <
            -((tolerance > right.tolerance) ? right.tolerance : tolerance));
   }

      // Comparison operator (greater-than).
   bool Epoch::operator>(const Epoch &right) const 
      throw()
   {
      return (operator-(right) >
            ((tolerance > right.tolerance) ? right.tolerance : tolerance));
   }
   
      // Comparison operator (less-than or equal-to).
   bool Epoch::operator<=(const Epoch &right) const 
      throw()
   {
      return !(operator>(right));
   }

      // Comparison operator (greater-than or equal-to).
   bool Epoch::operator>=(const Epoch &right) const 
      throw()
   {
      return !(operator<(right));
   }

   Epoch::operator CommonTime() const
      throw()
   {
      return core;
   }

   Epoch& Epoch::set(const TimeTag& tt)
      throw(Epoch::EpochException)
   {
      try
      {
         core = tt;
         return *this;
      }
      catch(InvalidParameter& ip)
      {
         EpochException ee(ip);
         GPSTK_THROW(ee);
      }
   }

   Epoch& Epoch::set(const WeekSecond& tt, short year)
      throw(Epoch::EpochException)
   {
      WeekSecond& ws = const_cast<WeekSecond&>(tt);
      ws.adjustToYear(year);
      set(ws);
      return *this;
   }

   Epoch& Epoch::set(const CommonTime& c)
      throw()
   {
      core = c;
      return *this;
   }

      // Set the object's time using the given GPSZcount.
      // System time is used to disambiguate which 1024 week 'zone'
      // is appropriate.
      // @param z the GPSZcount object to set to
      // @param f Time frame (see #TimeFrame)
      // @return a reference to this object.
   Epoch& Epoch::set(const GPSZcount& z)
      throw(Epoch::EpochException)
   {
      try
      {
         GPSWeekZcount wz(core);
         wz.week = z.getWeek();
         wz.zcount = z.getZcount();
         core = wz;
         return *this ;
      }
      catch(Exception& exc)
      {
         EpochException ee(exc);
         GPSTK_THROW(ee);
      }
   }

   Epoch& Epoch::setTime(const CommonTime& ct)
      throw(Epoch::EpochException)
   {
      try
      {
         long myDAY, mySOD, ctDAY, ctSOD;
         double myFSOD, ctFSOD;
         core.get(myDAY, mySOD, myFSOD);
         ct.get(ctDAY, ctSOD, ctFSOD);
         core.set(myDAY, ctSOD, ctFSOD);
         return *this;
      }
      catch(InvalidParameter& ip)
      {
         EpochException ee(ip);
         GPSTK_THROW(ee);
      }
   }
   
   Epoch& Epoch::setDate(const CommonTime& ct)
      throw(Epoch::EpochException)
   {
      try
      {
         long myDAY, mySOD, ctDAY, ctSOD;
         double myFSOD, ctFSOD;
         core.get(myDAY, mySOD, myFSOD);
         ct.get(ctDAY, ctSOD, ctFSOD);
         core.set(ctDAY, mySOD, myFSOD);
         return *this;
      }
      catch(InvalidParameter& ip)
      {
         EpochException ee(ip);
         GPSTK_THROW(ee);
      }
   }
   
      // set using local time
   Epoch& Epoch::setLocalTime()
      throw(Epoch::EpochException)
   {
      time_t t;
      time(&t);
      struct tm  *ltod;
      ltod = localtime(&t);
      return set(CivilTime(1900 + ltod->tm_year, 
                           ltod->tm_mon + 1,
                           ltod->tm_mday,
                           ltod->tm_hour,
                           ltod->tm_min,
                           ltod->tm_sec));
   }

   Epoch& Epoch::scanf(const string& str,
                       const string& fmt)
      throw(StringException, InvalidRequest)
   {
      try
      {
         scanTime( core, str, fmt );
         return *this;
      }
      catch (StringException& se)
      {
         GPSTK_RETHROW(se);
      }
   }

      // Format this time into a string.
   string Epoch::printf(const string& fmt) const
      throw(StringException)
   {
      try
      {
         return printTime( core, fmt );
      }
      catch (StringException& se)
      {
         GPSTK_RETHROW(se);
      }
   }

      // Stream output for Epoch objects.  Typically used for debugging.
      // @param s stream to append formatted Epoch to.
      // @param t Epoch to append to stream \c s.
      // @return reference to \c s.
   ostream& operator<<( ostream& s, 
                        const Epoch& e )
   {
      s << e.printf();
      return s;
   }

}   // end namespace gpstk
