/// @file WeekSecond.cpp  Implement full week, mod week and seconds-of-week time
/// representation.

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

#include "WeekSecond.hpp"
#include "TimeConstants.hpp"
#include "MJD.hpp"

namespace gpstk
{
   WeekSecond& WeekSecond::operator=( const WeekSecond& right )
   {
      Week::operator=(right);
      sow = right.sow;
      return *this;
   }

   CommonTime WeekSecond::convertToCommonTime() const
   {
      try
      {
	      //int dow = static_cast<int>( sow * DAY_PER_SEC );
         // Appears to have rounding issues on 32-bit platforms

         int dow = static_cast<int>( sow / SEC_PER_DAY );
         // NB this assumes MJDEpoch is an integer - what if epoch H:M:S != 0:0:0 ?
         long jday = MJD_JDAY + MJDEpoch() + (7 * week) + dow;
         double sod(sow - SEC_PER_DAY * dow);
         CommonTime ct;
         return ct.set( jday,
                        static_cast<long>(sod),
                        sod - static_cast<long>(sod),
                        timeSystem );
      }
      catch (InvalidParameter& ip)
      {
         GPSTK_RETHROW(ip);
      }
   }

   void WeekSecond::convertFromCommonTime( const CommonTime& ct )
   {
      if(static_cast<MJD>(ct).mjd < MJDEpoch())
      {
         InvalidRequest ir("Unable to convert to Week/Second - before Epoch.");
         GPSTK_THROW(ir);
      }

      long jday, sod;
      double fsod;
      ct.get( jday, sod, fsod, timeSystem );
         // find the number of days since the beginning of the Epoch
      jday -= MJD_JDAY + MJDEpoch();
         // find out how many weeks that is
      week = static_cast<int>( jday / 7 );
         // find out what the day of week is
      jday %= 7;

      sow = static_cast<double>( jday * SEC_PER_DAY + sod ) + fsod;
   }

   bool WeekSecond::isValid() const
   {
      return ( Week::isValid() &&
               sow < FULLWEEK );
   }

   void WeekSecond::reset()
   {
      Week::reset();
      sow = 0.0;
   }

   bool WeekSecond::operator==( const WeekSecond& right ) const
   {
      return ( Week::operator==(right) &&
               sow == right.sow );
   }

   bool WeekSecond::operator!=( const WeekSecond& right ) const
   {
      return ( !operator==( right ) );
   }

   bool WeekSecond::operator<( const WeekSecond& right ) const
   {
      if( Week::operator<(right) )
      {
         return true;
      }
      if( Week::operator>(right) )
      {
         return false;
      }
      if( sow < right.sow )
      {
         return true;
      }
      return false;
   }

   bool WeekSecond::operator>( const WeekSecond& right ) const
   {
      return ( !operator<=( right ) );
   }

   bool WeekSecond::operator<=( const WeekSecond& right ) const
   {
      return ( operator<( right ) || operator==( right ) );
   }

   bool WeekSecond::operator>=( const WeekSecond& right ) const
   {
      return ( !operator<( right ) );
   }

} // namespace
