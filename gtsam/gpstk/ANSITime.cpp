/// @file ANSITime.cpp

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
#include "ANSITime.hpp"
#include "TimeConstants.hpp"

namespace gpstk
{
   ANSITime& ANSITime::operator=( const ANSITime& right )
   {
      time = right.time;
      timeSystem = right.timeSystem;
      return *this;
   }

   CommonTime ANSITime::convertToCommonTime() const
   {
      try
      {
         CommonTime ct;
         return ct.set( ( MJD_JDAY + UNIX_MJD + time / SEC_PER_DAY ),
                        ( time % SEC_PER_DAY ),
                        0.,
                        timeSystem );
      }
      catch (InvalidParameter& ip)
      {
         InvalidRequest ir(ip);
         GPSTK_THROW(ir);
      }
   }

   void ANSITime::convertFromCommonTime( const CommonTime& ct )
   {
         /// This is the earliest CommonTime for which ANSITimes are valid,
         /// = UNIX_MJD = 40587 = Jan 1 1970
      static const CommonTime MIN_CT = ANSITime(0, TimeSystem::Any);
         /// This is the latest CommonTime for which ANSITimes are valid.
         /// 2^31 - 1 seconds = Jan 19 2038 3:14:7
      static const CommonTime MAX_CT = ANSITime(2147483647, TimeSystem::Any);

      if ( ct < MIN_CT || ct > MAX_CT )
      {
         InvalidRequest ir("Unable to convert given CommonTime to ANSITime.");
         GPSTK_THROW(ir);
      }

      long jday, sod;
      double fsod;
      ct.get( jday, sod, fsod, timeSystem );

      time =
         static_cast<time_t>((jday - MJD_JDAY - UNIX_MJD) * SEC_PER_DAY + sod);
   }

   std::string ANSITime::printf( const std::string& fmt) const
   {
      try
      {
         using StringUtils::formattedPrint;
         std::string rv( fmt );

         rv = formattedPrint( rv, getFormatPrefixInt() + "K",
                              "Klu", time );
         rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                              "Ps", timeSystem.asString().c_str() );
         return rv;
      }
      catch( StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }

   std::string ANSITime::printError( const std::string& fmt) const
   {
      try
      {
         using StringUtils::formattedPrint;
         std::string rv( fmt );

         rv = formattedPrint( rv, getFormatPrefixInt() + "K",
                              "Ks", getError().c_str() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                              "Ps", getError().c_str() );
         return rv;
      }
      catch( StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }

   bool ANSITime::setFromInfo( const IdToValue& info )
   {
      using namespace StringUtils;

      for( IdToValue::const_iterator i = info.begin(); i != info.end(); i++ )
      {
         switch( i->first )
         {
            case 'K':
               time = asInt( i->second );
               break;

            case 'P':
               timeSystem.fromString(i->second);
               break;

            default:
                  // do nothing
               break;
         };
      }

      return true;
   }

   bool ANSITime::isValid() const
   {
      ANSITime temp;
      temp.convertFromCommonTime( convertToCommonTime() );
      if( *this == temp )
      {
         return true;
      }
      return false;
   }

   void ANSITime::reset()
   {
      time = 0;
      timeSystem = TimeSystem::Unknown;
   }

   bool ANSITime::operator==( const ANSITime& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
     if ((timeSystem != TimeSystem::Any &&
          right.timeSystem != TimeSystem::Any) &&
	 timeSystem != right.timeSystem)
         return false;

      if( fabs(double(time - right.time)) < CommonTime::eps )
      {
         return true;
      }
      return false;
   }

   bool ANSITime::operator!=( const ANSITime& right ) const
   {
      return ( !operator==( right ) );
   }

   bool ANSITime::operator<( const ANSITime& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
     if ((timeSystem != TimeSystem::Any &&
          right.timeSystem != TimeSystem::Any) &&
         timeSystem != right.timeSystem)
      {
         InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
         GPSTK_THROW(ir);
      }

      return ( time < right.time );
   }

   bool ANSITime::operator>( const ANSITime& right ) const
   {
      return ( !operator<=( right ) );
   }

   bool ANSITime::operator<=( const ANSITime& right ) const
   {
      return ( operator<( right ) ||
               operator==( right ) );
   }

   bool ANSITime::operator>=( const ANSITime& right ) const
   {
      return ( !operator<( right ) );
   }

} // namespace
