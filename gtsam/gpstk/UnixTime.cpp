/// @file UnixTime.cpp

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

#include "UnixTime.hpp"
#include "TimeConstants.hpp"

namespace gpstk
{
   UnixTime& UnixTime::operator=( const UnixTime& right )
   {
      tv.tv_sec  = right.tv.tv_sec ;
      tv.tv_usec = right.tv.tv_usec;
      timeSystem = right.timeSystem;
      return *this;
   }

   CommonTime UnixTime::convertToCommonTime() const
   {
      try
      {
         CommonTime ct;
         return ct.set( ( MJD_JDAY + UNIX_MJD + tv.tv_sec / SEC_PER_DAY ),
                        ( tv.tv_sec % SEC_PER_DAY ),
                        ( static_cast<double>( tv.tv_usec ) * 1e-6 ),
                        timeSystem );
      }
      catch (InvalidParameter& ip)
      {
         InvalidRequest ir(ip);
         GPSTK_THROW(ip);
      }
   }

   void UnixTime::convertFromCommonTime( const CommonTime& ct )
   {
         /// This is the earliest CommonTime for which UnixTimes are valid.
      static const CommonTime MIN_CT = UnixTime(0, 0, TimeSystem::Any);
         /// This is the latest CommonTime for which UnixTimes are valid.
         /// (2^31 - 1) s and 999999 us
      static const CommonTime MAX_CT = UnixTime(2147483647, 999999, TimeSystem::Any);

      if ( ct < MIN_CT || ct > MAX_CT )
      {
         InvalidRequest ir("Unable to convert given CommonTime to UnixTime.");
         GPSTK_THROW(ir);
      }

      long jday, sod;
      double fsod;
      ct.get( jday, sod, fsod, timeSystem );

      tv.tv_sec = (jday - MJD_JDAY - UNIX_MJD) * SEC_PER_DAY + sod;

         // round to the nearest microsecond
      tv.tv_usec = static_cast<time_t>( fsod * 1e6 + 0.5 ) ;

      if (tv.tv_usec >= 1000000)
      {
         tv.tv_usec -= 1000000;
         ++tv.tv_sec;
      }
   }

   std::string UnixTime::printf( const std::string& fmt ) const
   {
      try
      {
         using gpstk::StringUtils::formattedPrint;
         std::string rv( fmt );

         rv = formattedPrint(rv, getFormatPrefixInt() + "U",
                             "Ulu", tv.tv_sec);
         rv = formattedPrint(rv, getFormatPrefixInt() + "u",
                             "ulu", tv.tv_usec);
         rv = formattedPrint(rv, getFormatPrefixInt() + "P",
                             "Ps", timeSystem.asString().c_str() );
         return rv;
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }

   std::string UnixTime::printError( const std::string& fmt ) const
   {
      try
      {
         using gpstk::StringUtils::formattedPrint;
         std::string rv( fmt );

         rv = formattedPrint(rv, getFormatPrefixInt() + "U",
                             "Us", getError().c_str());
         rv = formattedPrint(rv, getFormatPrefixInt() + "u",
                             "us", getError().c_str());
         rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                             "Ps", getError().c_str());
         return rv;
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }

   bool UnixTime::setFromInfo( const IdToValue& info )
   {
      using namespace gpstk::StringUtils;

      for( IdToValue::const_iterator i = info.begin(); i != info.end(); i++ )
      {
         switch( i->first )
         {
            case 'U':
               tv.tv_sec = asInt( i->second );
               break;

            case 'u':
               tv.tv_usec = asInt( i->second );
               break;

            case 'P':
               timeSystem = static_cast<TimeSystem>(asInt( i->second ));
               break;

            default:
                  // do nothing
               break;
         };
      }

      return true;
   }

   bool UnixTime::isValid() const
   {
      UnixTime temp;
      temp.convertFromCommonTime( convertToCommonTime() );
      if( *this == temp )
      {
         return true;
      }
      return false;
   }

   void UnixTime::reset()
   {
      tv.tv_sec = tv.tv_usec = 0;
      timeSystem = TimeSystem::Unknown;
   }

   bool UnixTime::operator==( const UnixTime& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
      if ((timeSystem != TimeSystem::Any &&
           right.timeSystem != TimeSystem::Any) &&
          timeSystem != right.timeSystem)
         return false;

      if( tv.tv_sec == right.tv.tv_sec  &&
          abs(tv.tv_usec - right.tv.tv_usec) < CommonTime::eps )
      {
         return true;
      }
      return false;
   }

   bool UnixTime::operator!=( const UnixTime& right ) const
   {
      return ( !operator==( right ) );
   }

   bool UnixTime::operator<( const UnixTime& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
      if ((timeSystem != TimeSystem::Any &&
           right.timeSystem != TimeSystem::Any) &&
          timeSystem != right.timeSystem)
      {
         gpstk::InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
         GPSTK_THROW(ir)
      }

      if( tv.tv_sec  <  right.tv.tv_sec )
      {
         return true;
      }
      if( tv.tv_sec  == right.tv.tv_sec  &&
          tv.tv_usec <  right.tv.tv_usec   )
      {
         return true;
      }
      return false;
   }

   bool UnixTime::operator>( const UnixTime& right ) const
   {
      return ( !operator<=( right ) );
   }

   bool UnixTime::operator<=( const UnixTime& right ) const
   {
      return ( operator<( right ) ||
               operator==( right ) );
   }

   bool UnixTime::operator>=( const UnixTime& right ) const
   {
      return ( !operator<( right ) );
   }

} // namespace
