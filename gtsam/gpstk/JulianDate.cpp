/// @file JulianDate.cpp

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
#include "JulianDate.hpp"
#include "TimeConstants.hpp"

namespace gpstk
{
   JulianDate& JulianDate::operator=( const JulianDate& right )
   {
      jd = right.jd;
      timeSystem = right.timeSystem;
      return *this;
   }

   CommonTime JulianDate::convertToCommonTime() const
   {
      try
      {
         long double temp_jd( jd + 0.5 );
         long jday( static_cast<long>( temp_jd ) );
         long double sod =
            ( temp_jd - static_cast<long double>( jday ) ) * SEC_PER_DAY;

         CommonTime ct;
         return ct.set( jday,
                        static_cast<long>( sod ),
                        static_cast<double>( sod - static_cast<long>( sod ) ),
                        timeSystem );
      }
      catch (InvalidParameter& ip)
      {
         InvalidRequest ir(ip);
         GPSTK_THROW(ir);
      }
   }

   void JulianDate::convertFromCommonTime( const CommonTime& ct )
   {
      long jday, sod;
      double fsod;
      ct.get( jday, sod, fsod, timeSystem );

      jd =   static_cast<long double>( jday ) +
           (   static_cast<long double>( sod )
             + static_cast<long double>( fsod ) ) * DAY_PER_SEC
           - 0.5;
   }

   std::string JulianDate::printf( const std::string& fmt ) const
   {
      try
      {
         using gpstk::StringUtils::formattedPrint;
         std::string rv( fmt );

         rv = formattedPrint( rv, getFormatPrefixFloat() + "J",
                              "JLf", jd );
         rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                              "Ps", timeSystem.asString().c_str() );
         return rv;
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }

   std::string JulianDate::printError( const std::string& fmt ) const
   {
      try
      {
         using gpstk::StringUtils::formattedPrint;
         std::string rv( fmt );

         rv = formattedPrint( rv, getFormatPrefixFloat() + "J",
                              "Js", getError().c_str() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                              "Ps", getError().c_str() );
         return rv;
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }

   bool JulianDate::setFromInfo( const IdToValue& info )
   {
      using namespace gpstk::StringUtils;

      for( IdToValue::const_iterator i = info.begin(); i != info.end(); i++ )
      {
         switch( i->first )
         {
            case 'J':
               jd = asLongDouble( i->second );
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

   bool JulianDate::isValid() const
   {
      JulianDate temp;
      temp.convertFromCommonTime( convertToCommonTime() );
      if( *this == temp )
      {
         return true;
      }
      return false;
   }

   void JulianDate::reset()
   {
      jd = 0.0;
      timeSystem = TimeSystem::Unknown;
   }

   bool JulianDate::operator==( const JulianDate& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
      if ((timeSystem != TimeSystem::Any &&
           right.timeSystem != TimeSystem::Any) &&
          timeSystem != right.timeSystem)
         return false;

      if( fabs(jd - right.jd) < CommonTime::eps )
      {
         return true;
      }
      return false;
   }

   bool JulianDate::operator!=( const JulianDate& right ) const
   {
      return ( !operator==( right ) );
   }

   bool JulianDate::operator<( const JulianDate& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
      if ((timeSystem != TimeSystem::Any &&
           right.timeSystem != TimeSystem::Any) &&
          timeSystem != right.timeSystem)
      {
         gpstk::InvalidRequest ir("CommonTime objects not in same time system, cannot be compared");
         GPSTK_THROW(ir);
      }

      if( jd < right.jd )
      {
         return true;
      }
      return false;
   }

   bool JulianDate::operator>( const JulianDate& right ) const
   {
      return ( !operator<=( right ) );
   }

   bool JulianDate::operator<=( const JulianDate& right ) const
   {
      return ( operator<( right ) ||
               operator==( right ) );
   }

   bool JulianDate::operator>=( const JulianDate& right ) const
   {
      return ( !operator<( right ) );
   }

} // namespace
