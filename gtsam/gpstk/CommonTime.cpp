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

#include <limits>
#include <iomanip>
#include <cmath>
#include "CommonTime.hpp"
#include "MathBase.hpp"
#include "StringUtils.hpp"

namespace gpstk
{
      // 'julian day' of earliest epoch expressible by CommonTime; 1/1/4713 B.C.
   const long CommonTime::BEGIN_LIMIT_JDAY = 0L;
      // 'julian day' of latest 'julian day' expressible by CommonTime,
      // 1/1/4713 A.D.
   const long CommonTime::END_LIMIT_JDAY = 3442448L;

      // earliest representable CommonTime
   const CommonTime
   CommonTime::BEGINNING_OF_TIME(CommonTime::BEGIN_LIMIT_JDAY,0,0.0,TimeSystem::Any);
      // latest representable CommonTime
   const CommonTime
   CommonTime::END_OF_TIME(CommonTime::END_LIMIT_JDAY,0,0.0,TimeSystem::Any) ;

   //@{
   /// Default tolerance for time equality, applied to milliseconds.
   const double CommonTime::eps = 4.*std::numeric_limits<double>::epsilon();
   //@}


   CommonTime::CommonTime( const CommonTime& right )
     : m_day( right.m_day ), m_msod( right.m_msod ), m_fsod( right.m_fsod ),
       m_timeSystem( right.m_timeSystem )
   {}

   CommonTime& CommonTime::operator=( const CommonTime& right )
   {
      m_day  = right.m_day;
      m_msod = right.m_msod;
      m_fsod = right.m_fsod;
      m_timeSystem = right.m_timeSystem;
      return *this;
   }


   CommonTime& CommonTime::set( long day,
                                long sod,
                                double fsod,
                                TimeSystem timeSystem )
   {
         // Use temp variables so that we don't modify our
         // data members until we know these values are good.
      if( day < BEGIN_LIMIT_JDAY || day > END_LIMIT_JDAY )
      {
         InvalidParameter ip( "Invalid day: "
                                     + StringUtils::asString( day ) );
         GPSTK_THROW( ip );
      }

      if( sod < 0 || sod >= SEC_PER_DAY )
      {
         InvalidParameter ip( "Invalid seconds of day: "
                                     + StringUtils::asString( sod ) );
         GPSTK_THROW( ip );
      }

      if( fsod < 0.0 || fsod >= 1 )
      {
         InvalidParameter ip( "Invalid fractional-seconds: "
                                     + StringUtils::asString( fsod ) );
         GPSTK_THROW( ip );
      }

         // get the number of milliseconds in the fractional seconds argument
      long msec = static_cast<long>( fsod * MS_PER_SEC );

         // subtract whole milliseconds to obtain the "fractional milliseconds"
      fsod -= static_cast<double>( msec ) * SEC_PER_MS;

      m_day  = day;
      m_msod = sod * MS_PER_SEC + msec;
      m_fsod = fsod;

      m_timeSystem = timeSystem;

      return *this;
   }

   CommonTime& CommonTime::set( long day,
                                double sod,
                                TimeSystem timeSystem )
   {
         // separate whole and fractional seconds, then use set()
      long sec = static_cast<long>( sod );
      sod -= sec;

      return set( day, sec, sod, timeSystem );
   }

  CommonTime& CommonTime::set( double day,
                               TimeSystem timeSystem )
   {
         // separate whole and fractional days
      long lday = static_cast<long>( day );
      double sec = ( day - lday ) * SEC_PER_DAY;
      return set( lday, sec, timeSystem );
   }

   CommonTime& CommonTime::setInternal( long day,
                                        long msod,
                                        double fsod,
                                        TimeSystem timeSystem )
   {
      if( day < BEGIN_LIMIT_JDAY || day > END_LIMIT_JDAY )
      {
         InvalidParameter ip( "Invalid day: "
                                     + StringUtils::asString( day ) );
         GPSTK_THROW( ip );
      }

      if( msod < 0 || msod >= MS_PER_DAY )
      {
         InvalidParameter ip( "Invalid milliseconds of day: "
                                     + StringUtils::asString( msod ) );
         GPSTK_THROW( ip );
      }

      if( fsod < 0.0 || fsod >= SEC_PER_MS )
      {
         InvalidParameter ip( "Invalid fractional-milliseconds: "
                                     + StringUtils::asString( fsod ) );
         GPSTK_THROW( ip );
      }

      m_day  = day;
      m_msod = msod;
      m_fsod = fsod;

      m_timeSystem = timeSystem;

      return *this;
   }

   void CommonTime::get( long& day,
                         long& sod,
                         double& fsod,
                         TimeSystem& timeSystem ) const
   {
      day = m_day;
      sod = m_msod / MS_PER_SEC;
      long msec = m_msod - sod * MS_PER_SEC;  // m_msod % MS_PER_SEC
      fsod = static_cast<double>( msec ) * SEC_PER_MS + m_fsod;
      timeSystem = m_timeSystem;
   }

   void CommonTime::get( long& day,
                         long& sod,
                         double& fsod ) const
   {
      TimeSystem ts;
      CommonTime::get( day, sod, fsod, ts );
   }

   void CommonTime::get( long& day,
                         double& sod,
                         TimeSystem& timeSystem ) const
   {
      day = m_day;
      sod = (double)m_msod / MS_PER_SEC + m_fsod;
      timeSystem = m_timeSystem;
   }

   void CommonTime::get( long& day,
                         double& sod ) const
   {
      TimeSystem ts;
      CommonTime::get( day, sod, ts );
   }

  void CommonTime::get( double& day,
                        TimeSystem& timeSystem ) const
   {
         // convert everything to days
      day = static_cast<double>( m_day ) +
            static_cast<double>( m_msod ) / MS_PER_DAY +
            m_fsod / SEC_PER_DAY;
      timeSystem = m_timeSystem;
   }

   void CommonTime::get( double& day ) const
   {
      TimeSystem ts;
      CommonTime::get( day, ts );
   }

   double CommonTime::getDays() const
   {
      double day;
      get( day );
      return day;
   }

   double CommonTime::getSecondOfDay() const
   {
      long day;
      double sod;
      get( day, sod );
      return sod;
   }

   TimeSystem CommonTime::getTimeSystem() const
   {
      return m_timeSystem;
   }

   double CommonTime::operator-( const CommonTime& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
      if ( (m_timeSystem != TimeSystem::Any &&
               right.m_timeSystem != TimeSystem::Any) &&
               m_timeSystem != right.m_timeSystem )
      {
         InvalidRequest ir("CommonTime objects not in same time system, cannot be differenced");
         GPSTK_THROW( ir );
      }

      return( SEC_PER_DAY * static_cast<double>( m_day  - right.m_day  ) +
              SEC_PER_MS  * static_cast<double>( m_msod - right.m_msod ) +
              m_fsod - right.m_fsod ) ;
   }

   CommonTime CommonTime::operator+( double sec ) const
   {
      return CommonTime( *this ).addSeconds( sec );
   }

   CommonTime CommonTime::operator-( double sec ) const
   {
      return CommonTime( *this ).addSeconds( -sec );
   }

   CommonTime& CommonTime::operator+=( double sec )
   {
      addSeconds( sec );
      return *this;
   }

   CommonTime& CommonTime::operator-=( double sec )
   {
      addSeconds( -sec );
      return *this;
   }

   CommonTime& CommonTime::addSeconds( double seconds )
   {
      long days = 0, ms = 0;

      if ( ABS(seconds) >= SEC_PER_DAY )
      {
         days = static_cast<long>( seconds * DAY_PER_SEC );
         seconds -= days * SEC_PER_DAY;
      }

      if ( ABS(seconds) >= SEC_PER_MS )
      {
         ms = static_cast<long>( seconds * MS_PER_SEC );
         seconds -= static_cast<double>( ms ) / MS_PER_SEC;
      }

      add( days, ms, seconds );
      return *this;
   }

   CommonTime& CommonTime::addSeconds( long seconds )
   {
      long days( 0 );
      if( ABS( seconds ) > SEC_PER_DAY )
      {
         days  = seconds / SEC_PER_DAY;
         seconds -= days * SEC_PER_DAY;  // seconds %= SEC_PER_DAY
      }
      add( days, seconds * MS_PER_SEC, 0. );

      return *this;
   }

   CommonTime& CommonTime::addDays( long days )
   {
      add( days, 0, 0.0 );
      return *this;
   }

   CommonTime& CommonTime::addMilliseconds( long msec )
   {
      add( 0, msec, 0.0 );
      return *this;
   }

   bool CommonTime::operator==( const CommonTime& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
      if ((m_timeSystem != TimeSystem::Any &&
           right.m_timeSystem != TimeSystem::Any) &&
          m_timeSystem != right.m_timeSystem)
         return false;

      return (m_day        == right.m_day     &&
              m_msod       == right.m_msod    &&
              fabs(m_fsod-right.m_fsod) < eps   );
   }

   bool CommonTime::operator!=( const CommonTime& right ) const
   {
      return !operator==(right);
   }

   bool CommonTime::operator<( const CommonTime& right ) const
   {
     /// Any (wildcard) type exception allowed, otherwise must be same time systems
      if ((m_timeSystem != TimeSystem::Any &&
           right.m_timeSystem != TimeSystem::Any) &&
          m_timeSystem != right.m_timeSystem)
      {
         InvalidRequest ir("CommonTime objects not in same time system, cannot be compared: " +
                           m_timeSystem.asString() + " != " + right.m_timeSystem.asString());
         GPSTK_THROW( ir );
      }

      if (m_day < right.m_day)
         return true;
      if (m_day > right.m_day)
         return false;

      if (m_msod < right.m_msod)
         return true;
      if (m_msod > right.m_msod)
         return false;

      if (m_fsod < right.m_fsod)
         return true;

      return false;
   }

   bool CommonTime::operator>( const CommonTime& right ) const
   {
      return !operator <=(right);
   }

   bool CommonTime::operator<=( const CommonTime& right ) const
   {
      return (operator<(right) || operator==(right));
   }

   bool CommonTime::operator>=( const CommonTime& right ) const
   {
      return !operator<(right);
   }

   std::string CommonTime::asString() const
   {
      using namespace std;
      ostringstream oss;
      oss << setfill('0')
          << setw(7) << m_day  << " "
          << setw(8) << m_msod << " "
          << fixed << setprecision(15) << setw(17) << m_fsod
          << " " << m_timeSystem.asString() ;
      return oss.str();
   }

      /// protected functions
   bool CommonTime::add( long days,
                         long msod,
                         double fsod )
   {
      m_day  += days;
      m_msod += msod;
      m_fsod += fsod;

      return normalize();
   }

   bool CommonTime::normalize()
   {
      std::numeric_limits<double> eps;

      if( ABS( m_fsod ) >= SEC_PER_MS - eps.epsilon() ) // allow for machine rounding errors
      {
         long ms = static_cast<long>( (m_fsod + eps.epsilon()) * MS_PER_SEC ); // again
         m_msod += ms;
         m_fsod -= static_cast<double>( ms ) * SEC_PER_MS;
      }

      if( ABS( m_msod ) >= MS_PER_DAY )
      {
         long day = m_msod / MS_PER_DAY;
         m_day  += day;
         m_msod -= day * MS_PER_DAY;
      }

      if( ABS(m_fsod) < 1e-15 )
      {
         m_fsod = 0.0;
      }

      if( m_fsod < 0 )
      {
         m_fsod += SEC_PER_MS;
         --m_msod;
      }

      if( m_msod < 0 )
      {
         m_msod = m_msod + MS_PER_DAY;
         --m_day;
      }

      // byte logic to repair fixed-point math
      if ( m_fsod >= static_cast<double>(0.999) )
         m_fsod -= static_cast<double>(0.999);

      return ( ( m_day >= BEGIN_LIMIT_JDAY ) &&
               ( m_day <  END_LIMIT_JDAY   ) );
   }

   std::ostream& operator<<(std::ostream& o, const CommonTime& ct)
   {
      o << ct.asString();
      return o;
   }

} // namespace
