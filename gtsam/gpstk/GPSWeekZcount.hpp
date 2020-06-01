#pragma ident "$Id$"



#ifndef GPSTK_GPSWEEKZCOUNT_HPP
#define GPSTK_GPSWEEKZCOUNT_HPP

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

#include "GPSWeek.hpp"
#include "TimeConstants.hpp"
#include "TimeSystem.hpp"

namespace gpstk
{
      /**
       * This class encapsulates the "Full GPS Week and GPS Z-count" time
       * representation.
       */
   class GPSWeekZcount : public GPSWeek
   {
   public:
         /// This is just a 19-bit mask.
      static const unsigned int bits19 = 0x7FFFF;

         /**
          * @defgroup gwzbo GPSWeekZcount Basic Operations
          * Default and Copy Constructors, Assignment Operator and Destructor.
          */
         //@{
         /**
          * Default Constructor.
          * All elements are initialized to zero.
          */
      GPSWeekZcount( int w = 0,
                     int z = 0,
                     TimeSystem ts = TimeSystem::Unknown )
            : GPSWeek( w ), zcount( z )
      { timeSystem = ts; }

         /**
          * Copy Constructor.
          * @param right a reference to the GPSWeekZcount object to copy
          */
      GPSWeekZcount( const GPSWeekZcount& right )
            : GPSWeek( right.week ), zcount( right.zcount )
      { timeSystem = right.timeSystem; }

         /**
          * Alternate Copy Constructor.
          * Takes a const TimeTag reference and copies its contents via
          * conversion to CommonTime.
          * @param right a const reference to the BasicTime object to copy
          * @throw InvalidRequest on over-/under-flow
          */
      GPSWeekZcount( const TimeTag& right )
      {
         convertFromCommonTime( right.convertToCommonTime() );
      }

         /**
          * Alternate Copy Constructor.
          * Takes a const CommonTime reference and copies its contents via
          * the convertFromCommonTime method.
          * @param right a const reference to the CommonTime object to copy
          * @throw InvalidRequest on over-/under-flow
          */
      GPSWeekZcount( const CommonTime& right )
      {
         convertFromCommonTime( right );
      }

         /**
          * Assignment Operator.
          * @param right a const reference to the GPSWeekZcount to copy
          * @return a reference to this GPSWeekZcount
          */
      GPSWeekZcount& operator=( const GPSWeekZcount& right );

         /// Virtual Destructor.
      virtual ~GPSWeekZcount()
      {}
         //@}

         // The following functions are required by TimeTag.
      virtual CommonTime convertToCommonTime() const;

      virtual void convertFromCommonTime( const CommonTime& ct );

         /// This function formats this time to a string.  The exceptions
         /// thrown would only be due to problems parsing the fmt string.
      virtual std::string printf( const std::string& fmt ) const;

         /// This function works similarly to printf.  Instead of filling
         /// the format with data, it fills with error messages.
      virtual std::string printError( const std::string& fmt) const;

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
         return GPSWeek::getPrintChars() + "wzZcC";
      }

         /// Return a string containing the default format to use in printing.
      inline virtual std::string getDefaultFormat() const
      {
         return GPSWeek::getDefaultFormat() + " %06Z %P";
      }

      virtual bool isValid() const
      {
	      return ( GPSWeek::isValid() && zcount < ZCOUNT_PER_WEEK );
      }

      inline virtual void reset()
      {
	      GPSWeek::reset();
	      zcount = 0;
      }

         /// @name Special Zcount-related Methods.
         /// @note The 29- and 32-bit Zcounts cannot represent time from
         /// GPS weeks over 1023 and 8191 respectively.
         //@{
      inline unsigned int getZcount29() const
      {
         return (getWeek10() << 19) | zcount;
      }

      inline unsigned int getZcount32() const
      {
         return (week << 19) | zcount;
      }

      GPSWeekZcount& setZcount29(unsigned int z);

      GPSWeekZcount& setZcount32(unsigned int z);
         //@}

      inline virtual unsigned int getDayOfWeek() const
      {
         return static_cast<unsigned int>(zcount) / ZCOUNT_PER_DAY;
      }

         /**
          * @defgroup gwzco GPSWeekZcount Comparison Operators
          * All comparison operators have a parameter "right" which corresponds
          *  to the GPSWeekZcount object to the right of the symbol.
          * All comparison operators are const and return true on success
          *  and false on failure.
          */
         //@{
     //
      inline bool operator==( const GPSWeekZcount& right ) const
      {
         return ( GPSWeek::operator==(right) && zcount == right.zcount );
      }

      inline bool operator!=( const GPSWeekZcount& right ) const
      {
   	   return ( !operator==( right ) );
      }

      inline bool operator<( const GPSWeekZcount& right ) const
      {
   	   if( GPSWeek::operator<(right) )
   	   {
   	      return true;
   	   }
   	   if( GPSWeek::operator>(right) )
   	   {
   	      return false;
   	   }
   	   if( zcount < right.zcount )
   	   {
   	      return true;
   	   }
   	   return false;
      }

      inline bool operator>( const GPSWeekZcount& right ) const
      {
   	   return ( !operator<=( right ) );
      }

      inline bool operator<=( const GPSWeekZcount& right ) const
      {
   	   return ( operator<( right ) || operator==( right ) );
      }

      inline bool operator>=( const GPSWeekZcount& right ) const
      {
   	   return ( !operator<( right ) );
      }
         //@}

      unsigned int zcount;
   };

} // namespace

#endif // GPSTK_GPSWEEKZCOUNT_HPP
