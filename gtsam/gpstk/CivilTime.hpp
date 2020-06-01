#pragma ident "$Id$"

#ifndef GPSTK_CIVILTIME_HPP
#define GPSTK_CIVILTIME_HPP

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
#include "TimeSystem.hpp"

namespace gpstk
{
      /**
       * This class encapsulates the representation of time consisting of
       * year, month, day (of month), hour (of day), minute (of hour), and
       * second (of minute).
       */
   class CivilTime : public TimeTag
   {
   public:

         /**
          * @defgroup caltbo CivilTime Basic Operations
          * Default and Copy Constructors, Assignment Operator and Destructor.
          */
         //@{
         /**
          * Default Constructor.
          * All elements default to zero.
          */
      CivilTime( int yr = 0,
                 int mo = 0,
                 int dy = 0,
                 int hr = 0,
                 int mn = 0,
                 double s = 0.0,
                 TimeSystem ts = TimeSystem::Unknown )
	    : year(yr), month(mo), day(dy), hour(hr), minute(mn), second(s)
      { timeSystem = ts; }

         /**
          * Copy Constructor.
          * @param right a const reference to the CivilTime object to copy
          */
      CivilTime( const CivilTime& right )
            : year( right.year ), month( right.month )  , day( right.day ),
              hour( right.hour ), minute( right.minute ), second( right.second )
      { timeSystem = right.timeSystem; }

         /**
          * Alternate Copy Constructor.
          * Takes a const TimeTag reference and copies its contents via
          * conversion to CommonTime.
          * @param right a const reference to the TimeTag-based object to copy
          * @throw InvalidRequest on over-/under-flow
          */
      CivilTime( const TimeTag& right )
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
      CivilTime( const CommonTime& right )
      {
         convertFromCommonTime( right );
      }

         /**
          * Assignment Operator.
          * @param right a const reference to the CivilTime object to copy
          * @return a reference to this CivilTime object
          */
      CivilTime& operator=( const CivilTime& right );

         /// Virtual Destructor.
      virtual ~CivilTime()
      {}
         //@}

         /// Long month names for converstion from numbers to strings
      static const char *MonthNames[];

         /// Short month names for converstion from numbers to strings
      static const char *MonthAbbrevNames[];

         // The following functions are required by TimeTag.
      virtual CommonTime convertToCommonTime() const;

      virtual void convertFromCommonTime( const CommonTime& ct );

         /// This function formats this time to a string.  The exceptions
         /// thrown would only be due to problems parsing the fmt string.
      virtual std::string printf(const std::string& fmt) const;

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
      virtual std::string getPrintChars() const
      {
         return "YymbBdHMSfP";
      }

         /// Return a string containing the default format to use in printing.
      virtual std::string getDefaultFormat() const
      {
         return "%02m/%02d/%04Y %02H:%02M:%02S %P";
      }

      virtual bool isValid() const;

      virtual void reset();

         /**
          * @defgroup ctco CivilTime Comparison Operators
          * All comparison operators have a parameter "right" which corresponds
          *  to the CivilTime object to the right of the symbol.
          * All comparison operators are const and return true on success
          *  and false on failure.
          */
         //@{
      bool operator==( const CivilTime& right ) const;
      bool operator!=( const CivilTime& right ) const;
      bool operator<( const CivilTime& right ) const;
      bool operator>( const CivilTime& right ) const;
      bool operator<=( const CivilTime& right ) const;
      bool operator>=( const CivilTime& right ) const;
         //@}

      int year;
      int month;
      int day;
      int hour;
      int minute;
      double second;

   };

      // -----------CivilTime operator<< -----------
      //
      /**
       * Stream output for CivilTime objects.  Typically used for debugging.
       * @param s stream to append formatted CivilTime to.
       * @param cit CivilTime to append to stream \c s.
       * @return reference to \c s.
       */
   std::ostream& operator<<( std::ostream& s,
                             const gpstk::CivilTime& cit );

} // namespace

#endif // GPSTK_CIVILTIME_HPP
