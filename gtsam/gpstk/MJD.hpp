#pragma ident "$Id$"

#ifndef GPSTK_MJD_HPP
#define GPSTK_MJD_HPP

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
       * This class encapsulates the "Modified Julian Date" time representation.
       */
   class MJD : public TimeTag
   {
   public:
         /**
          * @defgroup mjdbo MJD Basic Operations
          * Default and Copy Constructors, Assignment Operator and Destructor.
          */
         //@{
         /**
          * Default Constructor.
          * All elements are initialized to zero.
          */
      MJD( long double m = 0.,
           TimeSystem ts = TimeSystem::Unknown )
            : mjd( m )
      { timeSystem = ts; }

         /**
          * Copy Constructor.
          * @param right a reference to the MJD object to copy
          */
      MJD( const MJD& right )
            : mjd( right.mjd )
      { timeSystem = right.timeSystem; }

         /**
          * Alternate Copy Constructor.
          * Takes a const TimeTag reference and copies its contents via
          * conversion to CommonTime.
          * @param right a const reference to the BasicTime object to copy
          * @throw InvalidRequest on over-/under-flow
          */
      MJD( const TimeTag& right )
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
      MJD( const CommonTime& right )
      {
         convertFromCommonTime( right );
      }

         /**
          * Assignment Operator.
          * @param right a const reference to the MJD to copy
          * @return a reference to this MJD
          */
      MJD& operator=( const MJD& right );

         /// Virtual Destructor.
      virtual ~MJD()
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
      virtual std::string getPrintChars() const
      {
         return "QP";
      }

         /// Return a string containing the default format to use in printing.
      virtual std::string getDefaultFormat() const
      {
         return "%.9Q %P";
      }

      virtual bool isValid() const;

      virtual void reset();

         /**
          * @defgroup mjdco MJD Comparison Operators
          * All comparison operators have a parameter "right" which corresponds
          *  to the MJD object to the right of the symbol.
          * All comparison operators are const and return true on success
          *  and false on failure.
          */
         //@{
      bool operator==( const MJD& right ) const;
      bool operator!=( const MJD& right ) const;
      bool operator<( const MJD& right ) const;
      bool operator>( const MJD& right ) const;
      bool operator<=( const MJD& right ) const;
      bool operator>=( const MJD& right ) const;
         //@}

      long double mjd;
   };

} // namespace

#endif // GPSTK_MJD_HPP
