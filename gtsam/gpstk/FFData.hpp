#pragma ident "$Id$"



/**
 * @file FFData.hpp
 * Formatted File Data base class
 */

#ifndef GPSTK_FFDATA_HPP
#define GPSTK_FFDATA_HPP

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
#include <typeinfo>

#include <gtsam/gpstk/FFStreamError.hpp>
#include <gtsam/gpstk/StringUtils.hpp>

namespace gpstk
{
   /** @addtogroup formattedfile */
   //@{

      /// Forward declaration of FFStream class and friend functions
   class FFStream;

      /**
       * This is the base class for all Formatted File Data (FFData).
       * The data in FFStream objects are read/written into classes derived
       * from FFData.  There are two interfaces for this - one with the
       * operator<<() and operator>>() and the equivalent with getRecord() and 
       * putRecord().
       * 
       * Classes that derive from this should override the reallyGetRecord()
       * and reallyPutRecord() functions where the actual reading and
       * writing of the data should go as well as the isHeader() or isData()
       * functions as appropriate for the data type.  
       * Overriding dump() for debugging purposes is highly suggested.
       * 
       * All FFData derived classes should have all of their members defined
       * public.  These are meant to be simple container classes for reading,
       * writing, and manipulating file data. Any conversion of data types
       * should be made in constructors or cast operators.  For all FFData
       * objects, any internal time representation for the data should be
       * called \c time to ensure consistency between classes.
       *
       * Finally, for the sake of organization of the inheritence and
       * dependency trees, all related file data classes (i.e. RinexObsData and
       * RinexObsHeader) should derive from a common base (i.e. RinexObsBase).
       *
       * \sa FFStream for more information.
       */
   class FFData
   {
   public:
         /// virtual desctuctor
      virtual ~FFData(void) {};

         /**
          * Send a "record" to the given stream.
          * @param s a FFStream-based stream
          */
      void putRecord(FFStream& s) const 
         throw(FFStreamError, gpstk::StringUtils::StringException);

         /**
          * Retrieve a "record" from the given stream.
          * By default, this won't throw exceptions.  You can change this
          * by enabling exceptions with this code:
          * \code
          * filestream.exceptions(fstream::failbit);
          * \endcode
          * @param s a FFStream-based stream
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      void getRecord(FFStream& s) 
         throw(FFStreamError, gpstk::StringUtils::StringException);

         /**
          * Send debug output to the given stream.
          * @param s a generic output stream
          */
      virtual void dump(std::ostream& s) const {}

         /**
          * Returns whether or not this class is a header.
          * Have your overriding function return the correct value.
          */
      virtual bool isHeader() const {return false;}
         /** 
          * Returns whether or not this class is data.
          * Have your overriding function return the correct value.
          */
      virtual bool isData() const {return false;}

         /**
          * Our generic formatted output operator
          * This function provides compatibility with the ostream_iterator
          * class.
          * @param o the stream to write to
          * @param f the data to write
          * @return a reference to \c o
          * @warning The ostream provided MUST be an FFStream.
          */
      friend std::ostream& operator<<(std::ostream& o, const FFData& f)
         throw(FFStreamError, gpstk::StringUtils::StringException);

         /**
          * Generic formatted input operator.
          * This should throw the same exceptions as
          * FFData::getRecord().  Please read that for more info.
          * @see FFData::getRecord().
          * @warning see FFData::getRecord() for more info on exceptions
          * @param f formatted file data record.
          * @param i the stream to read from
          * @throw FFStreamError if the file being read is formatted
          * incorrectly or some other file error occurs
          * @throw StringException when StringUtils messes up.
          * @warning read the notes for getRecord() for whatever
          * class you'll be using this with.
          * @return a reference to \c i.
          * This function provides compatibility with the istream_iterator
          * class.
          * @warning The istream provided MUST be an FFStream.
          */
      friend std::istream& operator>>(std::istream& i, FFData& f)
         throw(FFStreamError, gpstk::StringUtils::StringException);

      friend class FFStream;

   protected:
         /// Does the actual reading from the stream into this FFData object.
      virtual void reallyGetRecord(FFStream& s)
         throw(std::exception, gpstk::StringUtils::StringException, 
               gpstk::FFStreamError) = 0;

         /// Does the actual writing from the stream into this FFData object.
      virtual void reallyPutRecord(FFStream& s) const
         throw(std::exception, gpstk::StringUtils::StringException, 
               gpstk::FFStreamError) = 0;
   }; // class

   //@}

} // namespace

#endif

