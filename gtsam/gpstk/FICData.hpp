#pragma ident "$Id$"



/**
 * @file FICData.hpp
 * gpstk::FICData - container for the FIC file data.
 */

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






#ifndef FICDATA_HPP
#define FICDATA_HPP

#include <vector>
#include <map>

#include "FFStream.hpp"
#include "FICBase.hpp"
#include "EngEphemeris.hpp"
#include "AlmOrbit.hpp"
#include "CommonTime.hpp"

namespace gpstk
{
      /// This gets thrown if we don't have the right data for cast operations.
      /// @ingroup exceptionclass
   NEW_EXCEPTION_CLASS(WrongBlockNumber, gpstk::Exception);
      
      /// This gets thrown if we don't have the wrong format block is used.
      /// @ingroup exceptionclass
   NEW_EXCEPTION_CLASS(WrongBlockFormat, gpstk::Exception);

      /**
       * This does all the grunt-work in reading/decoding 
       * FIC-formatted files (ASCII And Binary).
       * 
       * \sa fic_test.cpp, fic_read_write.cpp, fica_test.cpp for examples.
       *
       * \sa FICStream, FICAStream, and FICHeader.
       */
   class FICData : public FICBase
   {
   public:
         /// Default constructor
      FICData() : blockNum(0) {}

         /*
          * Construct a FICData from an EngEphemeris object.
          * @param engEph the EngEphemeris to copy
          */
         // if you uncomment this, fix the doxygen comment above as well
         //FICData(const EngEphemeris& engEph) throw();
      
         /// Destructor
      virtual ~FICData() {}
         /**
          * Returns whether or not this FICData is valid.
          * Checks the sizes of the f, i, and c vectors using different
          * metrics for each FIC Record Type.
          * See the ICD-GPS-200 for complete details.
          */
      bool isValid() const;      

         //! This class is "data" so this function always returns "true". 
      virtual bool isData() const {return true;}

         /**
          * Debug output function. 
          * Dump the contents of each of the f, i, and c vectors to the 
          * given ostream \c s.
          */ 
      virtual void dump(std::ostream& s) const;

         /**
          * Prints the FIC data in a nice format, labeling all the important
          * subframes and quantities.
          */
      void prettyDump(std::ostream& os) const;

         /**
          * Sets the transmit time of the current block of data.
          * @param dt (output) a CommonTime object containing the transmit time
          * @return if the process was sucessful, return true, else false
          */
      bool getTransmitTime(CommonTime& dt) const;

         /**  
          * Cast *this to an Engineering Ephemeiris Object.
          * @return the constructed EngEphemeris object
          */
      operator EngEphemeris() const throw(WrongBlockNumber);

         /**
          * cast *this into an AlmOrbit, only for block 62s
          * @return the constructed AlmOrbit object
          */
      operator AlmOrbit() const throw(WrongBlockNumber);

         /// Generates a unique key for this FIC data so that redundant
         /// messages can be filtered.  Uses the same criteria
         /// as other generateUniqueKey() functions.
         /// Currently only for blocks 9 and 62 (otherwise it throws)
         /// @sa NavSF::generateUniqueKey()
         /// @sa EngEphData::generateUniqueKey()
      std::string generateUniqueKey() const throw(WrongBlockNumber,
                                                  WrongBlockFormat);

        /***
         * Standard equality operator.
         */
      bool operator==(const FICData& rhs);

     /***
      * Returns a human readable string describing an entry
      * in a FIC vector.
      * @param block Block number: 9, 109, 62, 162.
      * @param type Which vector type. 'f' for floating, 'i' for integer, 'c' for character.
      * @param indx The number of the element in the vector to describe. Starts with zero.
      * @return String describing the entry.
      */
      std::string getElementLabel(char type, size_t indx) const;
  

           /// @name data members
         //@{
      static const std::string blockString; ///< "BLK " record header constsnt
      long blockNum;             ///< Block number for this FIC Record.
      std::vector<double> f;          ///< Vector of floating point numbers.
      std::vector<long> i;            ///< Vector of long integers.
      std::vector<char> c;            ///< Vector of characters.
         //@}

   protected:
         //! Writes this record to the stream \a s.
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, gpstk::StringUtils::StringException, 
               gpstk::FFStreamError);
         /**
          * Read a "FICData" record from the FFStream \c s. 
          * If an error is encountered, the function will 
          * return the stream to its original state and mark its fail-bit.
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s)
         throw(std::exception, gpstk::StringUtils::StringException, 
               gpstk::FFStreamError);

   private:
         /// nicely prints the data from a block 9 record
      void prettyDump9(std::ostream& o) const;
         /// nicely prints the data from a block 109 record
      void prettyDump109(std::ostream& o) const;
         /// nicely prints the data from a block 62 record
      void prettyDump62(std::ostream& o) const;
         /// nicely prints the data from a block 162 record
      void prettyDump162(std::ostream& o) const;

         /// nicely prints the time
      void timeDisplay( std::ostream & os, const char * legend,
                        const short week, const double SOW, 
                        const short headerFlag ) const;

         /// shortcut is used in prettyDump9 to convert the
         /// HOW word time to D:H:M:S and print it out.
      void shortcut(std::ostream & os, const double HOW ) const;
   };
   
}

#endif // FICDATA_HPP

