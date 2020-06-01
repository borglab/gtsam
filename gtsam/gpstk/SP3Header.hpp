#pragma ident "$Id$"

/**
 * @file SP3Header.hpp
 * Encapsulate header of SP3 file data, including I/O
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

#ifndef GPSTK_SP3HEADER_HPP
#define GPSTK_SP3HEADER_HPP

#include <string>
#include <map>
#include <vector>
#include "SP3Base.hpp"
#include "SP3SatID.hpp"
#include "CommonTime.hpp"
#include "TimeSystem.hpp"

namespace gpstk
{
   /** @addtogroup SP3ephem */
   //@{

      /// This class models the header for a SP3 file.
      ///
      /// @note A valid header MUST be read before data can be read from an SP3 file
      /// because the header contains the file version or format. The version in
      /// this Header is used by SP3Stream to determine the format of output SP3Data.
      ///
      /// @sa gpstk::SP3Stream and gpstk::SP3Data for more information.
   class SP3Header : public SP3Base
   {
   public:

         /// Supported SP3 versions (file formats) : 'a' 'b' or 'c'
         /// See the SP3 format definition documents.
      enum Version
      {
         undefined,        ///< Unknown or undefined SP3 file format
         SP3a,             ///< SP3 version a
         SP3b,             ///< SP3 version b (very similar to SP3a)
         SP3c              ///< SP3 version c (contains a/b as a subset)
      };

         /// constructor
      SP3Header() : version(undefined), numberOfEpochs(0),
                    system(1, SP3SatID::systemGPS), timeSystem(TimeSystem::Any),
                    basePV(0.0), baseClk(0.0)
                    {}

         /// destructor
      virtual ~SP3Header() {}

         /// access the version or file format
         /// @return the current Version
      Version getVersion(void) const throw() { return version; }

         /// access the version or file format as a character
         /// @return a character version of the current Version
      char versionChar(void) const throw()
      {
         char ch;
         switch(version) {
            case SP3a:
               ch = 'a'; break;
            case SP3b:
               ch = 'b'; break;
            case SP3c:
               ch = 'c'; break;
            case undefined: default:
               ch = 'U'; break;
         };
         return ch;
      }

         /// access the version or file format as a string
         /// @return a string version of the current Version
      std::string versionString(void) const throw()
      {
         std::string str;
         switch(version) {
            case SP3a:
               str = std::string("SP3a"); break;
            case SP3b:
               str = std::string("SP3b"); break;
            case SP3c:
               str = std::string("SP3c"); break;
            case undefined: default:
               str = std::string("Undefined"); break;
         };
         return str;
      }

         /// set the version or file format. Note that reading an SP3 file
         /// automatically sets the version in the SP3Header object that is read.
         /// @param ver the Version to be assigned to this header
         /// @return the current Version
      Version setVersion(const Version ver) throw()
      {
         Version oldFormat = version;
         version = ver;
         return oldFormat;
      }

         /// return a string with time system name
      std::string timeSystemString() const throw()
      { return timeSystem.asString(); };

         // The next four lines is our common interface
         /// SP3Header is a "header" so this function always returns true.
      virtual bool isHeader() const { return true; }
     
         /// Dump contents to an ostream
      virtual void dump(std::ostream& s=std::cout) const throw();


         ///@name data members
         //@{

         /// The SP3 version (file format) is initially undefined, but it will be
         /// assigned by reallyGetRecord() while reading, and may be reassigned
         /// by the user before writing.
      Version version;           ///< SP3 Version or file format
      bool containsVelocity;     ///< If true, file contains velocities
      CommonTime time;           ///< Time of first Epoch in file
      double epochInterval;      ///< Duration of Epoch in seconds
      int numberOfEpochs;        ///< Number of epochs in this file
      std::string dataUsed;      ///< Types of data input into the positions
      std::string coordSystem;   ///< Coordinate System of the data
      std::string orbitType;     ///< Type of Orbit Estimate
      std::string agency;        ///< Agency generating the Orbit

      // the following are specific to version 'c'
      SP3SatID system;        ///< system of satellites in file, e.g. G for GPS
      TimeSystem timeSystem;  ///< Time system used
      double basePV;          ///< Base used in Pos or Vel (mm or 10**-4mm/sec)
      double baseClk;         ///< Base used in Clk or rate (psec or 10**-4psec/sec)
      /// Map<SP3SatID,accuracy flag> (all SVs in file)
      std::map<SP3SatID, short> satList;
      /// vector of 4 comment lines
      std::vector<std::string> comments;

         //@}

      friend class SP3Data;

   protected:
         /// Writes the record formatted to the FFStream \a s.
         /// @throws StringException when a StringUtils function fails
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError,
               StringUtils::StringException);

         /// This function retrieves the SP3 header from the given FFStream.
         /// If an error is encountered in the retrieval of the header, the
         /// stream is reset to its original position and its fail-bit is set.
         /// @throws StringException when a StringUtils function fails
         /// @throws FFStreamError when exceptions(failbit) is set and
         ///  a read or formatting error occurs.  This also resets the
         ///  stream to its pre-read position.
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError,
               StringUtils::StringException);

   }; // end class SP3Header

   //@}

}  // namespace

#endif

