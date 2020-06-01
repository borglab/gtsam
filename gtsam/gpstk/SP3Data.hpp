#pragma ident "$Id$"

/**
 * @file SP3Data.hpp
 * Encapsulate SP3 file data, versions a,b,c, including I/O
 */

#ifndef GPSTK_SP3DATA_HPP
#define GPSTK_SP3DATA_HPP

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

#include "SP3SatID.hpp"
#include "SP3Base.hpp"
#include "CommonTime.hpp"
#include <iomanip>

namespace gpstk
{
   /** @addtogroup SP3ephem */
   //@{

      /**
       * This class encapsulates data for satellite orbits and clocks, including
       * positions, velocities and other orbit and estimation information read
       * as found in I/O of SP3 format (versions a, b, or c) files.
       *
       * This class is used in conjuction with class SP3Stream, which handles the I/O,
       * and SP3Header, which holds information from the SP3 file header.
       * Note that the version of SP3 is stored ONLY in the SP3Header object.
       * This version is set when an SP3 header is read into SP3Header, and it may
       * be set by the user using SP3Header::setVersion().
       * On output, SP3Stream uses the version stored in the SP3Header to determine
       * how SP3Data (this object) is output.
       *
       * @code
       * SP3Stream ss("igr14080.sp3");
       * SP3Header sh;
       * SP3Data sd;
       *
       * ss >> sh;
       *
       * while (ss >> sd)
       * {
       *    // Interesting stuff...
       * }    
       *
       * SP3Stream ssout("myfile.sp3", ios::out);
       * sh.setVersion(SP3Header::SP3c);
       * ssout << sh;
       * for(...) {
       *    // perhaps modify sd
       *    ssout << sd
       * }
       * @endcode
       *
       * @sa gpstk::SP3Header and gpstk::SP3Stream for more information.
       * @sa petest.cpp for an example.
       */
   class SP3Data : public SP3Base
   {
   public:
         /// Constructor.
      SP3Data() : RecType(' '), time(CommonTime::BEGINNING_OF_TIME),
                  clockEventFlag(false), clockPredFlag(false),
                  orbitManeuverFlag(false), orbitPredFlag(false),
                  correlationFlag(false)
         {}
     
         /// Destructor
      virtual ~SP3Data() {}
     
         // The next four lines is our common interface
         /// SP3Data is "data" so this function always returns true.
      virtual bool isData() const {return true;}

         /// Debug output function.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
       virtual void dump(std::ostream& s=std::cout, bool includeC=true) const throw();
#pragma clang diagnostic pop
         ///@name data members
         //@{
      char RecType;    ///< Data type indicator. P position, V velocity, * epoch
      SatID sat;       ///< Satellite ID
      CommonTime time; ///< Time of epoch for this record
      double x[3];     ///< The three-vector for position | velocity (m | dm/s).
      double clk;      ///< The clock bias or drift for P|V (microsec|1).

      /// the rest of the member are for version c only
      int sig[4];      ///< Four-vector of integer exponents for estimated sigma 
                       ///< of position,clock or velocity,clock rate; sigma = base**n
                       ///< units are mm,psec or 10^-4 mm/sec,psec/sec); base in head.
                       ///< n is >= 0, and n = -1 means unknown (blank in file)
      bool clockEventFlag;    ///< clock event flag, 'E' in file
      bool clockPredFlag;     ///< clock prediction flag, 'P' in file
      bool orbitManeuverFlag; ///< orbit maneuver flag, 'M' in file
      bool orbitPredFlag;     ///< orbit prediction flag, 'P' in file
      /// data for optional P|V Correlation record
      bool correlationFlag;   ///< If true, on input: a correlation record was read;
                              ///< on output: stream should output correlation.
      unsigned sdev[4];  ///< std dev of 3 positions (XYZ,mm) and clock (psec)
                         ///< or velocities(10^-4 mm/sec) and clock rate (10^-4 ps/s)
      int correlation[6];///< elements of correlation matrix: xy,xz,xc,yz,yc,zc
         //@}
      
   protected:

         /// Writes the formatted record to the FFStream \a s.
         /// @warning This function is currently unimplemented
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException);

         /**
          * This function reads a record from the given FFStream.
          * If an error is encountered in retrieving the record, the 
          * stream is reset to its original position and its fail-bit is set.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException);
   };

   //@}

}  // namespace

#endif
