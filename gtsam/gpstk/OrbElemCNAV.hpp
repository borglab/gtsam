#pragma ident "$Id$"

/**
 * @file OrbElemCNAV.hpp
 * Designed to support loading CNAV data
 */

#ifndef GPSTK_ORBELEMCNAV_HPP
#define GPSTK_ORBELEMCNAV_HPP

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

#include <string>
#include <iostream>

#include "OrbElemICE.hpp"
#include "FICData.hpp"
#include "PackedNavBits.hpp"

namespace gpstk
{
   class OrbElemCNAV : public OrbElemICE
   {
   public:
         /// Default constructor
      OrbElemCNAV();
  
        /// Need description here...
      OrbElemCNAV( const ObsID& obsIDArg,
                   const SatID& satIDArg,
                   const PackedNavBits& message10,
                   const PackedNavBits& message11,
                   const PackedNavBits& messageClk)
         throw( InvalidParameter);
      
         /// Destructor
      virtual ~OrbElemCNAV() {}

         /// Clone method
      virtual OrbElemCNAV* clone() const;
        
         /**
          * Store a subframe 2 in this object.
          * @param osbIDArg the carrier and code from which the message was obtained.
          * @param satIDArg the system and ID of the transmitting SV
          * @param message10 - 300 bits of Message Type 10
          * @param message11- 300 bits of Message Type 11
          * @param messageClk - 300 bits of any of Message Type 30-37
          * @throw InvalidParameter if message data is invalid
          */
      void loadData( const ObsID& obsIDArg,
                     const SatID& satIDArg,
                     const PackedNavBits& message10,
                     const PackedNavBits& message11,
                     const PackedNavBits& messageClk)
         throw(gpstk::InvalidParameter); 

      virtual std::string getName() const
      {
         return "OrbElemCNAV";
      }

      virtual std::string getNameLong() const
      {
         return "Civilian Navigation (CNAV) Message";
      }
      
      virtual void dumpHeader(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      CommonTime ctMsg10;          /**< Message 10 transmit time */
      CommonTime ctMsg11;          /**< Message 11 transmit time */
      CommonTime ctMsgClk;         /**< Message 30-37 transmit time */
  
      short  L1Health;             /**< SV L1 health */
      short  L2Health;             /**< SV L2 health */
      short  L5Health;             /**< SV L5 health */
      short  ITOW;		   /**< Interval time of week */

      short  L2CPhasing;   /**< L2C Phasing flag */
          
   }; // end class OrbElemCNAV

   std::ostream& operator<<(std::ostream& s, 
                                    const OrbElemCNAV& eph);
} // end namespace

#endif // GPSTK_OrbElemCNAV_HPP

