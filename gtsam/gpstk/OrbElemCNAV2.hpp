#pragma ident "$Id$"

/**
 * @file OrbElemCNAV2.hpp
 * Designed to support loading CNAV2 data
 */

#ifndef GPSTK_ORBELEMCNAV2_HPP
#define GPSTK_ORBELEMCNAV2_HPP

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
   class OrbElemCNAV2 : public OrbElemICE
   {
   public:
         /// Default constructor
      OrbElemCNAV2();
  
        /// Need description here...
      OrbElemCNAV2(  const ObsID& obsIDArg,
                     const short PRNIDArg,
                     const int subframe1,
                     const PackedNavBits& subframe2) 
         throw( InvalidParameter);
      
         /// Destructor
      virtual ~OrbElemCNAV2() {}

         /// Clone method
      virtual OrbElemCNAV2* clone() const;

         /**
           * Store a subframe 2 in this object.
           * @param OsbIDArg identifies the carrier and code from which the message was obtained.
           * @param PRNIDArg identifies the PRN ID of the transmitting SV
           * @param subframe1 the immediately preceding subframe 1 data (needed for the TOI count)
           * @param subframe2 600 bit navigation message stored as a PackedNavBits object
           * @throw InvalidParameter if message data is invalid
           *
           */
      void loadData( const ObsID& obsIDArg,
                     const short PRNIDArg,
                     const int subframe1,
                     const PackedNavBits& subframe2)
         throw( InvalidParameter ); 

      virtual std::string getName() const
      {
         return "OrbElemCNAV2";
      }

      virtual std::string getNameLong() const
      {
         return "L1C Navigation Message (CNAV-2)";
      }
      
      virtual void dumpHeader(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      short  L1CHealth;            /**< SV health */
      short  ITOW;		   /**< Interval time of week */
      double Tgd;                  /**< L1 and L2 correction term */
      double ISCP;		   /**< L1Cp correction term */
      double ISCD;                 /**< L1Cd correction term */
      
   }; // end class OrbElemCNAV2

   std::ostream& operator<<(std::ostream& s, 
                                    const OrbElemCNAV2& eph);
} // end namespace

#endif // GPSTK_OrbElemCNAV2_HPP

