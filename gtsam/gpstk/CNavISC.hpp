#pragma ident "$Id:$"

/**
 * @file CNavISC.hpp
 * Designed to support loading CNAV Tgd and ISC data
 */

#ifndef GPSTK_CNAVISC_HPP
#define GPSTK_CNAVISC_HPP

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
//  Copyright 2013, The University of Texas at Austin
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

#include "CNavDataElement.hpp"
#include "PackedNavBits.hpp"

namespace gpstk
{
   class CNavISC : public CNavDataElement
   {
   public:
         /// Default constructor
      CNavISC();
  
        /// Need description here...
      CNavISC(const PackedNavBits& message30)
         throw( InvalidParameter);
      
         /// Destructor
      virtual ~CNavISC() {}

         /// Clone method
      virtual CNavISC* clone() const;

      virtual bool isSameData(const CNavDataElement* right) const;     
        
         /**
          * Store the contents of message type 30 in this object.
          * @param osbIDArg the carrier and code from which the message was obtained.
          * @param satIDArg the system and ID of the transmitting SV
          * @param message30 - 300 bits of Message Type 30
          * @throw InvalidParameter if message data is invalid
          */
      void loadData(const PackedNavBits& message30)
         throw(gpstk::InvalidParameter); 

      virtual std::string getName() const
      {
         return "CNavISC";
      }

      virtual std::string getNameLong() const
      {
         return "Civilian Navigation (CNAV) Group Delay and ISC";
      }
      
      virtual void dumpBody(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      double Tgd;
      double ISC_L1CA;
      double ISC_L2C;
      double ISC_L5I5;
      double ISC_L5Q5;

         // See IS-GPS-705 20.3.3.3.1.2.  If transmitted data is "1000000000000" the
         // term is not available.  The following members are set accordingly.
      bool   avail_Tgd;
      bool   avail_L1CA;
      bool   avail_L2C;
      bool   avail_L5I5;
      bool   avail_L5Q5;

         // NOTE: units are sec, sec/rad, sec/rad**2, and sec/rad**3
      double alpha[4];    
      double beta[4];     
   }; // end class CnavISC

   std::ostream& operator<<(std::ostream& s, 
                                    const CNavISC& eph);
} // end namespace

#endif // GPSTK_CNAVISC_HPP

