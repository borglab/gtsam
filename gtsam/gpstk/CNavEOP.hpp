#pragma ident "$Id:$"

/**
 * @file CnavEOP.hpp
 * Designed to support loading CNAV EOP data
 * (Message Type 32)
 */

#ifndef GPSTK_CNAVEOP_HPP
#define GPSTK_CNAVEOP_HPP

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
   class CNavEOP : public CNavDataElement
   {
   public:
         /// Default constructor
      CNavEOP();
  
        /// Need description here...
      CNavEOP(const PackedNavBits& message32)
         throw( InvalidParameter);
      
         /// Destructor
      virtual ~CNavEOP() {}

         /// Clone method
      virtual CNavEOP* clone() const;
      
      virtual bool isSameData(const CNavDataElement* right) const;      
        
         /**
          * Store the contents of message type 33 in this object.
          * @param message30 - 300 bits of Message Type 33
          * @throw InvalidParameter if message data is invalid
          */
      void loadData(const PackedNavBits& message32)
         throw(gpstk::InvalidParameter); 

      virtual std::string getName() const
      {
         return "CNavEOP";
      }

      virtual std::string getNameLong() const
      {
         return "Civilian Navigation (CNAV) EOP Parameters";
      }
      
      virtual void dumpBody(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      double PM_X;
      double PM_X_dot;
      double PM_Y;
      double PM_Y_dot;
      double deltaUT1;
      double deltaUT1_dot;
      double deltaTls;
      long   Teop;       // Note: This is stored for completeness,
                         // The epoch time variable provides a CommonTime representation.
   }; // end class CNavEOP

   std::ostream& operator<<(std::ostream& s, 
                                    const CNavEOP& eph);
} // end namespace

#endif // GPSTK_CNavEOP_HPP

