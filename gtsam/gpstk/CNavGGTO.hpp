#pragma ident "$Id:$"

/**
 * @file CNavGGTO.hpp
 * Designed to support loading CNAV GGTO data
 * (Message Type 35)
 */

#ifndef GPSTK_CNAVGGTO_HPP
#define GPSTK_CNAVGGTO_HPP

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
   class CNavGGTO : public CNavDataElement
   {
   public:
      static const int NO_DATA_AVAIL;
      static const int GALILEO_ID;
      static const int GLONASS_ID;
   
         /// Default constructor
      CNavGGTO();
  
        /// Need description here...
      CNavGGTO(const PackedNavBits& message35)
         throw( InvalidParameter);
      
         /// Destructor
      virtual ~CNavGGTO() {}

         /// Clone method
      virtual CNavGGTO* clone() const;

      virtual bool isSameData(const CNavDataElement* right) const;
       
         /**
          * Store the contents of message type 35 in this object.
          * @param message30 - 300 bits of Message Type 35
          * @throw InvalidParameter if message data is invalid
          */
      void loadData(const PackedNavBits& message35)
         throw(gpstk::InvalidParameter); 

      virtual std::string getName() const
      {
         return "CNavGGTO";
      }

      virtual std::string getNameLong() const
      {
         return "Civilian Navigation (CNAV) GGTO Parameters";
      }
      
      virtual void dumpBody(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      double A0GGTO;
      double A1GGTO;
      double A2GGTO;
      long   TGGTO;   // Note: This is stored for completeness,
      short  WNGGTO;  // The epoch time variable provides a CommonTime representation,
      short  GNSS_ID;
      
   }; // end class CNavGGTO

   std::ostream& operator<<(std::ostream& s, 
                                    const CNavGGTO& eph);
} // end namespace

#endif // GPSTK_CNAVGGTO_HPP

