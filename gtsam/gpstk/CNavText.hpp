#pragma ident "$Id:$"

/**
 * @file CNavText.hpp
 * Designed to support loading CNAV text data
 * (Message Type 15 and/or Type 36)
 */

#ifndef GPSTK_CNAVTEXT_HPP
#define GPSTK_CNAVTEXT_HPP

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
   class CNavText : public CNavDataElement
   {
   public:
         /// Default constructor
      CNavText();
  
        /// Need description here...
      CNavText(const PackedNavBits& pnb)
         throw( InvalidParameter);
      
         /// Destructor
      virtual ~CNavText() {}

         /// Clone method
      virtual CNavText* clone() const;
        
         /**
          * Store the contents of message this object.
          * @param pnb - 300 bits of Message Type 15 or 37
          * @throw InvalidParameter if message data is invalid
          */
      void loadData(const PackedNavBits& pnb)
         throw(gpstk::InvalidParameter); 

      virtual std::string getName() const
      {
         return "CnavText";
      }

      virtual std::string getNameLong() const
      {
         return "Civilian Navigation (CNAV) Text Message";
      }

      virtual bool isSameData(const CNavDataElement* right) const;
      
      virtual void dumpBody(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      std::string textMessage;
      int textPage;
      
   }; // end class CnavText

   std::ostream& operator<<(std::ostream& s, const CNavText& eph);
} // end namespace

#endif // GPSTK_CNAVTEXT_HPP

