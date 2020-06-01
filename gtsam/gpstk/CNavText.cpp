#pragma ident "$Id: $"

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

/**
 * @file CnavText.cpp
 * CnavText data encapsulated in engineering terms
 */
#include <iomanip>

#include "CNavText.hpp"
#include "StringUtils.hpp"
#include "TimeString.hpp"

namespace gpstk
{
   using namespace std;

   CNavText::CNavText()
      :CNavDataElement(),
       textPage(0)
   {
      ctEpoch = CommonTime::BEGINNING_OF_TIME;
      ctEpoch.setTimeSystem(TimeSystem::GPS);
      ctXmit = CommonTime::BEGINNING_OF_TIME;
      ctXmit.setTimeSystem(TimeSystem::GPS);
      textMessage = "";
   }

   CNavText::CNavText(const PackedNavBits& pnb)
      throw( InvalidParameter)
   {
      loadData(pnb);
   }

   CNavText* CNavText::clone() const
   {
      return new CNavText (*this); 
   }

     // In this case, since epoch time is arbitrarily set to Xmit, 
     // the epoch time is NOT a distinguishing factor.  (This is
     // worth noting because epoch time frequently is THE 
     // distinguishing factor.) 
   bool CNavText::isSameData(const CNavDataElement* right) const      
   {
      if (const CNavText* rp = dynamic_cast<const CNavText*>(right))
      {
         if (textMessage.compare(rp->textMessage)!=0)  return false;
         if (textPage!=rp->textPage)     return false;
         return true;      
      }
      return false;
   }
   
   void CNavText::loadData(const PackedNavBits& pnb)
      throw(InvalidParameter)
   {
         // First, verify the correct message type is being passed in. 
      long msgType = pnb.asUnsignedLong(14,6,1);
      if(msgType!=15 && msgType!=36)
      {
         char temp[80];
         sprintf(temp,"Expected CNAV MsgType 15 or 36.  Found MsgType %ld",msgType);
         std::string tstr(temp);
         InvalidParameter exc(tstr);
         GPSTK_THROW(exc);    
      } 
      
      obsID     = pnb.getobsID();
      satID     = pnb.getsatSys();
      ctXmit    = pnb.getTransmitTime();
      ctEpoch   = ctXmit;

      if (msgType==15)
      {
         textMessage  = pnb.asString(38,29);
         textPage     = pnb.asUnsignedLong(270, 4, 1);
      }
      else // Must be msgType 36
      {
         textMessage  = pnb.asString(127,18);
         textPage     = pnb.asUnsignedLong(271, 4, 1);
      }   
      
      dataLoadedFlag = true;   
   } // end of loadData()

   void CNavText::dumpBody(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
    
      s << endl
        << "           TEXT MESSAGE PARAMETERS"
        << endl
        << endl;
      s << "Text Page: " << textPage << endl;
      s << "Message  : '" << textMessage << "'" << endl;
      
   } // end of dumpBody()   

   ostream& operator<<(ostream& s, const CNavText& eph)
   {
      try
      {
         eph.dump(s);
      }
      catch(gpstk::Exception& ex)
      {
         GPSTK_RETHROW(ex);
      }
      return s;

   } // end of operator<<

} // end namespace
