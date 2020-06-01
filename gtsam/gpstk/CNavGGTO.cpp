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
 * @file CNavGGTO.cpp
 * CNAV GGTO data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "CNavGGTO.hpp"
#include "StringUtils.hpp"
#include "TimeString.hpp"

namespace gpstk
{
   using namespace std;

   const int CNavGGTO::NO_DATA_AVAIL = 0x0000;
   const int CNavGGTO::GALILEO_ID    = 0x0001;
   const int CNavGGTO::GLONASS_ID    = 0x0002;

   CNavGGTO::CNavGGTO()
      :CNavDataElement(),
       A0GGTO(0.0), 
       A1GGTO(0.0),
       A2GGTO(0.0),
       TGGTO(0L),
       WNGGTO(0)
   {
      GNSS_ID = NO_DATA_AVAIL;
   }

   CNavGGTO::CNavGGTO(const PackedNavBits& message35)
      throw( InvalidParameter)
   {
      loadData(message35);
   }

   CNavGGTO* CNavGGTO::clone() const
   {
      return new CNavGGTO (*this); 
   }

   bool CNavGGTO::isSameData(const CNavDataElement* right) const      
   {
      if (const CNavGGTO* rp = dynamic_cast<const CNavGGTO*>(right))
      {
         if (ctEpoch  !=rp->ctEpoch)   return false;
         if (A0GGTO   !=rp->A0GGTO)    return false;
         if (A1GGTO   !=rp->A1GGTO)    return false;
         if (A2GGTO   !=rp->A2GGTO)    return false;
         if (GNSS_ID  !=rp->GNSS_ID)   return false;
         return true;      
      }
      return false;
   }
   
   void CNavGGTO::loadData(const PackedNavBits& message35)
      throw(InvalidParameter)
   {
         // First, verify the correct message type is being passed in. 
      long msgType = message35.asUnsignedLong(14,6,1);
      if(msgType!=35)
      {
         char errStr[80];
         sprintf(errStr,"Expected CNAV MsgType 35.  Found MsgType %ld",msgType);
         std::string tstr(errStr);
         InvalidParameter exc(tstr);
         GPSTK_THROW(exc);    
      } 
      obsID     = message35.getobsID();
      satID     = message35.getsatSys();
      ctXmit    = message35.getTransmitTime();

      TGGTO     = message35.asLong(127,16,16);
      WNGGTO    = message35.asLong(143,13, 1);
      GNSS_ID   = message35.asLong(156, 3, 1); 
      A0GGTO    = message35.asSignedDouble(158,16,-35);
      A1GGTO    = message35.asSignedDouble(175,13,-51);
      A2GGTO    = message35.asSignedDouble(188, 7,-68);

      if (GNSS_ID>0)
         ctEpoch   = GPSWeekSecond(WNGGTO, TGGTO, TimeSystem::GPS);

      dataLoadedFlag = true;   
   } // end of loadData()

   void CNavGGTO::dumpBody(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
    
      s << endl
        << "           GPS/GNSS TIME OFFSET PARAMETERS"
        << endl
        << "Parameter        Value" << endl;

      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      s << "GNSS_ID:          " << GNSS_ID; 
      if (GNSS_ID==NO_DATA_AVAIL) 
      { 
         s << ", NO DATA AVAILABLE" << endl;
         return;
      }
      else if (GNSS_ID==GALILEO_ID) s << ", Galileo";
      else if (GNSS_ID==GLONASS_ID) s << ", GLONASS";
      else s << ", other GNSS";
      s << endl;        

      s.setf(ios::scientific, ios::floatfield);
      s.precision(8);
      s << "A(0GGTO):         " << A0GGTO << " sec" <<endl;
      s << "A(1GGTO):         " << A1GGTO << " sec/sec" << endl;
      s << "A(2GGTO):         " << A2GGTO << " sec/sec**2" << endl;
      
   } // end of dumpBody()   

   ostream& operator<<(ostream& s, const CNavGGTO& eph)
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
