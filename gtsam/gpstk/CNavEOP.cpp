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
 * @file CnavEOP.cpp
 * CnavEOP data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "CNavEOP.hpp"
#include "StringUtils.hpp"
#include "TimeString.hpp"

namespace gpstk
{
   using namespace std;

   CNavEOP::CNavEOP()
      :CNavDataElement(),
       PM_X(0.0), 
       PM_X_dot(0.0),
       PM_Y(0.0),
       PM_Y_dot(0.0),
       deltaUT1(0.0),
       deltaUT1_dot(0.0)
   {
   }

   CNavEOP::CNavEOP(const PackedNavBits& message32)
      throw( InvalidParameter)
   {
      loadData(message32);
   }

   CNavEOP* CNavEOP::clone() const
   {
      return new CNavEOP (*this); 
   }

     // In this case, since epoch time is arbitrarily set to Xmit, 
     // the epoch time is NOT a distinguishing factor.  (This is
     // worth noting because epoch time frequently is THE 
     // distinguishing factor.) 
   bool CNavEOP::isSameData(const CNavDataElement* right) const      
   {
      if (const CNavEOP* rp = dynamic_cast<const CNavEOP*>(right))
      {
         if (ctEpoch      !=rp->ctEpoch)      return false;
         if (PM_X         !=rp->PM_X)         return false;
         if (PM_X_dot     !=rp->PM_X_dot)     return false;
         if (PM_Y         !=rp->PM_Y)         return false;
         if (PM_Y_dot     !=rp->PM_Y_dot)     return false;
         if (deltaUT1     !=rp->deltaUT1)     return false;
         if (deltaUT1_dot !=rp->deltaUT1_dot) return false;
            // Note: Already tested Teop (indirectly) by ctEpoch test.
         return true;      
      }
      return false;
   }
   
   void CNavEOP::loadData(const PackedNavBits& message32)
      throw(InvalidParameter)
   {
         // First, verify the correct message type is being passed in. 
      long msgType = message32.asUnsignedLong(14,6,1);
      if(msgType!=32)
      {
         char errStr[80];
         sprintf(errStr,"Expected CNAV MsgType 32.  Found MsgType %ld",msgType);
         std::string tstr(errStr);
         InvalidParameter exc(tstr);
         GPSTK_THROW(exc);    
      } 
      obsID     = message32.getobsID();
      satID     = message32.getsatSys();
      ctXmit    = message32.getTransmitTime();

      Teop         = message32.asUnsignedLong(127,16, 16);
      PM_X         = message32.asSignedDouble(143,21,-20);
      PM_X_dot     = message32.asSignedDouble(164,15,-21);
      PM_Y         = message32.asSignedDouble(179,21,-20);
      PM_Y_dot     = message32.asSignedDouble(200,15,-21);
      deltaUT1     = message32.asSignedDouble(215,31,-24);
      deltaUT1_dot = message32.asSignedDouble(246,19,-25);

         // The message does not contain a week counter.
         // We'll assume the Teop is to be within 1/2 week 
         // of the transmit time. 
      long xmitSOW = (static_cast<GPSWeekSecond>(ctXmit)).sow;
      short xmitWeek = (static_cast<GPSWeekSecond>(ctXmit)).week;
      double timeDiff = Teop - xmitSOW;
      short epochWeek = xmitWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

      ctEpoch   = GPSWeekSecond(epochWeek, Teop, TimeSystem::GPS);

      dataLoadedFlag = true;   
   } // end of loadData()

   void CNavEOP::dumpBody(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
    
      s << endl
        << "           EARTH ORIENTATION PARAMETERS"
        << endl
        << "Parameter        Value" << endl;

      s.setf(ios::scientific, ios::floatfield);
      s.precision(8); 
      s.fill(' ');

      s << "PM_X:           " << setw(16) << PM_X         << " arc-sec" << endl;
      s << "PM_X(dot):      " << setw(16) << PM_X_dot     << " arc-sec/day" << endl;
      s << "PM_X:           " << setw(16) << PM_Y         << " arc-sec" << endl;
      s << "PM_X(dot):      " << setw(16) << PM_Y_dot     << " arc-sec/day" << endl;
      s << "deltaUT1:       " << setw(16) << deltaUT1     << " sec" << endl;
      s << "deltaUIT1(dot): " << setw(16) << deltaUT1_dot << " sec/day" << endl;
      
   } // end of dumpBody()   

   ostream& operator<<(ostream& s, const CNavEOP& eph)
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
