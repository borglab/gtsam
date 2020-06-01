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
 * @file CNavUTC.cpp
 * CNAV UTC data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "CNavUTC.hpp"
#include "StringUtils.hpp"
#include "TimeString.hpp"

namespace gpstk
{
   using namespace std;

   CNavUTC::CNavUTC()
      :CNavDataElement(),
       A0(0.0), 
       A1(0.0),
       A2(0.0),
       deltaTls(0),
       Tot(0L),
       WNot(0),
       WNlsf(0),
       DN(0),
       deltaTlsf(0)
   {
   }

   CNavUTC::CNavUTC(const PackedNavBits& message33)
      throw( InvalidParameter)
   {
      loadData(message33);
   }

   CNavUTC* CNavUTC::clone() const
   {
      return new CNavUTC (*this); 
   }

   bool CNavUTC::isSameData(const CNavDataElement* right) const      
   {
      if (const CNavUTC* rp = dynamic_cast<const CNavUTC*>(right))
      {
         if (ctEpoch  !=rp->ctEpoch)   return false;
         if (A0       !=rp->A0)        return false;
         if (A1       !=rp->A1)        return false;
         if (A2       !=rp->A2)        return false;
         if (deltaTls !=rp->deltaTls)  return false;
         // Note: Already tested Tot and WNot (indirectly) by ctEpoch test.
         if (WNlsf    !=rp->WNlsf)     return false;
         if (DN       !=rp->DN)        return false;
         if (deltaTlsf!=rp->deltaTlsf) return false;
         return true;      
      }
      return false;
   }
   
   void CNavUTC::loadData(const PackedNavBits& message33)
      throw(InvalidParameter)
   {
         // First, verify the correct message type is being passed in. 
      long msgType = message33.asUnsignedLong(14,6,1);
      if(msgType!=33)
      {
         char errStr[80];
         sprintf(errStr,"Expected CNAV MsgType 33.  Found MsgType %ld",msgType);
         std::string tstr(errStr);
         InvalidParameter exc(tstr);
         GPSTK_THROW(exc);    
      } 
      obsID     = message33.getobsID();
      satID     = message33.getsatSys();
      ctXmit    = message33.getTransmitTime();
 
      A0 = message33.asSignedDouble(127,16,-35);
      A1 = message33.asSignedDouble(143,13,-51);
      A2 = message33.asSignedDouble(156, 7,-68);

      deltaTls  = message33.asLong(163, 8, 1);
      Tot       = message33.asUnsignedLong(171,16,16);
      WNot      = message33.asUnsignedLong(187,13, 1);
      WNlsf     = message33.asUnsignedLong(200,13, 1);
      DN        = message33.asUnsignedLong(213, 4, 1);
      deltaTlsf = message33.asLong(217, 8, 1);

      ctEpoch   = GPSWeekSecond(WNot, Tot, TimeSystem::GPS);

      dataLoadedFlag = true;   
   } // end of loadData()

   void CNavUTC::dumpBody(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
    
      s << endl
        << "           UTC CORRECTION PARAMETERS"
        << endl
        << "Parameter        Value" << endl;

      s.setf(ios::scientific, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(8);
      s.fill(' ');

      s << "A(0-n):         " << setw(16) << A0 << " sec" <<endl;
      s << "A(1-n):         " << setw(16) << A1 << " sec/sec" << endl;
      s << "A(2-n):         " << setw(16) << A2 << " sec/sec**2" << endl;

      s.setf(ios::fixed, ios::floatfield);    
      s.precision(0);
      s << "dT(LS):         " << setw(16) << deltaTls  << " sec" << endl;
      s << "WN(LSF):        " << setw(16) << WNlsf     << " weeks" << endl;
      s << "DN:             " << setw(16) << DN        << " days" << endl;
      s << "dT(LSF):        " << setw(16) << deltaTlsf << " sec" << endl;
      
   } // end of dumpBody()   

   ostream& operator<<(ostream& s, const CNavUTC& eph)
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
