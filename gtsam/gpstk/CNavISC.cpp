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
 * @file CNavISC.cpp
 * CNAV ISC data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "CNavISC.hpp"
#include "StringUtils.hpp"
#include "TimeString.hpp"

namespace gpstk
{
   using namespace std;

   CNavISC::CNavISC()
      :CNavDataElement(),
       Tgd(0.0), 
       ISC_L1CA(0.0),
       ISC_L2C(0.0),
       ISC_L5I5(0.0),
       ISC_L5Q5(0.0),
       avail_Tgd(false),
       avail_L1CA(false),
       avail_L2C(false),
       avail_L5I5(false),
       avail_L5Q5(false)
   {
      for (int i=0;i<4;i++)
      {
         alpha[i] = 0.0;
         beta[i]  = 0.0;
      }
   }

   CNavISC::CNavISC(const PackedNavBits& message30)
      throw( InvalidParameter)
   {
      loadData(message30);
   }

   CNavISC* CNavISC::clone() const
   {
      return new CNavISC (*this); 
   }

     // In this case, since epoch time is arbitrarily set to Xmit, 
     // the epoch time is NOT a distinguishing factor.  (This is
     // worth noting because epoch time frequently is THE 
     // distinguishing factor.) 
   bool CNavISC::isSameData(const CNavDataElement* right) const      
   {
      if (const CNavISC* rp = dynamic_cast<const CNavISC*>(right))
      {
         if (avail_Tgd  != rp->avail_Tgd)  return false;
         if (avail_L1CA != rp->avail_L1CA) return false;
         if (avail_L2C  != rp->avail_L2C)  return false;
         if (avail_L5I5 != rp->avail_L5I5) return false;
         if (avail_L5Q5 != rp->avail_L5Q5) return false;

            // Whether avail values are true or false, 
            // the actual values should match.  That is to say,
            // if avail = false, the corresponding ISC is 0.0.
         if (Tgd      != rp->Tgd)      return false;
         if (ISC_L1CA != rp->ISC_L1CA) return false;
         if (ISC_L2C  != rp->ISC_L2C)  return false;
         if (ISC_L5I5 != rp->ISC_L5I5) return false;
         if (ISC_L5I5 != rp->ISC_L5I5) return false;

         for (int i=0;i<4;i++)
         {
            if (alpha[i] != rp->alpha[i]) return false;
            if ( beta[i] != rp->beta[i]) return false;
         }
         
         return true;      
      }
      return false;
   }
   
   void CNavISC::loadData(const PackedNavBits& message30)
      throw( InvalidParameter )
   {
         // First, verify the correct message type is being passed in. 
      long msgType = message30.asUnsignedLong(14,6,1);
      if(msgType!=30)
      {
         char errStr[80];
         sprintf(errStr,"Expected CNAV MsgType 30.  Found MsgType %ld",msgType);
         std::string tstr(errStr);
         InvalidParameter exc(tstr);
         GPSTK_THROW(exc);    
      } 
      obsID     = message30.getobsID();
      satID     = message30.getsatSys();
      ctXmit    = message30.getTransmitTime();
      ctEpoch   = ctXmit;                     // For ISC, no explicit epoch time.

           // Message Type 30 data
      unsigned long testAvail = 4096;    // Pattern in message of 0x1000
                                         // if data quantity not available
                                          
      unsigned long avail = message30.asUnsignedLong(127,13,1);
      avail_Tgd = false;
      if (avail!=testAvail)
      {
         avail_Tgd = true;
         Tgd       = message30.asSignedDouble(127, 13, -35);
      }

      avail = message30.asUnsignedLong(140,13,1);
      avail_L1CA = false;
      if (avail!=testAvail)
      {
         avail_L1CA = true;
         ISC_L1CA  = message30.asSignedDouble(140, 13, -35);
      }
      
      avail = message30.asUnsignedLong(153,13,1);
      avail_L2C = false;
      if (avail!=testAvail)
      {
         avail_L2C = true;
         ISC_L2C   = message30.asSignedDouble(153, 13, -35);
      }
      
      avail = message30.asUnsignedLong(166,13,1);
      avail_L5I5 = false;
      if (avail!=testAvail)
      {
         avail_L5I5 = true;
         ISC_L5I5  = message30.asSignedDouble(166, 13, -35);
      }

      avail = message30.asUnsignedLong(179,13,1);
      avail_L5Q5 = false;
      if (avail!=testAvail)
      {
         avail_L5Q5 = true;
         ISC_L5Q5  = message30.asSignedDouble(179, 13, -35);
      }

      alpha[0] = message30.asSignedDouble(192, 8, -30);
      alpha[1] = message30.asSignedDouble(200, 8, -27);
      alpha[2] = message30.asSignedDouble(208, 8, -24);
      alpha[3] = message30.asSignedDouble(216, 8, -24);
      beta[0]  = message30.asSignedDouble(224, 8,  11);
      beta[1]  = message30.asSignedDouble(232, 8,  14);
      beta[2]  = message30.asSignedDouble(240, 8,  16);
      beta[3]  = message30.asSignedDouble(248, 8,  16);

         // Need to convert from sec/semi-circle to sec/rad
      double conversion = 1.0 / PI; 
      alpha[1] *= conversion;
      beta[1]  *= conversion;
      alpha[2] *= conversion * conversion;
      beta[2]  *= conversion * conversion;
      alpha[3] *= conversion * conversion * conversion;
      beta[3]  *= conversion * conversion * conversion;

      dataLoadedFlag = true;   
   } // end of loadData()

   void CNavISC::dumpBody(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
    
      s << endl
        << "           GROUP DELAY CORRECTIONS"
        << endl
        << "Parameter    Avail?     Value" << endl;

      s.setf(ios::scientific, ios::floatfield);
      s.precision(8);
      s.setf(ios::right, ios::adjustfield);
      s.fill(' ');

      s << "Tgd            ";
      if (avail_Tgd)
         s << "Y       " << setw(16) << Tgd << endl;
      else 
         s << "N" << endl;

      s << "ISC(L1CA)      ";
      if (avail_L1CA)
         s << "Y       " << setw(16) << ISC_L1CA << endl;
      else 
         s << "N" << endl;

      s << "ISC(L2C)       ";
      if (avail_L2C)
         s << "Y       " << setw(16) << ISC_L2C << endl;
      else 
         s << "N" << endl;

      s << "ISC(L5I5)      ";
      if (avail_L5I5)
         s << "Y       " << setw(16) << ISC_L5I5 << endl;
      else 
         s << "N" << endl;

      s << "ISC(L5Q5)      ";
      if (avail_L5Q5)
         s << "Y       " << setw(16) << ISC_L5Q5 << endl;
      else 
         s << "N" << endl;

      s << endl
        << "           IONOSPHERIC PARAMETERS"
        << endl;
      s << "  Alpha 0: " << setw(16) << alpha[0] << " sec       "
        << "   Beta 0: " << setw(16) << beta[0]  << " sec       " << endl;
      s << "  Alpha 1: " << setw(16) << alpha[1] << " sec/rad   "
        << "   Beta 1: " << setw(16) << beta[1]  << " sec/rad   " << endl;
      s << "  Alpha 2: " << setw(16) << alpha[2] << " sec/rad**2"
        << "   Beta 2: " << setw(16) << beta[2]  << " sec/rad**2" << endl;
      s << "  Alpha 3: " << setw(16) << alpha[3] << " sec/rad**3"
        << "   Beta 3: " << setw(16) << beta[3]  << " sec/rad**3" << endl;
      
   } // end of dumpBody()   

   ostream& operator<<(ostream& s, const CNavISC& eph)
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
