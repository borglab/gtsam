#pragma ident "$Id$"



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






/**
 * @file EngAlmanac.cpp
 * Almanac data encapsulated in engineering terms
 */

#include "gps_constants.hpp"
#include "CommonTime.hpp"
#include "EngAlmanac.hpp"

using namespace std;
using namespace gpstk;

// This is a macro in order to retain useful location information in the exc
#define CHECK_SV_HERE(itty, prn) \
if (itty == almPRN.end()) \
{ \
   SVNotPresentException \
      exc("Attempt to get data from EngAlmanac from a SV that is not" \
          " present."); \
   GPSTK_THROW(exc); \
}

namespace gpstk
{
   EngAlmanac :: EngAlmanac()
      throw()
   {
      for (int n = 0; n < 4; n++)
      {
         alpha[n] = beta[n] = 0.0;
      }

      A0 = A1 = dt_ls = dt_lsf = 0.0;

      t_ot = t_oa = 0;

      wn_t = wn_lsf = 0;

      alm_wk = 0;

      dn = 0;

      haveUTC = false;
   }

   bool EngAlmanac::addSubframe(const long subframe[10],
                                const int gpsWeek)
      throw(InvalidParameter)
   {
      double ficked[60];

      if (!subframeConvert(subframe, gpsWeek, ficked))
         return false;

      short pat = getSubframePattern(subframe);

         // check tlm preamble, subframe id and format #
      if ((ficked[0] != 0x8b) || ((ficked[4] != 4) && (ficked [4] != 5))
          || (pat < 4) || (pat > 10))
      {
         InvalidParameter
            exc("EngAlmanac::addSubframe: Not a valid almanac page.");
         GPSTK_THROW(exc);
      }

      short svid = (subframe[3] >> 22) & 0x3F;
      short sfid = (subframe[2] >> 8) & 0x7;
      long tow = ((subframe[2] >> 13) & 0x1ffff) * 6;
#pragma unused(svid,sfid,tow)
       
      switch(pat)
      {
         case 4:
               /* Page with Orbital Elements */
               /*check PRN */
            if ((ficked[19] < 0) || (ficked[19] > MAX_PRN_GPS))
            {
               InvalidParameter exc("EngAlmanac::addSubframe, PRN out of range "
                                    + StringUtils::asString(ficked[19]));
               GPSTK_THROW(exc);
            }
            {
               int prn = static_cast<short>( ficked[19] );
               if (prn) {
                  SatID sat(prn,SatID::systemGPS);
                  almPRN[sat] = AlmOrbit(prn, ficked[7], ficked[9], ficked[10],
                                      ficked[12], ficked[13], ficked[14],
                                         ficked[15], ficked[16], ficked[17],
                                         static_cast<long>( ficked[8] ),
                                         static_cast<long>( ficked[2] ), gpsWeek,
                                         static_cast<short>( ficked[11] ));
               }
            }
            break;

         case 5:  /*  Page with Satellite health information 1-24 */
            for (int i=1; i <=24; i++)
               health[i] = static_cast<char>( ficked[7 + i] );
               // manually crack the t_oa and WNa
            t_oa = ((subframe[2] >> 14) & 0xFF) * 4096;
            convert8bit(gpsWeek, &ficked[7]);
            alm_wk = static_cast<int>(ficked[7]);
            break;

         case 6:
         case 7:
               /* ignore page becase no data for our structure */
            return true;

         case 8: /* Page with UTC and ionosphere parameters */
            alpha[0] = ficked[7];
            alpha[1] = ficked[8];
            alpha[2] = ficked[9];
            alpha[3] = ficked[10];
            beta[0] = ficked[11];
            beta[1] = ficked[12];
            beta[2] = ficked[13];
            beta[3] = ficked[14];
            A0 = ficked[15];
            A1 = ficked[16];
            dt_ls = ficked[19];
            t_ot = static_cast<long>( ficked[17] );
            wn_t = static_cast<int>( ficked[18] );
            wn_lsf = static_cast<int>( ficked[20] );
            dn = static_cast<char>( ficked[21] );
            dt_lsf = ficked[22];
            haveUTC = true;
            break;

         case 9:  /* Page with Health for 25-32 and AS/SV config */
            for (int i=1; i<=MAX_PRN_GPS; i++)
               SV_config[i] = static_cast<char>( ficked[6 + i] );

            for (int i=25; i<=MAX_PRN_GPS; i++)
               health[i] = static_cast<char>( ficked[14 + i] );
            break;

         case 10:/* Page with Special Message */
            special_msg = "";
            for (int i=0; i<22; i++)
               special_msg += static_cast<char>( ficked[7 + i] );
            break;

         default:
               // never reached, see if statement prior to this switch
            break;
      }
      return true;
   }

   double EngAlmanac::getEcc(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of the ecc for the given PRN
      return (*i).second.ecc;
   }

   double EngAlmanac::getIOffset(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of the iOffset for the given PRN
      return (*i).second.i_offset;
   }

   double EngAlmanac::getOmegadot(SatID sat) const
      throw(SVNotPresentException)
   {

      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of OMEGAdot for the given PRN
      return (*i).second.OMEGAdot;
   }

   short EngAlmanac::get6bitHealth(SatID sat) const
      throw(SVNotPresentException)
   {
      SVBitsMap::const_iterator i = health.find(sat.id);
      if (i == health.end())
      {
         SVNotPresentException svnpe("SV health not present for PRN " +
                                     StringUtils::asString(sat.id));
         GPSTK_THROW(svnpe);
      }

      return i->second;
   }

   short EngAlmanac::getSVHealth(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of SV_health for the given PRN
      return (*i).second.SV_health;
   }

   short EngAlmanac::getSVConfig(SatID sat) const
      throw(SVNotPresentException)
   {
      SVBitsMap::const_iterator i = SV_config.find(sat.id);
      if (i == SV_config.end())
      {
         SVNotPresentException svnpe("SV Configuration not present for PRN " +
                                     StringUtils::asString(sat.id));
         GPSTK_THROW(svnpe);
      }

      return i->second;
   }

   double EngAlmanac::getAhalf(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of Ahalf for the given PRN
      return (*i).second.Ahalf;
   }

   double EngAlmanac::getA(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of A for the given PRN
      return (*i).second.Ahalf * (*i).second.Ahalf;
   }

   double EngAlmanac::getOmega0(SatID sat) const
      throw(SVNotPresentException)
   {

      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of OMEGA0 for the given PRN
      return (*i).second.OMEGA0;
   }

   double EngAlmanac::getW(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of w for the given PRN
      return (*i).second.w;
   }

   double EngAlmanac::getM0(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of M0 for the given PRN
      return (*i).second.M0;
   }

   double EngAlmanac::getAf0(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of the af0 for the given PRN
      return (*i).second.AF0;
   }


   double EngAlmanac::getAf1(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of af1 for the given PRN
      return (*i).second.AF1;
   }


   double EngAlmanac::getToa() const throw()
   {
      return static_cast<double>( t_oa );
   }

   double EngAlmanac::getToa(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of the Toa for the given PRN
      return static_cast<double>( (*i).second.Toa );
   }


   double EngAlmanac::getXmitTime(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of the xmit_time for the given PRN
      return static_cast<double>( (*i).second.xmit_time );
   }


   short EngAlmanac::getFullWeek(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

      return (*i).second.getFullWeek();
   }

   void EngAlmanac::getIon(double a[4], double b[4]) const
      throw(InvalidRequest)
   {
      if (!haveUTC)
      {
         InvalidRequest
            exc("UTC offset (subframe 4, page 18) is not present.");
         GPSTK_THROW(exc);
      }
      for (int n = 0; n < 4; n++)
      {
         a[n] = alpha[n];
         b[n] = beta[n];
      }
   }

   void EngAlmanac::getUTC(double& a0, double& a1, double& deltaTLS,
                           long& tot, int& WNt, int& WNLSF,
                           int& DN, double& deltaTLSF) const
      throw(InvalidRequest)
   {
      if (!haveUTC)
      {
         InvalidRequest
            exc("UTC offset (subframe 4, page 18) is not present.");
         GPSTK_THROW(exc);
      }
      a0 = A0;
      a1 = A1;
      deltaTLS = dt_ls;
      tot = t_ot;
      WNt = wn_t;
      WNLSF = wn_lsf;
      DN = static_cast<int>( dn );
      deltaTLSF = dt_lsf;
   }

   short EngAlmanac::getAlmWeek() const throw()
   {
      return alm_wk;
   }

   AlmOrbit EngAlmanac::getAlmOrbElem(SatID sat) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of the orbit elm. for the given PRN
      return (*i).second;
   }

   Xvt EngAlmanac::svXvt(SatID sat, const CommonTime& t) const
      throw(SVNotPresentException)
   {
      AlmOrbits::const_iterator i = almPRN.find(sat);
      CHECK_SV_HERE(i, sat);

         // return value of the orbit elm. for the given PRN
      return (*i).second.svXvt(t);
   }

   bool EngAlmanac::isData(SatID sat) const throw()
   {
      return (almPRN.find(sat) != almPRN.end());
   }

   string int2bin(unsigned int v, int len=8)
   {
      string s;
      for (int i = 0; i < len; i++)
      {
         if (v & 1)
            s = "1" + s;
         else
            s = "0" + s;
         v = v >> 1;
      }
      return s;
   }


   bool EngAlmanac::check(ostream& s) const
   {
      bool good = false;

      if (!haveUTC)
         s << "UTC offset (subframe 4, page 18) is not present." << endl;

      double p51Toa=getToa();
      for (int prn=1; prn<=32; prn++)
      {
         try
         {
            double svToa = getToa(gpstk::SatID(prn, SatID::systemGPS));
            if (svToa != p51Toa)
            {
               s << "Toa mis-match on prn " << prn
                 << "  page 51 Toa=" << p51Toa
                 << ", SV Toa=" << svToa << endl;
               good = false;
            }
         }
         catch (SVNotPresentException& e)
         {
            cout << "No page for prn " << prn << endl;
         }
      }
      return good;
   }


   void EngAlmanac::dump(ostream& s, bool checkFlag) const
   {
      ios::fmtflags oldFlags = s.flags();

      s.fill(' ');

      s << "****************************************************************"
        << "***************" << endl
        << "Broadcast Almanac (Engineering Units)" << endl
        << endl;

      s << endl << "           Iono Parameters" << endl << endl;
      s << "Alpha:    " << scientific << setprecision(6);
      for (int i=0; i<4; i++)
         s << setw(13) << alpha[i] << "  ";
      s << " various" << endl;
      s << " Beta:    " << fixed << setprecision(1);
      for (int i=0; i<4; i++)
         s << setw(13) << beta[i] << "  ";
      s << " various" << endl;

      s << endl << "           UTC Paramters" << endl << endl;
      s << scientific << setprecision(8)
        << "A0:       " << setw(15) << A0      << " sec" << endl
        << "A1:       " << setw(15) << A1      << " sec/sec" << endl
        << fixed << setprecision(1)
        << "dt_ls:    " << setw(15) << dt_ls   << " sec" << endl
        << "t_ot:     " << setw(15) << t_ot    << " sec" << endl
        << "wn_t:     " << setw(15) << wn_t    << " week" << endl
        << "wn_lsf    " << setw(15) << wn_lsf  << " week" << endl
        << "dn:       " << setw(15) << (int)dn << " days" << endl
        << "dt_lsf:   " << setw(15) << dt_lsf  << " sec" << endl;

      s << endl << "           Orbit Parameters" << endl << endl;
      for (AlmOrbits::const_iterator i = almPRN.begin(); i != almPRN.end(); i++)
         s<< scientific << (*i).second;

      s << endl << "           Special Message" << endl << endl;
      StringUtils::hexDumpData(s, special_msg);


      s << endl << "           Page 25 Health, AS, & SV config" << endl << endl;

      s << "Toa:    " << setfill(' ') << setw(8) << t_oa
        << ", week: " << setw(5) << alm_wk << endl << endl
        << "PRN   health  AS  cfg    PRN   health  AS  cfg" << endl;
      string bits[33];

      for (SVBitsMap::const_iterator i = health.begin(); i != health.end(); i++)
      {
         int prn = i->first;
         if (prn >= 1 && prn <= 32)
            bits[prn] = int2bin(i->second, 6);
      }

      for (SVBitsMap::const_iterator i = SV_config.begin(); i != SV_config.end(); i++)
      {
         int prn = i->first;
         if (prn >= 1 && prn <= 32)
         {
            bits[prn] += "  " + int2bin(i->second, 4);
            bits[prn].insert(9, "   ");
         }
      }

      for (int i=1; i<=16; i++)
         s << setw(2) << i    << "    " << bits[i] << "    "
           << setw(2) << i+16 << "    " << bits[i+16] << endl;

      s << endl;

      if (checkFlag)
         check(s);

      s << endl;

      s.flags(oldFlags);
   } // end of dump()

   std::ostream& operator<<(std::ostream& s, const EngAlmanac& alm)
   {
      alm.dump(s);
      return s;
   }

} // namespace
