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
 * @file EngEphemeris.cpp
 * Ephemeris data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "MathBase.hpp"
#include "GPSEllipsoid.hpp"
#include "EngEphemeris.hpp"
#include "GPSWeekSecond.hpp"
#include "YDSTime.hpp"
#include "CivilTime.hpp"
#include "TimeSystem.hpp"
#include "GPS_URA.hpp"
#include "TimeString.hpp"
#include "SVNumXRef.hpp"

namespace gpstk
{
   using namespace std;

   EngEphemeris::EngEphemeris()
      throw()
   {
      haveSubframe[0] = haveSubframe[1] = haveSubframe[2] = false;

      tlm_message[0] = tlm_message[1] = tlm_message[2] = 0;

      satSys = "";

      PRNID = tracker = ASalert[0] = ASalert[1] = ASalert[2] = weeknum =
	   codeflags = health = L2Pdata = 0;

      HOWtime[0] = HOWtime[1] = HOWtime[2] = 0;

      IODC = IODE = 0;
      Tgd = 0.0;

      isFIC = true;

      fitint = 0;

      for (int j = 0; j<3; j++)
      {
         for (int i = 0; i<10; i++) subframeStore[j][i] = 0L;
      }
   }

   /**
   *  Historically, EngEphemeris allowed calling programs to add data
   *  one subframe at a time.  This functionality does not exist in
   *  KeplerOrbit and BroadcastClockCorrection.  Therefore, EngEphemeris
   *  must handle the subframe collection overhead before calling
   *  BrcKeplerOrbit.loadData() and BrcClockCorrection.loadData().
   *
   *  Note: Historically, the gpsWeek in the calling parameters to
   *  addSubframe is the full week number associated with the TRANSMIT
   *  time (not the epoch times).
   */

   bool EngEphemeris::addSubframe(const long subframe[10], const int gpsWeek,
                                  const short PRN, const short track)
      throw( InvalidParameter )
   {
         // Determine the subframe number
      unsigned long SFword2 = (unsigned long) subframe[1];
      SFword2 &= 0x00000700;      // Strip all but three bit subframe ID
      SFword2 >>= 8;              // Right shift to move to lsbs;
      short sfID = static_cast<short>( SFword2 );

      if (sfID<1 || sfID>3)
      {
         InvalidParameter exc("Invalid SF ID: "+StringUtils::asString(sfID));
         GPSTK_THROW(exc);
      }

         // Store the subframe in the appropriate location
         // and set the flag
      int sfNdx = sfID - 1;
      for (int i=0;i<10;++i) subframeStore[sfNdx][i] = subframe[i];
      haveSubframe[sfNdx] = true;

         // Determine if all subframes are available.  If so,
         // load the data. Otherwise return and try again later.
      bool result = true;  // default return OK in cases where no cracking occurs
      if (haveSubframe[0] &&
          haveSubframe[1] &&
          haveSubframe[2])
      {
         result = unifiedConvert( gpsWeek, PRN, track );
      }
      return(result);
   }

   bool EngEphemeris::addSubframeNoParity(const long subframe[10],
                                          const int  gpsWeek,
                                          const short PRN,
                                          const short track)
      throw( InvalidParameter )
   {
      long paddedSF[10];
      short PRNArg;
      short trackArg;

      for (int i=0;i<10;++i)
      {
         paddedSF[i] = subframe[i];
         paddedSF[i] <<= 6;
         paddedSF[i] &= 0x3FFFFFC0;    // Guarantee 2 msb and 6 lsb are zeroes
      }
      PRNArg = PRN;
      trackArg = track;
      return( addSubframe( paddedSF, gpsWeek, PRNArg, trackArg ));
   }

   bool EngEphemeris::addIncompleteSF1Thru3(
      const long sf1[8], const long sf2[8], const long sf3[8],
      const long sf1TransmitSOW, const int gpsWeek,
      const short PRN, const short track)
   {
         // Need to provide a valid subframe number in the handover word.
         // While we're at it, we'll fake the A-S bit such that it
         // appears A-S is ON, even though we warn the user NOT to trust
         // returns from the getASAlert() method.
      const long sf1Lead[2] = { 0x00000000, 0x00000900 };
      const long sf2Lead[2] = { 0x00000000, 0x00000A00 };
      const long sf3Lead[2] = { 0x00000000, 0x00000B00 };

         // Handover word times represent the time of the leading edge of the
         // NEXvt subframe.  Therefore, HOW should always correspond to
         //   :06/:36 for SF 1
         //   :12/:42 for SF 2
         //   :18/:48 for SF 3
         // This method hasn't a clue about the accuracy of the SOW input by the
         // user, but it WILL enforce this relationship.
      long frameCount = sf1TransmitSOW / 30;
      long SF1HOWTime = (frameCount * 30) + 6;
#pragma unused(SF1HOWTime)
         // Convert subframe 1 parameters
      subframeStore[0][0] = sf1Lead[0];
      subframeStore[0][1] = sf1Lead[1];
      int i;
      for (i=0; i<8; ++i) subframeStore[0][i+2] = sf1[i];
      haveSubframe[0] = true;

         // Convert subframe 2 parameters
      subframeStore[1][0] = sf2Lead[0];
      subframeStore[1][1] = sf2Lead[1];
      for (i=0; i<8; ++i) subframeStore[1][i+2] = sf2[i];
      haveSubframe[1] = true;

         // Convert subframe 3 parameters
      subframeStore[2][0] = sf3Lead[0];
      subframeStore[2][1] = sf3Lead[1];
      for (i=0; i<8; ++i) subframeStore[2][i+2] = sf3[i];
      haveSubframe[2] = true;

         // Call method to crack and load the data.
      bool result = unifiedConvert( gpsWeek, PRN, track );

      return(result);
   }

      /**
      * Each of the addSubframe( ) methods eventually calls unifiedConvert( )
      * in order to crack the raw subframe data into engineering units and
      * load the orbit and clock objects.
      */
   bool EngEphemeris::unifiedConvert( const int gpsWeek,
                                      const short PRN,
                                      const short track)
   {
      double ficked[60];

      if (!subframeConvert(subframeStore[0], gpsWeek, ficked))
         return false;
      tlm_message[0] = (subframeStore[0][0] >> 8) & 0x3fff;
      HOWtime[0] = static_cast<long>( ficked[2] );
      ASalert[0] = static_cast<short>( ficked[3] );
      weeknum    = static_cast<short>( ficked[5] );
      codeflags  = static_cast<short>( ficked[6] );
      short accFlag = static_cast<short>( ficked[7] );
      health     = static_cast<short>( ficked[8] );
      IODC       = static_cast<short>( ldexp( ficked[9], -11 ) );
      L2Pdata    = static_cast<short>( ficked[10] );
      Tgd        = ficked[11];
      double Toc = ficked[12];
      double af2 = ficked[13];
      double af1 = ficked[14];
      double af0 = ficked[15];
      tracker    = track;

      if (!subframeConvert(subframeStore[1], gpsWeek, ficked))
         return false;

      tlm_message[1] = (subframeStore[1][0] >> 8) & 0x3fff;
      HOWtime[1]     = static_cast<long>( ficked[2] );
      ASalert[1]     = static_cast<short>( ficked[3] );
      IODE           = static_cast<short>( ldexp( ficked[5], -11 ) );
      double Crs     = ficked[6];
      double dn      = ficked[7];
      double M0      = ficked[8];
      double Cuc     = ficked[9];
      double ecc     = ficked[10];
      double Cus     = ficked[11];
      double Ahalf   = ficked[12];
      double Toe     = ficked[13];
      fitint         = static_cast<short>( ficked[14] );
      AODO           = static_cast<long>( ficked[15] );


      if (!subframeConvert(subframeStore[2], gpsWeek, ficked))
         return false;

      tlm_message[2]   = (subframeStore[2][0] >> 8) & 0x3fff;
      HOWtime[2]       = static_cast<long>( ficked[2] );
      ASalert[2]       = static_cast<short>( ficked[3] );
      double Cic       = ficked[5];
      double OMEGA0    = ficked[6];
      double Cis       = ficked[7];
      double i0        = ficked[8];
      double Crc       = ficked[9];
      double w         = ficked[10];
      double OMEGAdot  = ficked[11];
      double idot      = ficked[13];

         // The system is assumed (legacy navigation message is from GPS)
      satSys = "G";
      PRNID = PRN;

         // The observation ID has a type of navigation, but the
         // carrier and code types are undefined.  They could be
         // L1/L2 C/A, P, Y,.....
      ObsID obsID(ObsID::otNavMsg, ObsID::cbUndefined, ObsID::tcUndefined);

      bool healthy = false;
      if (health==0) healthy = true;
      double Adot = 0.0;
      double dnDot = 0.0;
      double A = Ahalf * Ahalf;

      double timeDiff = Toe - HOWtime[1];
      short epochWeek = weeknum;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

         // Toe is now in CommonTime, and needs to be passed to BrcKeplerOrbit as a CommonTime variable.
         // URAoc and URAoe in legacy nav message are equal. Only CNAV and CNAV2 are they different
      CommonTime ToeCT = GPSWeekSecond(epochWeek, Toe, TimeSystem::GPS);
      CommonTime TocCT = GPSWeekSecond(epochWeek, Toc, TimeSystem::GPS);

      short fiti = static_cast<short>(ficked[14]);
#pragma unused(fiti)
      short fitHours = getLegacyFitInterval(IODC, fitint);
      long beginFitSOW = Toe - (fitHours/2)*3600;
      long endFitSOW = Toe + (fitHours/2)*3600;
      short beginFitWk = epochWeek;
      short endFitWk = epochWeek;
      if (beginFitSOW < 0)
      {
         beginFitSOW += FULLWEEK;
         beginFitWk--;
      }
      CommonTime beginFit = GPSWeekSecond(beginFitWk, beginFitSOW, TimeSystem::GPS);

      if (endFitSOW >= FULLWEEK)
      {
         endFitSOW -= FULLWEEK;
         endFitWk++;
      }
      CommonTime endFit = GPSWeekSecond(endFitWk, endFitSOW, TimeSystem::GPS);

      orbit.loadData(satSys, obsID, PRN, beginFit, endFit, ToeCT,
                     accFlag, healthy, Cuc, Cus, Crc, Crs, Cic, Cis, M0,
                     dn, dnDot, ecc, A, Ahalf, Adot, OMEGA0, i0, w, OMEGAdot, idot);

      bcClock.loadData( satSys, obsID, PRNID, TocCT,
                        accFlag, healthy, af0, af1, af2);

	  return true;
   }

   bool EngEphemeris::isData(short subframe) const
      throw( InvalidRequest )
   {
      if ((subframe < 1) || (subframe > 3))
      {
         InvalidRequest exc("Subframe "+StringUtils::asString(subframe)+
                            " is not a valid ephemeris subframe.");
         GPSTK_THROW(exc);
      }

      return haveSubframe[subframe-1];
   }

   void EngEphemeris::setAccuracy(const double& acc)
      throw( InvalidParameter )
   {
      if( acc < 0 )
      {
         InvalidParameter exc("SV Accuracy of " + StringUtils::asString(acc) +
                              " meters is invalid.");
         GPSTK_THROW(exc);
      }
      orbit.setAccuracy(acc);
   }

      /**
      *This is for Block II/IIA
      *Need update for Block IIR and IIF
      */
   short EngEphemeris :: getFitInterval() const
      throw( InvalidRequest )
   {
      short iodc = getIODC();
      short fiti = getFitInt();

         /* check the IODC */
      if (iodc < 0 || iodc > 1023)
      {
            /* error in iodc, return minimum fit */
         return 4;
      }

      if ((((fiti == 0) &&
           (iodc & 0xFF) < 240) || (iodc & 0xFF) > 255 ))
      {
            /* fit interval of 4 hours */
         return 4;
      }
      else if (fiti == 1)
      {
         if( ((iodc & 0xFF) < 240 || (iodc & 0xFF) > 255))
         {
               /* fit interval of 6 hours */
            return 6;
         }
         else if(iodc >=240 && iodc <=247)
         {
               /* fit interval of 8 hours */
            return 8;
         }
         else if((iodc >= 248 && iodc <= 255) || iodc == 496)
         {
               /* fit interval of 14 hours */
            return 14;
         }
         // Revised in IS-GPS-200 Revision D for Block IIR/IIR-M/IIF
         else if((iodc >= 497 && iodc <=503) || (iodc >= 1021 && iodc <= 1023))
         {
               /* fit interval of 26 hours */
            return 26;
         }
         else if(iodc >= 504 && iodc <=510)
         {
               /* fit interval of 50 hours */
            return 50;
         }
         else if(iodc == 511 || (iodc >= 752 && iodc <= 756))
         {
               /* fit interval of 74 hours */
            return 74;
         }
         /* NOTE:
          * The following represents old fit intervals for Block II (not IIA)
          * and is present only in old versions of the ICD-GPS-200 Rev. C.
          * Please do not remove them as there are still people that may
          * want to process old Block II data and none of the IODC intervals
          * overlap (so far) so there is no need to remove them.
          */
         else if(iodc >= 757 && iodc <= 763)
         {
               /* fit interval of 98 hours */
            return 98;
         }
         else if((iodc >= 764 && iodc <=767) || (iodc >=1008 && iodc <=1010))
         {
               /* fit interval of 122 hours */
            return 122;
         }
         else if(iodc >= 1011 && iodc <=1020)
         {
               /* fit interval of 146 hours */
            return 146;
         }
         else
         {
               /* error in the iodc or ephemeris, return minimum
                  fit */
            return 4;
         }
      }
      else
      {
            /* error in ephemeris/iodc, return minimum fit */
         return 4;
      }

      return 0; // never reached
   }


   Xvt EngEphemeris::svXvt(const CommonTime& t) const
      throw( InvalidRequest )
   {
      Xvt sv;

      Xv xv = orbit.svXv(t);

      sv.x = xv.x;
      sv.v = xv.v;

      sv.clkbias = bcClock.svClockBias(t);
      sv.relcorr = orbit.svRelativity(t);

      sv.clkdrift = bcClock.svClockDrift(t);

      return sv;
   }

   double EngEphemeris::svRelativity(const CommonTime& t) const
      throw( InvalidRequest )
   {
      return orbit.svRelativity(t);
   }

   double EngEphemeris::svClockBias(const CommonTime& t) const
      throw( InvalidRequest )
   {
      return bcClock.svClockBias(t);
   }

   double EngEphemeris::svClockDrift(const CommonTime& t) const
      throw( InvalidRequest )
   {
      return bcClock.svClockDrift(t);
   }

   unsigned EngEphemeris::getTLMMessage(short subframe) const
      throw( InvalidRequest )
   {
      if (!haveSubframe[subframe-1])
      {
         InvalidRequest exc("Subframe "+StringUtils::asString(subframe)+
                            " not stored.");
         GPSTK_THROW(exc);
      }
      return tlm_message[subframe-1];
   }

   CommonTime EngEphemeris::getTransmitTime() const
      throw()
   {
      CommonTime toReturn;
      toReturn = GPSWeekSecond(getFullWeek(), static_cast<double>(getTot()), TimeSystem::GPS);
      return toReturn;
   }

   CommonTime EngEphemeris::getEpochTime() const
      throw( InvalidRequest )
   {
      return bcClock.getEpochTime();
   }

   CommonTime EngEphemeris::getEphemerisEpoch() const
      throw( InvalidRequest )
   {
      return orbit.getOrbitEpoch();
   }

   BrcKeplerOrbit EngEphemeris::getOrbit() const
      throw(InvalidRequest )
   {
      if(!orbit.hasData())
      {
         InvalidRequest exc("getOrbit(): Required Orbit data not stored.");
         GPSTK_THROW(exc);
      }
      return (orbit);
   }

   BrcClockCorrection EngEphemeris::getClock() const
      throw(InvalidRequest )
   {
      if(!bcClock.hasData())
      {
         InvalidRequest exc("getClock(): Required Clock Correction data not stored.");
         GPSTK_THROW(exc);
      }
      return (bcClock);
   }

   short EngEphemeris::getPRNID() const
      throw( InvalidRequest )
   {
      if(!haveSubframe[0])
      {
         InvalidRequest exc("getPRNID(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return PRNID;
   }

   short EngEphemeris::getTracker() const
      throw( InvalidRequest )
   {
      if(!haveSubframe[0])
      {
         InvalidRequest exc("getTracker(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return tracker;
   }

   double EngEphemeris::getHOWTime(short subframe) const
      throw( InvalidRequest )
   {
      if (!haveSubframe[subframe-1])
      {
         InvalidRequest exc("getHOWTime(): Subframe "
                            +StringUtils::asString(subframe)+" not stored.");
         GPSTK_THROW(exc);
      }
         // This return as a double is necessary for sets into CommonTime
         // to not get confused.  Ints are Zcounts whereas doubles are seconds.
         // This should still return a double after CommonTime->CommonTime
         // conversion, for backwards compatibility. [DR]
      return static_cast<double>(HOWtime[subframe-1]);
   }

   short EngEphemeris::getASAlert(short subframe)  const
      throw( InvalidRequest )
   {
      if (!haveSubframe[subframe-1])
      {
         InvalidRequest exc("getASAlert(): Subframe "
                            +StringUtils::asString(subframe)+" not stored.");
         GPSTK_THROW(exc);
      }
      return ASalert[subframe-1];
   }

   short EngEphemeris::getFullWeek()  const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getFullWeek(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return weeknum;
   }

   short EngEphemeris::getCodeFlags()  const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getCodeFlags(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return codeflags;
   }

   double EngEphemeris::getAccuracy()  const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getAccuracy(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getAccuracy();
   }

   short EngEphemeris::getAccFlag()  const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getAccFlag(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getURAoe();
   }

   short EngEphemeris::getHealth() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getHealth(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return health;
   }

   short EngEphemeris::getL2Pdata() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getL2Pdata(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return L2Pdata;
   }

   short EngEphemeris::getIODC() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getIODC(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return static_cast<short>(IODC);
   }

   short EngEphemeris::getIODE() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getIODE(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return static_cast<short>(IODE);
   }

   long EngEphemeris::getAODO() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getAODO(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return AODO;
   }

   double EngEphemeris::getToc() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getToc(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.getToc();
   }

   double EngEphemeris::getAf0() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getAf0(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.getAf0();
   }

   double EngEphemeris::getAf1() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getAf1(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.getAf1();
   }

   double EngEphemeris::getAf2() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getAf1(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.getAf2();
   }

   double EngEphemeris::getTgd() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[0])
      {
         InvalidRequest exc("getTgd(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      return Tgd;
   }

   double EngEphemeris::getCus() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getCus(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getCus();
   }

   double EngEphemeris::getCrs() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getCrs(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getCrs();
   }

   double EngEphemeris::getCis() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getCis(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getCis();
   }

   double EngEphemeris::getCrc() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getCrc(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getCrc();
   }

   double EngEphemeris::getCuc() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getCuc(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getCuc();
   }

   double EngEphemeris::getCic() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getCic(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getCic();
   }

   double EngEphemeris::getToe() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getToe(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getToe();
   }

   double EngEphemeris::getM0() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getM0(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getM0();
   }

   double EngEphemeris::getDn() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getDn(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getDn();
   }

   double EngEphemeris::getEcc() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getEcc(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getEcc();
   }

   double EngEphemeris::getAhalf() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getAhalf(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getAhalf();
   }

   double EngEphemeris::getA() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getA(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getA();
   }

   double EngEphemeris::getOmega0() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getOmega0(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getOmega0();
   }

   double EngEphemeris::getI0() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getI0(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getI0();
   }

   double EngEphemeris::getW() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getW(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getW();
   }

   double EngEphemeris::getOmegaDot() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getOmegaDot(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getOmegaDot();
   }

   double EngEphemeris::getIDot() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[2])
      {
         InvalidRequest exc("getIDot(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getIDot();
   }

   short EngEphemeris::getFitInt() const
      throw( InvalidRequest )
   {
      if (!haveSubframe[1])
      {
         InvalidRequest exc("getFitInt(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      return fitint;
   }

   long EngEphemeris::getTot() const
      throw( InvalidRequest )
   {
      if(!haveSubframe[0])
      {
         InvalidRequest exc("getTot(): Required subframe 1 not stored.");
         GPSTK_THROW(exc);
      }
      if(!haveSubframe[1])
      {
         InvalidRequest exc("getTot(): Required subframe 2 not stored.");
         GPSTK_THROW(exc);
      }
      if(!haveSubframe[2])
      {
         InvalidRequest exc("getTot(): Required subframe 3 not stored.");
         GPSTK_THROW(exc);
      }

      // MSVC
#ifdef _MSC_VER
      long foo = static_cast<long>( getHOWTime(1) < getHOWTime(2) ) ? getHOWTime(1) : getHOWTime(2);
      foo = ( foo < getHOWTime(3) ) ? foo : getHOWTime(3) ;
#else
      long foo =
         static_cast<long>( std::min( getHOWTime(1),
                            std::min( getHOWTime(2), getHOWTime(3) ) ) );
#endif
         // The ephemeris comes on 30 second boundaries, so...
      foo/=30;
      foo*=30;
      return foo;
   }

   EngEphemeris& EngEphemeris::loadData( const std::string satSysArg, unsigned short tlm[3],
                                         const long how[3], const short asalert[3],
                                         const short Tracker, const short prn,
                                         const short fullweek, const short cflags, const short acc,
                                         const short svhealth, const short iodc, const short l2pdata,
                                         const long aodo, const double tgd, const double toc,
                                         const double Af2, const double Af1, const double Af0,
                                         const short iode, const double crs, const double Dn,
                                         const double m0, const double cuc, const double Ecc,
                                         const double cus, const double ahalf, const double toe,
                                         const short fitInt, const double cic, const double Omega0,
                                         const double cis, const double I0, const double crc,
                                         const double W, const double OmegaDot, const double IDot )
      throw()
   {
      PRNID = prn;
      tracker = Tracker;
      for (int i=0; i<3; i++)
      {
         tlm_message[i] = tlm[i];
         HOWtime[i] = how[i];
         ASalert[i] = asalert[i];
      }
      weeknum   = fullweek;
      codeflags = cflags;
      short accFlag = acc;
      double accuracy = gpstk::ura2accuracy(accFlag);
#pragma unused(accuracy)
       health    = svhealth;
      L2Pdata   = l2pdata;
      IODC      = iodc;
      IODE      = iode;
      AODO      = aodo;
      fitint    = fitInt;
      Tgd       = tgd;
      satSys = satSysArg;

      satSys = "G";

         // The observation ID has a type of navigation, but the
         // carrier and code types are undefined.  They could be
         // L1/L2 C/A, P, Y,.....
      ObsID obsID(ObsID::otNavMsg, ObsID::cbUndefined, ObsID::tcUndefined);

      CommonTime toeCT = GPSWeekSecond(weeknum, toe, TimeSystem::GPS);
      CommonTime tocCT = GPSWeekSecond(weeknum, toc, TimeSystem::GPS);

      double A = ahalf*ahalf;
      double dndot = 0.0;
      double Adot = 0.0;
      short fitHours = getLegacyFitInterval(IODC, fitint);
      long beginFitSOW = toe - (fitHours/2)*3600;
      long endFitSOW = toe + (fitHours/2)*3600;
      short beginFitWk = weeknum;
      short endFitWk = weeknum;
      if (beginFitSOW < 0)
      {
         beginFitSOW += FULLWEEK;
         beginFitWk--;
      }
      CommonTime beginFit = GPSWeekSecond(beginFitWk, beginFitSOW, TimeSystem::GPS);
      if (endFitSOW >= FULLWEEK)
      {
         endFitSOW += FULLWEEK;
         endFitWk++;
      }
      CommonTime endFit = GPSWeekSecond(endFitWk, endFitSOW, TimeSystem::GPS);

      orbit.loadData(satSys, obsID, PRNID, beginFit, endFit, toeCT,
                     accFlag, health, cuc, cus, crc, crs, cic, cis, m0, Dn,
                     dndot, Ecc, A, ahalf, Adot, Omega0, I0, W, OmegaDot, IDot);

      bcClock.loadData( satSys, obsID, PRNID, tocCT,
                        accFlag, health, Af0, Af1, Af2);
      haveSubframe[0] = true;
      haveSubframe[1] = true;
      haveSubframe[2] = true;
      return *this;
   }

   EngEphemeris& EngEphemeris::setSF1( unsigned tlm, double how, short asalert,
                                       short fullweek, short cflags, short acc,
                                       short svhealth, short iodc, short l2pdata,
                                       double tgd, double toc, double Af2,
                                       double Af1, double Af0, short Tracker,
                                       short prn )
      throw()
   {

      tlm_message[0] = tlm;
      HOWtime[0] = static_cast<long>( how );
      ASalert[0] = asalert;
      weeknum    = fullweek;
      codeflags  = cflags;
      accFlagTmp = acc;
      health     = svhealth;
      IODC       = iodc;
      L2Pdata    = l2pdata;
      Tgd        = tgd;
      tracker    = Tracker;
      PRNID      = prn;
      bool healthy = false;
      if (health == 0) healthy = true;

      double timeDiff = toc - HOWtime[0];
      short epochWeek = fullweek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

      CommonTime tocCT = GPSWeekSecond(epochWeek, toc, TimeSystem::GPS);

         // The system is assumed (legacy navigation message is from GPS)
      satSys = "G";

         // The observation ID has a type of navigation, but the
         // carrier and code types are undefined.  They could be
         // L1/L2 C/A, P, Y,.....
      ObsID obsID(ObsID::otNavMsg, ObsID::cbUndefined, ObsID::tcUndefined);

      bcClock.loadData( satSys, obsID, PRNID, tocCT,
                        accFlagTmp, healthy, Af0, Af1, Af2);
      haveSubframe[0] = true;
      return *this;
   }

   EngEphemeris& EngEphemeris::setSF2( unsigned tlm, double how, short asalert,
                                       short iode, double crs, double Dn,
                                       double m0, double cuc, double Ecc,
                                       double cus, double ahalf, double toe,
                                       short fitInt )
      throw( InvalidRequest )
   {
      tlm_message[1] = tlm;
      HOWtime[1] = static_cast<long>( how );
      ASalert[1] = asalert;
      IODE       = iode;
      fitint     = fitInt;

      if (!haveSubframe[0])
      {
         InvalidRequest exc("Need to load subframe 1 before subframe 2");
         GPSTK_THROW(exc);
      }
      bool healthy = false;
      if (health == 0) healthy = true;

      double timeDiff = toe - HOWtime[1];
      short epochWeek = weeknum;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

         // The observation ID has a type of navigation, but the
         // carrier and code types are undefined.  They could be
         // L1/L2 C/A, P, Y,.....
      ObsID obsID(ObsID::otNavMsg, ObsID::cbUndefined, ObsID::tcUndefined);

      short accFlag = accFlagTmp;   // accFlagTmp set in setSF1( )
      //local variables in SF3 that are needed to load SF2
      double crc = 0.0;
      double cis = 0.0;
      double cic = 0.0;
      double Omega0 = 0.0;
      double I0 = 0.0;
      double W = 0.0;
      double OmegaDot = 0.0;
      double IDot = 0.0;
      //also need locals for modernized nav quantaties not in SF2 or SF3
      double A = ahalf*ahalf;     // TEMP fix BWT
      double dndot = 0.0;
      double Adot = 0.0;

      short fitHours = getLegacyFitInterval(IODC, fitint);
      long beginFitSOW = toe - (fitHours/2)*3600.0;
      long endFitSOW = toe + (fitHours/2)*3600.0;
      short beginFitWk = weeknum;
      short endFitWk = weeknum;
      if (beginFitSOW < 0)
      {
         beginFitSOW += FULLWEEK;
         beginFitWk--;
      }
      CommonTime beginFit = GPSWeekSecond(beginFitWk, beginFitSOW, TimeSystem::GPS);
      if (endFitSOW >= FULLWEEK)
      {
         endFitSOW += FULLWEEK;
         endFitWk++;
      }
      CommonTime endFit = GPSWeekSecond(endFitWk, endFitSOW, TimeSystem::GPS);

      CommonTime toeCT = GPSWeekSecond(epochWeek, toe, TimeSystem::GPS);

      orbit.loadData(satSys, obsID, PRNID, beginFit, endFit, toeCT,
                     accFlag, healthy, cuc, cus, crc, crs, cic, cis, m0, Dn,
                     dndot, Ecc, A, ahalf, Adot, Omega0, I0, W, OmegaDot, IDot);
      haveSubframe[1] = true;
      return *this;
   }

   EngEphemeris& EngEphemeris::setSF3( unsigned tlm, double how, short asalert,
                                       double cic, double Omega0, double cis,
                                       double I0, double crc, double W,
                                       double OmegaDot, double IDot )
      throw( InvalidRequest )
   {
      tlm_message[2] = tlm;
      HOWtime[2] = static_cast<long>( how );
      ASalert[2] = asalert;

      if (!haveSubframe[1])
      {
         InvalidRequest exc("Need to load subframe 2 before subframe 3");
         GPSTK_THROW(exc);
      }
      bool healthy = false;
      if (health == 0) healthy = true;

      double timeDiff = orbit.getToe() - HOWtime[2];
      short epochWeek = weeknum;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

         // The observation ID has a type of navigation, but the
         // carrier and code types are undefined.  They could be
         // L1/L2 C/A, P, Y,.....
      ObsID obsID(ObsID::otNavMsg, ObsID::cbUndefined, ObsID::tcUndefined);

      short accFlag = 0;
      double toe = 0.0;
      double cuc = 0.0;
      double cus = 0.0;
      double crs = 0.0;
      double m0 = 0.0;
      double Dn = 0.0;
      double dndot = 0.0;
      double Ecc = 0.0;
      double A = 0.0;
      double ahalf = 0.0;
      double Adot = 0.0;
      CommonTime beginFit;
      CommonTime endFit;
      try
      {
         accFlag = orbit.getURAoe();
         toe = orbit.getToe();
         cuc = orbit.getCuc();
         cus = orbit.getCus();
         dndot = orbit.getDnDot();
         A = orbit.getA();
         Adot = orbit.getAdot();
         crs = orbit.getCrs();
         m0 = orbit.getM0();
         Dn = orbit.getDn();
         Ecc = orbit.getEcc();
         ahalf = orbit.getAhalf();
         beginFit = orbit.getBeginningOfFitInterval();
         endFit = orbit.getEndOfFitInterval();
      }
      catch(InvalidRequest)
      {
         //Should not get to this point because of the if(!haveSubFrame[1]) check above.
         haveSubframe[1] = false;
         haveSubframe[2] = false;
         return *this;
      }

      CommonTime toeCT = GPSWeekSecond(epochWeek, toe, TimeSystem::GPS);

      orbit.loadData( satSys, obsID, PRNID, beginFit, endFit, toeCT,
                      accFlag, healthy, cuc, cus, crc, crs, cic, cis, m0, Dn,
                      dndot, Ecc, A, ahalf, Adot, Omega0, I0, W, OmegaDot, IDot);

      haveSubframe[2] = true;
      return *this;
   }

  void EngEphemeris::setFIC(const bool arg)
  {
	isFIC = arg;
  }

   static void timeDisplay( ostream & os, const CommonTime& t )
   {
         // Convert to CommonTime struct from GPS wk,SOW to M/D/Y, H:M:S.
      GPSWeekSecond dummyTime;
      dummyTime = GPSWeekSecond(t);
      os << setw(4) << dummyTime.week << "(";
      os << setw(4) << (dummyTime.week & 0x03FF) << ")  ";
      os << setw(6) << setfill(' ') << dummyTime.sow << "   ";

      switch (dummyTime.getDayOfWeek())
      {
         case 0: os << "Sun-0"; break;
         case 1: os << "Mon-1"; break;
         case 2: os << "Tue-2"; break;
         case 3: os << "Wed-3"; break;
         case 4: os << "Thu-4"; break;
         case 5: os << "Fri-5"; break;
         case 6: os << "Sat-6"; break;
         default: break;
      }
      os << "   " << (static_cast<YDSTime>(t)).printf("%3j   %5.0s   ")
         << (static_cast<CivilTime>(t)).printf("%02m/%02d/%04Y   %02H:%02M:%02S");
   }

   static void shortcut(ostream & os, const long HOW )
   {
      short DOW, hour, min, sec;
      long SOD, SOW;
      short SOH;

      SOW = static_cast<long>( HOW );
      DOW = static_cast<short>( SOW / SEC_PER_DAY );
      SOD = SOW - static_cast<long>( DOW * SEC_PER_DAY );
      hour = static_cast<short>( SOD/3600 );

      SOH = static_cast<short>( SOD - (hour*3600) );
      min = SOH/60;

      sec = SOH - min * 60;
      switch (DOW)
      {
         case 0: os << "Sun-0"; break;
         case 1: os << "Mon-1"; break;
         case 2: os << "Tue-2"; break;
         case 3: os << "Wed-3"; break;
         case 4: os << "Thu-4"; break;
         case 5: os << "Fri-5"; break;
         case 6: os << "Sat-6"; break;
         default: break;
      }

      os << ":" << setfill('0')
         << setw(2) << hour
         << ":" << setw(2) << min
         << ":" << setw(2) << sec
         << setfill(' ');
   }
    void EngEphemeris :: dumpTerse(ostream& s) const
      throw(InvalidRequest )
   {

       // Check if the subframes have been loaded before attempting
       // to dump them.
      if (!haveSubframe[0] || !haveSubframe[1] || !haveSubframe[2])
      {
         InvalidRequest exc("Need to load subframes 1,2 and 3");
         GPSTK_THROW(exc);
      }

      ios::fmtflags oldFlags = s.flags();

      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      SVNumXRef svNumXRef;
      int NAVSTARNum = 0;
      try
      {
	      NAVSTARNum = svNumXRef.getNAVSTAR(PRNID, bcClock.getEpochTime());
         s << setw(2) << " " << NAVSTARNum << "  ";
      }
      catch(NoNAVSTARNumberFound)
      {
	      s << "  XX  ";
      }

      s << setw(2) << PRNID << " ! ";

      string tform = "%3j %02H:%02M:%02S";

      s << printTime(getTransmitTime(), tform) << " ! ";
      s << printTime(bcClock.getEpochTime(), tform) << " ! ";
      s << printTime(orbit.getEndOfFitInterval(), tform) << " !  ";

      s << setw(4) << setprecision(1) << getAccuracy() << "  ! ";
      s << "0x" << setfill('0') << hex << setw(3) << IODC << " ! ";
      s << "0x" << setfill('0')  << setw(2) << health;
      s << setfill(' ') << dec;
      s << "   " << setw(2) << health << " ! ";

      s << endl;
      s.flags(oldFlags);

    } // end of SF123::dumpTerse()



   void EngEphemeris :: dump(ostream& s) const
      throw( InvalidRequest )
   {


       // Check if the subframes have been loaded before attempting
       // to dump them.
      if (!haveSubframe[0] || !haveSubframe[1] || !haveSubframe[2])
      {
         InvalidRequest exc("Need to load subframes 1,2 and 3");
         GPSTK_THROW(exc);
      }

      ios::fmtflags oldFlags = s.flags();

      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      s << "****************************************************************"
        << "************" << endl
        << "Broadcast Ephemeris (Engineering Units)";
	if(isFIC)
	{
		s << " -FIC" << endl;
	}
	else
	{
		s << " -RINEX" << endl;
	}
      s << endl;
      s << "PRN : " << setw(2) << PRNID << endl;
      s << endl;


      s << "              Week(10bt)     SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n";
      s << "Clock Epoch:  ";

      timeDisplay(s, bcClock.getEpochTime());
      s << endl;
      s << "Eph Epoch:    ";
      timeDisplay(s, orbit.getOrbitEpoch());
      s << endl;
/*
#if 0
         // FIX when moved from sf123, the tot got zapped.. because in
         // order for EngEphemeris to be able to use such a thing, it
         // needs to be pulled out of as-broadcast bits somehow.
      s << "Transmit time:" << setw(4) << weeknum << ", sow=" << Tot.GPSsecond() << endl
        << "Fit interval flag :  " << setw(2) << fitint
        << " (" << fitintlen << " hours)" << endl;
#elsif 0
      s << "Transmit time:" << setw(4) << weeknum << endl
        << "Fit interval flag :  " << setw(2) << fitint
        << " (" << getFitInt() << " hours)" << endl;
#endif
         // nuts to the above, let's just make it look like navdump output
*/
      s << "Transmit Time:";
	timeDisplay(s, getTransmitTime());
      s << endl;
      s << "Fit interval flag :  " << fitint << endl;
      if(isFIC)
      {
     	 s << endl
      	  << "          SUBFRAME OVERHEAD"
      	  << endl
      	  << endl
      	  << "               SOW    DOW:HH:MM:SS     IOD    ALERT   A-S\n";
     	 for (int i=0;i<3;i++)
     	 {
        	 s << "SF" << setw(1) << (i+1)
          	 << " HOW:   " << setw(7) << HOWtime[i]
          	 << "  ";

       	 	 shortcut( s, HOWtime[i]);
        	 if (i==0)
        	    s << "   ";
        	 else
        	    s << "    ";

       		  s << "0x" << setfill('0') << hex;

        	 if (i==0)
           		 s << setw(3) << IODC;
        	 else
        	    s << setw(2) << IODE;

       		  s << dec << "      " << setfill(' ');

        	 if (ASalert[i] & 0x0002)    // "Alert" bit handling
        	    s << "1     ";
        	 else
        	    s << "0     ";

       		  if (ASalert[i] & 0x0001)     // A-S flag handling
        	    s << " on";
        	 else
        	    s << "off";
        	 s << endl;
       }
     }
     else
     {
        s << endl;
	s << "IODC: 0x"
	  << setfill('0') << hex;
	s << setw(3) << IODC << endl;
	s << "IODE:  0x"
	  << setfill('0') << hex;
	s << setw(2) << IODE << endl;
    }
      s.setf(ios::scientific, ios::floatfield);
      s.precision(8);
      s.fill(' ');

      s << endl
        << "           CLOCK"
        << endl
        << endl
        << "Bias T0:     " << setw(16) << bcClock.getAf0() << " sec" << endl
        << "Drift:       " << setw(16) << bcClock.getAf1() << " sec/sec" << endl
        << "Drift rate:  " << setw(16) << bcClock.getAf2() << " sec/(sec**2)" << endl
        << "Group delay: " << setw(16) << Tgd << " sec" << endl;

      s << endl
        << "           ORBIT PARAMETERS"
        << endl
        << endl
        << "Semi-major axis:       " << setw(16) << orbit.getAhalf()  << " m**.5" << endl
        << "Motion correction:     " << setw(16) << orbit.getDn()     << " rad/sec"
        << endl
        << "Eccentricity:          " << setw(16) << orbit.getEcc()    << endl
        << "Arg of perigee:        " << setw(16) << orbit.getW()      << " rad" << endl
        << "Mean anomaly at epoch: " << setw(16) << orbit.getM0()     << " rad" << endl
        << "Right ascension:       " << setw(16) << orbit.getOmega0() << " rad    "
        << setw(16) << orbit.getOmegaDot() << " rad/sec" << endl
        << "Inclination:           " << setw(16) << orbit.getI0()     << " rad    "
        << setw(16) << orbit.getIDot()     << " rad/sec" << endl;

      s << endl
        << "           HARMONIC CORRECTIONS"
        << endl
        << endl
        << "Radial        Sine: " << setw(16) << orbit.getCrs() << " m    Cosine: "
        << setw(16) << orbit.getCrc() << " m" << endl
        << "Inclination   Sine: " << setw(16) << orbit.getCis() << " rad  Cosine: "
        << setw(16) << orbit.getCic() << " rad" << endl
        << "In-track      Sine: " << setw(16) << orbit.getCus() << " rad  Cosine: "
        << setw(16) << orbit.getCuc() << " rad" << endl;

      s << endl
        << "           SV STATUS"
        << endl
        << endl
        << "Health bits:   0x" << setfill('0')  << setw(2) << health
        << "      URA index: " << setfill(' ') << setw(4) << orbit.getURAoe() << endl
        << "Code on L2:   ";

      switch (codeflags)
      {
         case 0:
            s << "reserved ";
            break;

         case 1:
            s << " P only  ";
            break;

         case 2:
            s << " C/A only";
            break;

         case 3:
            s << " P & C/A ";
            break;

         default:
            break;

      }
      if(isFIC)
      {
      	s << "  L2 P Nav data:          ";
     	if (L2Pdata!=0)
        	s << "off";
      	else
         	s << "on";
      }

      s << endl;
      s.flags(oldFlags);

   } // end of SF123::dump()

   ostream& operator<<(ostream& s, const EngEphemeris& eph)
   {
     try
{
      eph.dump(s);
}
     catch(gpstk::Exception& ex)
{
     ex.addLocation(FILE_LOCATION);
     GPSTK_RETHROW(ex);
}
      return s;

   } // end of operator<<

} // namespace
