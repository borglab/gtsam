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
 * @file CNAV2EphClk.cpp
 * Ephemeris data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "MathBase.hpp"
#include "GPSEllipsoid.hpp"
#include "CNAV2EphClk.hpp"
#include "GPS_URA.hpp"

namespace gpstk
{
   using namespace std;

   CNAV2EphClk::CNAV2EphClk()
      throw()
   {
      dataLoaded = false;

      satSys = "";

      PRNID = TOWWeek = L1Health = 
      TOWCount = Top = 0;

      Tgd = ISCL1cp = ISCL1cd = 0.0;
   }

   void CNAV2EphClk::loadData( const std::string satSysArg, const ObsID obsIDArg,
                               const short PRNIDArg, const short TOWWeekArg,
                               const long TOWArg, const long TopArg,
                               const bool  L1HealthArg, const short URAoeArg,
                               const long ToeArg, const double deltaAArg,
                               const double AdotArg, const double dnArg,
                               const double dndotArg, const double M0Arg,
                               const double eccArg, const double wArg,
                               const double OMEGA0Arg, const double i0Arg,
                               const double OMEGAdotArg, const double idotArg,
                               const double CicArg, const double CisArg,
                               const double CrcArg, const double CrsArg,
                               const double CucArg, const double CusArg,
                               const short URAocArg, const short URAoc1Arg,
                               const short URAoc2Arg, const double Af0Arg,
                               const double Af1Arg, const double Af2Arg,
                               const double TgdArg, const double ISCL1cpArg,
                               const double ISCL1cdArg)
   {
      satSys       = satSysArg;
	   obsID        = obsIDArg;
	   PRNID        = PRNIDArg;
      TOWWeek      = TOWWeekArg;
      TOWCount     = TOWArg;
      Top          = TopArg;
      long Toe     = ToeArg;
      L1Health     = L1HealthArg;
      short URAoe  = URAoeArg;
      double deltaA = deltaAArg;
      short URAoc   = URAocArg;
      short URAoc1  = URAoc1Arg;
      short URAoc2  = URAoc2Arg;
      Tgd          = TgdArg;
      ISCL1cp      = ISCL1cpArg;
      ISCL1cd      = ISCL1cdArg;
      bool healthy = false;
      if (L1Health == 0) healthy = true;

      double A     = A_REF_GPS + deltaA;
      double Ahalf = SQRT(A);
      satSys       = "G";
      double timeDiff = ToeArg - TOWCount;
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

         // The observation ID has a type of navigation, but the
         // carrier and code types are undefined.  They could be
         // L1/L2 C/A, P, Y,.....
      ObsID obsID(ObsID::otNavMsg, ObsID::cbUndefined, ObsID::tcUndefined);

      long beginFitSOW = ((TOWCount)/7200)*7200;
      long endFitSOW   = beginFitSOW + 10800;
      short beginFitWk = TOWWeek;
      short endFitWk   = TOWWeek;

      CommonTime beginFit = GPSWeekSecond(beginFitWk, beginFitSOW, TimeSystem::GPS);

      if (endFitSOW >= FULLWEEK)
      {
         endFitSOW -= FULLWEEK;
         endFitWk++;
      }
      CommonTime endFit = GPSWeekSecond(endFitWk, endFitSOW, TimeSystem::GPS);   

      CommonTime TopCT = GPSWeekSecond(epochWeek, Top, TimeSystem::GPS);
      CommonTime ToeCT = GPSWeekSecond(epochWeek, Toe, TimeSystem::GPS);

      orbit.loadData( satSys, obsID, PRNID, beginFit, endFit, ToeCT, URAoe,
                      healthy, CucArg, CusArg, CrcArg, CrsArg, CicArg, 
                      CisArg, M0Arg, dnArg, dndotArg, eccArg, A, Ahalf, AdotArg, 
		                OMEGA0Arg, i0Arg, wArg, OMEGAdotArg, idotArg );

      bcClock.loadData( satSys, obsID, PRNID, ToeCT, TopCT, URAoc,
                        URAoc1, URAoc2, healthy, Af0Arg, Af1Arg, Af2Arg); 
      dataLoaded  = true;   
   }

   void CNAV2EphClk::loadData( const ObsID obsIDArg, const short PRNIDArg,
                               const int subframe1, const PackedNavBits subframe2)
      throw( InvalidParameter)
   {
      obsID  = obsIDArg;
      PRNID  = PRNIDArg;
      satSys = "G";

      TOWWeek              = subframe2.asUnsignedLong(0, 13, 1);
      short ITOW           = subframe2.asUnsignedLong(13, 8 ,1);
      Top                  = subframe2.asUnsignedLong(21, 11, 300);
      L1Health             = subframe2.asUnsignedLong(32, 1, 1);
      TOWCount             = subframe1*18 + ITOW * 7200;
      short URAoe          = subframe2.asLong(33, 5, 1);
      double Toe           = subframe2.asUnsignedLong(38, 11, 300);
      double deltaA        = subframe2.asSignedDouble(49, 26, -9);
      double Adot          = subframe2.asSignedDouble(75, 25, -21);
      double dn            = subframe2.asDoubleSemiCircles(100, 17, -44);
      double dndot         = subframe2.asDoubleSemiCircles(117, 23, -57);
      double M0            = subframe2.asDoubleSemiCircles(140, 33, -32);
      double ecc           = subframe2.asUnsignedDouble(173, 33, -34);
      double w             = subframe2.asDoubleSemiCircles(206, 33, -32);
      double OMEGA0        = subframe2.asDoubleSemiCircles(239, 33, -32);
      double i0            = subframe2.asDoubleSemiCircles(272, 33, -32);
      double deltaOMEGAdot = subframe2.asDoubleSemiCircles(305, 17, -44);
      double idot          = subframe2.asDoubleSemiCircles(322, 15, -44);
      double Cis           = subframe2.asSignedDouble(337, 16, -30);
      double Cic           = subframe2.asSignedDouble(353, 16, -30);
      double Crs           = subframe2.asSignedDouble(369, 24, -8);
      double Crc           = subframe2.asSignedDouble(393, 24, -8);
      double Cus           = subframe2.asSignedDouble(417, 21, -30);
      double Cuc           = subframe2.asSignedDouble(438, 21, -30);
      short URAoc          = subframe2.asLong(459, 5, 1);
      short URAoc1         = subframe2.asUnsignedLong(464, 3, 1);
      short URAoc2         = subframe2.asUnsignedLong(467, 3, 1);
      double af0           = subframe2.asSignedDouble(470, 26, -35);
      double af1           = subframe2.asSignedDouble(496, 20, -48);
      double af2           = subframe2.asSignedDouble(516, 10, -60);
      Tgd                  = subframe2.asSignedDouble(526, 13, -35);
      ISCL1cp              = subframe2.asSignedDouble(539, 13, -35);
      ISCL1cd              = subframe2.asSignedDouble(552, 13, -35);
      short sflag          = subframe2.asUnsignedLong(565, 1, 1);

      double A        = A_REF_GPS + deltaA;
      double Ahalf    = SQRT(A);
      double OMEGAdot = OMEGADOT_REF_GPS + deltaOMEGAdot;

      bool healthy = false;
      if (L1Health == 0) healthy = true;
      double timeDiff = Toe - TOWCount;
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

      double accuracy1 = gpstk::ura2CNAVaccuracy(URAoe);
#pragma unused(sflag,accuracy1)
       
     // deleted because algorithms in -705 and -800 have changed
     // double accuracy2 = gpstk::uraoc2CNAVaccuracy(URAoc, URAoc1, URAoc2, GPSWeekSecond(epochWeek, TOWCount, TimeSystem::GPS), GPSWeekSecond(epochWeek, Top, TimeSystem::GPS));

      long beginFitSOW = ((TOWCount)/7200)*7200;
      long endFitSOW   = beginFitSOW + 10800;
      short beginFitWk = TOWWeek;
      short endFitWk   = TOWWeek;

      CommonTime beginFit = GPSWeekSecond(beginFitWk, beginFitSOW, TimeSystem::GPS);

      if (endFitSOW >= FULLWEEK)
      {
         endFitSOW -= FULLWEEK;
         endFitWk++;
      }
      CommonTime endFit = GPSWeekSecond(endFitWk, endFitSOW, TimeSystem::GPS);

      CommonTime TopCT = GPSWeekSecond(epochWeek, Top, TimeSystem::GPS);
      CommonTime ToeCT = GPSWeekSecond(epochWeek, Toe, TimeSystem::GPS);   

      orbit.loadData( satSys, obsID, PRNID, beginFit, endFit, ToeCT, URAoe,
                      healthy, Cuc, Cus, Crc, Crs, Cic, Cis, M0, 
                      dn, dndot, ecc, A, Ahalf, Adot, OMEGA0, i0, w, OMEGAdot, idot );

      bcClock.loadData( satSys, obsID, PRNID, ToeCT, TopCT, URAoc, 
                        URAoc1, URAoc2, healthy, af0, af1, af2); 
      dataLoaded  = true;   
   }

   bool CNAV2EphClk::hasData() 
   {
      return(dataLoaded);
   }
            
   Xvt CNAV2EphClk::svXvt(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("svXvt: Required data not stored.");
         GPSTK_THROW(exc);
      }
      Xvt sv;
      Xv xv = orbit.svXv(t);

      sv.x = xv.x;
      sv.v = xv.v;

      sv.clkbias = bcClock.svClockBias(t);
      sv.relcorr = orbit.svRelativity(t);
      sv.clkdrift = bcClock.svClockDrift(t);
      
      return sv;
   }
      
   CommonTime CNAV2EphClk::getTransmitTime() const
      throw(InvalidRequest)
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getTransmitTime: Required data not stored.");
         GPSTK_THROW(exc);
      }
      GPSWeekSecond gws(TOWWeek, TOWCount, TimeSystem::GPS);
      return (gws.convertToCommonTime());
   }

   CommonTime CNAV2EphClk::getTimeOfPrediction() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getTimeOfPrediction(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      short week(TOWWeek);
      if( Top - TOWCount < -HALFWEEK)
      week++;
      else if ( Top - TOWCount > HALFWEEK)
      week--;
      CommonTime toReturn;
      toReturn = GPSWeekSecond(week, Top, TimeSystem::GPS);
      return toReturn;
   }

   short CNAV2EphClk::getPRNID() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getPRNID(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return PRNID;
   }

   short CNAV2EphClk::getURAoe() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getURAoe(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getURAoe();
   }

   short CNAV2EphClk::getHealth() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getHealth(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return L1Health;
   }

   short CNAV2EphClk::getURAoc(const short ndx) const
      throw( InvalidRequest, InvalidParameter )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getURAoc(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.getURAoc(ndx);
   }

   double CNAV2EphClk::getTgd() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getTgd(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Tgd;
   }

   double CNAV2EphClk::getISCL1cp() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getISCL1cp(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return ISCL1cp;
   }

   double CNAV2EphClk::getISCL1cd() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getISCL1cd(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return ISCL1cd;
   }

   double CNAV2EphClk::svRelativity(const CommonTime& t) const
      throw( InvalidRequest )
   {
      return orbit.svRelativity(t);
   }

   double CNAV2EphClk::svClockBias(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("svClockBias: Required data not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.svClockBias(t);
   }

   double CNAV2EphClk::svClockDrift(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("svClockDrift(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.svClockDrift(t);
   }

   long CNAV2EphClk::getTop() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Top;
   }
  
   BrcKeplerOrbit CNAV2EphClk::getOrbit() const
      throw(InvalidRequest )
   {
      if(!orbit.hasData())
      {
         InvalidRequest exc("getOrbit(): Required Orbit data not stored.");
         GPSTK_THROW(exc);
      }
      return (orbit);
   }

   BrcClockCorrection CNAV2EphClk::getClock() const
      throw(InvalidRequest )
   {
      if(!bcClock.hasData())
      {
         InvalidRequest exc("getClock(): Required Clock Correction data not stored.");
         GPSTK_THROW(exc);
      }
      return (bcClock);
   }

   static void timeDisplay( ostream & os, const CommonTime& t )
   {
         // Convert to CommonTime struct from GPS wk,SOW to M/D/Y, H:M:S.
      GPSWeekSecond dummyTime;
      dummyTime = GPSWeekSecond(t);
      os << dec;
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
      os << "   " << (static_cast<YDSTime>(t)).printf("%3j   %5.0s  ") 
         << (static_cast<CivilTime>(t)).printf("%02m/%02d/%04Y   %02H:%02M:%02S");
   }
    
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"

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
#pragma clang diagnostic pop
    
   void CNAV2EphClk :: dump(ostream& s) const
      throw()
   { 
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');
      
      s << "****************************************************************"
        << "************" << endl
        << "CNAV-2 Subframe 2" << endl
        << endl
        << "PRN: " << setw(2) << PRNID << "       "
        << "System: " << satSys << endl<<endl;
  
      s << "                  Week        SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n";
      s << "Transmit Time:  ";

      timeDisplay(s, getTransmitTime());
      s << endl;
      s << "Time of Predict:";
      timeDisplay(s, getTimeOfPrediction());
      s << endl;

      s << "Transmit Week:  " << setw(4) << TOWWeek << endl;  
     
      s << endl
        << "          ACCURACY PARAMETERS"
        << endl
        << endl
        << "URAoe index:  " <<setw(3) << getURAoe() << endl
        << "URAoc index:  " <<setw(3) << getURAoc(0) << "    " << setw(3) << getURAoc(1) 
        << "    " << setw(3) << getURAoc(2) << endl;

      s.setf(ios::scientific, ios::floatfield);
      s.precision(11);

      s << endl
      << "           SIGNAL PARAMETERS"
      << endl
      << endl
      << "Health bit:      " << setw(2) << L1Health << endl
      << "Group Delay:     " << setfill(' ') << setw(18) << Tgd << " sec" << endl
      << "ISC L1cp:        " << setw(18) << ISCL1cp << " sec" << endl
      << "ISC L1cd:        " << setw(18) << ISCL1cd << " sec" << endl;
        
      s << endl
        << "           CLOCK"
        << endl
        << endl
        << "Bias T0:     " << setw(18) << bcClock.getAf0() << " sec" << endl
        << "Drift:       " << setw(18) << bcClock.getAf1() << " sec/sec" << endl
        << "Drift rate:  " << setw(18) << bcClock.getAf2() << " sec/(sec**2)" << endl;
      
      s << endl
        << "           ORBIT PARAMETERS"
        << endl
        << endl
        << "Semi-major axis:       " << setw(18) << orbit.getAhalf()  << " m**.5" << endl
        << "Motion correction:     " << setw(18) << orbit.getDn()     << " rad/sec"
        << endl
        << "Eccentricity:          " << setw(18) << orbit.getEcc()    << endl
        << "Arg of perigee:        " << setw(18) << orbit.getW()      << " rad" << endl
        << "Mean anomaly at epoch: " << setw(18) << orbit.getM0()     << " rad" << endl
        << "Right ascension:       " << setw(18) << orbit.getOmega0() << " rad    "
        << setw(18) << orbit.getOmegaDot() << " rad/sec" << endl
        << "Inclination:           " << setw(18) << orbit.getI0()     << " rad    "
        << setw(18) << orbit.getIDot()     << " rad/sec" << endl;
      
      s << endl
        << "           HARMONIC CORRECTIONS"
        << endl
        << endl
        << "Radial        Sine: " << setw(18) << orbit.getCrs() << " m    Cosine: "
        << setw(18) << orbit.getCrc() << " m" << endl
        << "Inclination   Sine: " << setw(18) << orbit.getCis() << " rad  Cosine: "
        << setw(18) << orbit.getCic() << " rad" << endl
        << "In-track      Sine: " << setw(18) << orbit.getCus() << " rad  Cosine: "
        << setw(18) << orbit.getCuc() << " rad" << endl;    
      
      s << "****************************************************************"
        << "************" << endl;
      
   } // end of CNAV2EphClk::dump()

   ostream& operator<<(ostream& s, const CNAV2EphClk& eph)
   {
      eph.dump(s);
      return s;
   } // end of operator<<

} // namespace
