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
 * @file CNAVEphemeris.cpp
 * Ephemeris data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>
#include <iostream>

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "MathBase.hpp"
#include "GPSEllipsoid.hpp"
#include "CNAVEphemeris.hpp"
#include "GPS_URA.hpp"

namespace gpstk
{
   using namespace std;

   CNAVEphemeris::CNAVEphemeris()
      throw()
   {
      dataLoaded = false;

      satSys = "";

      PRNID = TOWWeek = L1Health = L2Health = L5Health =
      TOWCount[0] = TOWCount[1] = Alert[0] = Alert[1] = Top = 0;

   }

   void CNAVEphemeris::loadData( const std::string satSysArg, const ObsID obsIDArg, 
                                 const short PRNIDArg, const short AlertMsg10Arg, 
                                 const long TOWMsg10Arg, const short AlertMsg11Arg,
                                 const long TOWMsg11Arg, const short TOWWeekArg, 
                                 const long TopArg, const short URAoeArg,
		                           const short L1HealthArg, const short L2HealthArg, 
		                           const short L5HealthArg, const double ToeArg, 
	                              const double accuracyArg, const double CucArg, 
                                 const double CusArg, const double CrcArg, 
                                 const double CrsArg, const double CicArg, 
                                 const double CisArg, const double M0Arg, 
                                 const double dnArg, const double dndotArg,
		                           const double eccArg, const double deltaAArg, 
                                 const double AdotArg, const double OMEGA0Arg, 
                                 const double i0Arg, const double wArg, 
		                           const double deltaOMEGAdotArg, const double idotArg )
   {
      satSys        = satSysArg;
	   obsID         = obsIDArg;
	   PRNID         = PRNIDArg;
      Alert[0]      = AlertMsg10Arg;
      Alert[1]      = AlertMsg11Arg;
      TOWCount[0]   = TOWMsg10Arg;
      TOWCount[1]   = TOWMsg11Arg;
      TOWWeek       = TOWWeekArg;
      L1Health      = L1HealthArg;
      L2Health      = L2HealthArg;
      L5Health      = L5HealthArg;
      Top           = TopArg;
      short URAoe   = URAoeArg;
      double deltaA = deltaAArg;
      bool healthy  = false;
      if (obsIDArg.band == ObsID::cbL2 && L2Health == 0) healthy = true;
      if (obsIDArg.band == ObsID::cbL5 && L5Health == 0) healthy = true;

      double A             = A_REF_GPS + deltaA;
      double Ahalf         = SQRT(A);
      double deltaOMEGAdot = deltaOMEGAdotArg;
      double OMEGAdot      = OMEGADOT_REF_GPS + deltaOMEGAdot;
      satSys = "G";
      double timeDiff = ToeArg - TOWCount[0];
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;
    
         // The observation ID has a type of navigation. The code type could
         // be L1, L2, or L5.
      ObsID obsID(ObsID::otNavMsg, obsIDArg.band, obsIDArg.code);

      long beginFitSOW = ((TOWCount[0])/7200)*7200;
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

      CommonTime ToeCT = GPSWeekSecond(epochWeek, ToeArg, TimeSystem::GPS);

      orbit.loadData(satSys, obsID, PRNID, beginFit, endFit, ToeCT,
                     URAoe, healthy, CucArg, CusArg, CrcArg, CrsArg, CicArg,
                     CisArg, M0Arg, dnArg, dndotArg, eccArg, A, Ahalf, AdotArg, 
		               OMEGA0Arg, i0Arg, wArg, OMEGAdot, idotArg);
      dataLoaded  = true;   
   }

   void CNAVEphemeris::loadData( const ObsID obsIDArg, 
                                 const short PRNIDArg,
                                 const PackedNavBits message10,
                                 const PackedNavBits message11)

      throw( InvalidParameter)
   {
      obsID         = obsIDArg;
      PRNID         = PRNIDArg;
      satSys        = "G";

         // Message Type 10 data
      Alert[0]      = message10.asUnsignedLong(37, 1, 1);
      TOWCount[0]   = message10.asUnsignedLong(20, 17, 300);
      TOWWeek       = message10.asUnsignedLong(38, 13, 1);
      L1Health      = message10.asUnsignedLong(51, 1, 1);
      L2Health      = message10.asUnsignedLong(52, 1, 1);
      L5Health      = message10.asUnsignedLong(53, 1, 1);
      Top           = message10.asUnsignedLong(54, 11, 300);
      short URAoe   = message10.asLong(65, 5, 1);
      double Toe    = message10.asUnsignedLong(70, 11, 300);
      double deltaA = message10.asSignedDouble(81, 26, -9);
      double Adot   = message10.asSignedDouble(107, 25, -21);
      double dn     = message10.asDoubleSemiCircles(132, 17, -44);
      double dndot  = message10.asDoubleSemiCircles(149, 23, -57);
      double M0     = message10.asDoubleSemiCircles(172, 33, -32);
      double ecc    = message10.asUnsignedDouble(205, 33, -34);
      double w      = message10.asDoubleSemiCircles(238, 33, -32);
      
         // Message Type 11 data
      Alert[1]             = message11.asUnsignedLong(37, 1, 1);
      TOWCount[1]          = message11.asUnsignedLong(20, 17, 300);
      double OMEGA0        = message11.asDoubleSemiCircles(49, 33, -32);
      double i0            = message11.asDoubleSemiCircles(82, 33, -32);
      double deltaOMEGAdot = message11.asDoubleSemiCircles(115, 17, -44);
      double idot          = message11.asDoubleSemiCircles(132, 15, -44);
      double Cis           = message11.asSignedDouble(147, 16, -30);
      double Cic           = message11.asSignedDouble(163, 16, -30);
      double Crs           = message11.asSignedDouble(179, 24, -8);
      double Crc           = message11.asSignedDouble(203, 24, -8);
      double Cus           = message11.asSignedDouble(227, 21, -30);
      double Cuc           = message11.asSignedDouble(248, 21, -30);

      double A        = A_REF_GPS + deltaA;
      double Ahalf    = SQRT(A);
      double OMEGAdot = OMEGADOT_REF_GPS + deltaOMEGAdot;

      bool healthy = false;
      if (obsIDArg.band == ObsID::cbL2 && L2Health == 0) healthy = true;
      if (obsIDArg.band == ObsID::cbL5 && L5Health == 0) healthy = true;

      double timeDiff = Toe - TOWCount[0];
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

      long beginFitSOW = ((TOWCount[0])/7200)*7200;
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

      CommonTime ToeCT = GPSWeekSecond(epochWeek, Toe, TimeSystem::GPS);

      orbit.loadData(satSys, obsID, PRNID, beginFit, endFit, ToeCT,
                     URAoe, healthy, Cuc, Cus, Crc, Crs, Cic, Cis, M0, dn,
                     dndot, ecc, A, Ahalf, Adot, OMEGA0, i0, w, OMEGAdot, idot);
      dataLoaded = true;
   }

   bool CNAVEphemeris::hasData() 
   {
      return(dataLoaded);
   }
            
    Xv CNAVEphemeris::svXv(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("svXvt: Required data not stored.");
         GPSTK_THROW(exc);
      }
      Xv sv;
      Xv xv = orbit.svXv(t);
      sv.x = xv.x; // Position
      sv.v = xv.v; // Velocity
      return sv;
   }

   double CNAVEphemeris::svRelativity(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("svRelativity(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.svRelativity(t);
   }
      
   CommonTime CNAVEphemeris::getTransmitTime() const
      throw(InvalidRequest)
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getTransmitTime: Required data not stored.");
         GPSTK_THROW(exc);
      }
      GPSWeekSecond gws(TOWWeek, TOWCount[0], TimeSystem::GPS);
      return (gws.convertToCommonTime());
   }

   CommonTime CNAVEphemeris::getTimeOfPrediction() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getTimeOfPrediction(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      short week(TOWWeek);
      if( Top - TOWCount[0] < -HALFWEEK)
      week++;
      else if ( Top - TOWCount[0] > HALFWEEK)
      week--;
      CommonTime toReturn;
      toReturn = GPSWeekSecond(week, Top, TimeSystem::GPS);
      return toReturn;
   }

   short CNAVEphemeris::getPRNID() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getPRNID(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return PRNID;
   }

   short CNAVEphemeris::getAlert(short messageNum)  const
      throw( InvalidRequest, Exception )
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("getAlert(): messageNum not stored.");
         GPSTK_THROW(exc);
      }
      if (messageNum == 10) return Alert[0];
      if (messageNum == 11) return Alert[1];

	  stringstream os;
	  os << messageNum;
	  Exception e("getAlert(): unrecognized value for messageNum: " + os.str());
	  GPSTK_THROW(e);
   }

   double CNAVEphemeris::getAccuracy()  const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getAccuracy();
   }  

   short CNAVEphemeris::getURAoe() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getURAoe(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return orbit.getURAoe();
   }

   short CNAVEphemeris::getHealth(const ObsID::CarrierBand cb) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getHealth(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      short retVal;
      switch (cb)
      {
         case ObsID::cbL1:
            retVal = L1Health;
            break;
         case ObsID::cbL2:
            retVal = L2Health;
            break;
         case ObsID::cbL5:
            retVal = L5Health;
            break;
         default:
            InvalidRequest exc("getHealth(): Invalid carrier selection");
            GPSTK_THROW(exc);
      }
      return retVal;
   }

   long CNAVEphemeris::getTop() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Top;
   }
  
   BrcKeplerOrbit CNAVEphemeris::getOrbit() const
      throw(InvalidRequest )
   {
      if(!orbit.hasData())
      {
         InvalidRequest exc("getOrbit(): Required Orbit data not stored.");
         GPSTK_THROW(exc);
      }
      return (orbit);
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
   void CNAVEphemeris :: dump(ostream& s) const
      throw()
   { 
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');
      
      s << "****************************************************************"
        << "************" << endl
        << "CNAV Message Types 10 and 11" << endl
        << endl
        << "PRN : " << setw(2) << PRNID << "      "
        << "System : " << satSys << "      "
        << "Carrier: " << ObsID::cbDesc[obsID.band] << "      "
        << "Code: " << ObsID::tcDesc[obsID.code] << endl << endl;
  

      s << "                  Week        SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n";
      s << "Transmit Time:  ";

      timeDisplay(s, getTransmitTime());
      s << endl;
      s << "Time of Predict:";
      timeDisplay(s, getTimeOfPrediction());
      s << endl;
     
      s << endl
        << "          ACCURACY PARAMETERS"
        << endl
        << endl
        << "URAoe index:  " << setw(4) << orbit.getURAoe() << endl;

      s.setf(ios::scientific, ios::floatfield);
      s.precision(11);

      s << endl
      << "           SIGNAL PARAMETERS"
      << endl
      << endl
      << "L1 Health bit:  " << setw(2) << L1Health << endl
      << "L2 Health bit:  " << setw(2) << L2Health << endl
      << "L5 Health bit:  " << setw(2) << L5Health << endl
      << setfill(' ') << endl;  
     
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

   } // end of CNAVEphemeris::dump()

   ostream& operator<<(ostream& s, const CNAVEphemeris& eph)
   {
      eph.dump(s);
      return s;
   } // end of operator<<

} // namespace
