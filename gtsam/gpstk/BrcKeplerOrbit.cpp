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
 * @file BrcKeplerOrbit.cpp
 * Ephemeris data encapsulated in engineering terms
 */
#include <stdio.h>
#include "BrcKeplerOrbit.hpp"
#include <cmath>

namespace gpstk
{
   using namespace std;
   using namespace gpstk;

   BrcKeplerOrbit::BrcKeplerOrbit()
      throw()
   {
      dataLoaded = false;

      PRNID = 0;

      satSys = "";

      healthy = false;     
     
      URAoe = -16;

      Cuc = Cus = Crc = Crs = Cic = Cis = M0 = dn = dndot = 
	   ecc = A = Ahalf =Adot = OMEGA0 = i0 = w = OMEGAdot = idot = 0.0;
   }

   BrcKeplerOrbit::BrcKeplerOrbit(const std::string satSysArg, const ObsID obsIDArg,
                                  const short PRNIDArg, const CommonTime beginFitArg,
                                  const CommonTime endFitArg, const CommonTime ToeArg,
                                  const short URAoeArg, const bool healthyArg, 
                                  const double CucArg, const double CusArg, 
                                  const double CrcArg, const double CrsArg, 
                                  const double CicArg, const double CisArg, 
                                  const double M0Arg, const double dnArg, 
                                  const double dndotArg, const double eccArg, 
		                            const double AArg, const double AhalfArg, 
                                  const double AdotArg, const double OMEGA0Arg, 
                                  const double i0Arg, const double wArg, 
                                  const double OMEGAdotARg, const double idotArg )
   {
      loadData(satSysArg, obsIDArg, PRNIDArg, beginFitArg, endFitArg, ToeArg,
               URAoeArg, healthyArg, CucArg, CusArg, CrcArg,
               CrsArg, CicArg, CisArg, M0Arg, dnArg, dndotArg, eccArg, AArg,
               AhalfArg, AdotArg, OMEGA0Arg, i0Arg, wArg, OMEGAdotARg, idotArg );
   }

		/// Legacy GPS Subframe 1-3  
   BrcKeplerOrbit::BrcKeplerOrbit(const ObsID obsIDArg, const short PRNID,
                                  const short fullweeknum,
		                            const long subframe1[10],
		                            const long subframe2[10],
		                            const long subframe3[10] )
   {
      loadData(obsIDArg, PRNID,fullweeknum, subframe1, subframe2, subframe3 );
   }



   void BrcKeplerOrbit::loadData(const std::string satSysArg, const ObsID obsIDArg,
                                 const short PRNIDArg, const CommonTime beginFitArg,
                                 const CommonTime endFitArg, const CommonTime ToeArg,
                                 const short URAoeArg, const bool healthyArg, 
                                 const double CucArg, const double CusArg, 
                                 const double CrcArg, const double CrsArg, 
                                 const double CicArg, const double CisArg,  
                                 const double M0Arg, const double dnArg, 
                                 const double dndotArg, const double eccArg, 
		                           const double AArg, const double AhalfArg, 
                                 const double AdotArg, const double OMEGA0Arg, 
                                 const double i0Arg, const double wArg, 
                                 const double OMEGAdotARg, const double idotArg )
   {
	   satSys      = satSysArg;
	   obsID       = obsIDArg;
	   PRNID       = PRNIDArg;
      beginFit    = beginFitArg;
      endFit      = endFitArg;
	   Toe         = ToeArg;
	   URAoe       = URAoeArg;
	   healthy     = healthyArg;
	   Cuc         = CucArg;
	   Cus         = CusArg;
	   Crc         = CrcArg;
	   Crs         = CrsArg;
	   Cic         = CicArg;
	   Cis         = CisArg;
	   M0          = M0Arg;
	   dn          = dnArg;
	   dndot       = dndotArg;
	   ecc         = eccArg;
	   A           = AArg;
      Ahalf       = AhalfArg;
	   Adot        = AdotArg;
	   OMEGA0      = OMEGA0Arg;
	   i0          = i0Arg;
	   w           = wArg;
	   OMEGAdot    = OMEGAdotARg;
	   idot        = idotArg;
	   dataLoaded  = true;
   }

   void BrcKeplerOrbit::loadData(const ObsID obsIDArg, const short PRNIDArg,
                                 const short fullweeknum,
		                           const long subframe1[10],
		                           const long subframe2[10],
		                           const long subframe3[10])
	   throw(InvalidParameter)
   {
      double ficked[60];

 	    //Load overhead members
  	   satSys = "G";
	   obsID = obsIDArg;
	   PRNID = PRNIDArg;
      short iodc = 0;

	    //Convert Subframe 1
	   if (!subframeConvert(subframe1, fullweeknum, ficked))
	   {
	      InvalidParameter exc("Subframe 1 not valid.");
	      GPSTK_THROW(exc);
	   }

	   short weeknum = static_cast<short>( ficked[5] );
	   short accFlag = static_cast<short>( ficked[7] );
	   short health  = static_cast<short>( ficked[8] );
	   URAoe = accFlag;
	   healthy = false;
	   if (health == 0)
	   healthy = true;
      iodc = static_cast<short>( ldexp( ficked[9], -11 ) );
	  
	    //Convert Subframe 2
	   if (!subframeConvert(subframe2, fullweeknum, ficked))
	   {
	      InvalidParameter exc("Subframe 2 not valid.");
	      GPSTK_THROW(exc);
	   }

      Crs    = ficked[6];
	   dn     = ficked[7];
	   M0     = ficked[8];
	   Cuc    = ficked[9];
	   ecc    = ficked[10];
	   Cus    = ficked[11];
	   Ahalf  = ficked[12];
	   A      = Ahalf*Ahalf;
	   double ToeSOW = ficked[13];
/*
      double diff = Txmit - ToeSOW;
      if (diff > HALFWEEK)          // NOTE: This USED to be in DayTime, but DayTime is going away.  Where is it now?
         weeknum++;                 // Convert week # of transmission to week # of epoch time when Toc is forward across a week boundary
      else if (diff < -HALFWEEK)
         weeknum--;                 // Convert week # of transmission to week # of epoch time when Toc is back across a week boundary
*/
      Toe = GPSWeekSecond(weeknum, ToeSOW, TimeSystem::GPS);
      short fiti = static_cast<short>(ficked[14]);
      short fitHours = getLegacyFitInterval(iodc, fiti);
      long beginFitSOW = ToeSOW - (fitHours/2)*3600;
      long endFitSOW = ToeSOW + (fitHours/2)*3600;
      short beginFitWk = weeknum;
      short endFitWk = weeknum;
      if (beginFitSOW < 0)
      {
         beginFitSOW += FULLWEEK;
         beginFitWk--;
      }
      beginFit = GPSWeekSecond(beginFitWk, beginFitSOW, TimeSystem::GPS);

      if (endFitSOW >= FULLWEEK)
      {
         endFitSOW -= FULLWEEK;
         endFitWk++;
      }
      endFit = GPSWeekSecond(endFitWk, endFitSOW, TimeSystem::GPS);       

	       //Convert Subframe 3
	   if (!subframeConvert(subframe3, fullweeknum, ficked))
	   {
	      InvalidParameter exc("Subframe3 not valid.");
	      GPSTK_THROW(exc);
	   }

	   Cic      = ficked[5];
	   OMEGA0   = ficked[6];
	   Cis      = ficked[7];
	   i0       = ficked[8];
	   Crc      = ficked[9];
	   w        = ficked[10];
	   OMEGAdot = ficked[11];
	   idot     = ficked[13];

	   dndot      = 0.0;
	   Adot       = 0.0;
	   dataLoaded = true;
	 
	   return;
   }
	     
   bool BrcKeplerOrbit::hasData() const
   {
      return(dataLoaded);
   }

   bool BrcKeplerOrbit::isHealthy() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {   
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return(healthy);
   }

   bool BrcKeplerOrbit::withinFitInterval(const CommonTime ct) const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {   
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      if (ct >= beginFit && ct <= endFit) return(true);    
      return(false);
   }

   Xv BrcKeplerOrbit::svXv(const CommonTime& t) const
      throw(InvalidRequest)
   {
      Xv sv;

      GPSWeekSecond gpsws = (Toe);
      double ToeSOW = gpsws.sow;
      double ea;              // eccentric anomaly //
      double delea;           // delta eccentric anomaly during iteration */
      double elapte;          // elapsed time since Toe 
      //double elaptc;          // elapsed time since Toc 
      double q,sinea,cosea;
      double GSTA,GCTA;
      double amm;
      double meana;           // mean anomaly 
      double F,G;             // temporary real variables 
      double alat,talat,c2al,s2al,du,dr,di,U,R,truea,AINC;
      double ANLON,cosu,sinu,xip,yip,can,san,cinc,sinc;
      double xef,yef,zef,dek,dlk,div,domk,duv,drv;
      double dxp,dyp,vxef,vyef,vzef;
      GPSEllipsoid ell;

      double sqrtgm = SQRT(ell.gm());

         // Check for ground transmitter
      double twoPI = 2.0e0 * PI;
      double lecc;               // eccentricity
      double tdrinc;            // dt inclination

      lecc = ecc;
      tdrinc = idot;

         // Compute time since ephemeris & clock epochs
      elapte = t - getOrbitEpoch();
      //CommonTime orbEp = getOrbitEpoch();
      //elapte = t - orbEp;

         // Compute mean motion
      amm  = (sqrtgm / (A*Ahalf)) + dn;


         // In-plane angles
         //     meana - Mean anomaly
         //     ea    - Eccentric anomaly
         //     truea - True anomaly

      meana = M0 + elapte * amm;
      meana = fmod(meana, twoPI);
   
      ea = meana + lecc * ::sin(meana);

      int loop_cnt = 1;
      do  {
         F = meana - ( ea - lecc * ::sin(ea));
         G = 1.0 - lecc * ::cos(ea);
         delea = F/G;
         ea = ea + delea;
         loop_cnt++;
      } while ( (fabs(delea) > 1.0e-11 ) && (loop_cnt <= 20) );
   
         // Compute true anomaly
      q     = SQRT( 1.0e0 - lecc*lecc);
      sinea = ::sin(ea);
      cosea = ::cos(ea);
      G     = 1.0e0 - lecc * cosea;
   
         //  G*SIN(TA) AND G*COS(TA)
      GSTA  = q * sinea;
      GCTA  = cosea - lecc;

         //  True anomaly
      truea = atan2 ( GSTA, GCTA );

         // Argument of lat and correction terms (2nd harmonic)
      alat  = truea + w;
      talat = 2.0e0 * alat;
      c2al  = ::cos( talat );
      s2al  = ::sin( talat );

      du  = c2al * Cuc +  s2al * Cus;
      dr  = c2al * Crc +  s2al * Crs;
      di  = c2al * Cic +  s2al * Cis;

         // U = updated argument of lat, R = radius, AINC = inclination
      U    = alat + du;
      R    = A*G  + dr;
      AINC = i0 + tdrinc * elapte  +  di;

         //  Longitude of ascending node (ANLON)
      ANLON = OMEGA0 + (OMEGAdot - ell.angVelocity()) *
              elapte - ell.angVelocity() * ToeSOW;

         // In plane location
      cosu = ::cos( U );
      sinu = ::sin( U );

      xip  = R * cosu;
      yip  = R * sinu;

         //  Angles for rotation to earth fixed
      can  = ::cos( ANLON );
      san  = ::sin( ANLON );
      cinc = ::cos( AINC  );
      sinc = ::sin( AINC  );
 
         // Earth fixed - meters
      xef  =  xip*can  -  yip*cinc*san;
      yef  =  xip*san  +  yip*cinc*can;
      zef  =              yip*sinc;

      sv.x[0] = xef;
      sv.x[1] = yef;
      sv.x[2] = zef;

         // Compute velocity of rotation coordinates
      dek = amm * A / R;
      dlk = Ahalf * q * sqrtgm / (R*R);
      div = tdrinc - 2.0e0 * dlk *
         ( Cic  * s2al - Cis * c2al );
      domk = OMEGAdot - ell.angVelocity();
      duv = dlk*(1.e0+ 2.e0 * (Cus*c2al - Cuc*s2al) );
      drv = A * lecc * dek * sinea - 2.e0 * dlk *
         ( Crc * s2al - Crs * c2al );

      dxp = drv*cosu - R*sinu*duv;
      dyp = drv*sinu + R*cosu*duv;

         // Calculate velocities
      vxef = dxp*can - xip*san*domk - dyp*cinc*san
         + yip*( sinc*san*div - cinc*can*domk);
      vyef = dxp*san + xip*can*domk + dyp*cinc*can
         - yip*( sinc*can*div + cinc*san*domk);
      vzef = dyp*sinc + yip*cinc*div;

         // Move results into output variables
      sv.v[0] = vxef;
      sv.v[1] = vyef;
      sv.v[2] = vzef;

      return sv;
   }

   double BrcKeplerOrbit::svRelativity(const CommonTime& t) const
      throw( InvalidRequest )
   {
      GPSEllipsoid ell;
      double twoPI  = 2.0e0 * PI;
      double sqrtgm = SQRT(ell.gm());
      double elapte = t - getOrbitEpoch();
      double amm    = (sqrtgm / (A*Ahalf)) + dn;
      double meana,F,G,delea;
      
      meana = M0 + elapte * amm; 
      meana = fmod(meana, twoPI);
      double ea = meana + ecc * ::sin(meana);

      int loop_cnt = 1;
      do {
         F     = meana - ( ea - ecc * ::sin(ea));
         G     = 1.0 - ecc * ::cos(ea);
         delea = F/G;
         ea    = ea + delea;
         loop_cnt++;
      } while ( (ABS(delea) > 1.0e-11 ) && (loop_cnt <= 20) );
      double dtr = REL_CONST * ecc * Ahalf * ::sin(ea);
      return dtr;
   }

   CommonTime BrcKeplerOrbit::getOrbitEpoch() const
      throw(InvalidRequest)
   {
      return Toe;
   }
   
   CommonTime BrcKeplerOrbit::getBeginningOfFitInterval() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {   
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return beginFit;
   }

   CommonTime BrcKeplerOrbit::getEndOfFitInterval() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {   
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return endFit;
   }

   short BrcKeplerOrbit::getPRNID() const
      throw(InvalidRequest)
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return PRNID;
   }   
  
   short BrcKeplerOrbit::getFullWeek()  const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      GPSWeekSecond gpsws(Toe);
      return (gpsws.week);
   }   
  
   double BrcKeplerOrbit::getAccuracy()  const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      double accuracy = ura2CNAVaccuracy(URAoe);
      return accuracy;
   }   

   void BrcKeplerOrbit::setAccuracy(const double& acc)
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      URAoe = accuracy2ura(acc);
   }

   short BrcKeplerOrbit::getURAoe() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return URAoe;
   }
         
   double BrcKeplerOrbit::getCus() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Cus;
   }
   
   double BrcKeplerOrbit::getCrs() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Crs;
   }
   
   double BrcKeplerOrbit::getCis() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Cis;
   }
   
   double BrcKeplerOrbit::getCrc() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Crc;
   }
   
   double BrcKeplerOrbit::getCuc() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Cuc;
   }
  
   double BrcKeplerOrbit::getCic() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Cic;
   }
   
   double BrcKeplerOrbit::getToe() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      GPSWeekSecond gpsws(Toe);
      return gpsws.sow;
   }
   
   double BrcKeplerOrbit::getM0() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return M0;
   }
   
   double BrcKeplerOrbit::getDn() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return dn;
   }
   
   double BrcKeplerOrbit::getEcc() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return ecc;
   }
      
   double BrcKeplerOrbit::getA() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return A;
   }

   double BrcKeplerOrbit::getAhalf() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Ahalf;
   }

   double BrcKeplerOrbit::getAdot() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Adot;
   }

   double BrcKeplerOrbit::getDnDot() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return dndot;
   }
   
   double BrcKeplerOrbit::getOmega0() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return OMEGA0;
   }
   
   double BrcKeplerOrbit::getI0() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return i0;
   }
   
   double BrcKeplerOrbit::getW() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return w;
   }
   
   double BrcKeplerOrbit::getOmegaDot() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return OMEGAdot;
   }
   
   double BrcKeplerOrbit::getIDot() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return idot;
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
      
      SOW  = static_cast<long>( HOW );
      DOW  = static_cast<short>( SOW / SEC_PER_DAY );
      SOD  = SOW - static_cast<long>( DOW * SEC_PER_DAY );
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
   void BrcKeplerOrbit::dump(ostream& s) const
      throw()
   {
      ios::fmtflags oldFlags = s.flags();
#pragma unused(oldFlags)
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');
      
      s << "****************************************************************"
        << "************" << endl
        << "Broadcast Ephemeris (Engineering Units)" << endl
        << endl
        << "PRN : " << setw(2) << PRNID << endl
        << endl;
  
      s << "              Week(10bt)     SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n";
      
      s << endl;
      s << "Eph Epoch:    ";
      timeDisplay(s, getOrbitEpoch());
      s << endl;

      s.setf(ios::scientific, ios::floatfield);
      s.precision(8);
       
      s << endl
        << "           ORBIT PARAMETERS"
        << endl
        << endl
        << "Semi-major axis:       " << setw(16) << Ahalf  << " m**.5" << endl
        << "Motion correction:     " << setw(16) << dn     << " rad/sec"
        << endl
        << "Eccentricity:          " << setw(16) << ecc    << endl
        << "Arg of perigee:        " << setw(16) << w      << " rad" << endl
        << "Mean anomaly at epoch: " << setw(16) << M0     << " rad" << endl
        << "Right ascension:       " << setw(16) << OMEGA0 << " rad    "
        << setw(16) << OMEGAdot << " rad/sec" << endl
        << "Inclination:           " << setw(16) << i0     << " rad    "
        << setw(16) << idot     << " rad/sec" << endl;
      
      s << endl
        << "           HARMONIC CORRECTIONS"
        << endl
        << endl
        << "Radial        Sine: " << setw(16) << Crs << " m    Cosine: "
        << setw(16) << Crc << " m" << endl
        << "Inclination   Sine: " << setw(16) << Cis << " rad  Cosine: "
        << setw(16) << Cic << " rad" << endl
        << "In-track      Sine: " << setw(16) << Cus << " rad  Cosine: "
        << setw(16) << Cuc << " rad" << endl;    
      
      s << endl;
      
   } // end of BrcKeplerOrbit::dump()
   
   ostream& operator<<(ostream& s, const BrcKeplerOrbit& eph)
   {
      eph.dump(s);
      return s;

   } // end of operator<<
     
} // namespace
