/// @file OrbElem.cpp Encapsulates the "least common denominator" orbit parameters
/// that determine a satellite ephemeris, that is, clock model, Kepler orbit elements
/// plus harmonic perturbations with time of ephemeris, satellite ID, and begin and
/// end times of validity.
/// Although it can also be used alone, this class is most often to be used as a base
/// class for a fuller implementation of the ephemeris and clock, by adding health
/// and accuracy information, fit interval, ionospheric correction terms and data
/// flags. It serves as the base class for broadcast ephemerides for GPS, QZSS,
/// Galileo, and BeiDou, with RINEX Navigation input, among others.

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

#include "OrbitEph.hpp"

#include "MathBase.hpp"
#include "GNSSconstants.hpp"
#include "CivilTime.hpp"
#include "GPSWeekSecond.hpp"
#include "GALWeekSecond.hpp"
#include "BDSWeekSecond.hpp"
#include "QZSWeekSecond.hpp"
#include "GPSEllipsoid.hpp"
#include "TimeString.hpp"

using namespace std;

namespace gpstk {

   // Returns true if the time, ct, is within the period of validity of
   // this OrbitEph object.
   // throw Invalid Request if the required data has not been stored.
   bool OrbitEph::isValid(const CommonTime& ct) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));
      if(ct < beginValid || ct > endValid) return false;
      return true;
   }

   // Compute the satellite clock bias (seconds) at the given time
   // throw Invalid Request if the required data has not been stored.
   double OrbitEph::svClockBias(const CommonTime& t) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      double dtc, elaptc;
      elaptc = t - ctToc;
      dtc = af0 + elaptc * (af1 + elaptc * af2);
      return dtc;
   }

   // Compute the satellite clock drift (sec/sec) at the given time
   // throw Invalid Request if the required data has not been stored.
   double OrbitEph::svClockDrift(const CommonTime& t) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      double drift, elaptc;
      elaptc = t - ctToc;
      drift = af1 + elaptc * af2;
      return drift;
   }

   // Compute satellite position at the given time.
   // throw Invalid Request if the required data has not been stored.
   Xvt OrbitEph::svXvt(const CommonTime& t) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      Xvt sv;
      double ea;              // eccentric anomaly
      double delea;           // delta eccentric anomaly during iteration
      double elapte;          // elapsed time since Toe
      double elaptc;          // elapsed time since Toc
      double dtc,dtr,q,sinea,cosea;
#pragma unused(elaptc,dtr,dtc)
       
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
      double twoPI = 2.0e0 * PI;
      double lecc;            // eccentricity
      double tdrinc;          // dt inclination
      double Ahalf = SQRT(A); // A is semi-major axis of orbit
      double ToeSOW = GPSWeekSecond(ctToe).sow;    // SOW is time-system-independent

      lecc = ecc;
      tdrinc = idot;

      // Compute time since ephemeris & clock epochs
      elapte = t - ctToe;

      // Compute A at time of interest (LNAV: Adot==0)
      double Ak = A + Adot * elapte;

      // Compute mean motion (LNAV: dndot==0)
      double dnA = dn + 0.5*dndot*elapte;
      amm  = (sqrtgm / (A*Ahalf)) + dnA;     // Eqn specifies A0, not Ak

      // In-plane angles
      //     meana - Mean anomaly
      //     ea    - Eccentric anomaly
      //     truea - True anomaly
      meana = M0 + elapte * amm;
      meana = fmod(meana, twoPI);
      ea = meana + lecc * ::sin(meana);

      int loop_cnt = 1;
      do  {
         F = meana - (ea - lecc * ::sin(ea));
         G = 1.0 - lecc * ::cos(ea);
         delea = F/G;
         ea = ea + delea;
         loop_cnt++;
      } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

      // Compute clock corrections
      sv.relcorr = svRelativity(t);
      sv.clkbias = svClockBias(t);
      sv.clkdrift = svClockDrift(t);
      sv.frame = ReferenceFrame::WGS84;

      // Compute true anomaly
      q     = SQRT(1.0e0 - lecc*lecc);
      sinea = ::sin(ea);
      cosea = ::cos(ea);
      G     = 1.0e0 - lecc * cosea;

      //  G*SIN(TA) AND G*COS(TA)
      GSTA  = q * sinea;
      GCTA  = cosea - lecc;

      //  True anomaly
      truea = atan2 (GSTA, GCTA);

      // Argument of lat and correction terms (2nd harmonic)
      alat  = truea + w;
      talat = 2.0e0 * alat;
      c2al  = ::cos(talat);
      s2al  = ::sin(talat);

      du  = c2al * Cuc +  s2al * Cus;
      dr  = c2al * Crc +  s2al * Crs;
      di  = c2al * Cic +  s2al * Cis;

      // U = updated argument of lat, R = radius, AINC = inclination
      U    = alat + du;
      R    = Ak*G  + dr;
      AINC = i0 + tdrinc * elapte  +  di;

      //  Longitude of ascending node (ANLON)
      ANLON = OMEGA0 + (OMEGAdot - ell.angVelocity()) *
              elapte - ell.angVelocity() * ToeSOW;

      // In plane location
      cosu = ::cos(U);
      sinu = ::sin(U);
      xip  = R * cosu;
      yip  = R * sinu;

      //  Angles for rotation to earth fixed
      can  = ::cos(ANLON);
      san  = ::sin(ANLON);
      cinc = ::cos(AINC);
      sinc = ::sin(AINC);

      // Earth fixed coordinates in meters
      xef  =  xip*can  -  yip*cinc*san;
      yef  =  xip*san  +  yip*cinc*can;
      zef  =              yip*sinc;
      sv.x[0] = xef;
      sv.x[1] = yef;
      sv.x[2] = zef;

      // Compute velocity of rotation coordinates
      dek = amm * Ak / R;
      dlk = Ahalf * q * sqrtgm / (R*R);
      div = tdrinc - 2.0e0 * dlk * (Cic  * s2al - Cis * c2al);
      domk = OMEGAdot - ell.angVelocity();
      duv = dlk*(1.e0+ 2.e0 * (Cus*c2al - Cuc*s2al));
      drv = Ak * lecc * dek * sinea - 2.e0 * dlk * (Crc * s2al - Crs * c2al);
      dxp = drv*cosu - R*sinu*duv;
      dyp = drv*sinu + R*cosu*duv;

      // Calculate velocities
      vxef = dxp*can - xip*san*domk - dyp*cinc*san
               + yip*(sinc*san*div - cinc*can*domk);
      vyef = dxp*san + xip*can*domk + dyp*cinc*can
               - yip*(sinc*can*div + cinc*san*domk);
      vzef = dyp*sinc + yip*cinc*div;

      // Move results into output variables
      sv.v[0] = vxef;
      sv.v[1] = vyef;
      sv.v[2] = vzef;

      return sv;
   }

   // Compute satellite relativity correction (sec) at the given time
   // throw Invalid Request if the required data has not been stored.
   double OrbitEph::svRelativity(const CommonTime& t) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      GPSEllipsoid ell;
      double twoPI  = 2.0 * PI;
      double sqrtgm = SQRT(ell.gm());
      double elapte = t - ctToe;

      // Compute A at time of interest
      double Ak = A + Adot*elapte;                 // LNAV: Adot==0
      double dnA = dn + 0.5*dndot*elapte;          // LNAV: dndot==0
#pragma unused(dnA)
       
      double Ahalf = SQRT(A);
      double amm = (sqrtgm / (A*Ahalf)) + dn;      // Eqn specifies A0 not Ak
      double meana,F,G,delea;

      meana = M0 + elapte * amm;
      meana = fmod(meana, twoPI);
      double ea = meana + ecc * ::sin(meana);

      int loop_cnt = 1;
      do {
         F     = meana - (ea - ecc * ::sin(ea));
         G     = 1.0 - ecc * ::cos(ea);
         delea = F/G;
         ea    = ea + delea;
         loop_cnt++;
      } while ((ABS(delea) > 1.0e-11) && (loop_cnt <= 20));

      return (REL_CONST * ecc * SQRT(Ak) * ::sin(ea));
   }

   // Dump the overhead information as a string containing a single line.
   // @throw Invalid Request if the required data has not been stored.
   string OrbitEph::asString(void) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      try {
         ostringstream os;
         string sys;
         switch(satID.system) {
            case SatID::systemGPS: sys = "G"; break;
            case SatID::systemGalileo: sys = "E"; break;
            case SatID::systemBeiDou: sys = "C"; break;
            case SatID::systemQZSS: sys = "J"; break;
            default:
               os << "EPH Error - invalid satellite system "
                  << SatID::convertSatelliteSystemToString(satID.system) << endl;
               return os.str();
         }

         CivilTime ct;
         os << "EPH " << sys << setfill('0') << setw(2) << satID.id << setfill(' ');
         ct = CivilTime(beginValid);
         os << printTime(ct," | %4Y %3j %02H:%02M:%02S |");
         ct = CivilTime(ctToe);
         os << printTime(ct," %3j %02H:%02M:%02S |");
         ct = CivilTime(ctToc);
         os << printTime(ct," %3j %02H:%02M:%02S |");
         ct = CivilTime(endValid);
         os << printTime(ct," %3j %02H:%02M:%02S |");

         return os.str();
      }
      catch(Exception& e) { GPSTK_RETHROW(e);
      }
   }

   // Utility routine for dump(); override if GPSWeekSecond is not right
   string OrbitEph::timeDisplay(const CommonTime& t, bool showHead) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      try {
         if(showHead) return string("Week( mod)     SOW     DOW   UTD     SOD"
                                    "   MM/DD/YYYY   HH:MM:SS SYS");

         ostringstream os;
         WeekSecond *ptr;
         if(     t.getTimeSystem() == TimeSystem::GAL)
            ptr = new GALWeekSecond(t);
         else if(t.getTimeSystem() == TimeSystem::BDT)
            ptr = new BDSWeekSecond(t);
         else if(t.getTimeSystem() == TimeSystem::QZS)
            ptr = new QZSWeekSecond(t);
         else 
            ptr = new GPSWeekSecond(t);

         os << setw(4) << ptr->week << "(";
         os << setw(4) << (ptr->week & ptr->bitmask()) << ")  ";
         os << setw(6) << setfill(' ') << ptr->sow << "   ";

         switch (ptr->getDayOfWeek())
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

         os << printTime(t,"   %3j   %5.0s   %02m/%02d/%04Y   %02H:%02M:%02S %P");

         return os.str();
      }
      catch(Exception& e) { GPSTK_RETHROW(e);
      }
   }

   // Dump the overhead information to the given output stream.
   // throw Invalid Request if the required data has not been stored.
   void OrbitEph::dumpHeader(ostream& os) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      os << "****************************************************************"
        << "************" << endl
        << "Broadcast Orbit Ephemeris of class " << getName() << endl;
      os << "Satellite: " << SatID::convertSatelliteSystemToString(satID.system)
         << " " << setfill('0') << setw(2) << satID.id << setfill(' ') << endl;
   }

   // Dump the orbit, etc information to the given output stream.
   // throw Invalid Request if the required data has not been stored.
   void OrbitEph::dumpBody(ostream& os) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));

      os << "           TIMES OF INTEREST" << endl;
      os << "              " << timeDisplay(beginValid,true) << endl;
      os << "Begin Valid:  " << timeDisplay(beginValid) << endl;
      os << "Clock Epoch:  " << timeDisplay(ctToc) << endl;
      os << "Eph Epoch:    " << timeDisplay(ctToe) << endl;
      os << "End Valid:    " << timeDisplay(endValid) << endl;

      os << scientific << setprecision(8)
         << "           CLOCK PARAMETERS\n"
         << "Bias T0:     " << setw(16) << af0 << " sec" << endl
         << "Drift:       " << setw(16) << af1 << " sec/sec" << endl
         << "Drift rate:  " << setw(16) << af2 << " sec/(sec**2)" << endl;

      os << "           ORBIT PARAMETERS\n"
         << "Semi-major axis:       " << setw(16) <<  A  << " m" << endl
         << "Motion correction:     " << setw(16) <<  dn << " rad/sec" << endl
         << "Eccentricity:          " << setw(16) << ecc << endl
         << "Arg of perigee:        " << setw(16) << w << " rad" << endl
         << "Mean anomaly at epoch: " << setw(16) << M0 << " rad" << endl
         << "Right ascension:       " << setw(16) << OMEGA0 << " rad    "
         << setw(16) << OMEGAdot << " rad/sec" << endl
         << "Inclination:           " << setw(16) << i0 << " rad    "
         << setw(16) << idot << " rad/sec" << endl;

      os << "           HARMONIC CORRECTIONS\n"
         << "Radial        Sine: " << setw(16) << Crs << " m    Cosine: "
         << setw(16) << Crc << " m" << endl
         << "Inclination   Sine: " << setw(16) << Cis << " rad  Cosine: "
         << setw(16) << Cic << " rad" << endl
         << "In-track      Sine: " << setw(16) << Cus << " rad  Cosine: "
         << setw(16) << Cuc << " rad" << endl;
   }

/*
   // Define this OrbitEph by converting the given RINEX navigation data.
   // NB this will be both overridden and called by the derived classes
   // NB currently has fixes for MGEX data.
   // @param rnd Rinex3NavData
   // @return true if OrbitEph was defined, false otherwise
   bool OrbitEph::load(const Rinex3NavData& rnd)
   {
      try {
         // Glonass and Geosync do not have a orbit-based ephemeris
         if(rnd.satSys == "R" || rnd.satSys == "S") return false;

         // first get times and TimeSytem
         CommonTime gpstoe = rnd.time;
         unsigned int year = static_cast<CivilTime>(gpstoe).year;

         // Get week for clock, to build Toc
         double dt = rnd.Toc - rnd.HOWtime;
         int week = rnd.weeknum;
         if(dt < -HALFWEEK) week++;
         else if(dt > HALFWEEK) week--;
      
         //MGEX NB MGEX data has GPS week numbers in all systems except BeiDou,
         //MGEX so must implement temporary fixes: use GPS Toc for GAL and QZSS
         CommonTime gpstoc = GPSWeekSecond(week, rnd.Toc, TimeSystem::GPS);   //MGEX
         //cout << "gpstoc is " << printTime(gpstoc,"%Y/%m/%d %H:%M:%S %P")
         //<< " year " << year << " week " << week << " rnd.Toc " << rnd.Toc <<endl;

         // based on satellite ID, define Toc with TimeSystem
         if(rnd.satSys == "G") {
            ctToc = GPSWeekSecond(week, rnd.Toc, TimeSystem::GPS);
            ctToc.setTimeSystem(TimeSystem::GPS);
         }
         else if(rnd.satSys == "E") {
            //MGEX GALWeekSecond galws(week, rnd.Toc, TimeSystem::GAL);
            //MGEX galws.adjustToYear(year);
            //MGEX ctToc = CommonTime(galws);
            ctToc = gpstoc;        //MGEX
            ctToc.setTimeSystem(TimeSystem::GAL);
         }
         else if(rnd.satSys == "C") {
            BDSWeekSecond bdsws(week, rnd.Toc, TimeSystem::BDT);
            bdsws.adjustToYear(year);
            ctToc = CommonTime(bdsws);
            ctToc.setTimeSystem(TimeSystem::BDT);
         }
         else if(rnd.satSys == "J") {
            //MGEX QZSWeekSecond qzsws(week, rnd.Toc, TimeSystem::BDT);
            //MGEX qzsws.adjustToYear(year);
            //MGEX ctToc = CommonTime(qzsws);
            ctToc = gpstoc;        //MGEX
            ctToc.setTimeSystem(TimeSystem::QZS);
         }
         else
            GPSTK_THROW(Exception("Unknown satellite system: " + rnd.satSys));

         //cout << "ctToc " << printTime(oeptr->ctToc,"%Y/%m/%d %H:%M:%S %P") << endl;

         // Overhead
         RinexSatID sat;
         sat.fromString(rnd.satSys + StringUtils::asString(rnd.PRNID));
         satID = SatID(sat);
         //obsID = ?? ObsID obsID; // Defines carrier and tracking code
         ctToe = rnd.time;

         // clock model
         af0 = rnd.af0;
         af1 = rnd.af1;
         af2 = rnd.af2;
   
         // Major orbit parameters
         M0 = rnd.M0;
         dn = rnd.dn;
         ecc = rnd.ecc;
         A = rnd.Ahalf * rnd.Ahalf;
         OMEGA0 = rnd.OMEGA0;
         i0 = rnd.i0;
         w = rnd.w;
         OMEGAdot = rnd.OMEGAdot;
         idot = rnd.idot;
         // modern nav msg
         dndot = 0.;
         Adot = 0.;
   
         // Harmonic perturbations
         Cuc = rnd.Cuc;
         Cus = rnd.Cus;
         Crc = rnd.Crc;
         Crs = rnd.Crs;
         Cic = rnd.Cic;
         Cis = rnd.Cis;
   
         dataLoadedFlag = true;
         adjustValidity();

         return true;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }
*/

   // Output object to stream
   ostream& operator<<(ostream& os, const OrbitEph& eph)
   {
      eph.dump(os);
      return os;
   }

}  // end namespace
