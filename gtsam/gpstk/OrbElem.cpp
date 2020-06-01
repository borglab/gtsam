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
 * @file OrbElem.cpp
 * OrbElem data encapsulated in engineering terms
 */

#include "OrbElem.hpp"
#include "StringUtils.hpp"
#include "GPSEllipsoid.hpp"
#include "YDSTime.hpp"
#include "CivilTime.hpp"
#include "TimeSystem.hpp"
#include "TimeString.hpp"
#include "MathBase.hpp"


namespace gpstk
{
   using namespace std;
   using namespace gpstk;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
    OrbElem::OrbElem()
      :dataLoadedFlag(false),
       ctToe(CommonTime::BEGINNING_OF_TIME),
       ctToc(CommonTime::BEGINNING_OF_TIME),
       beginValid(CommonTime::BEGINNING_OF_TIME),
       endValid(CommonTime::BEGINNING_OF_TIME),
       healthy(false)
   {
      ctToe.setTimeSystem(TimeSystem::GPS);
      ctToc.setTimeSystem(TimeSystem::GPS);
      beginValid.setTimeSystem(TimeSystem::GPS);
      endValid.setTimeSystem(TimeSystem::GPS);
   }
#pragma clang diagnostic pop

    bool OrbElem::isValid(const CommonTime& ct) const
      throw(InvalidRequest)
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      if (ct >= beginValid && ct <= endValid) return(true);
      return(false);
   }


   bool OrbElem::dataLoaded() const
   {
      return(dataLoadedFlag);
   }

   bool OrbElem::isHealthy() const
      throw(InvalidRequest)
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return(healthy);
   }

   double OrbElem::svClockBias(const CommonTime& t) const
      throw(gpstk::InvalidRequest)
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      double dtc,elaptc;
      elaptc = t - ctToc;
      dtc = af0 + elaptc * ( af1 + elaptc * af2 );
      return dtc;
   }

    double OrbElem::svClockBiasM(const CommonTime& t) const
      throw(gpstk::InvalidRequest)
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      double ret = svClockBias(t);
      ret = ret*C_MPS;
      return (ret);
   }

   double OrbElem::svClockDrift(const CommonTime& t) const
      throw(gpstk::InvalidRequest)
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      double drift,elaptc;
      elaptc = t - ctToc;
      drift = af1 + elaptc * af2;
      return drift;
   }

      // Implements equations of motion as defined in IS-GPS-200.
      // This code has its origins in 1980's FORTRAN code that has
      // been ported to C, then C++, then became part of the toolkit.
      // The original code was based on IS-GPS-200 Table 20-IV.
      // In July 2013, the code was modified to conform to Table 30-II
      // which includes additional time-dependent terms (A(dot) 
      // and delta n(dot)) that are in CNAV but not in LNAV.  These
      // changes should be backward compatible with LNAV as long as the 
      // Adot and dndot variables are appropriately set to 0.0 by the 
      // LNAV loaders. 
   Xvt OrbElem::svXvt(const CommonTime& t) const
      throw(InvalidRequest)
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      Xvt sv;

      GPSWeekSecond gpsws = (ctToe);
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

         // Compute time since ephemeris & clock epochs
      elapte = t - ctToe;

      double sqrtgm = SQRT(ell.gm());

         // Compute A at time of interest
      double Ak = A + Adot * elapte;

      double twoPI = 2.0e0 * PI;
      double lecc;               // eccentricity
      double tdrinc;            // dt inclination

      lecc = ecc;
      tdrinc = idot;

         // Compute mean motion
      double dnA = dn + 0.5 * dndot * elapte;
      double Ahalf = SQRT(A); 
      amm  = (sqrtgm / (A*Ahalf)) + dnA;  // NOT Ak because this equation
                                          // specifies A0, not Ak.  

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

      // Compute clock corrections
      sv.relcorr = svRelativity(t);
      sv.clkbias = svClockBias(t);
      sv.clkdrift = svClockDrift(t);
      sv.frame = ReferenceFrame::WGS84;

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
      R    = Ak*G  + dr;
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
      dek = amm * Ak / R;            
      dlk = SQRT(Ak) * q * sqrtgm / (R*R);  
      div = tdrinc - 2.0e0 * dlk *
         ( Cic  * s2al - Cis * c2al );
      domk = OMEGAdot - ell.angVelocity();
      duv = dlk*(1.e0+ 2.e0 * (Cus*c2al - Cuc*s2al) );
      drv = Ak * lecc * dek * sinea - 2.e0 * dlk *
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

     double OrbElem::svRelativity(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      GPSEllipsoid ell;
      double twoPI  = 2.0e0 * PI;
      double sqrtgm = SQRT(ell.gm());
      double elapte = t - ctToe;
      
         // Compute A at time of interest
      double Ak = A + Adot * elapte;   
      
      double dnA = dn + 0.5 * dndot * elapte;
      double Ahalf = SQRT(A);
      double amm    = (sqrtgm / (A*Ahalf)) + dnA;// NOT Ak because this equation
                                                 // specifies A0, not Ak.  
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
      
         // Use Ak as we are interested in semi-major axis at this moment.
      double dtr = REL_CONST * ecc * SQRT(Ak) * ::sin(ea); 
      return dtr;
   }

   void OrbElem::shortcut(ostream & os, const long HOW )
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


   void OrbElem::timeDisplay( ostream & os, const CommonTime& t )
   {
      os.setf(ios::dec, ios::basefield);
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
      os << printTime(t,"   %3j   %5.0s   %02m/%02d/%04Y   %02H:%02M:%02S");
   }

   void OrbElem::dumpBody(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      ios::fmtflags oldFlags = s.flags();
#pragma unused(oldFlags)
       
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      s << endl;
      s << "           TIMES OF INTEREST"
        << endl << endl;
      s << "              Week(10bt)     SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n";
      s << "Begin Valid:  ";
      timeDisplay(s, beginValid);
      s << endl;
      s << "Clock Epoch:  ";
      timeDisplay(s, ctToc);
      s << endl;
      s << "Eph Epoch:    ";
      timeDisplay(s, ctToe);
      s << endl;
      s << "End Valid:    ";
      timeDisplay(s, endValid);

      s.setf(ios::scientific, ios::floatfield);
      s.precision(8);
      s.fill(' ');

      s << endl
        << endl
        << "           CLOCK PARAMETERS"
        << endl
        << endl
        << "Bias T0:     " << setw(16) << af0 << " sec" << endl
        << "Drift:       " << setw(16) << af1 << " sec/sec" << endl
        << "Drift rate:  " << setw(16) << af2 << " sec/(sec**2)" << endl;

      s << endl
        << "           ORBIT PARAMETERS"
        << endl
        << endl
        << "Semi-major axis:       " << setw(16) <<  A     << " m       "
        << setw(16) << Adot  << "   m/sec" << endl
        << "Motion correction:     " << setw(16) <<  dn    << " rad/sec "
        << setw(16) << dndot << " rad/(sec**2)" << endl
        << "Eccentricity:          " << setw(16) << ecc << endl
        << "Arg of perigee:        " << setw(16) << w << " rad" << endl
        << "Mean anomaly at epoch: " << setw(16) << M0 << " rad" << endl
        << "Right ascension:       " << setw(16) << OMEGA0 << " rad     "
        << setw(16) << OMEGAdot << " rad/sec" << endl
        << "Inclination:           " << setw(16) << i0     << " rad     "
        << setw(16) << idot << " rad/sec" << endl;

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
   } // end of dumpBody()

   void OrbElem::dumpHeader(ostream& s) const
      throw( InvalidRequest )
   {
      s << "****************************************************************"
        << "************" << endl
        << "Broadcast Ephemeris (Engineering Units) - " << getNameLong();
      s << endl;

      SVNumXRef svNumXRef;
      int NAVSTARNum = 0;

      s << endl;
      s << "PRN : " << setw(2) << satID.id << " / "
        << "SVN : " << setw(2);
      try
      {
         NAVSTARNum = svNumXRef.getNAVSTAR(satID.id, ctToe );
         s << NAVSTARNum << "  ";
      }
      catch(NoNAVSTARNumberFound)
      {
         s << "XX";
      }
      s << endl
        << endl;
   }

   void OrbElem::dumpFooter(ostream& s) const
      throw( InvalidRequest )
   {}

   void OrbElem::dump(ostream& s) const
      throw( InvalidRequest )
   {
      dumpHeader(s);
      dumpBody(s);
      dumpFooter(s);
   }

} // namespace


















