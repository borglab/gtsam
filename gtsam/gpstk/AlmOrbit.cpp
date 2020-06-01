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
 * @file AlmOrbit.cpp
 * Encapsulate almanac data, and compute satellite orbit, etc.
 */

#include "GNSSconstants.hpp"
#include "GPSEllipsoid.hpp"
#include "AlmOrbit.hpp"
#include "GPSWeekSecond.hpp"
#include <cmath>

namespace gpstk
{
   AlmOrbit :: AlmOrbit() throw()
   {
      ecc = i_offset = OMEGAdot = Ahalf = OMEGA0 = w = M0 = AF0 = AF1 = 0.0;

      Toa = xmit_time = 0;

      week = SV_health = 0;
   }

   AlmOrbit :: AlmOrbit(short prn, double aEcc, double ai_offset,
                        double aOMEGAdot, double aAhalf, double aOMEGA0,
                        double aw, double aM0, double aAF0, double aAF1,
                        long aToa, long axmit_time, short aweek,
                        short aSV_health)
         : PRN(prn), ecc(aEcc), i_offset(ai_offset), OMEGAdot(aOMEGAdot), Ahalf(aAhalf),
           OMEGA0(aOMEGA0), w(aw), M0(aM0), AF0(aAF0), AF1(aAF1), Toa(aToa),
           xmit_time(axmit_time), week(aweek), SV_health(aSV_health) 
   {
   }

   Xvt AlmOrbit :: svXvt(const CommonTime& t) const
      throw(InvalidRequest)
   {
      Xvt sv;
      GPSEllipsoid ell;

      double elapt;                 /* elapsed time since Toa */
      double A;                     /* semi-major axis */
      double n;                       /* mean motion */
      double meana;                 /* mean anomoly */
      double ea;                    /* eccentric anomoly */
      short loop;                   /* counter */
      double f,g,delea,q,gsta,gcta; /* temp. variables */
      double dtc;                   /* corrected time */
      double ta;                    /* true anomoly */
      double sinea,cosea,sinu,cosu;
      double alat;                  /* arguement of latitude */
      double ualat;                 /* corrected arguement of latitude */
      double r;                     /* radius */
      double i;                     /* inclination */
      double anlon;                 /* corrected longitue of ascending node */
      double xip,yip,can,san,cinc,sinc,xef,yef,zef,dek,dlk,div,domk,duv,
         drv,dxp,dyp,vxef,vyef,vzef;
      double sqrtgm = ::sqrt(ell.gm());

/*   Compute time since Almanac epoch (Toa) including week change */
      elapt = t - getToaTime();

         /* compute mean motion from semi-major axis */
      A = Ahalf * Ahalf;
      n = sqrtgm / (Ahalf * A);

         /* compute the mean anomaly */
      meana = M0 + elapt * n;
      meana = ::fmod(meana, 2.0 * PI);

         /* compute eccentric anomaly by iteration */

      ea = meana + ecc * ::sin(meana);
      loop = 1;

      do {
         f = meana - (ea - ecc * ::sin(ea));
         g = 1.0 - ecc * ::cos(ea);
         delea = f / g;
         ea += delea;
         loop++;
      }  while ( ::fabs(delea) > 1.0e-11 && (loop <= 20));

         /* compute clock corrections (no relativistic correction computed) */
      dtc = AF0 + elapt * AF1;
      sv.clkbias = dtc;

         /* compute the true anomaly */
      q = ::sqrt (1.0e0 - ecc * ecc);
      sinea = ::sin(ea);
      cosea = ::cos(ea);
      gsta = q * sinea;
      gcta = cosea  - ecc;
      ta = ::atan2(gsta,gcta);

         /* compute argument of latitude for orbit */
      alat = ta + w;

         /* compute correction terms ( no pertubation ) */
      ualat = alat;
      r = A * (1.0 - ecc * cosea);
      i = i_offset + 0.3e0 * PI;

         /* compute corrected longitude of ascending node */
      anlon = OMEGA0 +
         (OMEGAdot - ell.angVelocity()) * elapt -
         ell.angVelocity() * (double)Toa;

         /* compute positions in orbital plane */
      cosu = ::cos(ualat);
      sinu = ::sin(ualat);
      xip = r * cosu;
      yip = r * sinu;

         /* compute earch fixed coordinates (in meters) */
      can = ::cos (anlon);
      san = ::sin (anlon);
      cinc = ::cos(i);
      sinc = ::sin(i);

      xef = xip * can - yip * cinc * san;
      yef = xip * san + yip * cinc * can;
      zef =             yip * sinc;

      sv.x[0] = xef;
      sv.x[1] = yef;
      sv.x[2] = zef;

         /* compute velocity of rotation coordinates & velocity of sat. */
      dek = n * A / r;
      dlk = sqrtgm * Ahalf * q / (r * r);
      div = 0.0e0;
      domk = OMEGAdot - ell.angVelocity();
      duv = dlk;
      drv = A * ecc * dek * sinea;

      dxp = drv * cosu - r * sinu * duv;
      dyp = drv * sinu + r * cosu * duv;

      vxef = dxp * can - xip * san * domk - dyp * cinc * san
         + yip * (sinc * san * div - cinc * can * domk);
      vyef = dxp * san + xip * can * domk + dyp * cinc * can
         - yip * (sinc * can * div + cinc * san * domk);
      vzef = dyp * sinc + yip * cinc * div;

      sv.v[0] = vxef;
      sv.v[1] = vyef;
      sv.v[2] = vzef;

      return sv;
   }

   CommonTime AlmOrbit::getTransmitTime() const throw()
   {
      return GPSWeekSecond(getFullWeek(), xmit_time);
   }

   short AlmOrbit::getFullWeek() const throw()
   {
         // return value of the transmit week for the given PRN
      short xmit_week = week;
      double sow_diff = (double)(Toa - xmit_time);
      if (sow_diff < -HALFWEEK)
         xmit_week--;
      else if (sow_diff > HALFWEEK)
         xmit_week++;

      return xmit_week;
   }

   CommonTime AlmOrbit::getToaTime() const throw()
   {
      return GPSWeekSecond(week, Toa);
   }

   void AlmOrbit::dump(std::ostream& s, int verbosity) const
   {
      using std::endl;
      using std::setw;

      s << std::setprecision(4);
      s.setf(std::ios::scientific);
      switch (verbosity)
      {
         case 0:
            s << PRN       << ", "
              << Toa       << ", "
              << week       << ", "
              << std::hex
              << SV_health << ", "
              << std::dec
              << AF0       << ", "
              << AF1       << ", "
              << ecc       << ", "
              << w         << ", "
              << Ahalf     << ", "
              << M0        << ", "
              << OMEGA0    << ", "
              << OMEGAdot  << ", "
              << i_offset
              << endl;
            break;

         case 1:
            s << "PRN:" << PRN
              << " Toa:" << Toa
              << " H:" << SV_health
              << " AFO:" << AF0
              << " AF1:" << AF1
              << " Ecc:" << ecc
              << endl
              << "   w:" << w
              << " Ahalf:" << Ahalf
              << " M0:" << M0
              << endl
              << "   OMEGA0:" << OMEGA0
              << " OMEGAdot:" << OMEGAdot
              << " Ioff:" << i_offset
              << endl;
            break;

         default:
            s << "PRN:                   " << PRN << endl
              << "Toa:                   " << Toa << endl
              << "xmit_time:             " << xmit_time << endl
              << "week:                  " << week << endl
              << "SV_health:             " << SV_health << endl
              << "AFO:                   " << setw(12) << AF0  << " sec" << endl
              << "AF1:                   " << setw(12) << AF1  << " sec/sec" << endl
              << "Sqrt A:                " << setw(12) << Ahalf  << " sqrt meters" << endl
              << "Eccentricity:          " << setw(12) << ecc    << endl
              << "Arg of perigee:        " << setw(12) << w      << " rad" << endl
              << "Mean anomaly at epoch: " << setw(12) << M0     << " rad" << endl
              << "Right ascension:       " << setw(12) << OMEGA0 << " rad    " << setw(16) << OMEGAdot << " rad/sec" << endl
              << "Inclination offset:    " << setw(12) << i_offset << " rad    " << endl;
      }
   }

   std::ostream& operator<<(std::ostream& s, const AlmOrbit& ao)
   {
      ao.dump(s);
      return s;
   }

} // namespace
