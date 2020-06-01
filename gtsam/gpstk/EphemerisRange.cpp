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
 * @file EphemerisRange.cpp
 * Computation of range and associated quantities from EphemerisStore,
 * given receiver position and time.
 */

#include "EphemerisRange.hpp"
#include "MiscMath.hpp"
#include "GPSEllipsoid.hpp"
#include "GNSSconstants.hpp"
#include "geometry.hpp"

using namespace std;
using namespace gpstk;

namespace gpstk
{
   // Compute the corrected range at RECEIVE time, from receiver at position Rx,
   // to the GPS satellite given by SatID sat, as well as all the CER quantities,
   // given the nominal receive time tr_nom and an EphemerisStore. Note that this
   // routine does not intrinsicly account for the receiver clock error
   // like the ComputeAtTransmitTime routine does.
   double CorrectedEphemerisRange::ComputeAtReceiveTime(
      const CommonTime& tr_nom,
      const Position& Rx,
      const SatID sat,
      const XvtStore<SatID>& Eph)
   {
      try {
         int nit;
         double tof,tof_old;
         GPSEllipsoid ellipsoid;

         nit = 0;
         tof = 0.07;       // initial guess 70ms
         do {
            // best estimate of transmit time
            transmit = tr_nom;
            transmit -= tof;
            tof_old = tof;
            // get SV position
            try {
               svPosVel = Eph.getXvt(sat, transmit);
            }
            catch(InvalidRequest& e) {
               GPSTK_RETHROW(e);
            }

            rotateEarth(Rx);
            // update raw range and time of flight
            rawrange = RSS(svPosVel.x[0]-Rx.X(),
                           svPosVel.x[1]-Rx.Y(),
                           svPosVel.x[2]-Rx.Z());
            tof = rawrange/ellipsoid.c();

         } while(ABS(tof-tof_old)>1.e-13 && ++nit<5);

         updateCER(Rx);

         return (rawrange-svclkbias-relativity);
      }
      catch(gpstk::Exception& e) {
         GPSTK_RETHROW(e);
      }
   }  // end CorrectedEphemerisRange::ComputeAtReceiveTime


      // Compute the corrected range at TRANSMIT time, from receiver at position Rx,
      // to the GPS satellite given by SatID sat, as well as all the CER quantities,
      // given the nominal receive time tr_nom and an EphemerisStore, as well as
      // the raw measured pseudorange.
   double CorrectedEphemerisRange::ComputeAtTransmitTime(
      const CommonTime& tr_nom,
      const double& pr,
      const Position& Rx,
      const SatID sat,
      const XvtStore<SatID>& Eph)
   {
      try {
         CommonTime tt;

         // 0-th order estimate of transmit time = receiver - pseudorange/c
         transmit = tr_nom;
         transmit -= pr/C_MPS;
         tt = transmit;

         // correct for SV clock
         for(int i=0; i<2; i++) {
            // get SV position
            try {
               svPosVel = Eph.getXvt(sat,tt);
            }
            catch(InvalidRequest& e) {
               GPSTK_RETHROW(e);
            }
            tt = transmit;
            // remove clock bias and relativity correction
            tt -= (svPosVel.clkbias + svPosVel.relcorr);
         }

         rotateEarth(Rx);
         // raw range
         rawrange = RSS(svPosVel.x[0]-Rx.X(),
                        svPosVel.x[1]-Rx.Y(),
                        svPosVel.x[2]-Rx.Z());

         updateCER(Rx);

         return (rawrange-svclkbias-relativity);
      }
      catch(gpstk::Exception& e) {
         GPSTK_RETHROW(e);
      }
   }  // end CorrectedEphemerisRange::ComputeAtTransmitTime


   double CorrectedEphemerisRange::ComputeAtTransmitTime(
      const CommonTime& tr_nom,
      const Position& Rx,
      const SatID sat,
      const XvtStore<SatID>& Eph)
   {
      try {
         gpstk::GPSEllipsoid gm;
         svPosVel = Eph.getXvt(sat, tr_nom);
         double pr = svPosVel.preciseRho(Rx, gm);
         return ComputeAtTransmitTime(tr_nom, pr, Rx, sat, Eph);
      }
      catch(gpstk::Exception& e) {
         GPSTK_RETHROW(e);
      }
   }


   double CorrectedEphemerisRange::ComputeAtTransmitSvTime(
      const CommonTime& tt_nom,
      const double& pr,
      const Position& rx,
      const SatID sat,
      const XvtStore<SatID>& eph)
   {
      try {
         svPosVel = eph.getXvt(sat, tt_nom);

         // compute rotation angle in the time of signal transit

         // While this is quite similiar to rotateEarth, its not the same
         // and jcl doesn't know which is really correct
         // BWT this uses the measured pseudorange, corrected for SV clock and
         // relativity, to compute the time of flight; rotateEarth uses the value
         // computed from the receiver position and the ephemeris. They should be
         // very nearly the same, and multiplying by angVel/c should make the angle
         // of rotation very nearly identical.
         GPSEllipsoid ell;
         double range(pr/ell.c() - svPosVel.clkbias - svPosVel.relcorr);
         double rotation_angle = -ell.angVelocity() * range;
         svPosVel.x[0] = svPosVel.x[0] - svPosVel.x[1] * rotation_angle;
         svPosVel.x[1] = svPosVel.x[1] + svPosVel.x[0] * rotation_angle;
         svPosVel.x[2] = svPosVel.x[2];

         rawrange =rx.slantRange(svPosVel.x);
         updateCER(rx);

         return rawrange - svclkbias - relativity;
      }
      catch (Exception& e) {
         GPSTK_RETHROW(e);
      }
   }


   void CorrectedEphemerisRange::updateCER(const Position& Rx)
   {
      relativity = svPosVel.computeRelativityCorrection() * C_MPS;

      svclkbias = svPosVel.clkbias * C_MPS;
      svclkdrift = svPosVel.clkdrift * C_MPS;

      cosines[0] = (Rx.X()-svPosVel.x[0])/rawrange;
      cosines[1] = (Rx.Y()-svPosVel.x[1])/rawrange;
      cosines[2] = (Rx.Z()-svPosVel.x[2])/rawrange;

      Position SV(svPosVel);
      elevation = Rx.elevation(SV);
      azimuth = Rx.azimuth(SV);
      elevationGeodetic = Rx.elevationGeodetic(SV);
      azimuthGeodetic = Rx.azimuthGeodetic(SV);
   }


   void CorrectedEphemerisRange::rotateEarth(const Position& Rx)
   {
      GPSEllipsoid ellipsoid;
      double tof = RSS(svPosVel.x[0]-Rx.X(),
                       svPosVel.x[1]-Rx.Y(),
                       svPosVel.x[2]-Rx.Z())/ellipsoid.c();
      double wt = ellipsoid.angVelocity()*tof;
      double sx =  ::cos(wt)*svPosVel.x[0] + ::sin(wt)*svPosVel.x[1];
      double sy = -::sin(wt)*svPosVel.x[0] + ::cos(wt)*svPosVel.x[1];
      svPosVel.x[0] = sx;
      svPosVel.x[1] = sy;
      sx =  ::cos(wt)*svPosVel.v[0] + ::sin(wt)*svPosVel.v[1];
      sy = -::sin(wt)*svPosVel.v[0] + ::cos(wt)*svPosVel.v[1];
      svPosVel.v[0] = sx;
      svPosVel.v[1] = sy;
   }


   double RelativityCorrection(const Xvt& svPosVel)
   {
      // relativity correction
      // dtr = -2*dot(R,V)/(c*c) = -4.4428e-10(s/sqrt(m)) * ecc * sqrt(A(m)) * sinE
      // compute it separately here, in units seconds.
      double dtr = ( -2.0 *( svPosVel.x[0] * svPosVel.v[0]
                             + svPosVel.x[1] * svPosVel.v[1]
                             + svPosVel.x[2] * svPosVel.v[2] ) / C_MPS ) / C_MPS;
      return dtr;
   }

}  // namespace gpstk
