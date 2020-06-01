#pragma ident "$Id$"

/**
 * @file Xvt.cpp
 * Position and velocity as Triples, clock bias and drift as doubles.
 */

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

#include <iostream>
#include "Xvt.hpp"

// Output operator for Xvt
// @param os output stream to which \c xvt is sent
// @param xvt Xvt that is sent to \c os
std::ostream& operator<<(std::ostream& os, const gpstk::Xvt& xvt) throw()
{
   os << "x:" << xvt.x
     << ", v:" << xvt.v
     << ", clk bias:" << xvt.clkbias
     << ", clk drift:" << xvt.clkdrift
     << ", relcorr:" << xvt.relcorr;
   return os;
}

// compute the relativity correction
double gpstk::Xvt::computeRelativityCorrection(void)
{
   relcorr = -2.0*( (x[0]/C_MPS)*(v[0]/C_MPS)
                   +(x[1]/C_MPS)*(v[1]/C_MPS)
                   +(x[2]/C_MPS)*(v[2]/C_MPS) );
   return relcorr;
}

// Function to find the range and position from a ground
// location, rxPos, to the spacecraft position (*this).x
// Use the pseudorange corrected for SV clock effects to get a
// rough time of flight (dt).  Account directly for Earth
// rotation, then compute a rough receiver bias by differencing
// the initial time of flight with the new estimate.  Then
// correct the rotation by a small amount.
double gpstk::Xvt::preciseRho(const Triple& rxPos,
                              const EllipsoidModel& ellips,
                              double correction) const 
   throw()
{
   // Compute initial time of flight estimate using the
   // geometric range at transmit time.  This fails to account
   // for the rotation of the earth, but should be good to
   // within about 40 m
   double sr1 = rxPos.slantRange(x);
   double dt = sr1 / ellips.c();

   // compute rotation angle in the time of signal transit
   double rotation_angle = -ellips.angVelocity() * dt;

   // rotate original GS coordinates to new values to correct for
   // rotation of ECEF frame
   // Ref: Satellite Geodesy, Gunter Seeber, 1993, pg 291  and the 
   // ICD-GPS-200 sheet 102 May 1993 version
   //   xnew[0]=xg[0]*cos(rotation_angle)-xg[1]*sin(rotation_angle);
   //   xnew[1]=xg[1]*cos(rotation_angle)+xg[0]*sin(rotation_angle);
   //   xnew[2]=xg[2];
   // since cosine and sine are small, approximate by the first
   // order terms in an expansion.
   gpstk::Triple xnew;
   for (int i = 0; i < 2; i++)
   {
      xnew[0] = x[0] - x[1] * rotation_angle;
      xnew[1] = x[1] + x[0] * rotation_angle;
      xnew[2] = x[2];

      // Compute geometric slant range from ground station to
      // the rotated new coord's
      sr1 = rxPos.slantRange(xnew);
   
      // Recompute the time of flight (dt) based on PR, with the
      // time of flight based on geometric range.  Note that
      // this is a really unneeded, in that the change in PR is
      // < 40 m, hence the change in tof is < 20 ns
      dt = sr1 / ellips.c();
   
      // Compute new rotation in this time 
      rotation_angle = -ellips.angVelocity() * dt;  
   }

   // Account for SV clock drift, relativity and user input correction
   double rho = sr1 - (clkbias + relcorr) * ellips.c() - correction;

   return rho;

} // end of preciseRho()

