#pragma ident "$Id$"

/**
 * @file Xvt.hpp
 * Position and velocity as Triples, clock bias and drift as doubles.
 */

#ifndef GPSTK_XVT_INCLUDE
#define GPSTK_XVT_INCLUDE

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
#include "Triple.hpp"
#include "EllipsoidModel.hpp"
#include "ReferenceFrame.hpp"
#include "GNSSconstants.hpp"

namespace gpstk
{
   /** @addtogroup geodeticgroup */
   //@{

   /// Earth-Centered, Earth-Fixed Cartesian position, velocity, clock bias and drift
   class Xvt
   {
   public:

      /// Default constructor
      Xvt() : x(0.,0.,0.), v(0.,0.,0.),
              clkbias(0.), clkdrift(0.),
              relcorr(0.), frame(ReferenceFrame::Unknown)
         {};

      /// Destructor.
      virtual ~Xvt()
         {}

      /// access the position, ECEF Cartesian in meters
      Triple getPos() throw()
         { return x; }

      /// access the velocity in m/s
      Triple getVel() throw()
         { return v; }

      /// access the clock bias, in second
      double getClockBias() throw()
         { return clkbias; }

      /// access the clock drift, in second/second
      double getClockDrift() throw()
         { return clkdrift; }

      /// access the relativity correction, in seconds
      double getRelativityCorr() throw()
         { return relcorr; }

      /// Compute and return the relativity correction (-2R dot V/c^2) in seconds
      /// NB -2*dot(R,V)/(c*c) = -4.4428e-10(s/sqrt(m)) * ecc * sqrt(A(m)) * sinE
      double computeRelativityCorrection(void);

      /// Given the position of a ground location, compute the range
      /// to the spacecraft position.
      /// @param rxPos ground position at broadcast time in ECEF.
      /// @param ellipsoid geodetic parameters.
      /// @param correction offset in meters (include any factors other
      /// than the SV clock correction and the relativity correction).
      /// @return Range in meters
      double preciseRho(const Triple& rxPos, 
                        const EllipsoidModel& ellipsoid,
                        double correction = 0) const throw();

      // member data

      Triple x;        ///< Sat position ECEF Cartesian (X,Y,Z) meters
      Triple v;        ///< satellite velocity in ECEF Cartesian, meters/second
      double clkbias;  ///< Sat clock correction in seconds
      double clkdrift; ///< satellite clock drift in seconds/second
      double relcorr;  ///< relativity correction (standard 2R.V/c^2 term), seconds
      ReferenceFrame frame;   ///< reference frame of this data

   }; // end class Xvt

   //@}

}  // end namespace gpstk

/**
 * Output operator for Xvt
 * @param os output stream to which \c xvt is sent
 * @param xvt Xvt that is sent to \c os
 */
std::ostream& operator<<(std::ostream& os, const gpstk::Xvt& xvt) throw();

#endif // GPSTK_XVT_INCLUDE
