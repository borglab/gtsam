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
 * @file WGS84Ellipsoid.hpp
 * WGS 1984 model of the Ellipsoid
 */

#ifndef GPSTK_WGS84ELLIPSOID_HPP
#define GPSTK_WGS84ELLIPSOID_HPP

#include "EllipsoidModel.hpp"

namespace gpstk
{
    /** @addtogroup geodeticgroup */
    //@{

      /// This class represents the ellipsoid model defined in NIMA
      /// TR8350.2, "Department of Defense World Geodetic System 1984".
   class WGS84Ellipsoid : public EllipsoidModel
   {
   public:
   
         /// Defined in TR8350.2, Appendix A.1
         /// @return semi-major axis of Earth in meters.
      virtual double a() const throw()
      { return 6378137.0; }

         /// Derived from TR8350.2, Appendix A.1
         /// @return semi-major axis of Earth in km.
      virtual double a_km() const throw()
      { return a() / 1000.0; }

         /**
          * Derived from TR8350.2, Appendix A.1
          * @return flattening (ellipsoid parameter).
          */
      virtual double flattening() const throw()
      { return 0.335281066475e-2; }

         /// Defined in TR8350.2, Table 3.3
         /// @return eccentricity (ellipsoid parameter).
      virtual double eccentricity() const throw()
      { return 8.1819190842622e-2; }

         /// Defined in TR8350.2, Table 3.3
         /// @return eccentricity squared (ellipsoid parameter).
      virtual double eccSquared() const throw()
      { return 6.69437999014e-3; }

         /// Defined in TR8350.2, 3.2.4 line 3-6, or Table 3.1
         /// @return angular velocity of Earth in radians/sec.
      virtual double angVelocity() const throw()
      { return 7.292115e-5; }

         /// Defined in TR8350.2, Table 3.1
         /// @return geocentric gravitational constant in m**3 / s**2
      virtual double gm() const throw()
      { return 3986004.418e8; }

         /// Derived from TR8350.2, Table 3.1
         /// @return geocentric gravitational constant in km**3 / s**2
      virtual double gm_km() const throw()
      { return 398600.4418; }

         /// Defined in TR8350.2, 3.3.2 line 3-11
         /// @return Speed of light in m/s.
      virtual double c() const throw()
      { return 299792458; }

         /// Derived from TR8350.2, 3.3.2 line 3-11
         /// @return Speed of light in km/s
      virtual double c_km() const throw()
      { return c()/1000.0; }

      /// Destructor.
      virtual ~WGS84Ellipsoid() throw() {};

   }; // class WGS84Ellipsoid

   //@}

} // namespace

#endif
