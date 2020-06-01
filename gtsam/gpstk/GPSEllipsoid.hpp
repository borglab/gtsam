#pragma ident "$Id$"
/**
 * @file GPSEllipsoid.hpp
 * The GPS Ellipsoid defined in ICD-GPS-200
 */

#ifndef GPSTK_GPSELLIPSOID_HPP
#define GPSTK_GPSELLIPSOID_HPP

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

#include "WGS84Ellipsoid.hpp"
#include "GNSSconstants.hpp"

namespace gpstk
{
       /** @addtogroup geodeticgroup */
       //@{

      /**
       * This class represents the ellipsoidal model defined in
       * ICD-GPS-200.  This ellipsoid model was based on the WGS 84
       * model, which has changed since the ICD-GPS-200 was written.
       * To save the cost of having to update all of the fielded
       * receivers, these constants were kept for GPS purposes.
       *
       * Only some of the WGS 84 ellipsoid model parameters are defined
       * as part of the 200.  These parameters are explicitly defined
       * here.  Other parameters are inherited from the WGS84Ellipsoid
       * definition.
       */
   class GPSEllipsoid : public WGS84Ellipsoid
   {
   public:
         /// defined in ICD-GPS-200C, 20.3.3.4.3.3 and Table 20-IV
         /// @return angular velocity of Earth in radians/sec.
      virtual double angVelocity() const throw()
      { return 7.2921151467e-5; }

         /// defined in ICD-GPS-200C, 20.3.3.4.3.3 and Table 20-IV
         /// @return geocentric gravitational constant in m**3 / s**2
      virtual double gm() const throw()
      { return 3.986005e14; }

         /// derived from ICD-GPS-200C, 20.3.3.4.3.3 and Table 20-IV
         /// @return geocentric gravitational constant in km**3 / s**2
      virtual double gm_km() const throw()
      { return 3.9860034e5; }

         /// defined in ICD-GPS-200C, 20.3.4.3
         /// @return Speed of light in m/s.
      virtual double c() const throw()
      { return C_MPS; }

         /// derived from ICD-GPS-200C, 20.3.4.3
         /// @return Speed of light in km/s
      virtual double c_km() const throw()
      { return (C_MPS / 1000); }

   }; // class GPSEllipsoid

   //@}

} // namespace

#endif
