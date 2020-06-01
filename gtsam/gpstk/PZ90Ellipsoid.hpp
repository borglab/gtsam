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
 * @file PZ90Ellipsoid.hpp
 * PZ90.02 model of the Ellipsoid (as defined in table 3.2 of ICD-2008, v5.1).
 */

#ifndef GPSTK_PZ90ELLIPSOID_HPP
#define GPSTK_PZ90ELLIPSOID_HPP

#include "EllipsoidModel.hpp"

namespace gpstk
{
    /** @addtogroup geodeticgroup */
    //@{

   class PZ90Ellipsoid : public EllipsoidModel
   {
   public:

   		///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return semi-major axis of Earth in meters.
      virtual double a() const throw()
      { return 6378136.0; }
		 
         ///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return semi-major axis of Earth in km.
      virtual double a_km() const throw()
      { return a() / 1000.0; }

         /**
          * Defined in table 3.2 of the GLONASS ICD-2008 (v5.1)
          * @return inverse o flattening (ellipsoid parameter).
          */
      virtual double flatteningInverse() const throw()
      { return 298.25784; }

         /**
          * Computed from inverse flattening value as given in table 3.2
          * of the GLONASS ICD-2008 (v5.1)
          * @return flattening (ellipsoid parameter).
          */
      virtual double flattening() const throw()
      { return 3.35280373518e-3; }
     
         // The eccentricity and eccSquared values were computed from the
         // flattening value via the formula:
         // ecc2 = 1 - (1 - f)^2 = f*(2.0 - f)
         // ecc = sqrt(ecc2)
         /// @return eccentricity (ellipsoid parameter).
      virtual double eccentricity() const throw()
      { return 8.1819106432923e-2; }

         /// @return eccentricity squared (ellipsoid parameter).
      virtual double eccSquared() const throw()
      { return 6.69436617748e-3; }

         ///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return angular velocity of Earth in radians/sec.
      virtual double angVelocity() const throw()
      { return 7.292115e-5; }

         ///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return geocentric gravitational constant in m**3 / s**2
      virtual double gm() const throw()
      { return 398600.4418e9; }

         ///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return geocentric gravitational constant in km**3 / s**2
      virtual double gm_km() const throw()
      { return 398600.4418; }

         ///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return Speed of light in m/s.
      virtual double c() const throw()
      { return 299792458; }

         ///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return Speed of light in km/s
      virtual double c_km() const throw()
      { return c()/1000.0; }

         ///Defined in table 3.2 of ICD-2008 (v5.1)
         /// @return Returns second zonal harmonic of the geopotential.
      virtual double j20() const throw()
      { return (-1.08262575e-3); }

         /// Destructor.
      virtual ~PZ90Ellipsoid() throw() {};

   }; // End of class 'PZ90Ellipsoid'

   //@}

}  // End of namespace gpstk

#endif   // GPSTK_PZ90ELLIPSOID_HPP
