#pragma ident "$Id$"



/**
 * @file EllipsoidModel.hpp
 * Abstract base class modeling a Ellipsoid
 */

#ifndef GPSTK_ELLIPSOIDMODEL_HPP
#define GPSTK_ELLIPSOIDMODEL_HPP

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



namespace gpstk
{
      /** @addtogroup geodeticgroup */
      //@{

      /**
       * This abstract class encapsulates ellipsoid models (e.g. WGS84,
       * GPS, etc).
       */
   class EllipsoidModel
   {
   public:
         /// @return semi-major axis of Earth in meters.
      virtual double a() const throw() = 0;

         /// @return semi-major axis of Earth in km.
      virtual double a_km() const throw() = 0;

         /// @return flattening (ellipsoid parameter).
      virtual double flattening() const throw() = 0;

         /// @return eccentricity (ellipsoid parameter).
      virtual double eccentricity() const throw() = 0;

         /// @return eccentricity squared (ellipsoid parameter).
      virtual double eccSquared() const throw()
      { return eccentricity() * eccentricity(); }

         /// @return angular velocity of Earth in radians/sec.
      virtual double angVelocity() const throw() = 0;

         /// @return geocentric gravitational constant in m**3 / s**2
      virtual double gm() const throw() = 0;

         /// @return geocentric gravitational constant in m**3 / s**2
      virtual double gm_km() const throw() = 0;

         /// @return Speed of light in m/s.
      virtual double c() const throw() = 0;

         /// @return Speed of light in km/s
      virtual double c_km() const throw() = 0;

      /// Destructor.
      virtual ~EllipsoidModel() throw() {};

   }; // class EllipsoidModel

   //@}

} // namespace

#endif
