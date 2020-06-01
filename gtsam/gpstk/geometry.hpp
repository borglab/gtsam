#pragma ident "$Id$"



/**
 * @file geometry.hpp
 * Geometric constants (radian-degree conversion)
 */

#ifndef GPSTK_GEOMETRY_HPP
#define GPSTK_GEOMETRY_HPP

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
      /**
       * @addtogroup geodeticgroup
       * @name Geometric Constants
       */
      //@{
      //Miscellaneous Parameters

      /// Conversion Factor from degrees to radians (units: degrees^-1)
   static const double DEG_TO_RAD = 1.7453292519943e-2;

      /// Conversion Factor from radians to degrees (units: degrees)
   static const double RAD_TO_DEG = 57.295779513082;
      //@}
}

#endif
