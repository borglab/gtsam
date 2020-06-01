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
//  Copyright 2007, The University of Texas at Austin
//
//============================================================================




//
//
#ifndef GPSTK_NEDUTIL_HPP
#define GPSTK_NEDUTIL_HPP

// gpstk
#include "Triple.hpp"
#include "Matrix.hpp"
#include "Vector.hpp"
#include "Xvt.hpp"

namespace gpstk
{
    /** @addtogroup geodeticgroup */
    //@{

      /// A utility for converting from Cartesian in XZY to North-East-Down (NED)  
   class NEDUtil
   {
      public:
            // Constructors
          /**
          * Given a location as a (geodetic) latitude and longitude  
          * the constructor creates the appropriate rotation matrix 
          * from XYZ to NED and retains it for later use.
          * @param refGeodeticLatRad geodetic latitude of point of interest (radians)
          * @param refLonRad longitude of point of interest (radians).
          */
        NEDUtil( const double refGdLatRad,
                  const double refLonRad);
         
            // Methods
         /**
          * Convert from a vector in ECEF XYZ to ECEF NED using the
          * current rotation matrix.
          * @param inV,inVec,in vector of interest in ECEF XYZ.
          * @return Same type as input but with the vector in ECEF NED
          */
         gpstk::Vector<double> convertToNED( const gpstk::Vector<double>& inV ) const;
         gpstk::Triple         convertToNED( const gpstk::Triple& inVec ) const;
         gpstk::Xvt            convertToNED( const gpstk::Xvt& in ) const;
         
         /**
          * Update the rotation matrix to the new location without creating
          * a new object
          * @param refGdLatRad geodetic latitude of point of interest (radians)
          * @param refLonRad longitude of point of interest (radians).
          */
         void                  updatePosition( const double refLatRad,
                                               const double refLonRad );
                                  
            // Utilities
      protected:
         void compute( const double refLat,
                       const double refLon);
                       
         Matrix<double> rotMat;
   };

   //@}

}   
#endif      
