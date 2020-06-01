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
//
//

// gpstk
#include "ENUUtil.hpp"

namespace gpstk
{

//using namespace std; 

ENUUtil::ENUUtil(const double refLatRad,
                 const double refLonRad)
{
   compute( refLatRad, refLonRad ); 
}

//
//
void ENUUtil::compute( const double refLat,
                       const double refLon )
{
   rotMat.resize(3,3);
   rotMat (0,0) =  -std::sin(refLon);
   rotMat (1,0) =  -std::sin(refLat)*std::cos(refLon);
   rotMat (2,0) =   std::cos(refLat)*std::cos(refLon);
   rotMat (0,1) =   std::cos(refLon);
   rotMat (1,1) =  -std::sin(refLat)*std::sin(refLon);
   rotMat (2,1) =   std::cos(refLat)*std::sin(refLon);
   rotMat (0,2) =   0.0;
   rotMat (1,2) =   std::cos(refLat);
   rotMat (2,2) =   std::sin(refLat);
}

void ENUUtil::updatePosition( const double refLatRad,
                              const double refLonRad )
{
   compute( refLatRad, refLonRad );
}


gpstk::Vector<double> ENUUtil::convertToENU( const gpstk::Vector<double>& inV ) const
{
   gpstk::Vector<double> outV(3);

   if (inV.size()!=3)
   {
      gpstk::Exception e("Incompatible dimensions for Vector");
      GPSTK_THROW(e);
   }
   outV = rotMat * inV;
   return(outV);
}
   
gpstk::Triple ENUUtil::convertToENU( const gpstk::Triple& inVec ) const
{
   gpstk::Vector<double> v(3); 
   v[0] = inVec[0];
   v[1] = inVec[1];
   v[2] = inVec[2];
   
   gpstk::Vector<double> vOut = convertToENU( v );
   gpstk::Triple outVec( vOut[0], vOut[1], vOut[2] );
   return(outVec);
}
   
gpstk::Xvt ENUUtil::convertToENU( const gpstk::Xvt& in ) const
{
   gpstk::Xvt out;
   out.clkbias = in.clkbias;
   out.relcorr = in.relcorr;
   out.clkdrift = in.clkdrift;
   out.x = convertToENU( in.x );
   out.v = convertToENU( in.v );
   return(out);
}
}     // end namespace gpstk
