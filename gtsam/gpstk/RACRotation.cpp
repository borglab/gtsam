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




//
//
//#include <stdio.h>

// gpstk
#include "RACRotation.hpp"

namespace gpstk
{

//using namespace std;

RACRotation::RACRotation( const gpstk::Triple& SVPositionVector,
                          const gpstk::Triple& SVVelocityVector)
                          : gpstk::Matrix<double>(3,3)
{
   compute( SVPositionVector, SVVelocityVector );
}

RACRotation::RACRotation(const gpstk::Xvt& xvt)
                         : gpstk::Matrix<double>(3,3)
{
   compute( xvt.x, xvt.v );
}

//
//  Given the SV position vector and the SV velocity vector,
//  compute a rotation from ECEF XYZ to ECEF Radial,
//  Along-Track, Cross-Track (RAC).
//
//  Let the SV position vector be represented by R
//  Let the SV velocity vector be represented by V
//  1.) Form the unit vector R^ = R / |R|.
//  2.) Compute vector C = R^ cross V and unit vector C^ = C / |C|.  C^ is
//      perpendiculat to the RV plane
//  3.) Compute A^ = C^ corss R^.
//  4.) [R^, A^, C^] is an orthonormal triad and the rotation matrix between
//      XYZ and RAC is the matrix where R^, C^, and A^ are each a row of the
//      matrix.
//
void RACRotation::compute( const gpstk::Triple& SVPositionVector,
                           const gpstk::Triple& SVVelocityVector)
{

   gpstk::Triple unitR = SVPositionVector.unitVector();
   gpstk::Triple C = unitR.cross(SVVelocityVector);
   gpstk::Triple unitC = C.unitVector();
   gpstk::Triple unitA = unitC.cross(unitR);

   (*this) (0,0) = unitR[0];
   (*this) (0,1) = unitR[1];
   (*this) (0,2) = unitR[2];
   (*this) (1,0) = unitA[0];
   (*this) (1,1) = unitA[1];
   (*this) (1,2) = unitA[2];
   (*this) (2,0) = unitC[0];
   (*this) (2,1) = unitC[1];
   (*this) (2,2) = unitC[2];
}

gpstk::Vector<double> RACRotation::convertToRAC( const gpstk::Vector<double>& inV )
{
   gpstk::Vector<double> outV(3);

   /*
      My goal was to use the following statement.
   outV =  this * inV;
      However, for some reason, gcc refuses to recognize RACRotation as a
      Matrix subclass.  Therefore, I've incorporated the matrix multiply
      as a temporary kludge.
   */
   if (inV.size()!=3)
   {
      gpstk::Exception e("Incompatible dimensions for Vector");
      GPSTK_THROW(e);
   }
   size_t i, j;
   for (i = 0; i < 3; i++)
   {
      outV[i] = 0;
      for (j = 0; j < 3; j++)
      {
         double temp =  (*this)(i,j) * inV[j];
         outV[i] += temp;
      }
   }
   /* end kludge */
   return(outV);
}

gpstk::Triple RACRotation::convertToRAC( const gpstk::Triple& inVec )
{
   gpstk::Vector<double> v(3);
   v[0] = inVec[0];
   v[1] = inVec[1];
   v[2] = inVec[2];

   gpstk::Vector<double> vOut = convertToRAC( v );
   gpstk::Triple outVec( vOut[0], vOut[1], vOut[2] );
   return(outVec);
}

gpstk::Xvt RACRotation::convertToRAC( const gpstk::Xvt& in )
{
   gpstk::Xvt out;
   out.clkbias = in.clkbias;
   out.relcorr = in.relcorr;
   out.clkdrift = in.clkdrift;
   out.x = convertToRAC( in.x );
   out.v = convertToRAC( in.v );
   return(out);
}
}     // end namespace gpstk
