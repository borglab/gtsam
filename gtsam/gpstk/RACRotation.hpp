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
#ifndef GPSTK_RACROTATION_HPP
#define GPSTK_RACROTATION_HPP

// gpstk
#include "Triple.hpp"
#include "Matrix.hpp"
#include "Vector.hpp"
#include "Xvt.hpp"

namespace gpstk
{
   class RACRotation : public gpstk::Matrix<double>
   {
      public:
            // Constructors
         RACRotation( const gpstk::Triple& SVPositionVector,
                      const gpstk::Triple& SVVelocityVector);
         RACRotation(const gpstk::Xvt& xvt);
         
            // Methods
         gpstk::Vector<double> convertToRAC( const gpstk::Vector<double>& inV );
         gpstk::Triple         convertToRAC( const gpstk::Triple& inVec );
         gpstk::Xvt            convertToRAC( const gpstk::Xvt& in );
                                  
            // Utilities
      protected:
         void compute( const gpstk::Triple& SVPositionVector,
                       const gpstk::Triple& SVVelocityVector);
   };
}   
#endif      
