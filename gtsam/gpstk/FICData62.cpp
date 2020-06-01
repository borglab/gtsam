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
 * @file FICData62.cpp
 * Almanac data encapsulated in engineering terms
 */

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "FICData62.hpp"

#include <cmath>

namespace gpstk
{
   using namespace std;
   using namespace gpstk;

   const double FICData62::UNUSED_F = 0.0;
   const long FICData62::UNUSED_I = 0;

   FICData62::FICData62( const gpstk::FICData162 fic162  )
   {
      blockNum = 62;

      double output[60];
      bool stat = EngNav::subframeConvert(      &(fic162.i[1]),
                                            (int) fic162.i[14],
                                                        output );

      if (stat==false)
      {
         // need to throw something here
         return;
      }

      short format = EngNav::getSubframePattern( &(fic162.i[1]) );

      i.push_back( fic162.i[13] );
      i.push_back( (long) (output[2] - 6.0) );
      i.push_back( UNUSED_I );
      i.push_back( (long) output[6] );
      i.push_back( (long) format );
      i.push_back( fic162.i[14] );


      short maxNdx = 0;
      switch (format)
      {
         case 4: maxNdx = 20; break;
         case 5: maxNdx = 32; break;
         case 6: maxNdx = 14; break;
         case 7: maxNdx = 14; break;
         case 8: maxNdx = 23; break;
         case 9: maxNdx = 47; break;
         case 10:maxNdx = 29; break;

         default:
            // THROW SOMETHING
            break;
      }
      for (int ndx=0;ndx<maxNdx;++ndx)
      {
         f.push_back( output[ndx] );
      }
   }
}   // namespace
