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
 * @file FICData109.cpp
 * Ephemeris data encapsulated in engineering terms
 */

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "FICData109.hpp"

#include <cmath>

namespace gpstk
{
   using namespace std;
   using namespace gpstk;

      //
      //  Note: the subframes are assumed to be 11 elements long so
      //  elements 1-10 are used.
   FICData109::FICData109(const short PRNID,
                          const std::vector<uint32_t> sf1,
                          const std::vector<uint32_t> sf2,
                          const std::vector<uint32_t> sf3 )
   {
      blockNum = 109;

      long temp = sf1[3];
      temp &= 0x3FFFFFFF;       // Make certain top two bits are 0
      temp >>= 20;
                                 // DANGER WILL ROBINSON!!!!
                                 // HERE IS A TEMP KLUDGE
      temp += 1024;              // for the GPS Epoch.

      i.push_back( temp );
      i.push_back( (long) PRNID );

      for (int wndx=1;wndx<11;++wndx) i.push_back( (long) sf1[wndx] );
      for (int wndx=1;wndx<11;++wndx) i.push_back( (long) sf2[wndx] );
      for (int wndx=1;wndx<11;++wndx) i.push_back( (long) sf3[wndx] );
   }

}   // namespace
