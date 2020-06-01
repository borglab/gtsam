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
 * @file FICData162.cpp
 * Ephemeris data encapsulated in engineering terms
 */
#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "FICData162.hpp"

#include <cmath>

namespace gpstk
{
   using namespace std;
   using namespace gpstk;

      //
      //  Note: the subframes are assumed to be 11 elements long so
      //  elements 1-10 are used.
   FICData162::FICData162(const short xmitPRN,
                          const short SVID,
                          const short xmitWeek,
                          const short toaWeek,
                          const std::vector<uint32_t> subframe )
   {
      blockNum = 162;

      i.push_back( (long) SVID );
      for (int wndx=1;wndx<11;++wndx) i.push_back( (long) subframe[wndx] );
      long dummy = 0;
      i.push_back( (long) xmitPRN );
      i.push_back( dummy );
      i.push_back( (long) toaWeek );
      i.push_back( (long) xmitWeek );
   }

}   // namespace
