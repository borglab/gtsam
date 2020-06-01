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
 * @file OrbElemFIC109.cpp
 * OrbElemFIC109 data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "OrbElemFIC109.hpp"
#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "GPS_URA.hpp"
#include "GPSWeekSecond.hpp"
#include "SVNumXRef.hpp"
#include "TimeString.hpp"
#include "EngNav.hpp"

namespace gpstk
{
   using namespace std;

   OrbElemFIC109::OrbElemFIC109()
      :OrbElemLNav()
   {}

   OrbElemFIC109::OrbElemFIC109( const FICData& fic109 )
      throw( InvalidParameter )
   {
      loadData( fic109 );
   }

   OrbElemFIC109* OrbElemFIC109::clone() const
   {
      return new OrbElemFIC109 (*this); 
   }
   
   void OrbElemFIC109::loadData( const FICData& fic109)
      throw( InvalidParameter )
   {
      if (fic109.blockNum!=109)
      {
         InvalidParameter exc("Invalid FIC Block: "+StringUtils::asString(fic109.blockNum));
         GPSTK_THROW(exc);
      }
      short XmitGPSWeek = fic109.i[0];
      short PRNID = fic109.i[1];
      long SF1[10], SF2[10], SF3[10];
      for(int i = 0; i < 10; i++)
      {
         SF1[i] = fic109.i[2+i];
         SF2[i] = fic109.i[12+i];
         SF3[i] = fic109.i[22+i];
      }

         // FIC only stores GPS data, so the system is well-defined.
      SatID sid(PRNID, SatID::systemGPS);

      OrbElemLNav::loadData( SF1,
                             SF2,
                             SF3,
                             sid,
                             XmitGPSWeek );
   }

} // namespace


