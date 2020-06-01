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
 * @file ObsEpochMap.cpp
 * A class encapsulating observation data (roughly standard RINEX obs and met files).
 */

#include "ObsEpochMap.hpp"

using namespace std;
using namespace gpstk;

namespace gpstk
{
   // These are just to facilitate debugging. The format of the data output
   // is quite ad-hoc and may change.
   std::ostream& operator<<(std::ostream& s, const SvObsEpoch& obs)
      throw()
   {
      SvObsEpoch::const_iterator i;
      for (i=obs.begin(); i != obs.end(); i++)
      {
         if (i != obs.begin())
            s << ", ";
         s << i->first << ": " << i->second;
      }
      return s;
   }

   std::ostream& operator<<(std::ostream& s, const ObsEpoch& oe)
      throw()
   {
      s << oe.time << ", rxClock: " << oe.rxClock << endl;
      ObsEpoch::const_iterator i;
      for (i=oe.begin(); i!=oe.end(); i++)
         s << i->first << ": " << i->second << endl;

      return s;
   }
}  // namespace
