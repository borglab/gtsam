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
 * @file FICData9.cpp
 * Ephemeris data encapsulated in engineering terms
 */

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "GPSWeekSecond.hpp"
#include "FICData9.hpp"

#include <cmath>

namespace gpstk
{
   using namespace std;
   using namespace gpstk;

   const double FICData9::UNUSED = 0.0;

   FICData9::FICData9( const gpstk::FICData109 rawsf, const gpstk::EngEphemeris ee )
   {
      int i;
      blockNum = 9;

      firstFiveItems( 1, rawsf, ee );
      GPSWeekSecond gpstime(ee.getTransmitTime());
      f.push_back( (double) gpstime.week );
      f.push_back( (double) ee.getCodeFlags() );
      f.push_back( (double) ee.getAccFlag() );
      f.push_back( (double) ee.getHealth() );
      f.push_back( (double) (ee.getIODC() * 2048) );
      f.push_back( (double) ee.getL2Pdata() );
      f.push_back( ee.getTgd() );
      f.push_back( ee.getToc() );
      f.push_back( ee.getAf2() );
      f.push_back( ee.getAf1() );
      f.push_back( ee.getAf0() );

         // Two unused
      f.push_back( UNUSED );
      f.push_back( UNUSED );
      f.push_back( (double) ee.getTracker() );
      f.push_back( (double) ee.getPRNID() );

      firstFiveItems( 2, rawsf, ee );
      f.push_back( (double) (ee.getIODE() * 2048) );
      f.push_back( ee.getCrs() );
      f.push_back( ee.getDn() );
      f.push_back( ee.getM0() );
      f.push_back( ee.getCuc() );
      f.push_back( ee.getEcc() );
      f.push_back( ee.getCus() );
      f.push_back( ee.getAhalf() );
      f.push_back( ee.getToe() );
      f.push_back( (double) ee.getFitInt() );

         // Five unused
      for (i=0;i<5;++i) f.push_back( UNUSED );

      firstFiveItems( 3, rawsf, ee );
      f.push_back( ee.getCic() );
      f.push_back( ee.getOmega0() );
      f.push_back( ee.getCis() );
      f.push_back( ee.getI0() );
      f.push_back( ee.getCrc() );
      f.push_back( ee.getW() );
      f.push_back( ee.getOmegaDot() );
      f.push_back( (double) (ee.getIODE() * 2048) );
      f.push_back( ee.getIDot() );

         // Six unused
      for (i=0;i<6;++i) f.push_back( UNUSED );
   }

   void FICData9::firstFiveItems( const short sfNum,
                                  const gpstk::FICData109 rawsf,
                                  const gpstk::EngEphemeris ee )
   {
      int ndx = 2 + ((sfNum-1) * 10);
      long word01 = rawsf.i[ ndx ];
      long preamble = word01 >> 22;
      f.push_back( (double) preamble );
      f.push_back( (double) ee.getTLMMessage(sfNum) );
      f.push_back( ee.getHOWTime(sfNum) );
      f.push_back( (double) ee.getASAlert(sfNum) );
      f.push_back( (double) sfNum );
   }

}   // namespace
