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
 * @file BinUtils.cpp
 * Binary manipulation functions
 */
 
#include "BinUtils.hpp"

namespace gpstk
{
   namespace BinUtils
   {
      const CRCParam CRCCCITT(16, 0x1021, 0xffff, 0, true, false, false);
      const CRCParam CRC16(16, 0x8005, 0, 0, true, true, true);
      const CRCParam CRC32(32, 0x4c11db7, 0xffffffff, 0xffffffff, true, true, true);
      const CRCParam CRC24Q(24, 0x823ba9, 0, 0xffffffff, true, false, false);
      // CRC24Q (for GPS CNAV): 23 17 13 12 11 9 8 7 5 3 +1
      // 1000 0010 0011 1011 1010 1001 : 823ba9

      // CRC-16: 16 15 2 +1
      // 1000 0000 0000 0101: 8005

      // CRC-CCITT: 16 12 5 +1
      // 0001 0000 0010 0001: 1021

      // CRC-32: 32 26 23 22 16 12 11 10 8 7 5 4 2 +1
      // 0000 0100 1100 0001 0001 1101 1011 0101 : 04c11db5
   }
}
