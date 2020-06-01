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
// This software developed by Applied Research Laboratories at the University
// of Texas at Austin, under contract to an agency or agencies within the U.S. 
// Department of Defense. The U.S. Government retains all rights to use,
// duplicate, distribute, disclose, or release this software. 
//
// Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

/**
 * @file YumaAlmanacStore.cpp
 * Store Yuma almanac information, and access by satellite and time
 */

#include "YumaAlmanacStore.hpp"
#include "GPSWeekSecond.hpp"

namespace gpstk
{

   void YumaAlmanacStore::loadFile(const std::string& filename)
      throw(FileMissingException)
   {
      try
      {
         YumaStream strm(filename.c_str());
         if (!strm)
         {
            FileMissingException e("File " + filename + " could not be opened.");
            GPSTK_THROW(e);
         }
         
         YumaHeader header;
         strm >> header;
         addFile(filename, header);

         YumaData rec;
         while(strm >> rec)
	      {
               // If the user has indcated a time of interest and
               // the reference week number is less than 10 bits long, 
               // assume the almanac must be within 511 weeks of the 
               // time of interest
               // If necessary, adjust the GPS week number
            if (timeOfInterest>CommonTime::BEGINNING_OF_TIME &&
                rec.week < 1024)
            {
               short diff = GPSWeekSecond(timeOfInterest).week - rec.week;
               short nEpochs = (diff+512) / 1024;
               rec.week += nEpochs * 1024;
            }
	         addAlmanac(AlmOrbit(rec));
         }	 
      }
      catch (Exception& e)
      {
         GPSTK_RETHROW(e);
      }   
   }

}
