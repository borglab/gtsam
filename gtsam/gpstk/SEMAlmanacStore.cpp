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
 * @file SEMAlmanacStore.cpp
 * Store SEM almanac information, and access by satellite and time
 */

#include "SEMAlmanacStore.hpp"
#include "GPSWeekSecond.hpp"

namespace gpstk
{

   void SEMAlmanacStore::loadFile(const std::string& filename)
      throw(FileMissingException)
   {
      try
      {
         SEMStream strm(filename.c_str());
         if (!strm)
         {
            FileMissingException e("File " + filename + " could not be opened.");
            GPSTK_THROW(e);
         }
         
         SEMHeader header;
         strm >> header;
         
            // If the user has indcated a time of interest and
            // the reference week number is less than 10 bits long, 
            // assume the almanac must be within 511 weeks of the 
            // time of interest
            // If necessary, adjust the GPS week number
            //
            // NOTE: According to the SEM format documetation on the
            // USCG NAVCEN website, the week in the header should be
            // 0-1023; however there is anecdotal evidence that some
            // organizations have used the full GPS week number.  
            // This is an attempt to "do the right thing" in the 
            // broadest number of cases.
         if (timeOfInterest>gpstk::CommonTime::BEGINNING_OF_TIME &&
             header.week < 1024)
         {
            short diff = static_cast<GPSWeekSecond>(timeOfInterest).week - header.week;
            short nEpochs = (diff+512) / 1024;
            header.week += nEpochs * 1024;
         }
         addFile(filename, header);

         SEMData rec;
         while(strm >> rec)
	      {
               //This is a fix to get the header and the data to share Toa and week.
	         rec.Toa = header.Toa;
	         rec.week = header.week;
            
	         addAlmanac(AlmOrbit(rec));
         }
      }
      catch (Exception& e)
      {
         GPSTK_RETHROW(e);
      }  
   }
   
}
