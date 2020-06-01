/// @file GPSEphemerisStore.cpp
/// Class for storing and/or computing position, velocity, and clock data using
/// tables of <SatID, <time, GPSEphemeris> >. Inherits OrbitEphStore, which includes
/// initial and final times and search methods. GPSEphemeris inherits OrbitEph and
/// adds health and accuracy information, which this class makes use of.

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

#include <iostream>
#include <fstream>
#include <iomanip>

#include "GPSEphemerisStore.hpp"
#include "GPSWeekSecond.hpp"

using namespace std;
using namespace gpstk::StringUtils;

namespace gpstk
{
   //-----------------------------------------------------------------------------
   // See notes in the .hpp. This function is designed to be called AFTER all elements
   // are loaded. It can then make adjustments to time relationships based on
   // inter-comparisons between sets of elements that cannot be performed until the
   // ordering has been determined.
   void GPSEphemerisStore::rationalize(void)
   {
      // loop over satellites
      SatTableMap::iterator it;
      for (it = satTables.begin(); it != satTables.end(); it++) {
         TimeOrbitEphTable& table = it->second;
         TimeOrbitEphTable::iterator ei;
         TimeOrbitEphTable::iterator eiPrev;
         bool begin = true;
         double previousOffset = 0.0;
         bool previousIsOffset = false;
         bool currentIsOffset = false;

         //string tForm = "%02H:%02M:%02S";

         // Scan the map for this SV looking for uploads.  Uploads are identifed by
         // Toe values that are offset from an even hour.
         OrbitEph* oePrev = 0;
         for (ei = table.begin(); ei != table.end(); ei++)
         {
            currentIsOffset = false;      // start with this assumption
            OrbitEph* oe = ei->second;
            long Toe = (long) (static_cast<GPSWeekSecond> (oe->ctToe)).sow;
            double currentOffset = Toe % 3600;

            //cout << "Top of For Loop.  oe->beginValid = "
            // << printTime(oe->beginValid,tForm);
            //cout << ", currentOffset =" << currentOffset << endl;

            if ( (currentOffset)!=0) {
               currentIsOffset = true;

               //cout << "*** Found an offset" << endl;
               //cout << " currentIsOffest: " << currentIsOffset;
               //cout << " previousIsOffest: " << previousIsOffset;
               //cout << " current, previous Offset = " << currentOffset
               // << ", " << previousOffset << endl;

               // If this set is offset AND the previous set is offset AND
               // the two offsets are the same, then this is the SECOND
               // set of elements in an upload.  In that case the OrbitEph
               // load routines have conservatively set the beginning time
               // of validity to the transmit time because there was no
               // way to prove this was the second data set.  Since we can
               // now prove its second by observing the ordering, we can
               // adjust the beginning of validity as needed.
               // Since the algorithm is dependent on the message
               // format, this must be done in OrbitEph.
               // IMPORTANT NOTE:  We also need to adjust the
               // key in the map, which is based on the beginning
               // of validity.  However, we can't do it in this
               // loop without destroying the integrity of the
               // iterator.  This is handled later in a second
               // loop.  See notes on the second loop, below.
               if (previousIsOffset &&
                   currentIsOffset  &&
                   currentOffset==previousOffset)
               {
                  //cout << "*** Adjusting beginValid" << endl;
                  oe->adjustValidity();
               }

                  // If the previous set is not offset, then
                  // we've found an upload
                  // For that matter, if previous IS offset, but
                  // the current offset is different than the
                  // previous, then it is an upload.
               if (!previousIsOffset ||
                   (previousIsOffset && (currentOffset!=previousOffset) ) )
               {
                     // Record the offset for later reference
                  previousOffset = currentOffset;

                     // Adjust the ending time of validity of any elements
                     // broadcast BEFORE the new upload such that they
                     // end at the beginning validity of the upload.
                     // That beginning validity value should already be
                     // set to the transmit time (or earliest transmit
                     // time found) by OrbitEph and GPSEphemerisStore.addOrbitEph( )
                     // This adjustment is consistent with the IS-GPS-XXX
                     // rules that a new upload invalidates previous elements.
                     // Note that this may be necessary for more than one
                     // preceding set of elements.
                  if (!begin)
                  {
                     //cout << "*** Adjusting endValid Times" << endl;
                     TimeOrbitEphTable::iterator ri;
                     // We KNOW it exists in the map
                     ri = table.find(oePrev->beginValid);
                     bool done = false;
                     while (!done)
                     {
                        OrbitEph* oeRev = ri->second;
                        //cout << "Testing Toe of " << printTime(oeRev->ctToe,"%02H:%02M:%02S");
                        //cout << " with endValid of " << printTime(oeRev->endValid,"%02H:%02M:%02S") << endl;

                           // If the current set of elements has an ending
                           // validity prior to the upload, then do NOT
                           // adjust the ending and set done to true.
                        if (oeRev->endValid <= oe->beginValid) done = true;

                           // Otherwise, adjust the ending validity to
                           // match the begin of the upload.
                         else oeRev->endValid = oe->beginValid;

                           // If we've reached the beginning of the map, stop.
                           // Otherwise, decrement and test again.
                        if (ri!=table.begin()) ri--;
                         else done = true;
                     }
                  }
               }
            }

               // Update condition flags for next loop
            previousIsOffset = currentIsOffset;
            oePrev = oe;           // May need this for next loop.
            begin = false;
            //cout << "Bottom of For loop.  currentIsOffset: " << currentIsOffset <<
            //        ", previousIsOffset: " << previousIsOffset << endl;
         } //end inner for-loop

            // The preceding process has left some elements in a condition
            // when the beginValid value != the key in the map.  This
            // must be addressed, but the map key is a const (by definition).
            // We have to search the map for these disagreements.  When found,
            // the offending item need to be copied out of the map, the
            // existing entry deleted, the item re-entered with the new key.
            // Since it is unsafe to modify a map while traversing the map,
            // each time this happens, we have to reset the loop process.
            //
            // NOTE: Simplistically, we could restart the loop at the
            // beginning each time.  HOWEVER, we sometimes load a long
            // period in a map (e.g. a year).  At one upload/day, that
            // would mean ~365 times, each time one day deeper into the map.
            // As an alternate, we use the variable loopStart to note how
            // far we scanned prior to finding a problem and manipulating
            // the map.  Then we can restart at that point.
         bool done = false;
         CommonTime loopStart = CommonTime::BEGINNING_OF_TIME;
         while (!done)
         {
            ei = table.lower_bound(loopStart);
            while (ei!=table.end())
            {
              OrbitEph* oe = ei->second;
              if (ei->first!=oe->beginValid)
              {
                 OrbitEph* oeAdj= oe->clone();       // Adjustment was done in
                                                   // first loop above.
                 delete ei->second;                // oe becomes invalid.
                 table.erase(ei);                     // Remove the map entry.
                 table[oeAdj->beginValid] = oeAdj->clone(); // Add back to map
                 break;            // exit this while loop without finishing
              }
              loopStart = ei->first; // Scanned this far successfully, so
                                     // save this as a potential restart point.
              ei++;
              if (ei==table.end()) done = true;   // Successfully completed
                                               // the loop w/o finding any
                                               // mismatches.  We're done.
            }
        }

           // Well, not quite done.  We need to update the initial/final
           // times of the map.
        const TimeOrbitEphTable::iterator Cei = table.begin( );
        initialTime = Cei->second->beginValid;
        const TimeOrbitEphTable::reverse_iterator rCei = table.rbegin();
        finalTime   = rCei->second->endValid;

      } // end outer for-loop
   }

   //-----------------------------------------------------------------------------
   // Add all ephemerides to an existing list<GPSEphemeris> for given satellite
   // If sat.id is -1 (the default), all ephemerides are added.
   // @return the number of ephemerides added.
   int GPSEphemerisStore::addToList(list<GPSEphemeris>& gpslist, SatID sat) const
   {
      // get the list from OrbitEphStore
      list<OrbitEph*> oelst;
      OrbitEphStore::addToList(oelst,SatID(-1,SatID::systemGPS));

      int n(0);
      list<OrbitEph*>::const_iterator it;
      for(it = oelst.begin(); it != oelst.end(); ++it) {
         OrbitEph *ptr = *it;
         GPSEphemeris *gpsptr = dynamic_cast<GPSEphemeris*>(ptr);
         GPSEphemeris gpseph(*gpsptr);
         gpslist.push_back(gpseph);
         n++;
      }

      return n;
   }

} // namespace
