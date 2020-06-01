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
 * @file GPSOrbElemStore.cpp
 * Store GPS broadcast OrbElem information, and access by satellite and time
 */

#include <iostream>
#include <fstream>
#include <iomanip>

#include "StringUtils.hpp"
#include "GPSOrbElemStore.hpp"
#include "MathBase.hpp"
#include "CivilTime.hpp"
#include "TimeString.hpp"

using namespace std;
using namespace gpstk;
using gpstk::StringUtils::asString;


namespace gpstk
{

//--------------------------------------------------------------------------

   Xvt GPSOrbElemStore::getXvt(const SatID& sat, const CommonTime& t) const
      throw( InvalidRequest )
   {
      try
      {
         // test for GPS satellite system in sat?
         const OrbElem* eph = findOrbElem(sat,t);

         // If the orbital elements are unhealthy, refuse to 
         // calculate an SV position and throw.
         if (!eph->healthy)
         {
            InvalidRequest exc( std::string("SV is transmitting unhealhty navigation ")
                + std::string("message at time of interest.") );
            GPSTK_THROW( exc );
         }
         Xvt sv = eph->svXvt(t);
         return sv;
      }
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }
   }

//------------------------------------------------------------------------------

   void validSatSystem(const SatID& sat)
      throw( InvalidRequest )
   {
      InvalidRequest ire( std::string("Try to get NON-GPS sat position ")
          + std::string("from GPSOrbElemStore, and it's forbidden!") );
      if(sat.system!=SatID::systemGPS) GPSTK_THROW(ire);
   }

//--------------------------------------------------------------------------

   bool GPSOrbElemStore::isHealthy(const SatID& sat, const CommonTime& t) const
      throw( InvalidRequest )
   {
      try
      {
         validSatSystem(sat);

         // test for GPS satellite system in sat?
         const OrbElem* eph = findOrbElem(sat, t);
         
         return eph->isHealthy();
      }
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }
   } // end of GPSOrbElemStore::getHealth()

//--------------------------------------------------------------------------

   void GPSOrbElemStore::dump(std::ostream& s, short detail) const
      throw()
   {
      UBEMap::const_iterator it;
      static const string fmt("%4F %10.3g = %04Y/%02m/%02d %02H:%02M:%02S %P");

      s << "Dump of GPSOrbElemStore:\n";
      if (detail==0)
      {
         s << " Span is " << (initialTime == CommonTime::END_OF_TIME
                                      ? "End_time" : printTime(initialTime,fmt))
           << " to " << (finalTime == CommonTime::BEGINNING_OF_TIME
                                      ? "Begin_time" : printTime(finalTime,fmt))
           << " with " << size() << " entries."
           << std::endl;
      } // end if-block
      else if (detail==1)
      {
         for (it = ube.begin(); it != ube.end(); it++)
         {
            const OrbElemMap& em = it->second;
            s << "  BCE map for satellite " << it->first
              << " has " << em.size() << " entries." << std::endl;
            OrbElemMap::const_iterator ei;

            for (ei = em.begin(); ei != em.end(); ei++) 
            {
               const OrbElem* oe = ei->second;
               s << "PRN " << setw(2) << it->first
                 << " TOE " << printTime(oe->ctToe,fmt)
                 << " TOC " << fixed << setw(10) << setprecision(3)
                 << oe->ctToe
                 << " KEY " << printTime(ei->first,fmt);
                
               s << std::endl;
            } //end inner for-loop */

         } // end outer for-loop
   
         s << "  End of GPSOrbElemStore data." << std::endl << std::endl;

      } //end else-block

         // In this case the output is
         // key, beginValid,  Toe,   endValid
      else if (detail==2)
      {
         string tf1 = "%02m/%02d/%02y %02H:%02M:%02S";
         string tf2 = "%02H:%02M:%02S";
         
         for (it = ube.begin(); it != ube.end(); it++)
         {
            const OrbElemMap& em = it->second;
            s << "  Map for satellite " << it->first
              << " has " << em.size() << " entries." << std::endl;
            OrbElemMap::const_iterator ei;

            s << "  PRN  MM/DD/YY      Key     Begin       Toe       Toc      End" << endl;

            for (ei = em.begin(); ei != em.end(); ei++) 
            {
               const OrbElem* oe = ei->second;
               s << it->first << "  " << printTime(ei->first,tf1)
                              << "  " << printTime(oe->beginValid,tf2)
                              << "  " << printTime(oe->ctToe,tf2)
                              << "  " << printTime(oe->ctToc,tf2)
                              << "  " << printTime(oe->endValid,tf2);
               s << std::endl;

            } //end inner for-loop */

         } // end outer for-loop
      }    // end of else-block
      else
      {
         for (it = ube.begin(); it != ube.end(); it++)
         {
            const OrbElemMap& em = it->second;
            s << "  Map for satellite " << it->first
              << " has " << em.size() << " entries." << std::endl;
            OrbElemMap::const_iterator ei;

            for (ei = em.begin(); ei != em.end(); ei++) 
            {
               const OrbElem* oe = ei->second;
               oe->dump(s);
            }
         }
      }
   } // end GPSOrbElemStore::dump

//------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------
// Keeps only one OrbElem for a given SVN and Toe.
// It should keep the one with the earliest transmit time.
//------------------------------------------------------------------------------------ 
   bool GPSOrbElemStore::addOrbElem(const OrbElem& eph)
      throw(InvalidParameter,Exception)
   {
      bool rc = false;
     try
     {

     SatID sid = eph.satID;
     OrbElemMap& oem = ube[sid];
     string ts = "%02m/%02d/%02y %02H:%02M:%02S";
     int PRN = sid.id;
     int testPRN = 0;
#pragma unused(PRN,testPRN,rc)
         
       // if map is empty, load object and return
     if (oem.size()==0)
     {
        oem[eph.beginValid] = eph.clone();
        updateInitialFinal(eph);
        return (true);
     }
       // Search for beginValid in current keys.
       // If found candidate, should be same data
       // as already in table. Test this by comparing
       // Toe values.
     OrbElemMap::iterator it = oem.find(eph.beginValid);
     if(it!=oem.end())
     {
        const OrbElem* oe = it->second;
          // Found duplicate already in table
        if(oe->ctToe==eph.ctToe)
        {
            return (false);
        }
          // Found matching beginValid but different Toe - This shouldn't happen
        else
        {
           string str = "Unexpectedly found matching beginValid times";
           stringstream os;
           os << eph.satID.id;
           str += " but different Toe.   PRN= " + os.str();
           str += ", beginValid= " + printTime(eph.beginValid,ts);
           str += ", Toe(map)= " + printTime(eph.ctToe,ts);
           str += ", Toe(candidate)= "+ printTime(oe->ctToe," %6.0g");
           str += ". ";
           InvalidParameter exc( str );
           GPSTK_THROW(exc); 
        }
     }
        // Did not already find match to
        // beginValid in map
        // N.B:: lower_bound will reutrn element beyond key since there is no match
     it = oem.lower_bound(eph.beginValid);
        // Case where candidate is before beginning of map
     if(it==oem.begin())
     {
        const OrbElem* oe = it->second;
        if(oe->ctToe==eph.ctToe)
        {
           oem.erase(it);
           oem[eph.beginValid] = eph.clone();
           updateInitialFinal(eph);
           return (true);
        }
        oem[eph.beginValid] = eph.clone();
        updateInitialFinal(eph);
        return (true);
     }
          // Case where candidate is after end of current map
     if(it==oem.end())
     {
          // Get last item in map and check Toe
        OrbElemMap::reverse_iterator rit = oem.rbegin();
        const OrbElem* oe = rit->second;
        if(oe->ctToe!=eph.ctToe)
        {
           oem[eph.beginValid] = eph.clone();
           updateInitialFinal(eph);
           return (true);
        }
        return (false);
     }
        // case where candidate is "In the middle"
        // Check if iterator points to late transmission of
        // same OrbElem as candidate
     const OrbElem* oe = it->second;
     if(oe->ctToe==eph.ctToe)
     {
        oem.erase(it);
        oem[eph.beginValid] = eph.clone();
        updateInitialFinal(eph);
        return (true);
     }
        // Two cases:
        //    (a.) Candidate is late transmit copy of
        //         previous OrbElem in table - discard (do nothing)
        //    (b.) Candidate OrbElem is not in table
 
        // Already checked for it==oem.beginValid() earlier
     it--;
     const OrbElem* oe2 = it->second;
     if(oe2->ctToe!=eph.ctToe)
     {
        oem[eph.beginValid] = eph.clone();
        updateInitialFinal(eph);
        return (true);
     }
     return (false);
    
   }
   catch(Exception& e)
   {
      GPSTK_RETHROW(e)
   }
 }
    
//-----------------------------------------------------------------------------

   void GPSOrbElemStore::edit(const CommonTime& tmin, const CommonTime& tmax)
      throw()
   {
      for(UBEMap::iterator i = ube.begin(); i != ube.end(); i++)
      {
         OrbElemMap& eMap = i->second;

         OrbElemMap::iterator lower = eMap.lower_bound(tmin);
         if (lower != eMap.begin())
         { 
            for (OrbElemMap::iterator emi = eMap.begin(); emi != lower; emi++)
               delete emi->second;        
            eMap.erase(eMap.begin(), lower);
         } 

         OrbElemMap::iterator upper = eMap.upper_bound(tmax);
         if (upper != eMap.end())
         {
            for (OrbElemMap::iterator emi = upper; emi != eMap.end(); emi++)
               delete emi->second; 
            eMap.erase(upper, eMap.end());          
         }
      }

      initialTime = tmin;
      finalTime   = tmax;
   }

//-----------------------------------------------------------------------------

   unsigned GPSOrbElemStore::size() const
      throw()
   {
      unsigned counter = 0;
      for(UBEMap::const_iterator i = ube.begin(); i != ube.end(); i++)
         counter += i->second.size();
      return counter;
   }

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Goal is to find the set of orbital elements that would have been
// used by a receiver in real-time.   That is to say, the most recently
// broadcast elements (assuming the receiver has visibility to the SV
// in question).
//-----------------------------------------------------------------------------

   const OrbElem*
   GPSOrbElemStore::findOrbElem(const SatID& sat, const CommonTime& t) const
      throw( InvalidRequest )
   {
          // Check to see that there exists a map of orbital elements
	  // relevant to this SV.
      UBEMap::const_iterator prn_i = ube.find(sat);
      if (prn_i == ube.end())
      {
         InvalidRequest e("No orbital elements for satellite " + asString(sat));
         GPSTK_THROW(e);
      }

         // Define reference to the relevant map of orbital elements
      const OrbElemMap& em = prn_i->second;

         // The map is ordered by beginning times of validity, which
	 // is another way of saying "earliest transmit time".  A call
         // to em.lower_bound( t ) will return the element of the map
	 // with a key "one beyond the key" assuming the t is NOT a direct
	 // match for any key. 
	 
	 // First, check for the "direct match" case
      OrbElemMap::const_iterator it = em.find(t);
         // If that fails, then use lower_bound( )
      if (it == em.end( ))    
      {
	 it = em.lower_bound(t); 
	              
	 // Tricky case here.  If the key is beyond the last key in the table,
	 // lower_bound( ) will return em.end( ).  However, this doesn't entirely
	 // settle the matter.  It is theoretically possible that the final 
	 // item in the table may have an effectivity that "stretches" far enough
	 // to cover time t.   Therefore, if it==em.end( ) we need to check 
	 // the period of validity of the final element in the table against 
	 // time t.
	 //
	 if (it==em.end())	
         {
            OrbElemMap::const_reverse_iterator rit = em.rbegin();
            if (rit->second->isValid(t)) return(rit->second);   // Last element in map works

	       // We reached the end of the map, checked the end of the map,
	       // and we still have nothing.  
            string mess = "All orbital elements found for satellite " + asString(sat) + " are too early for time "
               + (static_cast<CivilTime>(t)).printf("%02m/%02d/%04Y %02H:%02M:%02S %P");
            InvalidRequest e(mess);
            GPSTK_THROW(e);
         }
      } 

         // If the algorithm found a direct match, then we should 
	 // probably use the PRIOR set since it takes ~30 seconds
	 // from beginning of transmission to complete reception.
	 // If lower_bound( ) was called, it points to the element
	 // after the time of the key.
	 // So either way, it points ONE BEYOND the element we want.
	 //
	 // The exception is if it is pointing to em.begin( ).  If that is the case,
	 // then all of the elements in the map are too late.
      if (it==em.begin())
      {
         string mess = "All orbital elements found for satellite " + asString(sat) + " are too late for time "
            + (static_cast<CivilTime>(t)).printf("%02m/%02d/%04Y %02H:%02M:%02S %P");
         InvalidRequest e(mess);
         GPSTK_THROW(e);
      }

	 // The iterator should be a valid iterator and set one beyond
	 // the item of interest.  However, there may be gaps in the 
	 // middle of the map and cases where periods of effectivity do
	 // not overlap.  That's OK, the key represents the EARLIEST 
	 // time the elements should be used.   Therefore, we can 
	 // decrement the counter and test to see if the element is
	 // valid. 
      it--; 
      if (!(it->second->isValid(t)))
      {
	    // If we reach this throw, the cause is a "hole" in the middle of a map. 
         string mess = "No orbital elements found for satellite " + asString(sat) + " at "
            + (static_cast<CivilTime>(t)).printf("%02m/%02d/%04Y %02H:%02M:%02S %P");
         InvalidRequest e(mess);
         GPSTK_THROW(e);
      }
      return(it->second);
   } 
 
   

 
//-----------------------------------------------------------------------------
 
     const OrbElem*
   GPSOrbElemStore::findNearOrbElem(const SatID& sat, const CommonTime& t) const
      throw(InvalidRequest)
   {
        // Check for any OrbElem for this SV            
      UBEMap::const_iterator prn_i = ube.find(sat);
      if (prn_i == ube.end())
      {
         InvalidRequest e("No OrbElem for satellite " + asString(sat));
         GPSTK_THROW(e);
      }
   
         // FIRST, try to find the elements that were
         // actually being broadcast at the time of 
         // interest.  That will ALWAYS be the most
         // correct response.   IF YOU REALLY THINK
         // OTHERWISE CALL ME AND LET'S TALK ABOUT 
         // IT - Brent Renfro
      try
      {
         const OrbElem* oep = findOrbElem(sat, t);
         return(oep);
      }      
         // No OrbElem in store for requested sat time  
      catch(InvalidRequest)
      {
           // Create references to map for this satellite
         const OrbElemMap& em = prn_i->second;
           /*
              Three Cases: 
                1. t is within a gap within the store
                2. t is before all OrbElem in the store
                3. t is after all OrbElem in the store
           */

           // Attempt to find next in store after t
         OrbElemMap::const_iterator itNext = em.lower_bound(t);
           // Test for case 2
         if(itNext==em.begin())
         {
            return(itNext->second);
         }
           // Test for case 3
         if(itNext==em.end())
         {
            OrbElemMap::const_reverse_iterator rit = em.rbegin();
            return(rit->second);
         }
           // Handle case 1
           // Know that itNext is not the beginning, so safe to decrement
         CommonTime nextBeginValid = itNext->first;
         OrbElemMap::const_iterator itPrior = itNext;
         itPrior--;
         CommonTime lastEndValid = itPrior->second->endValid;
         double diffToNext = nextBeginValid-t;
         double diffFromLast = t - lastEndValid;
         if(diffToNext>diffFromLast)
         {
            return(itPrior->second);
         }
         return(itNext->second);
      }
   }
      
// See notes in the .hpp.  This function is designed to be called 
// AFTER all elements are loaded.  It can then make adjustments to
// time relationships based on inter-comparisons between sets of 
// elements that cannot be performed until the ordering has been
// determined. 
//-----------------------------------------------------------------------------
   void GPSOrbElemStore::rationalize( )
   {
      UBEMap::iterator it;
      for (it = ube.begin(); it != ube.end(); it++)
      {
         OrbElemMap& em = it->second;
         OrbElemMap::iterator ei;
	      OrbElemMap::iterator eiPrev;
         bool begin = true;
         double previousOffset = 0.0;
         bool previousIsOffset = false; 
         bool currentIsOffset = false;

         //string tForm = "%02H:%02M:%02S";

            // Scan the map for this SV looking for 
            // uploads.  Uploads are identifed by 
            // Toe values that are offset from 
            // an even hour.  
         OrbElem* oePrev = 0;
         for (ei = em.begin(); ei != em.end(); ei++) 
         {
            currentIsOffset = false;      // start with this assumption
            OrbElem* oe = ei->second;
            long Toe = (long) (static_cast<GPSWeekSecond> (oe->ctToe)).sow;
            double currentOffset = Toe % 3600;

            //cout << "Top of For Loop.  oe->beginValid = " << printTime(oe->beginValid,tForm);
            //cout << ", currentOffset =" << currentOffset << endl;
            
            if ( (currentOffset)!=0) 
            {
               currentIsOffset = true; 

               //cout << "*** Found an offset" << endl;
               //cout << " currentIsOffest: " << currentIsOffset; 
               //cout << " previousIsOffest: " << previousIsOffset; 
               //cout << " current, previous Offset = " << currentOffset << ", " << previousOffset << endl; 
            
                  // If this set is offset AND the previous set is offset AND
                  // the two offsets are the same, then this is the SECOND
                  // set of elements in an upload.  In that case the OrbElem
                  // load routines have conservatively set the beginning time
                  // of validity to the transmit time because there was no
                  // way to prove this was the second data set.  Since we can
                  // now prove its second by observing the ordering, we can
                  // adjust the beginning of validity as needed.
                  // Since the algorithm is dependent on the message
                  // format, this must be done in OrbElem.
                  // 
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
                 oe->adjustBeginningValidity();
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
                     // time found) by OrbElem and GPSOrbElemStore.addOrbElem( )
                     // This adjustment is consistent with the IS-GPS-XXX 
                     // rules that a new upload invalidates previous elements.
                     // Note that this may be necessary for more than one
                     // preceding set of elements.
                  if (!begin)
                  {
                     //cout << "*** Adjusting endValid Times" << endl;
                     OrbElemMap::iterator ri;
                     ri = em.find(oePrev->beginValid);     // We KNOW it exists in the map
                     bool done = false;
                     while (!done)
                     {
                        OrbElem* oeRev = ri->second;
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
                        if (ri!=em.begin()) ri--;
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
            // NOTE: Simplisitically, we could restart the loop at the 
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
            ei = em.lower_bound(loopStart);
            while (ei!=em.end())
            {
              OrbElem* oe = ei->second;
              if (ei->first!=oe->beginValid)
              {
                 OrbElem* oeAdj= oe->clone();       // Adjustment was done in 
                                                   // first loop above.
                 delete ei->second;                // oe becomes invalid.
                 em.erase(ei);                     // Remove the map entry.
                 em[oeAdj->beginValid] = oeAdj->clone(); // Add back to map
                 break;            // exit this while loop without finishing
              }
              loopStart = ei->first; // Scanned this far successfully, so
                                     // save this as a potential restart point.
              ei++;
              if (ei==em.end()) done = true;   // Successfully completed 
                                               // the loop w/o finding any 
                                               // mismatches.  We're done. 
            } 
        }

           // Well, not quite done.  We need to update the initial/final 
           // times of the map.
        const OrbElemMap::iterator Cei = em.begin( );
        initialTime = Cei->second->beginValid;
        const OrbElemMap::reverse_iterator rCei = em.rbegin();
        finalTime   = rCei->second->endValid;

      } // end outer for-loop
   }


//-----------------------------------------------------------------------------

   int GPSOrbElemStore::addToList(std::list<OrbElem*>& v) const
      throw()
   {
      int n = 0;
      UBEMap::const_iterator prn_i;
      for (prn_i = ube.begin(); prn_i != ube.end(); prn_i++)
      {
         const OrbElemMap& em = prn_i->second;
         OrbElemMap::const_iterator ei;
         for (ei = em.begin(); ei != em.end(); ei++)
         {
            v.push_back(ei->second->clone());
            n++;
         }
      }
      return n;
   } 

//-----------------------------------------------------------------------------

   const GPSOrbElemStore::OrbElemMap&
   GPSOrbElemStore::getOrbElemMap( const SatID& sat ) const
      throw( InvalidRequest )
   {
      validSatSystem(sat);

      UBEMap::const_iterator prn_i = ube.find(sat);
      if (prn_i == ube.end())
      {
         InvalidRequest e("No OrbElem for satellite " + asString(sat));
         GPSTK_THROW(e);
      }
      return(prn_i->second);
   }
   
} // namespace
