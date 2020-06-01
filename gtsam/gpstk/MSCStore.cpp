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

/**
 * @file MSCStore.cpp
 * Store Monitor Station coordinate information and access by station ID and time
 */

#include <iostream>
#include <fstream>
#include <iomanip>

#include "StringUtils.hpp"
#include "CommonTime.hpp"
#include "MSCStore.hpp"
#include "MathBase.hpp"

using namespace std;
using namespace gpstk;
using gpstk::StringUtils::asString;

namespace gpstk
{
      // The number o seconds in a year is 365 and a quetar days time
      // the number of seconds in a day.
   const double MSCStore::SEC_YEAR = 365.25 * gpstk::SEC_PER_DAY;
   
   //--------------------------------------------------------------------------
   //--------------------------------------------------------------------------

   Xvt MSCStore::getXvt(const std::string& stationID, const CommonTime& t)
      const throw( gpstk::InvalidRequest )
   {
      try
      {
            // Find an appropriate MSCData object.  
         const MSCData& msc = findMSC( stationID, t ); 

         return( msc.getXvt(t) );
      }
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }
   } // end of MSCStore::getXvt()

   //--------------------------------------------------------------------------
   //--------------------------------------------------------------------------

   Xvt MSCStore::getXvt(unsigned long& stationIDno, const CommonTime& t)
      const throw(InvalidRequest)
   {
      try
      {
            // Find an appropriate MSCData object.  
         const MSCData& msc = findMSC( stationIDno, t ); 

         return( msc.getXvt(t) );
      }
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }
   } // end of MSCStore::getXvt()

   //--------------------------------------------------------------------------
   //--------------------------------------------------------------------------
   void MSCStore::dump(ostream& s, short detail) const
      throw()
   {
      MMci it;

      s << "Dump of MSCStore:\n";
      if (detail==0)
      {
         unsigned msc_count=0;
         for (it = mscMap.begin(); it != mscMap.end(); it++)
         {
            msc_count += it->second.size();
         }
         s << " Span is " << initialTime
           << " to " << finalTime
           << " with " << msc_count << " entries."
           << endl;
      }
      else
      {
         for (it = mscMap.begin(); it != mscMap.end(); it++)
         {
            s << "Coordinates for station '" << (string) it->first << "'" << endl;
            SMMci it2;
            for (it2 = it->second.begin(); it2 != it->second.end(); ++it2)
            {
               const MSCData& em = it2->second;

               s << "Mnemonic    " << em.mnemonic
                 << ", Ref epoch   " << em.refepoch
                 << ", Eff epoch   " << em.effepoch
                 << ", Coordinates " << em.coordinates
                 << ", Velocities  " << em.velocities   
                 << endl;
            }
         }
         s << "  End of MSCStore data." << endl << endl;
      }
   } // end of MSCStore::dump()

//-----------------------------------------------------------------------------
//
//  edit() is a little unusual in that the MSCData objects have a beginning-of-
//  effectivity, but do NOT have an end-of-effectivity.  Therefore, we end up
//  with the following sort of logic (BOT = BEGINNING_OF_TIME, EOT = END_OF_TIME).
//
// Case    tmin          tmax          action
//   1     BOT           EOT           no action
//   2     BOT           <initialTime  clear all 
//   3     >finalTime    EOT           keep last entry for each station
//                                     (no ending effectivity)
//   4     >initialTime  EOT           clear if effepoch < tmin && 
//         <finalTime                  not ending object
//   5     BOT           >initialTime  clear if effepoch > tmax
//                       <finalTime
//   6     >initialTime  >initialTime  clear if (effepoch > tmax or 
//         <finalTime    <finalTime             (effepoch < tmin and
//         <tmax         >tmax                   not ending object)
//
//   Cases 1-5 are degenerate examples of case 6.
//
//-----------------------------------------------------------------------------
   void MSCStore::edit(const CommonTime& tmin, const CommonTime& tmax)
      throw()
   {
      
         // For each station....      
      MMi mmi;
      for(mmi = mscMap.begin(); mmi != mscMap.end(); mmi++)
      {
            // If the effective epoch of the final entry is beyond
            // tmax, clear all objects
         StaMSCMap& smmr = mmi->second;
         StaMSCMap::reverse_iterator smmir;
         smmir = smmr.rbegin();
         if (smmir->first > tmax) 
         {
            smmr.clear();   
         }
            // Since we're in the else branch, there's at least
            // one entry that needs to be retained.
         else 
         {
            SMMi smmi = smmr.begin();
            SMMi lastTooEarly = smmi;// Just initialize to a valid value 
            while (smmi != smmr.end() && smmi->first < tmin)
            {
               lastTooEarly = smmi;
               ++smmi;
            }
            if (smmi != smmr.begin()) smmr.erase( smmr.begin(), lastTooEarly );
         }
      }
      initialTime = tmin;
      
   } // end of MSCStore::edit()

   //--------------------------------------------------------------------------
   //--------------------------------------------------------------------------
   void MSCStore::loadFile(const string& filename)
      throw(FileMissingException)
   {
      try
      {
         MSCStream strm(filename.c_str());
         if (!strm)
         {
            FileMissingException e("File " + filename + " could not be opened.");
            GPSTK_THROW(e);
         }
         
         MSCHeader header;
         strm >> header;
         addFile(filename, header);  // <<<---- Error here
         
         MSCData rec;
         while(strm >> rec)
         {
	         addMSC(rec);
         }	 
      }
      catch (gpstk::Exception& e)
      {
         GPSTK_RETHROW(e);
      } 
   }

   //--------------------------------------------------------------------------
   //--------------------------------------------------------------------------
   bool MSCStore::addMSC(const MSCData& msc)
      throw()
   {
         // Chop off any leading/trailing whitespace
      string key = StringUtils::stripTrailing(msc.mnemonic);
      key = StringUtils::stripLeading(key);
      
      StaMSCMap& mm = mscMap[key];
      mm[msc.effepoch] = msc;
      
      if (msc.effepoch < initialTime) initialTime = msc.effepoch;
      // NOTE: finalTime is purposely left at END_OF_TIME - 
      // MSCData objects have no ending time of effectivity.
      return(true);
   }

   //--------------------------------------------------------------------------
   //--------------------------------------------------------------------------
   const MSCData&
   MSCStore::findMSC(const unsigned long stationID, const CommonTime& t) 
      const throw(InvalidRequest)
   {
      try
      {
         string sid = StringUtils::asString( stationID );
         return( findMSC( sid, t ));
      }
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }
   }
   //--------------------------------------------------------------------------
   //--------------------------------------------------------------------------
   const MSCData&
   MSCStore::findMSC(const string& stationID, const CommonTime& t) 
      const throw(InvalidRequest)
   {
      try
      {
         string key = StringUtils::stripTrailing( stationID );
         key = StringUtils::stripLeading( key ); 
         MMci mm = mscMap.find( key );

         // Didn't find the station under the mnemonic.
            // Check for a possible station number use.
         if (mm == mscMap.end() && StringUtils::isDigitString(key) )
         {
            unsigned long staNum = StringUtils::asUnsigned(key);
            mm=mscMap.begin();
            while (mm != mscMap.end())
            {
               const StaMSCMap& smm = mm->second;
               SMMci smmi = smm.begin();
               if (smmi->second.station == staNum) break;
               ++mm;
            }
         }
         if (mm==mscMap.end())
         {
            InvalidRequest e("No station coordinates for station '" + key + "'" );
            GPSTK_THROW(e);
         }
         
            // If we reach this point, mm should hold a valid reference
            // to an StaMSCMap object.  Now to find the appropriate
            // entry in that object.
            // The effective epoch (which is the key for the list) represents
            // the earliest time that the MSCData object is applicable.  There
            // is no corresponding end time.  Therefore, we'll start at the 
            // END of the time-ordered list and select the first object
            // with a key <= the time of interest.  
            //
         StaMSCMap::const_reverse_iterator smmir;
         for (smmir = mm->second.rbegin(); smmir != mm->second.rend(); ++smmir)
         {
            const CommonTime& dtr = smmir->first;
            if (dtr<=t) return( smmir->second );
         }

            // If we reach this point, there's no time-approprate entry for 
            // this station
         InvalidRequest e("No station coordinates for station " + 
                           stationID + " at time " + t.asString() );
         GPSTK_THROW(e);
      }
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }
   }

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
   unsigned MSCStore::size() const throw()
   {
      unsigned counter = 0;
      for(MMci i = mscMap.begin(); i != mscMap.end(); ++i)
      {
         counter += i->second.size();
      }
      return counter;
   }


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
   int MSCStore::addToList(list<MSCData>& v) const throw()
   {
      int n=0;
      for (MMci i = mscMap.begin(); i != mscMap.end(); ++i)
      {
         SMMci i2;
         for (i2=i->second.begin(); i2 != i->second.end(); ++i2)
         {
            v.push_back(i2->second);
            n++;
         }
      }
      return n;
   }


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
   list<string> MSCStore::getIDList()
   {
      list<string> temp;
      MMci mmci;
      for(mmci = mscMap.begin(); mmci != mscMap.end(); mmci++)
      {
         temp.push_back(mmci->first);
      }
      return(temp);
   }

   
} // namespace
 
