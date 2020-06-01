#pragma ident "$Id$"



/**
 * @file RinexNavFilterOperators.hpp
 * Operators for FileFilter using Rinex navigation data
 */

#ifndef GPSTK_RINEXNAVFILTEROPERATORS_HPP
#define GPSTK_RINEXNAVFILTEROPERATORS_HPP

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






#include "GPSWeekSecond.hpp"

#include "FileFilter.hpp"
#include "RinexNavData.hpp"
#include "RinexNavHeader.hpp"
#include "GPSWeekSecond.hpp"

#include <set>

namespace gpstk
{
   /** @addtogroup RinexNav */
   //@{

      /// This compares all elements of the RinexNavData with less than.
   struct RinexNavDataOperatorLessThanFull : 
      public std::binary_function<gpstk::RinexNavData, 
         gpstk::RinexNavData, bool>
   {
   public:
      bool operator()(const gpstk::RinexNavData& l,
                      const gpstk::RinexNavData& r) const
         {
            gpstk::GPSWeekSecond lXmitTime(l.weeknum, (double)l.HOWtime);
            gpstk::GPSWeekSecond rXmitTime(r.weeknum, (double)r.HOWtime);


            if (lXmitTime < rXmitTime)
               return true;
            else if (lXmitTime == rXmitTime)
            {
                  // compare the times and all data members
               if (l.time < r.time)
                  return true;
               else if (l.time == r.time)
               {
                  std::list<double>
                     llist = l.toList(),
                     rlist = r.toList();
                  
                  std::list<double>::iterator 
                     litr = llist.begin(), 
                     ritr = rlist.begin();
                  
                  while (litr != llist.end())
                  {
                     if (*litr < *ritr)
                        return true;
                     else if (*litr > *ritr)
                        return false;
                     else
                     {
                        litr++;
                        ritr++;
                     }
                  }
               }
            } // if (lXmitTime == rXmitTime)

            return false;
         }
   };

      /// This compares all elements of the RinexNavData with equals
   struct RinexNavDataOperatorEqualsFull : 
      public std::binary_function<gpstk::RinexNavData, 
         gpstk::RinexNavData, bool>
   {
   public:
      bool operator()(const gpstk::RinexNavData& l,
                      const gpstk::RinexNavData& r) const
         {
               // compare the times and all data members
            if (l.time != r.time)
               return false;
            else // if (l.time == r.time)
            {
               std::list<double>
                  llist = l.toList(),
                  rlist = r.toList();

               std::list<double>::iterator 
                  litr = llist.begin(), 
                  ritr = rlist.begin();

               while (litr != llist.end())
               {
                  if (*litr != *ritr)
                     return false;
                  litr++;
                  ritr++;
               }
            }

            return true;
         }
   };

      /// Only compares time.  Suitable for sorting a RinexNav file.
   struct RinexNavDataOperatorLessThanSimple : 
      public std::binary_function<gpstk::RinexNavData, 
         gpstk::RinexNavData, bool>
   {
   public:
      bool operator()(const gpstk::RinexNavData& l,
                      const gpstk::RinexNavData& r) const
         {
            gpstk::GPSWeekSecond lXmitTime(l.weeknum, (double)l.HOWtime);
            gpstk::GPSWeekSecond rXmitTime(r.weeknum, (double)r.HOWtime);

            if (lXmitTime < rXmitTime)
               return true;
            return false;
         }
   };

      /// Combines RinexNavHeaders into a single header, combining comments
      /// This assumes that
      /// all the headers come from the same station for setting the other
      /// header fields. After running touch() on a list of RinexNavHeader,
      /// the internal theHeader will be the merged header data for
      /// those files.
   struct RinexNavHeaderTouchHeaderMerge :
      public std::unary_function<gpstk::RinexNavHeader, bool>
   {
   public:
      RinexNavHeaderTouchHeaderMerge()
            : firstHeader(true)
         {}

      bool operator()(const gpstk::RinexNavHeader& l)
         {
            if (firstHeader)
            {
               theHeader = l;
               firstHeader = false;
            }
            else
            {
               std::set<std::string> commentSet;

                  // insert the comments to the set
                  // and let the set take care of uniqueness
               copy(theHeader.commentList.begin(),
                    theHeader.commentList.end(),
                    inserter(commentSet, commentSet.begin()));
               copy(l.commentList.begin(),
                    l.commentList.end(),
                    inserter(commentSet, commentSet.begin()));
                  // then copy the comments back into theHeader
               theHeader.commentList.clear();
               copy(commentSet.begin(), commentSet.end(),
                    inserter(theHeader.commentList,
                             theHeader.commentList.begin()));
            }
            return true;
         }

      bool firstHeader;
      gpstk::RinexNavHeader theHeader;
   };

      /// Filter based on PRN ID.
   struct RinexNavDataFilterPRN : 
      public std::unary_function<gpstk::RinexNavData,  bool>
   {
   public:
      RinexNavDataFilterPRN(const std::list<long>& lst )
         :prnList(lst)
         {}
         /// This should return true when the data are to be erased
      bool operator()(const gpstk::RinexNavData& l) const
         {
            long testValue = (long) l.PRNID;
            return find(prnList.begin(), prnList.end(), testValue )
                                                       == prnList.end(); 
         }
   private:
      std::list<long> prnList;
   };

   //@}

}


#endif

