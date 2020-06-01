#pragma ident "$Id$"



/**
 * @file RinexObsFilterOperators.hpp
 * Operators for FileFilter using Rinex observation data
 */

#ifndef GPSTK_RINEXOBSFILTEROPERATORS_HPP
#define GPSTK_RINEXOBSFILTEROPERATORS_HPP

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






#include "FileFilter.hpp"
#include "RinexObsData.hpp"
#include "RinexObsHeader.hpp"

#include <set>
#include <algorithm>

namespace gpstk
{
   /** @addtogroup RinexObs */
   //@{

      /// This compares all elements of the RinexObsData with less than
      /// (only for those fields which the two obs data share).
   struct RinexObsDataOperatorLessThanFull :
      public std::binary_function<gpstk::RinexObsData,
         gpstk::RinexObsData, bool>
   {
   public:
         /// The set is a set of RinexObsType that the two files have in
         /// common.  This is easily generated with the set_intersection
         /// STL function.  See difftools/rowdiff.cpp for an example.
      RinexObsDataOperatorLessThanFull
      (const std::set<gpstk::RinexObsType>& rohset)
            : obsSet(rohset)
         {}

      bool operator()(const gpstk::RinexObsData& l,
                      const gpstk::RinexObsData& r) const
         {
               // compare the times, offsets, then only those elements
               // that are common to both.  this ignores the flags
               // that are set to 0
            if (l.time < r.time)
               return true;
            else if (l.time == r.time)
            {
               if (l.epochFlag < r.epochFlag)
                  return true;
               else if (l.epochFlag == r.epochFlag)
               {
                  if (l.clockOffset < r.clockOffset)
                     return true;
                  else if (l.clockOffset > r.clockOffset)
                     return false;
               }
               else
                  return false;
            }
            else
               return false;

               // for the obs, first check that they're the same size
               // i.e. - contain the same number of PRNs
            if (l.obs.size() < r.obs.size())
               return true;

            if (l.obs.size() > r.obs.size())
               return false;

               // then check that each PRN has the same data for each of the
               // shared fields
            gpstk::RinexObsData::RinexSatMap::const_iterator lItr =
               l.obs.begin(), rItr;

            gpstk::SatID sat;

            while (lItr != l.obs.end())
            {
               sat = (*lItr).first;
               rItr = r.obs.find(sat);
               if (rItr == r.obs.end())
                  return false;

               gpstk::RinexObsData::RinexObsTypeMap
                  lObs = (*lItr).second,
                  rObs = (*rItr).second;

               std::set<gpstk::RinexObsType>::const_iterator obsItr =
                  obsSet.begin();

               while (obsItr != obsSet.end())
               {
                  gpstk::RinexDatum lData, rData;
                  lData = lObs[*obsItr];
                  rData = rObs[*obsItr];

                  if (lData.data < rData.data)
                     return true;

                  if ( (lData.lli != 0) && (rData.lli != 0) )
                     if (lData.lli < rData.lli)
                        return true;

                  if ( (lData.ssi != 0) && (rData.ssi != 0) )
                     if (lData.ssi < rData.ssi)
                        return true;

                  obsItr++;
               }

               lItr++;
            }

               // the data is either == or > at this point
            return false;
         }

   private:
      std::set<gpstk::RinexObsType> obsSet;
   };

      /// This is a much faster less than operator for RinexObsData,
      /// only checking time
   struct RinexObsDataOperatorLessThanSimple :
      public std::binary_function<gpstk::RinexObsData,
         gpstk::RinexObsData, bool>
   {
   public:
      bool operator()(const gpstk::RinexObsData& l,
                      const gpstk::RinexObsData& r) const
         {
            if (l.time < r.time)
               return true;
            return false;
         }
   };

      /// This simply compares the times of the two records
      /// for equality
   struct RinexObsDataOperatorEqualsSimple :
      public std::binary_function<gpstk::RinexObsData,
         gpstk::RinexObsData, bool>
   {
   public:
      bool operator()(const gpstk::RinexObsData& l,
                      const gpstk::RinexObsData& r) const
         {
            if (l.time == r.time)
               return true;
            return false;
         }
   };

      /// Combines RinexObsHeaders into a single header, combining comments
      /// and adding the appropriate RinexObsTypes.  This assumes that
      /// all the headers come from the same station for setting the other
      /// header fields. After running touch() on a list of RinexObsHeader,
      /// the internal theHeader will be the merged header data for
      /// those files and obsSet will be the set of RinexObsTypes that
      /// will be printed to the file.
   struct RinexObsHeaderTouchHeaderMerge :
      public std::unary_function<gpstk::RinexObsHeader, bool>
   {
   public:
      RinexObsHeaderTouchHeaderMerge()
            : firstHeader(true)
         {}

      bool operator()(const gpstk::RinexObsHeader& l)
         {
            if (firstHeader)
            {
               theHeader = l;
               firstHeader = false;
            }
            else
            {
               std::set<gpstk::RinexObsType> thisObsSet,
                  tempObsSet;
               std::set<std::string> commentSet;
               obsSet.clear();

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

                  // find the set intersection of the obs types
               copy(theHeader.obsTypeList.begin(),
                    theHeader.obsTypeList.end(),
                    inserter(thisObsSet, thisObsSet.begin()));
               copy(l.obsTypeList.begin(),
                    l.obsTypeList.end(),
                    inserter(tempObsSet, tempObsSet.begin()));
               set_intersection(thisObsSet.begin(), thisObsSet.end(),
                                tempObsSet.begin(), tempObsSet.end(),
                                inserter(obsSet, obsSet.begin()));
                  // then copy the obsTypes back into theHeader
               theHeader.obsTypeList.clear();
               copy(obsSet.begin(), obsSet.end(),
                    inserter(theHeader.obsTypeList,
                             theHeader.obsTypeList.begin()));
            }
            return true;
         }

      bool firstHeader;
      gpstk::RinexObsHeader theHeader;
      std::set<gpstk::RinexObsType> obsSet;
   };

   //@}

}


#endif
