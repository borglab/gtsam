#pragma ident "$Id$"



/**
 * @file FICFilterOperators.hpp
 * gpstk::FICFilterOperators - FIC filter operators.
 */

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






#ifndef FICFILTEROPERATORS_HPP
#define FICFILTEROPERATORS_HPP

#include "FileFilter.hpp"
#include "FICData.hpp"

namespace gpstk
{
      /// Removes all data that doesn't match the given SVID/PRN numbers.
   struct FICDataFilterPRN :
      public std::unary_function<gpstk::FICData, bool>
   {
   public:
      FICDataFilterPRN(const std::list<long>& lst)
         : prnList(lst)
         {}

         /// This should return true when the data is to be erased.
      bool operator() (const gpstk::FICData& l) const
         {
               // this criteria is the same as the r44 navdump criteria
            switch (l.blockNum)
            {
               case 9:
                  return find(prnList.begin(), prnList.end(), l.f[19]) ==
                     prnList.end();
                  break;

               case 109:
                  return find(prnList.begin(), prnList.end(), l.i[1]) ==
                     prnList.end();
                  break;

                     // For 62 and 162, all SVIDs 51-63 will be added
                     //  as well as any PRNs 1-32 that were selected.

               case 62:
                  return find(prnList.begin(), prnList.end(), l.i[3]) ==
                     prnList.end();
                  
                  break;

               case 162:
                  return find(prnList.begin(), prnList.end(), l.i[0]) ==
                     prnList.end();
                  break;

               default:
                  return true;
                  break;
            }
         }

   private:
      std::list<long> prnList;
   };

      /// Removes all data that doesn't match the given block numbers
   struct FICDataFilterBlock :
      public std::unary_function<gpstk::FICData, bool>
   {
   public:
      FICDataFilterBlock(const std::list<long>& lst)
         : blockList(lst)
         {}

      /// This should return true when the data is to be erased.
      bool operator() (const gpstk::FICData& l) const
         {
            return find(blockList.begin(), blockList.end(), l.blockNum) ==
               blockList.end();
         }

   private:
      std::list<long> blockList;
   };

   struct FICDataFilterStartTime :
      public std::unary_function<gpstk::FICData, bool>
   {
   public:
      FICDataFilterStartTime(const gpstk::CommonTime start)
         : stime(start)
         {}

      /// This should return true when the data is to be erased.
      bool operator() (const gpstk::FICData& l) const
         {
            CommonTime dt;
            if(l.getTransmitTime(dt)) //if valid trasmit time
              return dt <= stime;
            else
              return true; // exclude data if no transmit time
         }

   private:
      gpstk::CommonTime stime;
   };

   struct FICDataFilterEndTime :
      public std::unary_function<gpstk::FICData, bool>
   {
   public:
      FICDataFilterEndTime(const gpstk::CommonTime end)
         : etime(end)
         {}

      /// This should return true when the data is to be erased.
      bool operator() (const gpstk::FICData& l) const
         {
            CommonTime dt;
            if(l.getTransmitTime(dt)) //if valid trasmit time
              return dt >= etime;
            else
              return true; // exclude data if no transmit time
         }

   private:
      gpstk::CommonTime etime;
   };

      /// Finds all data that matches the given block numbers
   struct FICDataFindBlock :
      public std::unary_function<gpstk::FICData, bool>
   {
   public:
      FICDataFindBlock(const std::list<long>& lst)
         : blockList(lst)
         {}

      bool operator() (const gpstk::FICData& l) const
         {
            return find(blockList.begin(), blockList.end(), l.blockNum) !=
               blockList.end();
         }
   private:
      std::list<long> blockList;
   };

      /// Sorting only for block 9 FICData
   struct FICDataOperatorLessThanBlock9 : 
      public std::binary_function<gpstk::FICData, gpstk::FICData, bool>
   {
   public:
      bool operator() (const gpstk::FICData& l, 
                       const gpstk::FICData& r) const
         {
            if ( (l.blockNum != 9) || (r.blockNum != 9) )
               return false;

               // sort by transmit time, prn
            if (l.f[5] < r.f[5])
               return true;
            else if (l.f[5] == r.f[5])
               {if (l.f[33] < r.f[33])
                  return true;
               else if (l.f[33] == r.f[33])
                  if (l.f[19] < r.f[19])
                     return true;}
            
            return false;
         }
   };

      /// Sorting only for block 109 FICData
   struct FICDataOperatorLessThanBlock109 :
      public std::binary_function<gpstk::FICData, gpstk::FICData, bool>
   {
   public:
      bool operator() (const gpstk::FICData& l, 
                       const gpstk::FICData& r) const
         {
            if ( (l.blockNum != 109) || (r.blockNum != 109) )
               return false;

               // sort by transmit time, prn
            if(l.i[0] < r.i[0])  // week numbers
               return true;
            else if(l.i[0] == r.i[0])
            {
                  // crack the HOW.  Note: I know I'm not multiplying by 6.
               if( ((l.i[3] >> 13) & 0x1FFFF) < 
                   ((r.i[3] >> 13) & 0x1FFFF) )
                  return true;
               else if( ((l.i[3] >> 13) & 0x1FFFF) == 
                        ((r.i[3] >> 13) & 0x1FFFF) )
                  if(l.i[1] < r.i[1])
                     return true;
            }
               
            return false;
         }
   };

      /// Useful for FICDiff and others...
   struct FICDataOperatorLessThanFull : 
      public std::binary_function<gpstk::FICData, gpstk::FICData, bool>
   {
   public:
      bool operator() (const gpstk::FICData& l, 
                       const gpstk::FICData& r) const
         {
            if (l.blockNum < r.blockNum)
               return true;
            if (l.blockNum > r.blockNum)
               return false;

            if ( (l.f.size() < r.f.size()) ||
                 (l.i.size() < r.i.size()) ||
                 (l.c.size() < r.c.size()))
               return true;

            if ( (l.f.size() > r.f.size()) ||
                 (l.i.size() > r.i.size()) ||
                 (l.c.size() > r.c.size()))
               return false;
            
               // ok, they're the same block and type of data - check
               // the individual contents
            std::vector<double>::size_type findex;
            for (findex = 0; findex < l.f.size(); findex++)
            {
               if (l.f[findex] < r.f[findex])
                  return true;
               if (l.f[findex] > r.f[findex])
                  return false;
            }

            std::vector<long>::size_type iindex;
            for (iindex = 0; iindex < l.i.size(); iindex++)
            {
               if (l.i[iindex] < r.i[iindex])
                  return true;
               if (l.i[iindex] > r.i[iindex])
                  return false;
            }

            std::vector<char>::size_type cindex;
            for (cindex = 0; cindex < l.c.size(); cindex++)
            {
               if (l.c[cindex] < r.c[cindex])
                  return true;
               if (l.c[cindex] > r.c[cindex])
                  return false;
            }
               // they're equal
            return false;
         }
   };

      /// Uniqueness operator for block 9 FIC data
   struct FICDataUniqueBlock9 : 
      public std::binary_function<gpstk::FICData, gpstk::FICData, bool>
   {
   public:
      bool operator() (const gpstk::FICData& l, 
                       const gpstk::FICData& r) const
         {
            if ( (l.blockNum != 9) || (r.blockNum != 9) )
               return false;
               // the unique criteria are PRN, week, IODC, AS/alert bits
            return ( (l.f[19] == r.f[19]) &&
                     (l.f[3] == r.f[3]) &&
                     (l.f[5] == r.f[5]) &&
                     (l.f[23] == r.f[23]) &&
                     (l.f[43] == r.f[43]) &&
                     ( (l.f[9] / 2048) == (r.f[9] / 2048) ) );
         }
   };

      /// Uniqueness operator for block 109 FIC data
   struct FICDataUniqueBlock109 : 
      public std::binary_function<gpstk::FICData, gpstk::FICData, bool>
   {
   public:
      bool operator() (const gpstk::FICData& l, 
                       const gpstk::FICData& r) const
      {
         if ( (l.blockNum != 109) || (r.blockNum != 109) )
            return false;
            // the unique criteria are PRN, week, IODC, AS/alert bits
         return ( (l.i[1] == r.i[1]) &&
                  (l.i[0] == r.i[0]) &&
                     // crack IODC on subframe 1
                  ((((l.i[4] << 2) & 0x00000300) + ((l.i[9] >> 22) & 0xFF)) ==
                   (((r.i[4] << 2) & 0x00000300) + ((r.i[9] >> 22) & 0xFF))) &&
                      // crack AS/Alert on subframe 1
                  ( ((l.i[4] >> 11) & 0x3) == ((r.i[4] >> 11) & 0x3) ) &&
                     // crack AS/Alert on subframe 2
                  ( ((l.i[14] >> 11) & 0x3) == ((r.i[14] >> 11) & 0x3) ) &&
                     // crack AS/Alert on subframe 3
                  ( ((l.i[24] >> 11) & 0x3) == ((r.i[24] >> 11) & 0x3) ) );
      }
   };
   
      /// Uniqueness operator for block 62 FIC data
   struct FICDataUniqueBlock62:
      public std::binary_function<gpstk::FICData, gpstk::FICData, bool>
   {
   public:
      bool operator() (const gpstk::FICData& l,
                       const gpstk::FICData& r) const
         {
            if ( (l.blockNum != 62) || (r.blockNum != 62) )
               return false;
               // the unique criteria are:
                        // SV ID - dimensionless
  
            return ( (l.i[3] == r.i[3]) &&
                        // transmit week
                     (l.i[5] == r.i[5]) &&
                        // reference week - full GPS week number
                     (l.f[18] == r.f[18]) &&
                        // toa (time of epoch) - GPS sec of week
                     (l.f[8] == r.f[8]) &&
                        // AS/alert bit - dimensionless
                     (l.f[3] == r.f[3]) );         
         }   
   };
   
         /// Uniqueness operator for block 162 FIC data
   struct FICDataUniqueBlock162:
      public std::binary_function<gpstk::FICData, gpstk::FICData, bool>
   {
   public:
      bool operator() (const gpstk::FICData& l,
                       const gpstk::FICData& r) const
         {
           if ( (l.blockNum != 162) || (r.blockNum != 162) )
              return false;
               // the unique criteria are: 
                        // SV ID  - dimensionless
            return ( (l.i[0] == r.i[0]) &&
                        // transmit week
                     (l.i[14] == r.i[14]) &&
                        // reference week - full GPS week number
                     (l.i[13] == r.i[13]) &&
                        // toa (time of week) - GPS sec week
                     ((((l.i[2] & 0x3FFFFFFFL) >> 13) * 6) == 
                      (((r.i[2] & 0x3FFFFFFFL) >> 13) * 6)) );       
         }   
   };
   
}

#endif

