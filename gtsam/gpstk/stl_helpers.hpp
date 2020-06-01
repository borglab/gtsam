#pragma ident "$Id$"

/**
 * @file stl_helpers.hpp
 * Useful functions that take advantage of STL containers
 */

#ifndef GPSTK_STL_HELPERS_HPP
#define GPSTK_STL_HELPERS_HPP

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

#include <cmath>
#include <algorithm>
#include <list>

namespace gpstk
{
      /** @defgroup datastructsgroup STL helpers */
      //@{

      /// A simple way to get the max value of a list of numbers
   template<class For> For max(const std::list<For>& lst) 
   {
      return *max_element(lst.begin(), lst.end());
   }
   
      /// A simple way to get the minimum value of a list of numbers
   template<class For> For min(const std::list<For>& lst) 
   {
      return *min_element(lst.begin(), lst.end());
   }

      /// ListStats (i.e. Statistics on a list of numbers)
   template<class bt>
   struct ListStats
   {
         /// number of items used in the statistics taking
      unsigned n;
         /// statistics values
      bt mean, sigma, min, max;
         /// constructor
      ListStats():n(0),mean(0),sigma(0),min(0),max(0){};
   };

      /// Compute the statistics for a list of numbers.
      /// This algorithm is written to be stable in computing the standard
      /// deviation for sequences of number with a large mean value.
      /// (i.e. it doesn't accumulate sum of the value squared.)
   template<class bt> ListStats<bt> stats(const std::list<bt>& lst)
   {
      ListStats<bt> s;
      bt sum=0, sumsq=0;

      s.n = lst.size();
      if (s.n<1)
         return s;
      
      typename std::list<bt>::const_iterator li;
      li=lst.begin();
      s.min = s.max = *li;
      for(; li!=lst.end(); li++)
      {
         s.min = std::min(s.min, *li);
         s.max = std::max(s.max, *li);
         sum += *li;
      }
      s.mean = sum/s.n;

      if (s.n<2)
         return s;

      for(li=lst.begin(); li!=lst.end(); li++)
      {
         bt z=*li-s.mean;
         sumsq += z*z;
      }
   
      s.sigma = sqrt(sumsq/(s.n-1));

      return s;
   }

      /// find the index of the first element of a vector with a given value
      /// return -1 if not found
   template <class T> int vectorindex(const std::vector<T>& vec, const T& value) 
   {
      typename std::vector<T>::const_iterator it;
      it = find(vec.begin(), vec.end(), value);
      if(it == vec.end()) return -1;
      return int(it - vec.begin());
   }

      //@}

} // namespace
   
#endif
