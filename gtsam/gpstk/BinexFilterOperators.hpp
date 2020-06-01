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
//  Copyright 2008, The University of Texas at Austin
//
//============================================================================

/**
 * @file BinexFilterOperators.hpp
 * Operators for FileFilter using Binex data
 */

#ifndef GPSTK_BINEXFILTEROPERATORS_HPP
#define GPSTK_BINEXFILTEROPERATORS_HPP

#include "FileFilter.hpp"
#include "BinexData.hpp"

#include <set>

namespace gpstk
{
   /** @addtogroup Binex */
   //@{

   typedef std::binary_function<BinexData, BinexData, bool> BinexDataBinaryOperator;


      /// Determine if two BinexData objects are equal.
   struct BinexDataOperatorEquals : 
      public BinexDataBinaryOperator
   {
   public:
      bool operator()(const BinexData& l,
                      const BinexData& r) const
         {
            return (l == r);
         }
   };

   //@}

}

#endif // GPSTK_BINEXFILTEROPERATORS_HPP
