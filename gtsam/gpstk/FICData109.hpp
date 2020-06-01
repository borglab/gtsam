#pragma ident "$Id$"



/**
 * @file FICData109.hpp
 * Augment the FICData class to provide the ability to load
 * the FICData specifically for Block 109
 */

#ifndef GPSTK_FICDATA109_HPP
#define GPSTK_FICDATA109_HPP

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






#include "FICData.hpp"

namespace gpstk
{
   class FICData109 : public FICData
   {
   public:
         /// Default constructor
      FICData109( const short PRNID,
                  const std::vector<uint32_t> sf1,
                  const std::vector<uint32_t> sf2,
                  const std::vector<uint32_t> sf3 );
      
         /// Destructor
      virtual ~FICData109() {}

   }; // class FICData109

   //@}

} // namespace

#endif
