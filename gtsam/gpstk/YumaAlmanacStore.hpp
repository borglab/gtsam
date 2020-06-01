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
// This software developed by Applied Research Laboratories at the University
// of Texas at Austin, under contract to an agency or agencies within the U.S. 
// Department of Defense. The U.S. Government retains all rights to use,
// duplicate, distribute, disclose, or release this software. 
//
// Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

/**
 * @file YumaAlmanacStore.hpp
 * Store Yuma almanac information, and access by satellite and time
 */
 
#ifndef YUMAALMANACSTORE_HPP
#define YUMAALMANACSTORE_HPP

#include "GPSAlmanacStore.hpp"
#include "FileStore.hpp"
#include "YumaData.hpp"
#include "YumaStream.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{
   class YumaAlmanacStore : public FileStore<YumaHeader>, 
                            public GPSAlmanacStore
   {
   public:
      YumaAlmanacStore(const CommonTime& dtInterest =
                             CommonTime::BEGINNING_OF_TIME)
      {
         timeOfInterest = dtInterest;
      }
      
      void loadFile(const std::string& filename) 
         throw(FileMissingException);
         
      CommonTime timeOfInterest;
   };

   
   //@}
}

#endif
