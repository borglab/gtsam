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
 * @file RinexEphemerisStore.hpp
 * Read and store Rinex formated ephemeris data
 */

#ifndef GPSTK_RINEX_EPHEMERIS_STORE_HPP
#define GPSTK_RINEX_EPHEMERIS_STORE_HPP

#include <iostream>

#include "GPSEphemerisStore.hpp"
#include "FileStore.hpp"
#include "RinexNavStream.hpp"
#include "RinexNavData.hpp"
#include "RinexNavHeader.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   class RinexEphemerisStore : public GPSEphemerisStore,
                               public FileStore<RinexNavHeader>
   {
   public:
      RinexEphemerisStore()
         throw()
      { GPSEphemerisStore(); }

      /// destructor
      virtual ~RinexEphemerisStore() {}
      
      /** Dump the store to cout.
       * @param detail determines how much detail to include in the output
       *   0 list of filenames with their start, stop times.
       *   1 list of filenames with their start, stop times,
       *     other header information and prns/accuracy.
       *   2 above, plus dump all the PVT data (use judiciously).
       */
      void dump(std::ostream& s=std::cout, short detail=0)
         const throw();

      /// load the given Rinex file
      void loadFile(const std::string& filename) 
         throw(FileMissingException);
   };

   //@}

}  // namespace

#endif
