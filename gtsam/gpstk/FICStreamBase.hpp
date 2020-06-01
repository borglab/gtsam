#pragma ident "$Id$"



/**
 * @file FICStreamBase.hpp
 * gpstk::FICStreamBase stores common FIC stream data.
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






#ifndef FICSTREAMBASE_HPP
#define FICSTREAMBASE_HPP

#include <vector>
#include <map>

#include "FICHeader.hpp"

namespace gpstk
{
      /** 
       * Encapsulates FIC stream data for FIC and FICA files.
       */
   class FICStreamBase
   {
   public:
         /// Default constructor
      FICStreamBase()
            : headerRead(false)
         {}

         /// destructor per the coding standards
      virtual ~FICStreamBase() {}
      
         /// resets the header info for derived classes
      void open()
         { headerRead = false;  header = FICHeader(); }

         /// Whether or not the header's been read for this file.
      bool headerRead;  
         /// The FICHeader object for this file.
      FICHeader header; 

   };
}

#endif
