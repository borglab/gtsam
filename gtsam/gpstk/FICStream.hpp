#pragma ident "$Id$"



/**
 * @file FICStream.hpp
 * gpstk::FICStream - binary FIC file stream container.
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






#ifndef FICSTREAM_HPP
#define FICSTREAM_HPP

#include <vector>
#include <map>

#include "FICStreamBase.hpp"
#include "FFBinaryStream.hpp"
#include "FICHeader.hpp"

namespace gpstk
{
      /** 
       * This is a stream used to obtain data from a binary FIC File.
       * 
       * \sa fic_test.cpp and fic_read_write.cpp for an example.
       *
       * \sa FICData and FICHeader.
       */
   class FICStream : public FFBinaryStream, public FICStreamBase
   {
   public:
         /// default constructor
      FICStream() {}

         /**
          * Constructor
          * @param fn the name of the FIC file to be opened
          * @param mode the ios::openmode to be used on \a fn
          */
      FICStream(const char* fn,
                std::ios::openmode mode=std::ios::in|std::ios::binary)
            : FFBinaryStream(fn, mode)
         {}

         /// destructor per the coding standards
      virtual ~FICStream() {}

         /// overrides open to reset the header
      virtual void open(const char* fn, std::ios::openmode mode)
         { FFBinaryStream::open(fn, mode); FICStreamBase::open(); }

   }; // class FICStream
} // namespace gpstk

#endif
