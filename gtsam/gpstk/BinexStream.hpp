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
 * @file BinexStream.hpp
 * File stream for RINEX meteorological files
 */

#ifndef GPSTK_BINEXSTREAM_HPP
#define GPSTK_BINEXSTREAM_HPP

#include "FFBinaryStream.hpp"

namespace gpstk
{
   /** @addtogroup Binex */
   //@{

      /**
       * This class performs file i/o on a BINEX file for the 
       * BinexData classes.
       *
       * @sa binex_read_write.cpp for an example.
       * @sa binex_test.cpp for an example.
       * @sa BinexData.
       *
       */
   class BinexStream : public FFBinaryStream
   {
   public:
         /// Destructor
      virtual ~BinexStream() {}
      
         /// Default constructor
      BinexStream() {}
      
         /** Constructor 
          * Opens a file named \a fn using ios::openmode \a mode.
          */
      BinexStream(const char* fn,
                  std::ios::openmode mode=std::ios::in | std::ios::binary)
            : FFBinaryStream(fn, mode) {};

         /// Opens a file named \a fn using ios::openmode \a mode.
      virtual void
      open(const char* fn, std::ios::openmode mode)
      { 
         FFBinaryStream::open(fn, mode); 
      }
   };

   //@}

} // namespace gpstk

#endif // GPSTK_BINEXSTREAM_HPP
