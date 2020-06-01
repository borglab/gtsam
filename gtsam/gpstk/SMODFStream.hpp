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
 * @file SMODFStream.hpp
 * Read/Write smoothed measurement data
 */

#ifndef SMODFSTREAM_HPP
#define SMODFSTREAM_HPP

#include "FFTextStream.hpp"

namespace gpstk
{
      /** @addtogroup icd211group ICD-GPS-211 Classes */
      //@{

      /**
       * This is a stream used to obtain data from a 
       * Smoothed Measurement Data File.
       */
   class SMODFStream : public gpstk::FFTextStream
   {
   public:
      SMODFStream()
            : format(undefined)
         {}

         /**
          * Constructor.
          * @param fn the SMODF to open
          * @param mode the ios::openmode to use in opening \a fn
          */
      SMODFStream(const char* fn, std::ios::openmode mode=std::ios::in)
         throw()
            : gpstk::FFTextStream(fn, mode), format(undefined)
         {
               // open a file for write in icd211 mode
            if  ( (mode & std::ios::out) && !(mode & std::ios::in) )
               format = icd211;
         }

         /// Destructor per the coding standard
      virtual ~SMODFStream() {}

         /// overrides open to reset the header
      virtual void open(const char* fn, std::ios::openmode mode)
         { 
            gpstk::FFTextStream::open(fn, mode);
               // open a file for write in icd211 mode
            if  ( (mode & std::ios::out) && !(mode & std::ios::in) )
               format = icd211;
            else
               format = undefined;
         }

         /** 
          * These are the file formats SMODFStream recognizes.
          */
      enum FileFormat {
         undefined,      ///< Undefined format
         legacy,         ///< Legacy format
         icd211          ///< ICD-GPS-211 format
      };

      FileFormat format; ///< The FileFormat of the file currently being read.
   }; // class SMODFStream

      //@}

} // namespace sglmsn

#endif
