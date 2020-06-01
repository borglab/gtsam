#pragma ident "$Id$"


/**
 * @file YumaStream.hpp
 * gpstk::YumaStream - ASCII FIC file stream container.
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


#ifndef YUMASTREAM_HPP
#define YUMASTREAM_HPP


#include "FFTextStream.hpp"
#include "YumaHeader.hpp"

namespace gpstk
{
   /** @addtogroup Yuma */
   //@{

      /**
       * This class performs file i/o on a Yuma file for the 
       * YumaHeader and YumaData classes.
       *
       * @sa tests/Yuma for examples.
       * @sa YumaData.
       * @sa YumaHeader for information on writing Yuma files.
       *
       */
   class YumaStream : public FFTextStream
   {
   public:
         /// Default constructor
      YumaStream() {}
      
         /**
          * Constructor
          * @param fn the name of the ascuu FIC file to be opened
          * @param mode the ios::openmode to be used on \a fn
          */
      YumaStream(const char* fn,
                std::ios::openmode mode=std::ios::in)
            : FFTextStream(fn, mode), headerRead(false) {};

         /// destructor per the coding standards
      virtual ~YumaStream() {}

         /// overrides open to reset the header
      virtual void open(const char* fn, std::ios::openmode mode)
         {
	    FFTextStream::open(fn, mode);
	    headerRead = false;
	    header = YumaHeader();
	 }
	 /// YumaHeader for this file
      YumaHeader header;
      
         /// Flag showing whether or not the header has been read.
      bool headerRead;

   }; // class YumaStream
   
   //@}
   
} // namespace gpstk

#endif
