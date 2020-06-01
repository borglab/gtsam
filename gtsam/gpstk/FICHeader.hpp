#pragma ident "$Id$"



/**
 * @file FICHeader.hpp
 * gpstk::FICHeader - container for the FIC file header data.
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






#ifndef FICHEADER_HPP
#define FICHEADER_HPP

#include "FFStream.hpp"
#include "FICBase.hpp"

namespace gpstk
{
      /**
       * This is the Header for the FIC File Model.
       * There is one 40 character header at the start of each FIC file,
       * ASCII or binary.
       * 
       * \sa fic_test.cpp, fic_read_write.cpp, and fica_test.cpp for examples.
       *
       * \sa FICStream, FICAStream, and FICData.
       */
   class FICHeader : public FICBase
   {
   public:
         /// Default constructor
      FICHeader() {}

         /// Destructor
      virtual ~FICHeader() {}

         /// FICHeader is a header, so this function always returns true.
      virtual bool isHeader() const {return true;}
     
         /**
          * This function does \b nothing.
          */
      virtual void dump(std::ostream& s) const;

         /// The header string.
      std::string header;

         /// constant for the header size.
      static const int headerSize;

   protected:
         /// Writes the header string to the FFStream \a s.
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, gpstk::StringUtils::StringException, 
               gpstk::FFStreamError);

         /**
          * Retrieve the header string from the FFStream \a s.
          * If the read fails for some reason, the stream will
          * be reset to its original position, and its fail-bit
          * will be set.
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, gpstk::StringUtils::StringException, 
               gpstk::FFStreamError);

   }; // class FICHeader
} // namespace gpstk

#endif
