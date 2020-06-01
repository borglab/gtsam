#pragma ident "$Id$"



/**
 * @file FICHeader.cpp
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






#include "StringUtils.hpp"
#include "FICHeader.hpp"
#include "FICStream.hpp"
#include "FICAStream.hpp"

using namespace gpstk::StringUtils;

const int gpstk::FICHeader::headerSize = 40;


namespace gpstk
{
   using namespace std;

   
   void FICHeader::reallyPutRecord(FFStream& ffs) const 
      throw(std::exception, gpstk::StringUtils::StringException, 
            gpstk::FFStreamError)
   {
      string theHeader(header);
      if(FICStream* strm = dynamic_cast<FICStream*>(&ffs))
      {
            // This is a binary FIC stream, so
            // send the 40 character header, truncated or padded 
            //  with ' ' as needed.
         *strm << leftJustify(theHeader, headerSize, ' ');
      }
      else if (FICAStream* ficas = dynamic_cast<FICAStream*>(&ffs))
      {
            // If this is a FICA stream, add some extra stuff as well as
            // send the 40 character header, truncated or padded 
            //  with ' ' as needed.
         *ficas << "    " << leftJustify(theHeader, headerSize, ' ') << '\n';
      }
      else
      {
         gpstk::FFStreamError e("Attempt to write a FICHeader object"
                                " to a non-FIC(A)Stream FFStream.");
         GPSTK_THROW(e);
      }
   }

   void FICHeader::dump(ostream& s) const 
   {
      s << header << endl;
   };

   void FICHeader::reallyGetRecord(FFStream& ffs)
      throw(std::exception, gpstk::StringUtils::StringException, 
            gpstk::FFStreamError)
   {
      FICStreamBase *fsb = dynamic_cast<FICStreamBase *>(&ffs);
      if(fsb == NULL)
      {
         gpstk::FFStreamError e("Attempt to read a FICHeader object"
                                " from a non-FICStreamBase FFStream.");
         GPSTK_THROW(e);
      }

      char c[headerSize + 1];
      
         // if this is a FICA stream, get 4 characters
      FICAStream* ficas = dynamic_cast<FICAStream*>(&ffs);
      
      if (ficas)
      {
         const int blankChrs = 4;
         char whitespaces[blankChrs + 1];
         ffs.read(whitespaces, blankChrs);
      }
      
      ffs.read(c, headerSize);
      if (ffs.gcount() != headerSize)
      {
         FFStreamError e("Error reading header");
         GPSTK_THROW(e);
      }
      
      c[headerSize]='\0';
      header = c;
      fsb->headerRead=true;
      fsb->header.header = header;
      
      if (ficas)
      {
         string line;
         ficas->formattedGetLine(line);
         ficas->formattedGetLine(line);
      }
   }  // end of FICHeader::getRecord()

} // namespace gpstk
