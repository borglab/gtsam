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

/**
 * @file SEMHeader.hpp
 * Encapsulate SEM Almanac header data, including I/O
 */

#ifndef SEMHEADER_HPP
#define SEMHEADER_HPP

#include <vector>
#include <list>
#include <map>

#include "FFStream.hpp"
#include "AlmOrbit.hpp"
#include "SEMBase.hpp"
#include "StringUtils.hpp"

namespace gpstk
{
   /** @addtogroup SEM */
   //@{

      /** 
       * This class stores, reads, and writes SEM records. 
       * @warning The SEM header information and data information don't
       * correctly talk to each other at the time of completion of this file.
       * The current fix is in SEMAlamanacStore.hpp.
       *
       * @sa tests/SEM for examples
       * @sa SEMStream.
       * @sa SEMHeader for information on writing SEM files.
       */
   class SEMHeader : public SEMBase
   {
   public:
         /// Constructor.
      SEMHeader() {}

         /// Destructor
      virtual ~SEMHeader() {}

         /// This is is the nearest full GPS week to the 10-bit week
         /// available in the SEM file.  If this value is 0 it is ignored.
         /// Otherwise, the 10-bit week is moved into the GPS Epoch
         /// centered on the given full week.
      static short nearFullWeek;

      short numRecords;
      std::string Title;
      short week;
      long Toa;
      
      
         /**
          * Debug output function. 
          * Dump the contents of each of the SEM header to a
          * given ostream \c s.
          */ 
      virtual void dump(std::ostream& s) const;
      
         //! This class is a "header" so this function always returns "true". 
      virtual bool isHeader() const {return true;}
   
      

   protected:      
	 /**
          * Writes a correctly formatted record from this header to stream \a s.
          */
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException);  
  
         /**
          * This functions obtains a SEM header record from the given 
          * FFStream.
          * If there is an error in reading from the stream, it is reset
          * to its original position and its fail-bit is set.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException);  
      
   }; // class SEMHeader

   //@}

} // namespace

#endif
