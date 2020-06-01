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
 * @file SEMData.hpp
 * Encapsulate SEM Almanac file data, including I/O
 */

#ifndef SEMDATA_HPP
#define SEMDATA_HPP

#include <vector>
#include <list>
#include <map>

#include "FFStream.hpp"
#include "AlmOrbit.hpp"
#include "SEMBase.hpp"
#include "StringUtils.hpp"
#include "SEMHeader.hpp"

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
   class SEMData : public SEMBase
   {
   public:
   
         /// Constructor.
      SEMData() {}

         /// Destructor
      virtual ~SEMData() {}
      
         
      short PRN;
      short SVNnum;
      short URAnum;
      double ecc;
      double i_offset;
      double OMEGAdot;
      double Ahalf;
      double OMEGA0;
      double w;
      double M0;
      double AF0;
      double AF1;
      short SV_health;
      short satConfig;
      
      long xmit_time;
      
      long Toa;
      short week;
      
         /**
          * Debug output function. 
          * Dump the contents of each of the SEM class to a
          * given ostream \c s.
          */ 
      virtual void dump(std::ostream& s) const;
      
         //! This class is "data" so this function always returns "true". 
      virtual bool isData() const {return true;}

         /**
          * cast *this into an AlmOrbit
          * @return the constructed AlmOrbit object
          */
      operator AlmOrbit() const;
      
      
   protected:      
	 
	 /**
          * Writes a correctly formatted record from this data to stream \a s.
          */
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException);  
  
         /**
          * This functions obtains a SEM almanac record from the given 
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
      
   }; // class SEMData

   //@}

} // namespace

#endif
