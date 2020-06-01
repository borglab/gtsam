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
 * @file YumaData.hpp
 * Encapsulate Yuma Almanac file data, including I/O
 */

#ifndef YUMADATA_HPP
#define YUMADATA_HPP

#include <vector>
#include <list>
#include <map>

#include "FFStream.hpp"
#include "AlmOrbit.hpp"
#include "YumaBase.hpp"
#include "YumaHeader.hpp"
#include "StringUtils.hpp"

namespace gpstk
{
   /** @addtogroup Yuma */
   //@{

      /** 
       * This class stores, reads, and writes Yuma records. 
       *
       * @sa tests/Yuma for examples
       * @sa YumaStream.
       * @sa YumaHeader for information on writing Yuma files.
       */ 
   class YumaData : public YumaBase
   {
   public:
         /// Constructor.
      YumaData() {}

         /// Destructor
      virtual ~YumaData() {}
      
         /// This is is the nearest full GPS week to the 10-bit week
         /// available in the SEM file.  If this value is 0 it is ignored.
         /// Otherwise, the 10-bit week is moved into the GPS Epoch
         /// centered on the given full week.
      static short nearFullWeek;

      static const std::string sID;     // ID label string
      static const std::string sHlth;   // Satellite Health string
      static const std::string sEcc;    // Eccentricity string
      static const std::string sTOA;
      static const std::string sOrbI;
      static const std::string sRRA;
      static const std::string sSqrA;
      static const std::string sRtAs;
      static const std::string sArgP;
      static const std::string sMnAn;
      static const std::string sAf0;
      static const std::string sAf1;
      static const std::string sweek; 
      
      
      short PRN;
      short week;
      short SV_health;
      double ecc;
      long Toa;
      double i_offset;
      double OMEGAdot;
      double Ahalf;
      double OMEGA0;
      double w;
      double M0;
      double AF0;
      double AF1;
      long xmit_time;
      
      
         /**
          * Debug output function. 
          * Dump the contents of each of the Yuma class to a
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
      void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException);  
  
         /**
          * This functions obtains a Yuma almanac record from the given 
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

      std::string lineParser(const std::string& line, const std::string& s)
         const throw(FFStreamError);
      
   }; // class YumaData

   //@}

} // namespace

#endif
