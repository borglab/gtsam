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

#ifndef SMODFDATA_HPP
#define SMODFDATA_HPP

/**
 * @file SMODFData.hpp
 * smoothed measurement data file data
 */

#include <vector>

#include "StringUtils.hpp"
#include "FFData.hpp"
#include "CommonTime.hpp"

namespace gpstk
{
   /** @addtogroup icd211group ICD-GPS-211 Classes */
   //@{

   /**
    * Model for a Smoothed Measurement Data File Data Record.
    */
   class SMODFData : public gpstk::FFData
   {
   public:
      /// constructor
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
       SMODFData() : PRNID(0), time(gpstk::CommonTime::BEGINNING_OF_TIME)
      {}
#pragma clang diagnostic pop
      /// destructor
      virtual ~SMODFData() {}

      gpstk::CommonTime time;  ///< the date of this data (from year DOY, SOD)
      short PRNID;        ///< PRN number
      long  station;      ///< NIMA Monitor Station number (85408, etc.)
      short channel;      ///< receiver channel
      short type;         ///< Data type ( 0 = range, 9 = delta/doppler range )
      short lol;          ///< Loss of lock flag.  0 = OK, 1 = loss
                          ///<  ( only used for type = 9 )
      long double obs;    ///< Observed data (range or delta range meas)
      double stdDev;      ///< standard deviation of observation
      short tempSource;   ///< temperature source flag 
                          ///< (0 = not available, 1 = measured value, 2 = default value)
      short pressSource;  ///< pressure source flag
                          ///< (0 = not available, 1 = measured value, 2 = default value)
      short humidSource;  ///< humidity source flag
                          ///< (0 = not available, 1 = measured value, 2 = default value)
      double temp;        ///< temperature (degrees C)
      double pressure;    ///< pressure (mb)
      double humidity;    ///< relative humidity (%)

      /// SMODFData is data, so this function always returns true.
      virtual bool isData() const {return true;}

      virtual void dump(std::ostream& s) const;
      
         /// Convert a double to the "funny" format specified in the '211.
      static std::string doub2funny(const double& num,
                                    const std::string::size_type length,
                                    const std::string::size_type expLen);

   protected:
      /// Writes a smodfdata object in the format specified
      /// by the stream to the stream.
      virtual void reallyPutRecord(gpstk::FFStream& s) const
         throw(std::exception, gpstk::FFStreamError,
               gpstk::StringUtils::StringException);

      /**
       * Retrieve a SMODFData record from the given gpstk::FFStream.
       * If there is a problem with reading from the stream, it
       * is reset to its original position and its fail-bit is set.
       * @throws StringException when a gpstk::StringUtils function fails
       * @throws gpstk::FFStreamError when exceptions(failbit) is set and
       *  a read or formatting error occurs.  This also resets the
       *  stream to its pre-read position.
       */
      virtual void reallyGetRecord(gpstk::FFStream& s) 
         throw(std::exception, gpstk::FFStreamError,
               gpstk::StringUtils::StringException);
     
   private:
      static const int SMO_LEN_ICD211;  ///< Length of an ICD-GPS-211 SMODF record
      static const int SMO_LEN_LEGACY;  ///< Length of a Legacy SMODF record
      static const int BEGINGPS2DYEAR;  ///< Beginning of the GPS Two Digit Year
   }; // class SMODFData

      //@}

} // namespace gpstk

#endif
