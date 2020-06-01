#pragma ident "$Id$"

/**
 * @file OrbElemLNav.hpp
 * Designed to support loading GPS legacy navigation message 
 * data that is in the original format of three 300-bit subframes.
 */ 

#ifndef GPSTK_OrbElemLNav_HPP
#define GPSTK_OrbElemLNav_HPP

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

#include <string>
#include <iostream>

#include "OrbElem.hpp"
#include "FICData.hpp"

namespace gpstk
{
   class OrbElemLNav : public OrbElem
   {
   public:
         /// Default constructor
      OrbElemLNav();
   
         /**  Create an object based on the three subframes of navigation
          *   message data, the PRNID, and the week the data were transmitted.
          *   The SF1, SF2, and SF3 arrays hold the data collected from
          *   subframe 1, subframe 2, and subframe 3 respectively).  Each
          *   30-bit word of the navigation message is stored right-justificed
          *   in a single member of SF1, SF2, or SF3.   For example, Subframe 1,
          *   bits 1-30 are stored in the 30 lsb of SF1[0].
          *   XmitGPSWeek - The full GPS week the data were transmitted. This is  
          *   required in order to correctly set the GPS 1024-week "epoch"
          *   and correctly derive the complete epoch times.
          *   @throw InvalidParameter if the input data are inconsistent.
          */ 
      OrbElemLNav(   const long  SF1[10],
                     const long  SF2[10],
                     const long  SF3[10],
                     const SatID satIDArg,
                     const short XmitGPSWeek ) 
         throw( InvalidParameter);

         /** Load the object from the navigation message data contained in the
          *  arguments. Any existing data in the object is overwritten with the
          *  new data.
          *  See the corresponding constructor for a description of the arguments.
          *  @throw InvalidParameter if the input data are inconsistent.
          */ 

         /// Destructor
      virtual ~OrbElemLNav() {}

        /// Clone function
      virtual OrbElemLNav* clone() const;


      void loadData( const long  SF1[10],
                     const long  SF2[10],
                     const long  SF3[10],
                     const SatID satIDArg,
                     const short XmitGPSWeek ) 
         throw( InvalidParameter);

      virtual std::string getName() const
      {
         return "OrbElemLNav";
      }

      virtual std::string getNameLong() const
      {
         return "LNav SF 1/2/3";
      }
      
        /* Returns the upper bound of the URA range
         * @throw Invalid Request if the required data has not been stored
         */
      double getAccuracy()  const
         throw( InvalidRequest );

        /* Should only be used by GPSOrbElemStore::rationalize()
         */
      void adjustBeginningValidity();

      virtual void dumpHeader(std::ostream& s = std::cout) const
         throw( InvalidRequest );

        /** Generate a formatted human-readable one-line output that summarizes
          *  the critical times associated with this object and send it to the
          *  designated output stream (default to cout).
          *  @throw Invalid Parameter if the object has been instantiated, but not loaded.
          */   
      void dumpTerse(std::ostream& s = std::cout) const
         throw( InvalidRequest );


       /// Ephemeris overhead information
         //@{
      long  HOWtime[3];     /**< Time of subframe 1-3 (sec of week) */
      short ASalert[3];     /**< A-S and "alert" flags for each subframe. 
			        2 bit quantity with Alert flag the high
                                order bit and the A-S flag low order bit */
      CommonTime transmitTime; /** Estimated beginning time of this sample */

      short  codeflags;     /**< L2 codes */
      short  accFlag;       /**< User Range Accuracy flag */ 
      short  health;        /**< SV health */
      short  L2Pdata;       /**< L2 P data flag */
      short  IODC;          /**< Index of data-clock  */
      short  IODE;          /**< Index of data-eph    */
      short  fitint;        /**< Fit interval flag */
      double Tgd;           /**< L1 and L2 correction term */
      long   AODO;          /**< Age of Data offset from subframe 2.  Note: This field may not be present
                                 in older FIC data records.  A valid value will be greater than zero.  
                                 A value  of zero indicates the AODO is not available in this record. */ 
         //@}   

   }; // end class OrbElemLNav

   //@}

   std::ostream& operator<<(std::ostream& s, 
                                     const OrbElemLNav& eph);

} // end namespace

#endif // GPSTK_OrbElemLNav_HPP
