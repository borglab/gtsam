#pragma ident "$Id$"

/**
 * @file CNAVClock.hpp
 * GPS CNAV (L2C or L5) clock data encapsulated in engineering terms
 */

/**
*   This is one of four classes designed to contain GPS navigation message data.  The classes are
*      EngEphemeris -  Legacy GPS navigation message data from subframces 1,2,3 ( L1 C/A, L1 P(Y), L2 P(Y) )
*      CNAVEphemeris - GPS Civil navigation message data from Message Type 10/11 (L2C and L5)
*      CNAVClock - GPS Civil navigation message data from the "clock" portion of Message Types 30-37 (L2C and L5)
*      CNAV2EphClock - GPS Civil navigation message from subframe 2 of the L1C message
*/

#ifndef GPSTK_CNAVCLOCK_HPP
#define GPSTK_CNAVCLOCK_HPP

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

#include "Exception.hpp"
#include "BrcClockCorrection.hpp"
#include "PackedNavBits.hpp"

namespace gpstk
{
   /** @addtogroup ephemcalc */
   //@{

      /**
       * Clock information for a single satellite.  This class
       * encapsulates the clock navigation message (message types 30-37)
       * and provides functions to decode the as-broadcast
       * clock corrections.
       */
   class CNAVClock : public PackedNavBits
   {
   public:
         /// Default constructor
      CNAVClock() throw();

         /// Destructor
      virtual ~CNAVClock() {}

         /**
          * Store a subframe in this object.
	       * @param OsbID identifies the carrier and code from which the message was obtained. 
          * @param message3_ ten word navigation message stored in the
          * 30 least-significant bits of each array index.
          * @return true if successful.
          * @throw InvalidParameter if message data is invalid
	       *
	       * Note that a couple of items not in the legacy message are
	       * included in the message: PRN ID and 13 bit (full) week number.
          */
      void loadData( const ObsID obsIDArg, const short PRNIDArg,
                     const short TOWWeekArg, const PackedNavBits message3_)
         throw(gpstk::InvalidParameter);

      	/**
	 * Store data in the object
	 * In this case, the data may come from a file or other source
	 * and the navigation message format may no longer be present.
	 */
      void loadData( const std::string satSysArg, const ObsID obsIDArg,
                     const short PRNIDArg, const short AlertMsgArg,
                     const long TOWMsgArg, const short TOWWeekArg,
                     const long TopArg, const long TocArg,
                     const double accuracyArg, const short URAocArg,
                     const short URAoc1Arg, const short URAoc2Arg, 
                     const double af0Arg, const double af1Arg,
                     const double af2Arg );
       
         /**
          * Query presence of message number in this object.
          * @return true if the necessary data is present in this object.
          */
      bool hasData( );

         /** Returns the transmit time from the ephemeris */
      CommonTime getTransmitTime() const throw(gpstk::InvalidRequest);

         /** Returns the time of prediction */
      CommonTime getTimeOfPrediction() const throw(gpstk::InvalidRequest);

         /** Returns the epoch time (time of clock) from this ephemeris, correcting
          *  for half weeks. */
      CommonTime getClockEpoch() const throw(gpstk::InvalidRequest);
      
         /** This function returns the PRN ID of the SV. */
      short getPRNID() const throw(gpstk::InvalidRequest);
      
         /** This function returns the alert flag. */
      short getAlert() const throw(gpstk::InvalidRequest);
      
         /** This function returns the value of the SV accuracy (m)
          * computed from the accuracy flag in the nav message, or
          * as set by the setAccuracy() method. */
      double getAccuracy(const CommonTime& t) const throw(gpstk::InvalidRequest);

         /** This function returns the SV accuracy
          * index as it appears in the nav message. */
      short getURAoc(const short ndx) const 
         throw(gpstk::InvalidRequest,InvalidParameter);

         /** This function returns the time of prediction in GPS
          *  seconds of week. */
      long getTop() const throw(gpstk::InvalidRequest);

         /** This function returns the clock epoch in GPS seconds of
          * week. */
      double getToc() const throw(gpstk::InvalidRequest);

         /** Compute the satellite clock bias (sec) at the given time
          * @throw InvalidRequest if a required message type has not been stored.
          */
      double svClockBias(const CommonTime& t) const
         throw( gpstk::InvalidRequest );

         /** Compute the satellite clock drift (sec/sec) at the given time
          * @throw InvalidRequest if a required message type has not been stored.
          */
      double svClockDrift(const CommonTime& t) const
         throw( gpstk::InvalidRequest );

      	/** Return the clock parameter object */
      BrcClockCorrection getClock( ) const throw(gpstk::InvalidRequest);
     
         /** Output the contents of this ephemeris to the given stream. */
      void dump(std::ostream& s = std::cout) const throw();

   protected:
      bool dataLoaded;     /**< True if data is present, False otherwise */
      std::string satSys;  /**< System ID (based on RINEX definitins) */
      ObsID obsID;         /**< Defines carrier and code tracked */
      short PRNID;         /**< SV PRN ID */
      short Alert;         /**< "alert" flags for each subframe */
      long  TOWCount;      /**< TOW count associated with message 10/11 */
      long  Top;           /**< Time of Preditcion */
      double Toc;          /**< Clock epoch (sec of week) */
      short TOWWeek;       /**< GPS full week number that corresponds to the TOWtime of Message Type 10 */
      
         //@}
      
         /// Clock parameters
         //@{
      BrcClockCorrection bcClock;
         //@}

      friend std::ostream& operator<<(std::ostream& s, 
                                      const CNAVClock& eph);

   }; // class CNAVClock

   //@}

} // namespace

#endif
