#pragma ident "$Id$"

/**
 * @file CNAVEphemeris.hpp
 * GPS CNAV (L2C or L5) ephemeris data encapsulated in engineering terms
 */

/**
*   This is one of four classes designed to contain GPS navigation message data.  The classes are
*      EngEphemeris -  Legacy GPS navigation message data from subframces 1,2,3 ( L1 C/A, L1 P(Y), L2 P(Y) )
*      CNAVEphemeris - GPS Civil navigation message data from Message Type 10/11 (L2C and L5)
*      CNAVClock - GPS Civil navigation message data from the "clock" portion of Message Types 30-37 (L2C and L5)
*      CNAV2EphClock - GPS Civil navigation message from subframe 2 of the L1C message
*/

#ifndef GPSTK_CNAVEPHEMERIS_HPP
#define GPSTK_CNAVEPHEMERIS_HPP

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

#include "BrcKeplerOrbit.hpp"
#include "PackedNavBits.hpp"
#include "Exception.hpp"

namespace gpstk
{
   /** @addtogroup ephemcalc */
   //@{

      /**
       * Ephemeris information for a single satellite.  This class
       * encapsulates the ephemeris navigation message (message types 10 and 11)
       * and provides functions to decode the as-broadcast
       * ephemerides.
       */
   class CNAVEphemeris : public PackedNavBits
   {
   public:
         /// Default constructor
      CNAVEphemeris() throw();

         /// Destructor
      virtual ~CNAVEphemeris() {}

         /**
          * Store a subframe in this object.
	       * @param OsbID identifies the carrier and code from which the message was obtained. 
          * @param message10 ten word navigation message stored in the
          * 30 least-significant bits of each array index.
          * @param message11 ten word navigation message stored in the
          * 30 least-significant bits of each array index.
          * @return true if successful.
          * @throw InvalidParameter if message data is invalid
	       *
	       * Note that a couple of items not in the legacy message are
	       * included in the message: PRN ID and 13 bit (full) week number.
          */
      void loadData( const ObsID obsIDArg,
                    const short PRNIDArg,
                    const PackedNavBits message10, 
		              const PackedNavBits message11)
         throw(gpstk::InvalidParameter);

      	/**
	       * Store data in the object
	       * In this case, the data may come from a file or other source
	       * and the navigation message format may no longer be present.
	       */
      void loadData( const std::string satSysArg, const ObsID obsIDArg,
                     const short PRNIDArg, const short AlertMsg10Arg,
                     const long TOWMsg10Arg, const short AlertMsg11Arg, 
                     const long TOWMsg11Arg, const short TOWWeekArg,
                     const long TopArg, const short URAoeArg,
		               const short L1HealthArg, const short L2HealthArg, 
		               const short L5HealthArg, const double ToeArg,
	                  const double accuracyArg, const double CucArg, 
                     const double CusArg, const double CrcArg,
                     const double CrsArg, const double CicArg,
                     const double CisArg, const double M0Arg,
                     const double dnArg, const double dndotArg, 
                     const double eccArg, const double AArg, 
                     const double AdotArg, const double OMEGA0Arg,
                     const double i0Arg, const double wArg, 
		               const double deltaOMEGAdotARg, const double idotArg );
       
         /**
          * Query presence of message number in this object.
          * @return true if the necessary data is present in this object.
          */
      bool hasData( );

         /** Returns the transmit time from the ephemeris. */
      CommonTime getTransmitTime() const throw(gpstk::InvalidRequest);

         /** Returns the time of prediction */
      CommonTime getTimeOfPrediction() const throw(gpstk::InvalidRequest);
      
         /** This function returns the PRN ID of the SV. */
      short getPRNID() const throw(gpstk::InvalidRequest);
      
         /** This function returns the alert flag for either
          * message 10 or 11. */
      short getAlert(short messageNum) const throw(gpstk::InvalidRequest, Exception);
      
         /** This function returns the value of the SV accuracy (m)
          * computed from the accuracy flag in the nav message. */
      double getAccuracy() const throw(gpstk::InvalidRequest);

         /** This function returns the SV accuracy
          * index as it appears in the nav message. */
      short getURAoe() const throw(gpstk::InvalidRequest);
      
         /** This function returns the value of the SV health flag. */
      short getHealth( const ObsID::CarrierBand ) const throw(gpstk::InvalidRequest);

         /** This function returns the time of prediction in GPS
          *  seconds of week. */
      long getTop() const throw(gpstk::InvalidRequest);

         /** Compute satellite relativity correction (sec) at the given time
          * @throw InvalidRequest if a required subframe has not been stored.
          */
      double svRelativity(const CommonTime& t) const
         throw( gpstk::InvalidRequest );
   
      	/** Return the orbit parameter object */
      BrcKeplerOrbit getOrbit( ) const throw(gpstk::InvalidRequest);
      
         /** Compute satellite velocity/position at the given time
          * using this ephemeris.
          * @throw InvalidRequest if a required message type has not been stored.
          */
      Xv svXv(const CommonTime& t) const throw(gpstk::InvalidRequest);
      
      
         /** Output the contents of this ephemeris to the given stream. */
      void dump(std::ostream& s = std::cout) const throw();

   protected:
      bool dataLoaded;     /**< True if data is present, False otherwise */
      std::string satSys;  /**< System ID (based on RINEX definitins) */
      ObsID obsID;         /**< Defines carrier and code tracked */
      short PRNID;         /**< SV PRN ID */
      short Alert[2];      /**< A-S and "alert" flags for each subframe */
      long  TOWCount[2];   /**< TOW count associated with message 10/11 */
      long Top;            /**< Time of Preditcion */
      short TOWWeek;       /**< GPS full week number that corresponds to the TOWtime of Message Type 10 */
      short L1Health;      /**< SV health */
      short L2Health;
      short L5Health;
         //@}
      
         /// Orbit parameters
         //@{
      BrcKeplerOrbit orbit;
         //@}

      friend std::ostream& operator<<(std::ostream& s, 
                                      const CNAVEphemeris& eph);

   }; // class CNAVEphemeris

   //@}

} // namespace

#endif
