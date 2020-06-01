#pragma ident "$Id$"

/**
 * @file CNAV2EphClk.hpp
 * GPS CNAV-2 (L1C) ephemeris/clock data from Subframe 2 encapsulated 
 * in engineering terms
 */

/**
*   This is one of four classes designed to contain GPS navigation message data.  The classes are
*      EngEphemeris -  Legacy GPS navigation message data from subframces 1,2,3 ( L1 C/A, L1 P(Y), L2 P(Y) )
*      CNAVEphemeris - GPS Civil navigation message data from Message Type 10/11 (L2C and L5)
*      CNAVClock - GPS Civil navigation message data from the "clock" portion of Message Types 30-37 (L2C and L5)
*      CNAV2EphClock - GPS Civil navigation message from subframe 2 of the L1C message
*/

#ifndef GPSTK_CNAV2EPHCLK_HPP
#define GPSTK_CNAV2EPHCLK_HPP

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
#include "Xvt.hpp"
#include "BrcKeplerOrbit.hpp"
#include "BrcClockCorrection.hpp"
#include "PackedNavBits.hpp"

namespace gpstk
{
   /** @addtogroup ephemcalc */
   //@{

      /**
       * Ephemeris information for a single satellite.  This class
       * encapsulates the ephemeris navigation message (subframes 1-3)
       * and provides functions to decode the as-broadcast
       * ephemerides.
       */
   class CNAV2EphClk : public PackedNavBits
   {
   public:
         /// Default constructor
      CNAV2EphClk() throw();

         /// Destructor
      virtual ~CNAV2EphClk() {}

         /**
          * Store a subframe 2 in this object.
	       * @param OsbIDArg identifies the carrier and code from which the message was obtained. 
	       * @param PRNIDArg identifies the PRN ID of the transmitting SV
	       * @param subframe1 the immediately preceding subframe 1 data (needed for the TOI count)
          * @param message20 600 bit navigation message stored in the
          * 30 least-significant bits of each array index.
          * @return true if successful.
          * @throw InvalidParameter if message data is invalid
	       *
          */
      void loadData( const ObsID obsIDArg,
		               const short PRNIDArg,
                     const int subframe1,
                     const PackedNavBits subframe2)
         throw(gpstk::InvalidParameter);

      	/**
	       * Store data in the object
	       * In this case, the data may come from a file or other source
	       * and the navigation message format may no longer be present.
	       */
      void loadData( const std::string satSysArg, const ObsID obsIDArg,
                     const short PRNIDArg, const short TOWWeekArg,
                     const long TOWArg, const long TopArg,
                     const bool  L1HealthArg, const short URAoeArg,
                     const long ToeArg, const double delatAArg,
                     const double AdotArg, const double dnArg,
                     const double dndotArg, const double M0Arg,
                     const double eccArg, const double wArg, 
		               const double OMEGA0Arg, const double i0Arg,
		               const double OMEGAdotARg, const double idotArg,
		               const double CicArg, const double CisArg,
		               const double CrcArg, const double CrsArg,
		               const double CucArg, const double CusArg,
                     const short URAocArg, const short URAoc1Arg,
                     const short URAoc2Arg, const double Af0Arg,
                     const double Af1Arg, const double Af2Arg,
		               const double TgdArg, const double ISCL1cpArg,
                     const double ISCL1cdArg);

         /** 
          * Compute satellite position & velocity at the given time
          * using this ephemeris.
          * @throw InvalidRequest if a required subframe has not been stored.
          */
       Xvt svXvt(const CommonTime& t) const
         throw( gpstk::InvalidRequest );

         /**
          * Query presence of subframe in this object.
          * @return true if the necessary data is present in this object.
          */
      bool hasData( );

         /** Returns the transmit time from the ephemeris. */
      CommonTime getTransmitTime() const throw(gpstk::InvalidRequest);

         /** Returns the time of prediction */
      CommonTime getTimeOfPrediction() const throw(gpstk::InvalidRequest);
      
         /** This function returns the PRN ID of the SV. */
      short getPRNID() const throw(gpstk::InvalidRequest);

         /**
          * This function returns the SV accuracy (ephemeris-related user
          * range accuracy) index as it appears in the nav message. */
      short getURAoe() const throw(gpstk::InvalidRequest);

         /**
          * This function returns the SV accuracy (clock-related user range
          * accuracy) index as it appears in the nav message. */
      short getURAoc(const short ndx) const 
         throw(gpstk::InvalidRequest,InvalidParameter);

         /** 
          * This function returns the value of the inter-signal correction
          * for L1 or L2 P(Y) */
      double getTgd() const throw(gpstk::InvalidRequest);

         /** 
          * This function returns the value of the inter-signal correction
          * for L1Cp */
      double getISCL1cp() const throw(gpstk::InvalidRequest);

         /** 
          * This function returns the value of the inter-signal correction
          * for L1Cd */
      double getISCL1cd() const throw(gpstk::InvalidRequest);
      
         /** This function returns the value of the SV health flag. */
      short getHealth( ) const throw(gpstk::InvalidRequest);

         /** 
          * This function returns the value of the Time of Prediction
          * in GPS seconds of week. */
      long getTop() const throw(gpstk::InvalidRequest);      

         /** 
          * Compute satellite relativity correction (sec) at the given time
          * @throw InvalidRequest if a required subframe has not been stored.
          */
      double svRelativity(const CommonTime& t) const
         throw( gpstk::InvalidRequest );

         /** 
          * Compute the satellite clock bias (sec) at the given time
          * @throw InvalidRequest if a required subframe has not been stored.
          */
      double svClockBias(const CommonTime& t) const
         throw( gpstk::InvalidRequest );

         /** 
          * Compute the satellite clock drift (sec/sec) at the given time
          * @throw InvalidRequest if a required subframe has not been stored.
          */
      double svClockDrift(const CommonTime& t) const
         throw( gpstk::InvalidRequest );

      	/** Return the orbit parameter object */
      BrcKeplerOrbit getOrbit( ) const throw(gpstk::InvalidRequest);

      	/** Return the clock parameter object */
      BrcClockCorrection getClock( ) const throw(gpstk::InvalidRequest);
      
         /** Output the contents of this ephemeris to the given stream. */
      void dump(std::ostream& s = std::cout) const throw();

   protected:
         /// Ephemeris overhead information
         //@{
      bool  dataLoaded;    /**< True if data is present, False otherwise */
      std::string  satSys; /**< System ID (based on RINEX defintions */
      ObsID obsID;	      /**< Defines carrier and code tracked */ 
      short PRNID;         /**< SV PRN ID */
      long  TOWCount;      /**< TOW count associated with subframe 2 */
      long  Top;           /**< Time of prediction */
      short TOWWeek;       /**< GPS full week number that corresponds to the TOWtime of SF2 */
      short L1Health;      /**< SV health */
      double Tgd;
      double ISCL1cp;
      double ISCL1cd;
         //@}
      
         /// Orbit parameters
         //@{
      BrcKeplerOrbit orbit;
         //@}

	      /// Clock parameters
         //@{
      BrcClockCorrection bcClock;
         //@}

      friend std::ostream& operator<<(std::ostream& s, 
                                      const CNAV2EphClk& eph);

   }; // class CNAV2EphClk

   //@}

} // namespace

#endif
