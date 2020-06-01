/**
 * @file Rinex3ObsData.hpp
 * Encapsulate RINEX observation file data, including I/O
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

#ifndef RINEX3OBSDATA_HPP
#define RINEX3OBSDATA_HPP

#include <vector>
#include <list>
#include <map>

#include "CommonTime.hpp"
#include "FFStream.hpp"
#include "Rinex3ObsBase.hpp"
#include "Rinex3ObsHeader.hpp"

namespace gpstk
{

   /// A structure used to store a single RINEX Data point.
   #ifndef GPSTK_RINEXDATUM
   #define GPSTK_RINEXDATUM
   struct RinexDatum
   {
      RinexDatum() : data(0), lli(0), ssi(0) {} ///< constructor
      double data;  ///< The actual data point.
      short lli;    ///< See the RINEX Spec. for an explanation.
      short ssi;    ///< See the RINEX Spec. for an explanation.
   };
   #endif // GPSTK_RINEXDATUM


      /** @addtogroup Rinex3Obs */
      //@{

      /// This class models a RINEX Observation Data Record.
      /// @sa gpstk::Rinex3ObsStream and gpstk::Rinex3ObsHeader.
      /// @sa rinex_obs_test.cpp and rinex_obs_read_write.cpp for examples.
   class Rinex3ObsData : public Rinex3ObsBase
   {
   public:

         /// Constructor.
      Rinex3ObsData() : time(gpstk::CommonTime::BEGINNING_OF_TIME)
         {}

         /// Destructor
      virtual ~Rinex3ObsData() {}


         /// Map from RinexSatID to RinexDatum; order of the data matches the
         /// order of RinexObsIDs in the header
      typedef std::map<RinexSatID, std::vector<RinexDatum> > DataMap;

         /// Time corresponding to the observations
      CommonTime time;


         /// Epoch flag has the following values
         /// 0 ok
         /// 1 power failure since previous epoch
         /// 2 start moving antenna
         /// 3 new site occupation (end moving antenna)
         ///   at least MARKER NAME header record follows
         /// 4 header records follow
         /// 5 external event
         /// 6 cycle slip record - same format as observation, but slips not data,
         ///   and LLI and SSI are blank
      short epochFlag;


         /// Number of satellites in this observation, except when
         /// epochFlag=2-5, then number of auxiliary header records to follow.
      short numSVs;


      double clockOffset;        ///< optional clock offset in seconds

      DataMap obs;               ///< the map of observations

      Rinex3ObsHeader auxHeader; ///< auxiliary header records (epochFlag 2-5)


         /** This method returns the RinexDatum of a given observation
          *
          * @param sat     Satellite whose observation we want to fetch.
          * @param index   Index representing the observation type. It is
          *                obtained from corresponding RINEX Observation Header
          *                using method 'Rinex3ObsHeader::getObsIndex()'.
          */
      virtual RinexDatum getObs( const SatID& sat, int index ) const
         throw(InvalidRequest);


         /** This method returns the RinexDatum of a given observation
          *
          * @param sat  Satellite whose observation we want to fetch.
          * @param type String representing the observation type.
          * @param hdr  RINEX Observation Header for current RINEX file.
          */
      virtual RinexDatum getObs( const SatID& sat,
                                 std::string type,
                                 const Rinex3ObsHeader& hdr ) const
         throw(InvalidRequest);


         /// A Debug output function.
         /// Dumps the time of observations and the IDs of the Sats
         /// in the map.
      virtual void dump(std::ostream& s) const;


         /// Another dump, using information from the header
      void dump(std::ostream& s, Rinex3ObsHeader& head) const;


   protected:


         /// Writes a correctly formatted record from this data to stream \a s.
         /// When printing comment records, you'll need to format them
         /// correctly yourself. This means making sure that "COMMENT" is at
         /// the end of the line and that they're the correct length
         /// (<= 80 chrs).
         /// Also, make sure to correctly set the epochFlag to the correct
         /// number for the type of header data you want to write.
      virtual void reallyPutRecord(FFStream& s) const
         throw( std::exception, FFStreamError,
                gpstk::StringUtils::StringException );


         /** This functions obtains a RINEX 3 Observation record from the given
          *  FFStream. If there is an error in reading from the stream, it is
          *  reset to its original position and its fail-bit is set.
          *
          *  Because of the Rinex 3 Obs format, a Rinex3ObsData record returned
          *  might not have data.  Check the Rinex3SatMap for empty() before
          *  using any data in it.
          *
          *  @throws StringException When a StringUtils function fails
          *  @throws FFStreamError   When exceptions(failbit) is set and a read
          *          or formatting error occurs.  Also resets the stream to its
          *          pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s)
         throw( std::exception, FFStreamError,
                gpstk::StringUtils::StringException );


   private:


         /// Writes the CommonTime into RINEX 3 format.
         /// If it's a bad time, it will return blanks.
      std::string writeTime(const CommonTime& dt) const
         throw( gpstk::StringUtils::StringException );


         /** This function constructs a CommonTime object from the given
          *  parameters.
          *
          * @param line The encoded time string found in the RINEX record.
          * @param hdr  The RINEX Observation Header object for the current
          *             RINEX file.
          */
      CommonTime parseTime( const std::string& line,
                            const Rinex3ObsHeader& hdr,
                            const TimeSystem& ts) const
         throw( FFStreamError );


   }; // End of class 'Rinex3ObsData'

      //@}

} // End of namespace gpstk

#endif   // GPSTK_RINEX3OBSDATA_HPP
