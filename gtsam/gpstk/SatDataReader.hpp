#pragma ident "$Id$"

/**
 * @file SatDataReader.hpp
 * File stream for satellite file data in PRN_GPS-like format.
 */

#ifndef GPSTK_SATDATAREADER_HPP
#define GPSTK_SATDATAREADER_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2009
//
//============================================================================




#include <string>
#include <map>
#include "FFTextStream.hpp"
#include "StringUtils.hpp"
#include "CommonTime.hpp"
#include "SatID.hpp"


namespace gpstk
{
      /** @addtogroup formattedfile */
      //@{

      /** This is a class to read and parse satellite data from
       *  PRN_GPS-like files.
       *
       *
       * Jet Propulsion Laboratory (JPL) provides a file called "PRN_GPS"
       * with satellite information such as launch and deactivation dates,
       * block type GPS number, etc. This information is important for some
       * precise GPS data processing algorithms, and is used in Gipsy/OASIS
       * software.
       *
       * You may find this file using FTP:
       *
       * ftp://sideshow.jpl.nasa.gov:/pub/gipsy_products/gipsy_params
       *
       * where the PRN_GPS file resides, usually compressed in .gz format.
       *
       * A typical way to use this class follows:
       *
       * @code
       *   SatDataReader satread;
       *
       *   SatID prn28(28, SatID::systemGPS);
       *   CommonTime time(1995, 331, 43200);
       *
       *   satread.open("PRN_GPS");
       *
       *   string prn28Block = satread.getBlock(prn28, time);
       *   // From 1992 to 1997, PRN 28 belonged to a block IIA satellite
       * @endcode
       *
       * @warning Be aware that PRN numbers are recycled, so several
       * different satellites may have the same PRN number at different
       * epochs. Therefore, you must provide the epoch of interest when
       * calling get methods.
       */
   class SatDataReader : public FFTextStream
   {
   public:

         /// Default constructor
      SatDataReader() {}

         /** Common constructor. It will always open file for read and will
          * load satellite data in one pass.
          *
          * @param fn   Satellite data file to read
          *
          */
      SatDataReader(const char* fn) : FFTextStream(fn, std::ios::in)
      { loadData(); }


         /** Common constructor. It will always open file for read and will
          * load satellite data in one pass.
          *
          * @param fn   Satellite data file to read
          *
          */
      SatDataReader(const std::string& fn) : FFTextStream(fn.c_str(), std::ios::in)
      { loadData(); }
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"

         /// Method to open AND load satellite data file.
      virtual void open(const char* fn);


         /// Method to open AND load satellite data file.
      virtual void open(const std::string& fn);
#pragma clang diagnostic pop

         /// Method to clear all previously loaded satellite data.
      virtual SatDataReader& clearData()
      { SatelliteData.clear(); return (*this); };


         /** Method to get the block type of a given SV at a given epoch.
          *
          * @param sat   Satellite ID.
          * @param epoch Epoch of interest.
          *
          * @return String containing satellite's block. If satellite is
          * not found or epoch is out of proper launch/deactivation bounds,
          * this method will return an empty string.
          */
      virtual std::string getBlock(const SatID& sat,
                                   const CommonTime& epoch) const;


         /** Method to get the GPS number of a given SV at a given epoch.
          *
          * @param sat   Satellite ID.
          * @param epoch Epoch of interest.
          *
          * @return Integer containing satellite's block. If satellite is
          * not found or epoch is out of proper launch/deactivation bounds,
          * this method will return -1.
          */
      virtual int getGPSNumber(const SatID& sat,
                               const CommonTime& epoch) const;


         /** Method to get the launch date of a given SV.
          *
          * @param sat   Satellite ID.
          * @param epoch Epoch of interest.
          *
          * @return CommonTime object containing satellite's launch date. If
          * satellite is not found or epoch is out of proper
          * launch/deactivation bounds, this method will return
          * CommonTime::END_OF_TIME.
          */
      virtual CommonTime getLaunchDate(const SatID& sat,
                                    const CommonTime& epoch) const;


         /** Method to get the deactivation date of a given SV.
          *
          * @param sat   Satellite ID.
          * @param epoch Epoch of interest.
          *
          * @return CommonTime object containing satellite's deactivation date.
          * If satellite is not found, epoch is out of proper
          * launch/deactivation bounds or satellite is still active, this
          * method will return CommonTime::BEGINNING_OF_TIME.
          */
      virtual CommonTime getDeactivationDate(const SatID& sat,
                                          const CommonTime& epoch) const;


         /// Destructor
      virtual ~SatDataReader() {}


   private:


         /// A structure used to store satellite data.
      struct svData
      {
            // Default constructor initializing the data in the structure
         svData() : launchDate(CommonTime::BEGINNING_OF_TIME),
                    deactivationDate(CommonTime::END_OF_TIME),
                    gpsNumber(0),
                    block("") {};

         CommonTime launchDate;         ///< SV launch date.
         CommonTime deactivationDate;   ///< SV deactivation date.
         int gpsNumber;              ///< GPS number.
         std::string block;               ///< Block the SV belongs to
      };


         /// Handy iterator type
      typedef std::multimap<SatID, svData>::const_iterator satDataIt;


         /// Map holding the information regarding every satellite
      std::multimap<SatID, svData> SatelliteData;


         /** Method to store satellite data in this class' data map
          * @param sat   Satellite ID.
          * @param data  svData structure holding the SV data
          */
      void setData(const SatID& sat,
                   const svData& data)
      { SatelliteData.insert(std::pair<const SatID, svData>(sat, data)); }


         /// Method to load satellite data in this class' data map
      virtual void loadData(void)
         throw(FFStreamError, gpstk::StringUtils::StringException);


   }; // End of class 'SatDataReader'


      //@}

}  // End of namespace gpstk

#endif  // GPSTK_SATDATAREADER_HPP
