#pragma ident "$Id$"

/**
 * @file BLQDataReader.hpp
 * File stream for ocean tides harmonics data in BLQ file format.
 */

#ifndef GPSTK_BLQDATAREADER_HPP
#define GPSTK_BLQDATAREADER_HPP

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
#include "Matrix.hpp"


namespace gpstk
{
      /** @addtogroup formattedfile */
      //@{

      /** This is a class to read and parse ocean tides harmonics data
       *  in BLQ file format.
       *
       * Ocean loading displacement models usually use the ocean tide
       * harmonics in order to compute station biases due to this effect.
       *
       * A common format to encode such information is the so-called BLQ
       * format, where each station name is associated to a matrix with
       * 11 columns (corresponding to the most important harmonics) and
       * six rows: Three for amplitudes (radial, west, south), and three
       * for phases (radial, west, south).
       *
       * You may find this data using the "Ocean tide loading provider" at:
       *
       * http://www.oso.chalmers.se/~loading/
       *
       * A typical way to use this class follows:
       *
       * @code
       *   BLQDataReader blqread;
       *
       *   blqread.open("EBRE.GOT00.2");
       *
       *   Matrix<double> tides(6,11,0.0);
       *
       *   tides = blqread.getTideHarmonics("EBRE");
       * @endcode
       *
       * The eleven tide harmonics used are:
       *
       * - M2:  Principal lunar semidiurnal
       * - S2:  Principal solar semidiurnal
       * - N2:  Larger lunar elliptic semidiurnal
       * - K2:  Lunisolar semidiurnal
       * - K1:  Lunar diurnal
       * - O1:  Lunar diurnal
       * - P1:  Solar diurnal
       * - Q1:  Larger lunar elliptic diurnal
       * - MF:  Lunisolar fortnightly
       * - MM:  Lunar monthly
       * - SSA: Solar semiannual
       *
       * @warning Be aware that you may select several different tide models
       * to generate tide harmonics. It is advised to use the latest
       * models such as GOT00.2, FES99, TPXO.6.2, etc.
       */
   class BLQDataReader : public FFTextStream
   {
   public:

         /// Default constructor
      BLQDataReader()
      {};

         /** Common constructor. It will always open file a for read and will
          *  load ocean tide harmonics data in one pass.
          *
          * @param fn   BLQ data file to read
          *
          */
      BLQDataReader(const char* fn)
         : FFTextStream(fn, std::ios::in)
      { loadData(); };


         /** Common constructor. It will always open file for read and will
          *  load ocean tide harmonics data in one pass.
          *
          * @param fn   BLQ data file to read
          *
          */
      BLQDataReader(const std::string& fn)
         : FFTextStream(fn.c_str(), std::ios::in)
      { loadData(); };

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
         /// Method to open AND load ocean tide harmonics data file. It doesn't
         /// clear data previously loaded.
      virtual void open(const char* fn);


         /// Method to open AND load ocean tide harmonics data file. It doesn't
         /// clear data previously loaded.
      virtual void open(const std::string& fn);

#pragma clang diagnostic pop
         /// Method to clear all previously loaded ocean tide harmonics data.
      virtual BLQDataReader& clearData()
      { OceanTidesData.clear(); return (*this); };


         /** Method to get the ocean tide harmonics corresponding to a
          *  given station.
          *
          * @param station   Station name (case is NOT relevant).
          *
          * @return A Matrix<double> of siw rows and eleven columns
          * containing tide harmonics M2, S2, N2, K2, K1, O1, P1, Q1, MF,
          * MM and SSA for amplitudes (radial, west, south, in meters) and
          * phases (radial, west, south, in degrees). If station is 
          * not found, this method will return a matrix full of zeros.
          */
      virtual Matrix<double> getTideHarmonics(const std::string& station);


         /// Destructor
      virtual ~BLQDataReader() {};


   private:


         /// A structure used to store ocean tide harmonics data.
      struct tideData
      {
            // Default constructor initializing the data in the structure
         tideData() : harmonics(6,11,0.0) {};

         Matrix<double> harmonics;   ///< Tide harmonics data
      };


         /// Handy iterator type
      typedef std::map<std::string, tideData>::const_iterator tideDataIt;


         /// Map holding the information regarding ocean tide harmonics
      std::map<std::string, tideData> OceanTidesData;


         /** Method to store ocean tide harmonics data in this class'
          *  data map
          *
          * @param stationName String holding station name.
          * @param data        tideData structure holding the harmonics data
          */
      void setData( const std::string& stationName,
                    const tideData& data )
      { OceanTidesData.insert(make_pair(stationName, data)); };


         /// Method to store ocean tide harmonics data in this class' data map
      virtual void loadData(void)
         throw( FFStreamError, gpstk::StringUtils::StringException );


   }; // End of class 'BLQDataReader'


      //@}

}  // End of namespace gpstk

#endif  // GPSTK_BLQDATAREADER_HPP
