#pragma ident "$Id$"

/**
 * @file IonexData.cpp
 * Encapsulate IONEX file data, including I/O
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
//  Octavian Andrei - FGI ( http://www.fgi.fi ). 2008, 2009
//
//============================================================================


#include <cmath>

#include "StringUtils.hpp"
#include "IonexData.hpp"
#include "CivilTime.hpp"


using namespace std;
using namespace gpstk::StringUtils;


namespace gpstk
{
   const string IonexData::startTecMapString    =  "START OF TEC MAP";
   const string IonexData::startRmsMapString    =  "START OF RMS MAP";
   const string IonexData::startHgtMapString    =  "START OF HEIGHT MAP";
   const string IonexData::currentEpochString   =  "EPOCH OF CURRENT MAP";
   const string IonexData::dataBlockString      =  "LAT/LON1/LON2/DLON/H";
   const string IonexData::endTecMapString      =  "END OF TEC MAP";
   const string IonexData::endRmsMapString      =  "END OF RMS MAP";
   const string IonexData::endHgtMapString      =  "END OF HEIGHT MAP";
   const string IonexData::endOfFile            =  "END OF FILE";

   const IonexData::IonexValType IonexData::UN( "UN",
                                                "Unknown or Invalid",
                                                "unknown" );

   const IonexData::IonexValType IonexData::TEC( "TEC",
                                                 "Total Electron Content map",
                                                 "TECU" );

   const IonexData::IonexValType IonexData::RMS( "RMS",
                                                 "Root Mean Square error",
                                                 "TECU" );


   //--------------------------------------------------------------------
   //--------------------------------------------------------------------
   void IonexData::reallyPutRecord(FFStream& ffs) const
      throw( std::exception, FFStreamError, StringException )
   {

         // is there anything to write?
      if ( !valid || data.empty() )
         return;

      IonexStream& strm = dynamic_cast<IonexStream&>(ffs);
      string line;


         // write record opening TEC/RMS map
      line.clear();
      line += rightJustify( asString(mapID), 6 );
      line += string(54, ' ');

      if (type == IonexData::TEC)
      {
         line += leftJustify(startTecMapString,20);
      }
      else if (type == IonexData::RMS)
      {
         line += leftJustify(startRmsMapString,20);
      }
      else
      {
         FFStreamError err("This isn't a valid standard IONEX value type: " + 
            asString(type.description) );
         GPSTK_THROW(err);
      }
      strm << line << endl;
      strm.lineNumber++;


         // write epoch of current TEC/RMS map
      line.clear();
      line += writeTime(time);
      line += string(24, ' ');
      line += leftJustify(currentEpochString,20);
      strm << line << endl;
      strm.lineNumber++;


         // write TEC/RMS data sequence
      int nlat(dim[0]), nlon(dim[1]);

      for (int ilat = 0; ilat < nlat; ilat++)
      {
            // write record initializing a new TEC/RMS 
            // data block for latitude 'currLat'
         double currLat = lat[0] + ilat*lat[2];

         line.clear();
         line += string(2, ' ');
         line += rightJustify( asString(currLat,1), 6 );
         line += rightJustify( asString(lon[0],1), 6 );
         line += rightJustify( asString(lon[1],1), 6 );
         line += rightJustify( asString(lon[2],1), 6 );
         line += rightJustify( asString(hgt[0],1), 6 );
         line += string(28, ' ');
         line += leftJustify(dataBlockString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write single TEC/RMS data block
         line.clear();
         for (int ilon = 0; ilon < nlon; ilon++)
         {
            int index = ilat*dim[1]+ilon;

            double val = (data[index] != 999.9) ?
                         std::pow(10.0,-exponent)*data[index] : 9999.0;

               // we need to put there an integer, i.e., the neareast integer
            int valint = (val > 0.0) ? 
                         static_cast<int>(val+0.5) : static_cast<int>(val-0.5);
            line += rightJustify( asString<short>(valint), 5 );

            if (line.size() == 80)  // maximum 16 values per record
            {
               strm << line << endl;
               strm.lineNumber++;
               line.clear();
            }
            else if (ilon == nlon-1)  // last longitude
            {
               line += string(80-line.size(), ' ');
               strm << line << endl;
               strm.lineNumber++;
               line.clear();
            }

         }  // End of 'for (int ilon = 0; ilon < nlon; ilon++)'

      }  // End of 'for (int ilat = 0; ilat < nlat; ilat++)'


         // write closing TEC/RMS map
      line.clear();
      line += rightJustify( asString(mapID), 6 );
      line += string(54, ' ');

      if (type == IonexData::TEC)
      {
         line += leftJustify(endTecMapString,20);
      }
      else if (type == IonexData::RMS)
      {
         line += leftJustify(endRmsMapString,20);
      }
      else
      {
         FFStreamError err("This isn't a valid standard IONEX value type: " + 
            asString(type.description) );
         GPSTK_THROW(err);
      }
      strm << line << endl;
      strm.lineNumber++;

   }  // End of method 'IonexData::reallyPutRecord()'



      /** This function obtains a IONEX Data record from
       * the given FFStream.
       *
       * If there is an error reading the stream, it is reset
       * to its original position and its fail-bit is set.
       *
       * @throws StringException when a StringUtils function fails
       * @throws FFStreamError when exceptions(failbit) is set and
       *   a read or formatting error occurs.  This also resets the
       *   stream to its pre-read position.
       */
   void IonexData::reallyGetRecord(FFStream& ffs)
      throw( exception, FFStreamError, gpstk::StringUtils::StringException )
   {

      IonexStream& strm = dynamic_cast<IonexStream&>(ffs);

         // If the header has not been read, read it
      if(!strm.headerRead)
      {
         strm >> strm.header;
      }

         // Clear out this object
      IonexHeader& hdr = strm.header;

         // Let's be sure that nothing is set yet
      IonexData iod;
      *this = iod;

         // let's get some useful values from the header
      exponent = hdr.exponent;
      for (int i = 0; i < 3; i++)
      {
         lat[i] = hdr.lat[i];
         lon[i] = hdr.lon[i];
         hgt[i] = hdr.hgt[i];
      }


      string line;

         // some initializations before looping

         // ityp may be -1(initialize), 0(unknown), 1(TEC), 2(RMS), 3(HGT)
      int ityp(-1);

         // index for latitude
      int ilat(0);

      while (ityp != 0)
      {

         strm.formattedGetLine(line,true);
         StringUtils::stripTrailing(line);

         if (line.size() > 80)
         {
            FFStreamError e("Bad epoch line");
            GPSTK_THROW(e);
         }

            // skip empty lines
         if (line.length() == 0)
         {
            continue;
         }

         string label(line, 60, 20);

         if (label == startTecMapString)
         {

            type = IonexData::TEC;
            ityp = 1;
            mapID = asInt(line.substr(0,6));
            ilat = 0;

         }
         else if (label == startRmsMapString)
         {

            type = IonexData::RMS;
            ityp = 2;
            mapID = asInt(line.substr(0,6));
            ilat = 0;

         }
         else if (label == startHgtMapString)
         {

            ityp = 3;
            mapID = asInt(line.substr(0,6));
            ilat = 0;

         }
         else if (label == currentEpochString)
         {

            time = parseTime(line);

               // Get grid dimensions to know how much memory has to be
               // allocated for 'data' member of the object, and then
               // initialize all elements of the object member with dummy
               // values, i.e., 999.9
            dim[0] = static_cast<int>( ( hdr.lat[1] - hdr.lat[0] )
                            / hdr.lat[2] + 1 ); // consider Equator as well

            dim[1] = static_cast<int>( ( hdr.lon[1] - hdr.lon[0] )
                            / hdr.lon[2] + 1 ); // consider Greenwich meridian

            dim[2] = (hdr.hgt[2] == 0) ?
                     1 : static_cast<int>( (hdr.hgt[1]-hdr.hgt[0]) 
                                           / hdr.hgt[2]+1 );

            data.resize( dim[0]*dim[1]*dim[2], 999.9 );

         }
         else if (label == dataBlockString)
         {

            if (ityp == 0)
            {
               FFStreamError e(string("Map type undefined: " + line) );
               GPSTK_THROW(e);
            }

            double lat0,lon1,lon2,dlon,hgt;

            lat0 = asDouble(line.substr( 2,6));
            lon1 = asDouble(line.substr( 8,6));
            lon2 = asDouble(line.substr(14,6));
            dlon = asDouble(line.substr(20,6));
            hgt  = asDouble(line.substr(26,6));

               //read single data block
            for (int ival = 0,line_ndx = 0; ival < dim[1]; ival++, line_ndx++)
            {

                  // only 16 values per line - same as (line_ndx % 16 == 0)
               if (!(line_ndx % 16))
               {

                     // Get new line
                  strm.formattedGetLine(line);

                     // Set to zero again. There are only 16 values per line
                  line_ndx = 0;

                  if (line.size() > 80)
                  {

                    FFStreamError e("Error reading IONEX data. Bad epoch line");
                    GPSTK_THROW(e);

                  }

                     // skip empty lines if any
                  if (line.size() == 0)
                  {
                     continue;
                  }

               }  // End of 'if (!(line_ndx % 16))...'


                  // the 5th line doesn't have 80 characters, thus fill it
               line.resize(80, ' ');

                  // extract value
               int val = asInt(line.substr(line_ndx*5,5));

                  // add value
               data[ilat*dim[1]+ival] = (val != 9999) ?
                                        std::pow(10.0,exponent)*val : 999.9;

            }  // End of 'for (int ival = 0,line_ndx = 0; ival < dim[1];...'

               // next latitude
            ilat++;

         }  // End of 'if (label == dataBlockString)...'
         else if (label == endTecMapString)
         {

            ityp = 0;
            valid = true;

         }
         else if (label == endRmsMapString)
         {

            ityp = 0;
            valid = true;

         }
         else if (label == endOfFile)
         {

            // Remember that there is one more line in Ionex definition
            // before EOF. The IonexData object has been initialized already
            // but we mark the flag 'valid' as false. This helps when we shall
            // store this data into an IonexStore object.
            ityp = 0;
            valid = false;

         }
         else
         {

            FFStreamError e(string("Unidentified Ionex Data record " + line) );
            GPSTK_THROW(e);

         }

      }  // end of 'while (ityp != 0)' loop

      return;

   }  // End of method 'IonexData::reallyGetRecord()'



      // A debug output function.
   void IonexData::dump(std::ostream& os) const
   {

      os << endl;
      os << "IonexData dump() function"      << std::endl;
      os << "Epoch                       : " << time << std::endl;
      os << "Map index                   : " << mapID << std::endl;
      os << "Data type                   : " << type.type
                << " (" << type.units << ")" << std::endl;
      os << "Grid size (lat x lon x hgt) : " << dim[0]
                << " x " << dim[1]
                << " x " << dim[2] << std::endl;
      os << "Number of values            : " << data.size()
                << " values." << std::endl;
      os << "Valid object?               : " << isValid() << endl;

      return;

   }  // End of method 'IonexData::dump()'



      /** Get the position of a grid point based on input position
       *
       * @param in     input lat, lon and height (Triple object)
       * @param type   grid point to be returned
       *               (1) neareast grid point
       *               (2) lower left hand grid point
       * @param out    output lat, lon and height (Triple object)
       * @return       the index within the data
       *
       * @warning Keep in mind the assumptions of IONEX grid (i.e., latitude
       *          between [87.5, -87.5], longitude between [-180, 180])
       *          when you construct a Triple object.
       */
   int IonexData::getIndex( const Triple& in,
                            const int& igp,
                            Triple& ABC ) const
      throw(InvalidRequest)
   {

         // grid dimensions
      int nlat = dim[0];
      int nlon = dim[1];
      int nhgt = dim[2];

         // useful variables
      int ilat, ilon, ihgt, ncyc;
      double xlat, xlon, xhgt;

         // latitude
      xlat = (in[0] - lat[0]) / lat[2] + 1.0;

      ilat = (igp == 1) ?
             static_cast<int>(xlat+0.5) : static_cast<int>(xlat);

      if (ilat >= 1 && ilat <= nlat)
      {

         ABC[0] = lat[0] + (ilat-1)*lat[2];

      }
      else
      {

         InvalidRequest e( "Irregular latitude. Latitude "
                           + asString(in[0]) + " DEG" );

         GPSTK_THROW(e);

      }


         // longitude
      xlon = (in[1] - lon[0]) / lon[2] + 1.0;

      ilon = (igp == 1) ?
             static_cast<int>(xlon + 0.5) : static_cast<int>(xlon);

         // Round to neareast integer
      ncyc = static_cast<int>( ( 360.0 / std::abs(lon[2]) ) + 0.5 );


      if (ilon < 1)
      {

         ilon = ilon + ncyc;

      }
      else if (ilon > nlon)
      {

         ilon = ilon - ncyc;

      }


      if ( (ilon >= 1) && (ilon <= nlon) )
      {

         ABC[1] = lon[0] + (ilon-1) * lon[2];

      }
      else
      {

         InvalidRequest e( "Irregular longitude. Longitude: "
                           + asString(in[1]) + " DEG" );
         GPSTK_THROW(e);

      }

         // height
      if (hgt[2] == 0)
      {

         ihgt = 1;
         ABC[2] = hgt[0];

      }
      else
      {

         xhgt = (in[2]/1000.0 - hgt[0]) / hgt[2] + 1.0;

         ihgt = (igp == 1) ?
                static_cast<int>(xhgt + 0.5) : static_cast<int>(xhgt);

         if ( (ihgt >= 1) && (ihgt <= nhgt) )
         {

            ABC[2] = ( hgt[0] + (ihgt-1) * hgt[2] ) * 1000.0;  //meters

         }
         else
         {

            InvalidRequest e( "Irregular height. Height: "
                              + asString( in[2]/1000.0 ) + " km.");

            GPSTK_THROW(e);

         }  // End of 'if ( (ihgt >= 1) && (ihgt <= nhgt) )...'

      }  // End of 'if (hgt[2] == 0)...'


      return ( (ilon-1) + (ilat-1)*nlon + (ihgt-1)*nlon*nlat );

   }  // End of method 'IonexData::getIndex()'



      /** Get IONEX TEC or RMS value as a function of the position
       *  and nominal height.
       *
       * A simple 4-point formula is applied to interpolate between
       * grid points.
       *
       * For more information see page 3 of IONEX manual:
       *
       * http://igscb.jpl.nasa.gov/igscb/data/format/ionex1.pdf
       *
       * @param pos             input position (Position object).
       *
       * @return                Computed TEC or RMS value.
       *
       */
   double IonexData::getValue( const Position& p ) const
      throw(InvalidRequest,FFStreamError)
   {

         // this never should happen but just in case
      Position pos(p);
      if ( pos.getSystemName() != "Geocentric" )
      {

         InvalidRequest e( "Position object is not in GEOCENTRIC coordinates");

         GPSTK_THROW(e);

      }

         // some useful declarations
      Triple ABC[4], inarg;
      int e[4];
      double xsum = 0.0;

         // the object is required for AEarth to be consistent with 
         // Position::getIonosphericPiercePoint()
      WGS84Ellipsoid WGS84;

         // let's fetch the data
      double beta    = p.theArray[0];
      double lambda  = p.theArray[1];
      double height  = p.theArray[2]-WGS84.a();

         // we need this step because in the Position object the longitude is 
         // expressed in degrees E (i.e., [0 +360]), while in IONEX files 
         // longitude takes values within [-180 180]) (see IONEX manual) 
      if (lambda > 180.0)
      {
         lambda = lambda - 360.0;
      }

         // get position of lower left hand grid point E00
      inarg = Triple( beta, lambda, height);
      e[0] = getIndex( inarg, 2, ABC[0] );


         // compute factors P and Q
      double xp( (inarg[1] - ABC[0][1]) / lon[2] );
      double xq( (inarg[0] - ABC[0][0]) / lat[2] );

         // this never should happen but just in case
      if ( (xp < 0) || (xp > 1) || (xq < 0) || (xq > 1) )
      {

         throw(Exception("IonexData::getValue(): Wrong xp and xq factors!!!"));

      }

         // get E10's position index
      inarg = Triple( ABC[0][0], ABC[0][1]+lon[2], ABC[0][2] );
      e[1] = getIndex( inarg, 1, ABC[1] );

         // get E01's position index
      inarg = Triple( ABC[0][0]+lat[2], ABC[0][1], ABC[0][2] );
      e[2] = getIndex( inarg, 1, ABC[2] );

         // get E11's position index
      inarg = Triple( ABC[0][0]+lat[2], ABC[0][1]+lon[2], ABC[0][2] );
      e[3] = getIndex( inarg, 1, ABC[3] );

         // let's fetch the values
      double pntval[4];
      for (int i = 0; i < 4; i++)
      {

         double xval( data[e[i]] );

         if (xval != 999.9)
         {
            pntval[i] = xval;
         }
         else
         {
            FFStreamError e("Undefined TEC/RMS value(s).");
            GPSTK_THROW(e);
         }

      }  // End of 'for (int i = 0; i < 4; i++)...'

         // bivariate interpolation (pag.3, IONEX manual)
      xsum = (1.0-xp) * (1.0-xq) * pntval[0] +
                  xp  * (1.0-xq) * pntval[1] +
             (1.0-xp) *      xq  * pntval[2] +
                  xp  *      xq  * pntval[3];

      return xsum;

   }  // End of method 'IonexData::getValue()'



      /** This function constructs a CommonTime object from the given
       * parameters.
       *
       * @param line    Encoded time string found in the IONEX record.
       */
   CommonTime IonexData::parseTime( const std::string& line ) const
   {

      int year, month, day, hour, min, sec;

      year  = asInt(line.substr( 0,6));
      month = asInt(line.substr( 6,6));
      day   = asInt(line.substr(12,6));
      hour  = asInt(line.substr(18,6));
      min   = asInt(line.substr(24,6));
      sec   = asInt(line.substr(30,6));

      return CivilTime( year, month, day, hour, min, (double)sec );

   }  // End of method 'IonexData::parseTime()'



      /** Writes the CommonTime object into IONEX format. If it's a bad time,
       * it will return blanks.
       *
       * @param dt    time to be written into a IONEX data record.
       */
   string IonexData::writeTime(const CommonTime& dt) const
      throw(gpstk::StringUtils::StringException)
   {

      if (dt == CommonTime::BEGINNING_OF_TIME)
      {
         return string(36, ' ');
      }

      string line;
      line  = rightJustify(asString<short>(static_cast<CivilTime>(dt).year), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).month), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).day), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).hour), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).minute), 6);
      line += rightJustify(asString (static_cast<int>(static_cast<CivilTime>(dt).second)), 6);

      return line;

   }  // End of method 'IonexData::writeTime()'



}  // End of namespace gpstk
