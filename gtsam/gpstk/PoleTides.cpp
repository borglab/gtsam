#pragma ident "$Id$"

/**
 * @file PoleTides.cpp
 * Computes the effect of pole tides at a given position and epoch.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008
//
//============================================================================


#include "PoleTides.hpp"
#include "CivilTime.hpp"
#include "MJD.hpp"


namespace gpstk
{

      /* Returns the effect of pole tides (meters) at the given
       * position and epoch, in the Up-East-North (UEN) reference frame.
       *
       * @param[in]  t Epoch to look up
       * @param[in]  p Position of interest
       *
       * @return a Triple with the pole tide effect, in meters and in
       *    the UEN reference frame.
       *
       * @throw InvalidRequest If the request can not be completed for
       *    any reason, this is thrown. The text may have additional
       *    information about the reason the request failed.
       *
       * @warning In order to use this method, you must have previously
       *    set the current pole displacement parameters
       *
       */
   Triple PoleTides::getPoleTide( const CommonTime& t,
                                  const Position& p )
      throw(InvalidRequest)
   {

         // We will store here the results
      Triple res(0.0, 0.0, 0.0);

         // Declare J2000 reference time: January 1st, 2000, at noon
      const CivilTime j2000(2000, 1, 1, 12, 0, 0.0);


      try
      {

            // Get current position's latitude and longitude, in radians
         double latitude(p.geodeticLatitude()*DEG_TO_RAD);
         double longitude(p.longitude()*DEG_TO_RAD);

            // Compute appropriate running averages
            // Get time difference between current epoch and
            // J2000.0, in years
         double timedif((MJD(t).mjd - MJD(j2000).mjd)/365.25);

         double xpbar(0.054 + timedif*0.00083);
         double ypbar(0.357 + timedif*0.00395);

            // Now, compute m1 and m2 parameters
         double m1(xdisp-xpbar);
         double m2(ypbar-ydisp);

            // Now, compute some useful values
         double sin2lat(std::sin(2.0*latitude));
         double cos2lat(std::cos(2.0*latitude));
         double sinlat(std::sin(latitude));
         double sinlon(std::sin(longitude));
         double coslon(std::cos(longitude));

            // Finally, get the pole tide values, in UEN reference
            // frame and meters
         res[0] = -0.033 * sin2lat * ( m1*coslon + m2*sinlon );
         res[1] = +0.009 * sinlat  * ( m1*sinlon - m2*coslon );
         res[2] = -0.009 * cos2lat * ( m1*coslon + m2*sinlon );

            // Please be aware that the former equations take into account
            // that the IERS pole tide equations use CO-LATITUDE instead
            // of LATITUDE. See Wahr, 1985.

      } // End of try block
      catch(...)
      {

         InvalidRequest ir("Unknown error when computing pole tides.");
         GPSTK_THROW(ir);

      }

      return res;

   }  // End of method 'PoleTides::getPoleTide()'



      /* Method to set the pole displacement parameters
       *
       * @param x     Pole displacement x, in arcseconds
       * @param y     Pole displacement y, in arcseconds
       *
       * @return This same object
       */
   PoleTides& PoleTides::setXY( const double& x,
                                const double& y )
   {

      xdisp = x;
      ydisp = y;

      return (*this);

   }  // End of method 'PoleTides::setXY()'



}  // End of namespace gpstk
