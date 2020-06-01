#pragma ident "$Id$"

/**
 * @file SunPosition.cpp
 * Returns the approximate position of the Sun at the given epoch in the 
 * ECEF system.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007
//
//============================================================================


#include "SunPosition.hpp"
#include "CivilTime.hpp"
#include "YDSTime.hpp"


namespace gpstk
{


      // Time of the first valid time
   const CommonTime SunPosition::initialTime = CivilTime(1900, 3, 1, 0, 0, 0.0,TimeSystem::Any);

      // Time of the last valid time
   const CommonTime SunPosition::finalTime = CivilTime(2100, 2, 28, 0, 0, 0.0,TimeSystem::Any);


      // Returns the position of Sun ECEF coordinates (meters) at the 
      // indicated time.
      // @param[in] t the time to look up
      // @return the position of the Sun at time (as a Triple)
      // @throw InvalidRequest If the request can not be completed for any
      //    reason, this is thrown. The text may have additional
      //    information as to why the request failed.
   Triple SunPosition::getPosition(const CommonTime& t) const
      throw(InvalidRequest)
   {

         // Test if the time interval is correct
      if ( (t < SunPosition::initialTime) ||
           (t > SunPosition::finalTime) )
      {
         InvalidRequest ir("Provided epoch is out of bounds.");
         GPSTK_THROW(ir);
      }

         // We will store here the results
      Triple res;

      res = SunPosition::getPositionCIS(t);
      res = CIS2CTS(res, t);

      return res;
   } // End SunPosition::getPosition



      /* Function to compute Sun position in CIS system (coordinates 
       * in meters)
       * @param t Epoch
       */
   Triple SunPosition::getPositionCIS(const CommonTime& t) const
      throw(InvalidRequest)
   {

         // Test if the time interval is correct
      if ( (t < SunPosition::initialTime) ||
           (t > SunPosition::finalTime) )
      {
         InvalidRequest ir("Provided epoch is out of bounds.");
         GPSTK_THROW(ir);
      }

         // Compute the years, and fraction of year, pased since J1900.0
      int y(static_cast<YDSTime>(t).year);    // Current year
      int doy(static_cast<YDSTime>(t).doy);   // Day of current year
      double fd( (static_cast<YDSTime>(t).sod/86400.0 ) );   // Fraction of day
      int years( (y - 1900) );    // Integer number of years since J1900.0
      int iy4( ( ((y%4)+4)%4 ) ); // Is it a leap year?

         // Compute fraction of year
      double yearfrac = ( ( static_cast<double>(4*(doy-1/(iy4+1)) 
                            - iy4 - 2) + 4.0 * fd ) / 1461.0 );

      double time(years+yearfrac);

         // Compute the geometric mean longitude of the Sun
      double elm( fmod((4.881628 + TWO_PI*yearfrac + 
                        0.0001342*time), TWO_PI) );

         // Mean longitude of perihelion
      double gamma(4.90823 + 0.00030005*time);

         // Mean anomaly
      double em(elm-gamma);

         // Mean obliquity
      double eps0(0.40931975 - 2.27e-6*time);

         // Eccentricity
      double e(0.016751 - 4.2e-7*time);
      double esq(e*e);

         // True anomaly
      double v(em + 2.0*e*std::sin(em) + 1.25*esq*std::sin(2.0*em));

         // True ecliptic longitude
      double elt(v+gamma);

         // True distance
      double r( (1.0 - esq)/(1.0 + e*std::cos(v)) );

         // Moon's mean longitude
      double elmm( fmod((4.72 + 83.9971*time),TWO_PI) );

         // Useful definitions
      double coselt(std::cos(elt));
      double sineps(std::sin(eps0));
      double coseps(std::cos(eps0));
      double w1(-r*std::sin(elt));
      double selmm(std::sin(elmm));
      double celmm(std::cos(elmm));

      Triple result;

         // Sun position is the opposite of Earth position
      result.theArray[0] = (r*coselt+MeanEarthMoonBary*celmm)*AU_CONST;
      result.theArray[1] = (MeanEarthMoonBary*selmm-w1)*coseps*AU_CONST;
      result.theArray[2] = (-w1*sineps)*AU_CONST;

      return result;
   } // End SunPosition::getPositionCIS()


} // end namespace gpstk
