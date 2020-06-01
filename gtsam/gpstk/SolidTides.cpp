#pragma ident "$Id$"

/**
 * @file SolidTides.cpp
 * Computes the effect of solid Earth tides on a given position and epoch.
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


#include "SolidTides.hpp"


namespace gpstk
{


      // Love numbers
   const double SolidTides::H_LOVE(0.609), SolidTides::L_LOVE(0.0852);

      // Phase lag. Assumed as zero here because no phase lag has been 
      // detected so far.
      // const double SolidTides::PH_LAG(0.0);


      /* Returns the effect of solid Earth tides (meters) at the given
       * position and epoch, in the Up-East-Down (UEN) reference frame.
       *
       * @param[in] t Epoch to look up
       * @param[in] p Position of interest
       *
       * @return a Triple with the solid tidal effect, in meters and in
       * the UED reference frame.
       *
       * @throw InvalidRequest If the request can not be completed for any
       * reason, this is thrown. The text may have additional information
       * as to why the request failed.
       */
   Triple SolidTides::getSolidTide(const CommonTime& t,
                                   const Position& p) const
      throw(InvalidRequest)
   {

         // We will store here the results
      Triple res;

         // Objects to compute Sun and Moon positions
      SunPosition  sunPosition;
      MoonPosition moonPosition;


      try
      {

            // Variables to hold Sun and Moon positions
         Triple sunPos(sunPosition.getPosition(t));
         Triple moonPos(moonPosition.getPosition(t));


            // Compute the factors for the Sun
         double rpRs( p.X()*sunPos.theArray[0] + 
                      p.Y()*sunPos.theArray[1] + 
                      p.Z()*sunPos.theArray[2]);

         double Rs2(sunPos.theArray[0]*sunPos.theArray[0] +
                    sunPos.theArray[1]*sunPos.theArray[1] +
                    sunPos.theArray[2]*sunPos.theArray[2]);

         double rp2( p.X()*p.X() + p.Y()*p.Y() + p.Z()*p.Z() );

         double xy2p( p.X()*p.X() + p.Y()*p.Y() );
         double sqxy2p( std::sqrt(xy2p) );

         double sqRs2(std::sqrt(Rs2));

         double fac_s( 3.0*MU_SUN*rp2/(sqRs2*sqRs2*sqRs2*sqRs2*sqRs2) );

         double g1sun( fac_s*(rpRs*rpRs/2.0 - rp2*Rs2/6.0) );

         double g2sun( fac_s * rpRs * (sunPos.theArray[1]*p.X() -
                       sunPos.theArray[0]*p.Y()) * std::sqrt(rp2)/sqxy2p );

         double g3sun( fac_s * rpRs * ( sqxy2p* sunPos.theArray[2] -
                       p.Z()/sqxy2p * (p.X()*sunPos.theArray[0] +
                       p.Y()*sunPos.theArray[1]) ) );


            // Compute the factors for the Moon
         double rpRm( p.X()*moonPos.theArray[0] + 
                      p.Y()*moonPos.theArray[1] + 
                      p.Z()*moonPos.theArray[2]);

         double Rm2(moonPos.theArray[0]*moonPos.theArray[0] +
                    moonPos.theArray[1]*moonPos.theArray[1] +
                    moonPos.theArray[2]*moonPos.theArray[2]);

         double sqRm2(std::sqrt(Rm2));

         double fac_m( 3.0*MU_MOON*rp2/(sqRm2*sqRm2*sqRm2*sqRm2*sqRm2) );

         double g1moon( fac_m*(rpRm*rpRm/2.0 - rp2*Rm2/6.0) );

         double g2moon( fac_m * rpRm * (moonPos.theArray[1]*p.X() -
                        moonPos.theArray[0]*p.Y()) * std::sqrt(rp2)/sqxy2p );

         double g3moon( fac_m * rpRm * ( sqxy2p* moonPos.theArray[2] -
                        p.Z()/sqxy2p * (p.X()*moonPos.theArray[0] +
                        p.Y()*moonPos.theArray[1]) ) );

            // Effects due to the Sun
         double delta_sun1(H_LOVE*g1sun);
         double delta_sun2(L_LOVE*g2sun);
         double delta_sun3(L_LOVE*g3sun);

            // Effects due to the Moon
         double delta_moon1(H_LOVE*g1moon);
         double delta_moon2(L_LOVE*g2moon);
         double delta_moon3(L_LOVE*g3moon);

            // Combined effect
         res.theArray[0] = delta_sun1 + delta_moon1;
         res.theArray[1] = delta_sun2 + delta_moon2;
         res.theArray[2] = delta_sun3 + delta_moon3;

      } // End of try block
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }

      return res;

   } // End SolidTides::getSolidTide


} // end namespace gpstk
